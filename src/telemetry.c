#define _GNU_SOURCE

#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <poll.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/queue.h>

#include "telemetry.h"
#include "network.h"
#include "common.h"
#include "json.h"

struct tlm_q_entry {
	struct telemetry_data data;
	SIMPLEQ_ENTRY(tlm_q_entry) queue;
};

static pthread_t tlm_thr;
static pthread_mutex_t tlm_q_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t tlm_q_cond = PTHREAD_COND_INITIALIZER;
static SIMPLEQ_HEAD(tlm_q_head, tlm_q_entry) tlm_q_head = SIMPLEQ_HEAD_INITIALIZER(tlm_q_head);
static int tlm_q_count;

static int format_data(char **buf, size_t *len, const struct telemetry_data *d)
{
	FILE *f = open_memstream(buf, len);
	long long int ts = d->tv.tv_sec;

	xassert(f, return errno, "%d", errno);
	fprintf(f, "hvac.run.furnace_mode %d %lld\n", d->run.furnace_mode, ts);
	fprintf(f, "hvac.run.humid_mode %d %lld\n", d->run.humid_mode, ts);
	fprintf(f, "hvac.run.erv_mode %d %lld\n", d->run.erv_mode, ts);
	fprintf(f, "hvac.run.temp_sp_heat %.1f %lld\n", d->run.temp_sp_heat, ts);
	fprintf(f, "hvac.run.temp_sp_cool %.1f %lld\n", d->run.temp_sp_cool, ts);
	fprintf(f, "hvac.run.temp_thres %.1f %lld\n", d->run.temp_thres, ts);
	fprintf(f, "hvac.run.humid_sp %.1f %lld\n", d->run.humid_sp, ts);
	fprintf(f, "hvac.ctrl.furnace_blow %u %lld\n", d->ctrl.furnace_blow, ts);
	fprintf(f, "hvac.ctrl.furnace_heat %u %lld\n", d->ctrl.furnace_heat, ts);
	fprintf(f, "hvac.ctrl.furnace_cool %u %lld\n", d->ctrl.furnace_cool, ts);
	fprintf(f, "hvac.ctrl.humid_d_close %u %lld\n", d->ctrl.humid_d_close, ts);
	fprintf(f, "hvac.ctrl.humid_d_open %u %lld\n", d->ctrl.humid_d_open, ts);
	fprintf(f, "hvac.ctrl.humid_fan %u %lld\n", d->ctrl.humid_fan, ts);
	fprintf(f, "hvac.ctrl.humid_valve %u %lld\n", d->ctrl.humid_valve, ts);
	fprintf(f, "hvac.ctrl.erv_off %u %lld\n", d->ctrl.erv_off, ts);
	fprintf(f, "hvac.ctrl.erv_recirc %u %lld\n", d->ctrl.erv_recirc, ts);
	fprintf(f, "hvac.ctrl.erv_low %u %lld\n", d->ctrl.erv_low, ts);
	fprintf(f, "hvac.ctrl.erv_high %u %lld\n", d->ctrl.erv_high, ts);
	if (!isnan(d->sens.temp1))
		fprintf(f, "hvac.sens.th1.temp %.1f %lld\n", d->sens.temp1, ts);
	if (!isnan(d->sens.humid1))
		fprintf(f, "hvac.sens.th1.humid %.1f %lld\n", d->sens.humid1, ts);
	if (!isnan(d->sens.temp2))
		fprintf(f, "hvac.sens.th2.temp %.1f %lld\n", d->sens.temp2, ts);
	if (!isnan(d->sens.humid2))
		fprintf(f, "hvac.sens.th2.humid %.1f %lld\n", d->sens.humid2, ts);
	fclose(f);

	return 0;
}

static int dequeue_and_send(int sock)
{
	int rc;
	char *buf;
	size_t len;
	struct tlm_q_entry *entry;

	rc = pthread_mutex_lock(&tlm_q_mutex);
	xassert(!rc, return ENOTRECOVERABLE, "%d", rc);

	while (SIMPLEQ_EMPTY(&tlm_q_head)) {
		rc = pthread_cond_wait(&tlm_q_cond, &tlm_q_mutex);
		CHECK_CANCELED(goto abort_unlock);
		xassert(!rc, goto abort_unlock, "%d", rc);
	}

	entry = SIMPLEQ_FIRST(&tlm_q_head);
	SIMPLEQ_REMOVE_HEAD(&tlm_q_head, queue);
	tlm_q_count--;

	rc = pthread_mutex_unlock(&tlm_q_mutex);
	xassert(!rc, goto abort_free_entry, "%d", rc);

	rc = format_data(&buf, &len, &entry->data);
	free(entry);
	xassert(!rc, return ENOTRECOVERABLE, "%d", rc);

	rc = net_write(sock, buf, len);
	free(buf);
	if (!rc)
		return 0;
	if (errno == EINTR)
		return ENOTRECOVERABLE;
	xprintf(SD_ERR "Lost telemetry connection: %d\n", errno);

	return errno;
abort_unlock:
	rc = pthread_mutex_unlock(&tlm_q_mutex);
	xassert(!rc, NOOP, "%d", rc);
	return ENOTRECOVERABLE;
abort_free_entry:
	free(entry);
	return ENOTRECOVERABLE;
}

static void *tlm_main(void *arg)
{
	const char *host = json_get_string(arg, "host", "localhost");
	int port = json_get_number(arg, "port", 1, 65535, 2003);
	char pstr[6];
	int rc = 0, sock;

	snprintf(pstr, sizeof(pstr), "%d", port);

	while (rc != ENOTRECOVERABLE) {
		sock = net_connect_loop(host, pstr);
		if (sock < 0)
			break;
		while (!(rc = dequeue_and_send(sock)));
		close(sock);
	}

	return NULL;
}

int telemetry_init(cJSON *cfg)
{
	return pthread_create(&tlm_thr, NULL, tlm_main, cfg);
}

int telemetry_exit(void)
{
	int rc;

	/*
	 * The first SIGUSR2 sets the __canceled flag, but the thread may not
	 * be waiting in a syscall before a checkpoint. Keep sending signals to
	 * interrupt syscalls, and eventually the thread will reach a checkpoint.
	 * Since pthread_cond_wait() is not interrupted by signals, signal the
	 * condition every time, in addition to sending SIGUSR2.
	 *
	 * A delay is needed in the loop to give the target process enough time
	 * to progress. The thread is never ready to be joined immediately, so
	 * do the delay first, before trying to join.
	 */
	do {
		rc = pthread_kill(tlm_thr, SIGUSR2);
		xassert(!rc, return rc, "%d", rc);
		rc = pthread_cond_signal(&tlm_q_cond);
		xassert(!rc, return rc, "%d", rc);
		usleep(10000);
		rc = pthread_tryjoin_np(tlm_thr, NULL);
	} while (rc == EBUSY);

	xassert(!rc, return rc, "%d", rc);

	return 0;
}

int telemetry_send(const struct telemetry_data *data)
{
	struct tlm_q_entry *entry = malloc(sizeof(*entry));
	int rc, ret = 0;

	if (!entry)
		return -ENOMEM;

	rc = pthread_mutex_lock(&tlm_q_mutex);
	xassert(!rc, goto out_free, "%d", rc);

	if (tlm_q_count >= 7200) {
		xprintf(SD_WARNING "Telemetry queue is full; discarding data\n");
		ret = ENOSPC;
	} else {
		entry->data = *data;
		SIMPLEQ_INSERT_TAIL(&tlm_q_head, entry, queue);
		tlm_q_count++;
		pthread_cond_signal(&tlm_q_cond);
	}

	rc = pthread_mutex_unlock(&tlm_q_mutex);
	xassert(!rc, return rc, "%d", rc);

	return ret;

out_free:
	free(entry);
	return rc;
}
