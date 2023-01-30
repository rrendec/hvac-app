#define _GNU_SOURCE

#include <netdb.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/queue.h>

#include "telemetry.h"
#include "common.h"
#include "json.h"

struct tlm_q_entry {
	struct telemetry_data data;
	SIMPLEQ_ENTRY(tlm_q_entry) queue;
};

static volatile int sfd = -1;
static volatile int is_locked;
static pthread_t tlm_thr;
static pthread_mutex_t tlm_q_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t tlm_q_cond = PTHREAD_COND_INITIALIZER;
static SIMPLEQ_HEAD(tlm_q_head, tlm_q_entry) tlm_q_head = SIMPLEQ_HEAD_INITIALIZER(tlm_q_head);
static int tlm_q_count;

static void resolve_and_connect(const char *host, const char *port)
{
	static const struct addrinfo hints = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_STREAM,
	};

	struct addrinfo *ares, *ap;
	int rc;

	for (;;) {
		rc = getaddrinfo(host, port, &hints, &ares);
		xassert(!rc, ares = NULL, "%d %d", rc, errno);

		for (ap = ares; ap; ap = ap->ai_next) {
			sfd = socket(ap->ai_family, ap->ai_socktype, ap->ai_protocol);
			xassert(sfd != -1, continue, "%d", errno);

			rc = connect(sfd, ap->ai_addr, ap->ai_addrlen);
			if (!rc) {
				freeaddrinfo(ares);
				xprintf(SD_INFO "Connected to %s:%s\n", host, port);
				return;
			}

			close(sfd);
		}

		freeaddrinfo(ares);
		xprerrf(SD_NOTICE "Connect to %s:%s failed: %d\n", host, port, errno);

		sleep(60);
	}
}

static void cleanup_hdlr(void *arg)
{
	if (is_locked)
		pthread_mutex_unlock(&tlm_q_mutex);

	close(sfd);
}

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
	fprintf(f, "hvac.sens.th1.temp %.1f %lld\n", d->sens.temp1, ts);
	fprintf(f, "hvac.sens.th1.humid %.1f %lld\n", d->sens.humid1, ts);
	fprintf(f, "hvac.sens.th2.temp %.1f %lld\n", d->sens.temp2, ts);
	fprintf(f, "hvac.sens.th2.humid %.1f %lld\n", d->sens.humid2, ts);
	fclose(f);

	return 0;
}

static int dequeue_and_send(void)
{
	int rc;
	char *buf, *ptr;
	size_t len;
	struct tlm_q_entry *entry;

	rc = pthread_mutex_lock(&tlm_q_mutex);
	xassert(!rc, return ENOTRECOVERABLE, "%d", rc);

	/*
	 * No race condition here, because cancellation is deferred
	 * and pthread_mutex_lock is not a cancellation point.
	 */
	is_locked = 1;

	/*
	 * If the thread is cancelled while waiting, the mutex is
	 * reacquired before the cancellation handlers run.
	 */
	while (SIMPLEQ_EMPTY(&tlm_q_head)) {
		rc = pthread_cond_wait(&tlm_q_cond, &tlm_q_mutex);
		xassert(!rc, return ENOTRECOVERABLE, "%d", rc);
	}

	entry = SIMPLEQ_FIRST(&tlm_q_head);
	SIMPLEQ_REMOVE_HEAD(&tlm_q_head, queue);
	tlm_q_count--;

	rc = pthread_mutex_unlock(&tlm_q_mutex);
	xassert(!rc, return ENOTRECOVERABLE, "%d", rc);

	/*
	 * No race condition here, because cancellation is deferred
	 * and pthread_mutex_unlock is not a cancellation point.
	 */
	is_locked = 0;

	rc = format_data(&buf, &len, &entry->data);
	free(entry);
	xassert(!rc, return rc, "%d", rc);
	for (ptr = buf; ptr < buf + len; ptr += rc) {
		rc = TEMP_FAILURE_RETRY(write(sfd, ptr, buf + len - ptr));
		if (rc <= 0) {
			free(buf);
			xprintf(SD_ERR "Lost telemetry connection: %d %d\n",
				rc, errno);
			return rc ? (errno ? errno : EIO) : EIO;
		}
	}
	free(buf);

	return 0;
}

static void *tlm_main(void *arg)
{
	const char *host = json_get_string(arg, "host", "localhost");
	int port = json_get_number(arg, "port", 1, 65535, 2003);
	char pstr[6];
	int rc = 0;

	snprintf(pstr, sizeof(pstr), "%d", port);

	pthread_cleanup_push(cleanup_hdlr, NULL);
	while (rc != ENOTRECOVERABLE) {
		resolve_and_connect(host, pstr);
		while (!(rc = dequeue_and_send()));
	}
	pthread_cleanup_pop(1);

	return NULL;
}

int telemetry_init(cJSON *cfg)
{
	return pthread_create(&tlm_thr, NULL, tlm_main, cfg);
}

int telemetry_exit(void)
{
	int rc;

	rc = pthread_cancel(tlm_thr);
	xassert(!rc, return rc, "%d", rc);
	rc = pthread_join(tlm_thr, NULL);
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
