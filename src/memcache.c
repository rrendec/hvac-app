#define _GNU_SOURCE

#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/timerfd.h>

#include "common.h"
#include "network.h"
#include "memcache.h"
#include "json.h"
#include "gsdata.h"

static pthread_t mc_thr;
static int tfd = -1;
static int pfd[2] = {-1, -1};

/*
 * Read a line from the server and handle extra data.
 *
 * The assumption is that there is extra data after the "\r\n" line
 * terminator. Make the line itsel a null-terminated string by replacing
 * the '\r' with '\0', and return a pointer to the trailing data in xdata
 * and the length of the trailing data as the function return value.
 */
static int mc_read_response(int sock, char *buf, size_t len, char **xdata)
{
	int rc;
	char *p;

	rc = net_read_line(sock, buf, len);
	if (rc < 0)
		return rc;

	p = memmem(buf, rc, "\r\n", 2);
	if (!p) {
		errno = EPROTO;
		return -1;
	}

	*p = '\0';
	p += 2;

	if (xdata)
		*xdata = p;

	return rc - (p - buf);
}

static int mc_cmd_get(int sock, const char *key, void **data, size_t *len, int *flags)
{
	static const char trail_str[] = "\r\nEND\r\n";

	char buf[MEMCACHE_MAX_CMD];
	char *xdptr, *_data;
	int rc, xdlen, tlen, _flags, _len;

	rc = snprintf(buf, sizeof(buf), "get %s\r\n", key);
	xassert(rc < sizeof(buf), return ENOMEM, "%d", rc);

	rc = net_write(sock, buf, rc);
	xassert(rc != -1, return errno, "%d", errno);

	xdlen = mc_read_response(sock, buf, sizeof(buf), &xdptr);
	xassert(xdlen != -1, return errno, "%d", errno);

	if (!strcmp(buf, "END"))
		return ENOENT;

	if (sscanf(buf, "VALUE %*s %d %d", &_flags, &_len) != 2)
		return EPROTO;

	if (_len > MEMCACHE_MAX_ITEM)
		return ENOMEM;

	/*
	 * Expect extra bytes for the trail_str sequence at the end. For short
	 * variable data, the entire response comes in one network packet and
	 * fits into our buffer, so it's fetched by the first read.
	 */
	tlen = _len + STATIC_STRLEN(trail_str);
	_data = malloc(tlen);
	if (!_data)
		return ENOMEM;

	memcpy(_data, xdptr, xdlen);
	if (xdlen < tlen) {
		rc = net_read(sock, _data + xdlen, tlen - xdlen);
		xassert(rc != -1, goto out_free, "%d", errno);
	}
	if (STATIC_STRNCMP(&_data[_len], trail_str)) {
		errno = EPROTO;
		goto out_free;
	}

	_data[_len] = '\0';
	*data = _data;
	*len = _len;
	*flags = _flags;

	return 0;

out_free:
	free(_data);
	return errno;
}

static int mc_handle_timer(int sock)
{
	int rc, flags;
	uint64_t overruns;
	char *data;
	size_t len;
	cJSON *json, *node;
	struct ext_data ed;

	rc = RETRY_NC(read(tfd, &overruns, sizeof(overruns)));
	xassert(rc == sizeof(overruns), return ENOTRECOVERABLE, "%d", errno);

	rc = mc_cmd_get(sock, "outdoor.aggregate", (void **)&data, &len, &flags);
	xassert(!rc, return rc, "%d", rc);

	json = cJSON_ParseWithLength(data, len);
	free(data);
	xassert(json, return EILSEQ);

	rc = EINVAL;

	node = json_get_object_path(json, STR_ARRAY("weather", "temperature"));
	xassert(cJSON_IsNumber(node), goto out_free);
	ed.temp = round(cJSON_GetNumberValue(node) * 10.0);

	node = json_get_object_path(json, STR_ARRAY("weather", "humidity"));
	xassert(cJSON_IsNumber(node), goto out_free);
	ed.humid = round(cJSON_GetNumberValue(node) * 10.0);

	node = json_get_object_path(json, STR_ARRAY("weather", "pressure"));
	xassert(cJSON_IsNumber(node), goto out_free);
	ed.pres = cJSON_GetNumberValue(node);

	node = json_get_object_path(json, STR_ARRAY("airquality", "aqi", "epa"));
	xassert(cJSON_IsNumber(node), goto out_free);
	ed.aqi = cJSON_GetNumberValue(node);

	node = cJSON_GetObjectItem(json, "dt");
	xassert(cJSON_IsNumber(node), goto out_free);
	ed.dt = cJSON_GetNumberValue(node);

	rc = pthread_mutex_lock(&gs_mutex);
	xassert(!rc, goto out_free, "%d", rc);

	gs_ed = ed;

	rc = pthread_mutex_unlock(&gs_mutex);
	xassert(!rc, goto out_free, "%d", rc);

	rc = 0;

out_free:
	cJSON_Delete(json);
	return rc;
}

static int mc_handle_update(int sock)
{
	return 0;
}

static int mc_handle_event(int sock)
{
	struct pollfd fds[] = {
		{tfd, POLLIN},
		{pfd[0], POLLIN},
	};
	int rc;

	rc = RETRY_NC(poll(fds, ARRAY_SIZE(fds), -1), return ENOTRECOVERABLE);
	xassert(rc != -1, return ENOTRECOVERABLE, "%d", errno);

	if ((fds[0].revents & POLLIN)) {
		rc = mc_handle_timer(sock);
		if (rc)
			return rc;
	}

	if ((fds[1].revents & POLLIN)) {
		rc = mc_handle_update(sock);
		if (rc)
			return rc;
	}

	return 0;
}

static void *mc_main(void *arg)
{
	const char *host = json_get_string(arg, "host", "localhost");
	int port = json_get_number(arg, "port", 1, 65535, 11211);
	char pstr[6];
	int rc = 0, sock;

	pthread_setname_np(pthread_self(), "memcache");
	snprintf(pstr, sizeof(pstr), "%d", port);

	while (rc != ENOTRECOVERABLE && rc != EINTR) {
		sock = net_connect_loop(host, pstr);
		if (sock < 0)
			break;
		while (!(rc = mc_handle_event(sock)));
		close(sock);
	}

	return NULL;
}

int memcache_init(cJSON *cfg)
{
	static const struct itimerspec its = {
		.it_interval = {5, 0},
		.it_value = {1, 0},
	};
	int rc;

	tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
	xassert(tfd != -1, return errno, "%d", errno);

	rc = timerfd_settime(tfd, 0, &its, NULL);
	xassert(rc != -1, goto out_clo_tfd, "%d", errno);

	rc = pipe2(pfd, O_CLOEXEC);
	xassert(rc != -1, goto out_clo_tfd, "%d", errno);

	return pthread_create(&mc_thr, NULL, mc_main, cfg);

out_clo_tfd:
	rc = errno;
	close(tfd);

	return rc;
}

int memcache_exit(void)
{
	int rc;

	/* See telemetry_exit() for implementation details. */
	do {
		rc = pthread_kill(mc_thr, SIGUSR2);
		xassert(!rc, return rc, "%d", rc);
		usleep(10000);
		rc = pthread_tryjoin_np(mc_thr, NULL);
	} while (rc == EBUSY);
	xassert(!rc, NOOP, "%d", rc);

	close(tfd);

	return rc;
}

#if 0
int memcache_send(const struct memcache_data *data)
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
#endif
