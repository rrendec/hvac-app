#define _GNU_SOURCE

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <gpiod.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>
#include <math.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <modbus/modbus.h>
#include <civetweb.h>
#include <cjson/cJSON.h>

#include "common.h"

enum furnace_mode {
	FURNACE_OFF,
	FURNACE_FAN,
	FURNACE_HEAT,
	FURNACE_COOL,
	FURNACE_MAX = FURNACE_COOL
};

struct run_data {
	enum furnace_mode furnace_mode;
	int temp_sp_heat;
	int temp_sp_cool;
	int temp_thres;
	int hum_sp;
	// sync to disk delay counter
	int sync;
};

struct sensor_data {
	// AQ-N-LCD
	int temp1;
	int hum1;
	int aq;
	// XDUCER-D-TH
	int temp2;
	int hum2;
	// calculated
	int temp_avg;
	int hum_avg;
	int valid;
};

enum gpio_pins {
	GPIO_FURNACE_BLOW,
	GPIO_FURNACE_HEAT,
	GPIO_FURNACE_COOL,
#if 0
	GPIO_HUM_D_CLOSE,
	GPIO_HUM_D_OPEN,
#endif
	GPIO_HUM_FAN,
	GPIO_HUM_VALVE,
	NUM_GPIO_PINS
};

enum http_methods {
	HTTP_GET,
	HTTP_POST,
	NUM_HTTP_METHODS
};

#define TEMP_SP_HEAT_MIN	100
#define TEMP_SP_HEAT_MAX	300
#define TEMP_SP_COOL_MIN	150
#define TEMP_SP_COOL_MAX	350
#define HUM_SP_MIN		100
#define HUM_SP_MAX		500

const char * const rd_furnace_map[] = {
	[FURNACE_OFF]	= "off",
	[FURNACE_FAN]	= "fan",
	[FURNACE_HEAT]	= "heat",
	[FURNACE_COOL]	= "cool",
};

const char * const http_method_map[] = {
	[HTTP_GET]	= "GET",
	[HTTP_POST]	= "POST",
};

cJSON *config;

unsigned int gpio_pin_map[NUM_GPIO_PINS] = {
	[GPIO_FURNACE_BLOW]	= 17,	// GPIO_GEN0
	[GPIO_FURNACE_HEAT]	= 18,	// GPIO_GEN1
	[GPIO_FURNACE_COOL]	= 27,	// GPIO_GEN2
#if 0
	[GPIO_HUM_D_CLOSE]	= 23,	// GPIO_GEN4
	[GPIO_HUM_D_OPEN]	= 24,	// GPIO_GEN5
#endif
	[GPIO_HUM_FAN]		= 25,	// GPIO_GEN6
	[GPIO_HUM_VALVE]	= 4,	// GPCLK0
};

volatile int keep_going = 1;
volatile pid_t child_pid;

/* hw handles below */
struct gpiod_chip *chip;
struct gpiod_line_bulk bulk;
modbus_t *mb;
/* end of hw handles */

/* sensor data below */
struct sensor_data sd_snap;
pthread_mutex_t sd_mutex = PTHREAD_MUTEX_INITIALIZER;
/* end of sensor data */

/* run data below */
struct run_data rd_inst = {
	.furnace_mode = FURNACE_OFF,
	.temp_sp_heat = 220,	// 22.0 C
	.temp_sp_cool = 250,	// 25.0 C
	.temp_thres = 5,	// 0.5 C
	.hum_sp = 350,		// 35.0 %
};
pthread_mutex_t rd_mutex = PTHREAD_MUTEX_INITIALIZER;
/* end of run data */

int _map_find(const char * const *map, int size, const char *key)
{
	int i;

	for (i = 0; i < size; i++)
		if (strcmp(map[i], key) == 0)
			return i;

	return -1;
}

#define map_find(map, key) _map_find(map, ARRAY_SIZE(map), key)

const char *json_get_string(cJSON *json, const char *key, const char *_default)
{
	cJSON *node = cJSON_GetObjectItem(json, key);
	const char *value = cJSON_GetStringValue(node);

	return value ? value : _default;
}

#define json_map_string(json, key, _default, map) ({				\
	const char *str = json_get_string(json, key, _default);			\
	str ? map_find(map, str) : -1;						\
})

double json_get_number(cJSON *json, const char *key,
		       double min, double max, double _default)
{
	cJSON *node = cJSON_GetObjectItem(json, key);
	double value = cJSON_GetNumberValue(node);

	return isnan(value) ||
		(!isnan(min) && value < min) ||
		(!isnan(max) && value > max) ?
		_default : value;
}

int json_read(const char *path, cJSON **json, int *errl, int *errc)
{
	int fd, ret = 0;
	off_t len;
	char *buf;

	fd = open(path, O_RDONLY, 0);
	if (fd < 0)
		return errno;

	len = lseek(fd, 0, SEEK_END);
	if (len == (off_t)-1) {
		ret = errno;
		goto out_close;
	}

	buf = mmap(NULL, len, PROT_READ, MAP_SHARED, fd, 0);
	if (buf == MAP_FAILED) {
		ret = errno;
		goto out_close;
	}

	*json = cJSON_ParseWithLength(buf, len);

	if (!*json) {
		const char *err = cJSON_GetErrorPtr();
		const char *last = err, *p;
		int line = 1;

		for (p = buf; p < err; p++)
			if (*p == '\n') {
				line++;
				last = p + 1;
			}

		if (errl)
			*errl = line;
		if (errc)
			*errc = err - last;

		ret = EILSEQ;
	}

	munmap(buf, len);
out_close:
	close(fd);

	return ret;
}

int json_write(const char *path, cJSON *json)
{
	FILE *f = fopen(path, "w");
	char *str;
	int rc;

	if (!f)
		return errno;

	str = cJSON_Print(json);
	rc = fwrite(str, strlen(str), 1, f) ? 0 : EIO;
	free(str);

	rc = fclose(f) ? errno : rc;

	return rc;
}

static inline const char *cfg_get_string(const char *key, const char *_default)
{
	return json_get_string(config, key, _default);
}

static inline double cfg_get_number(const char *key,
				    double min, double max, double _default)
{
	return json_get_number(config, key, min, max, _default);
}

static inline const char *nvram_path(void)
{
	return cfg_get_string("nvram_path", "nvram.json");
}

/*
 * This function is called once during initialization, before other threads
 * (such as civetweb) are started. No synchronization is required to access
 * rd_inst. Furthermore, we can use the default values from the rd_inst
 * initializer to avoid duplicating the default values.
 */
int nvram_read(void)
{
	cJSON *json = NULL;
	int rc;

	rc = json_read(nvram_path(), &json, NULL, NULL);
	if (rc)
		return rc;

	rd_inst.furnace_mode = json_get_number(json, "furnace_mode",
		0, FURNACE_MAX, rd_inst.furnace_mode);
	rd_inst.temp_sp_heat = json_get_number(json, "temp_sp_heat",
		TEMP_SP_HEAT_MIN, TEMP_SP_HEAT_MAX, rd_inst.temp_sp_heat);
	rd_inst.temp_sp_cool = json_get_number(json, "temp_sp_cool",
		TEMP_SP_COOL_MIN, TEMP_SP_COOL_MAX, rd_inst.temp_sp_cool);
	rd_inst.hum_sp = json_get_number(json, "hum_sp",
		HUM_SP_MIN, HUM_SP_MAX, rd_inst.hum_sp);

	cJSON_Delete(json);

	return 0;
}

/*
 * This function assumes exclusive access to rd_inst. It is called either:
 *   - during initialization, before other threads (such as civetweb) are
 *     started, and in this case no synchronization is needed; or
 *   - in the delay loop, and in this case rd_mutex is locked externally.
 */
int nvram_write(void)
{
	cJSON *json = cJSON_CreateObject();
	const char *cfg_path;
	char *tmp_path = NULL;
	int rc = ENOMEM;

	xprintf(SD_DEBUG "Sync run mode data to non-volatile storage\n");

	cJSON_AddItemToObject(json, "furnace_mode", cJSON_CreateNumber(rd_inst.furnace_mode));
	cJSON_AddItemToObject(json, "temp_sp_heat", cJSON_CreateNumber(rd_inst.temp_sp_heat));
	cJSON_AddItemToObject(json, "temp_sp_cool", cJSON_CreateNumber(rd_inst.temp_sp_cool));
	cJSON_AddItemToObject(json, "hum_sp", cJSON_CreateNumber(rd_inst.hum_sp));

	cfg_path = nvram_path();
	if (asprintf(&tmp_path, "%s~", cfg_path) >= 0) {
		if ((rc = json_write(tmp_path, json)))
			unlink(tmp_path);
		else
			rc = rename(tmp_path, cfg_path) ? errno : 0;
		free(tmp_path);
	}

	cJSON_Delete(json);

	return rc;
}

int sensor_read(modbus_t *mb, struct sensor_data *data)
{
	uint16_t reg[2];
	int rc, ret = 0;

	memset(data, 0, sizeof(*data));

	// Note: Delay required between consecutive Modbus transactions with
	//       the same sensor, or else the sensor will ignore the second
	//       transaction. Experimentally, 5 ms is too little and 8 ms is
	//       okay. Use 10 ms just to be on the safe side.

	rc = modbus_set_slave(mb, 1);
	xassert(!rc, goto read2, "%d", errno);

	usleep(10000);
	rc = modbus_read_registers(mb, 136, 2, reg);
	xassert(rc != -1, ret = errno, "%d", errno);
	data->temp1 = reg[0];
	data->hum1 = reg[1];

	usleep(10000);
	rc = modbus_read_registers(mb, 184, 1, reg);
	xassert(rc != -1, ret = errno, "%d", errno);
	data->aq = reg[0];

read2:
	rc = modbus_set_slave(mb, 2);
	xassert(!rc, return errno, "%d", errno);

	usleep(10000);
	rc = modbus_read_registers(mb, 34, 2, reg);
	xassert(rc != -1, ret = errno, "%d", errno);
	data->temp2 = reg[0] + 9; // FIXME: hard-coded calibration
	data->hum2 = reg[1] - 44; // FIXME: hard-coded calibration

	if (ret)
		return ret;

	data->temp_avg = (data->temp1 + 3 * data->temp2) / 4;
	data->hum_avg = (data->hum1 + 3 * data->hum2) / 4;
	data->valid = 1;

	return 0;
}

void sig_hdlr(int signal)
{
	keep_going = 0;

	if (child_pid > 0)
		kill(child_pid, SIGTERM);
}

/**
 * @note gpiod_line_request_output() additionally sets the pin state. The side
 *       effect is that we initialize the output pins in a clean state. For
 *       example, the furnace is turned off.
 */
int gpio_init(void)
{
	int i, rc;

	chip = gpiod_chip_open_by_name("gpiochip0");
	xassert(chip, return errno, "%d", errno);

	rc = gpiod_chip_get_lines(chip, gpio_pin_map, NUM_GPIO_PINS, &bulk);
	xassert(!rc, return errno, "%d", errno);

	for (i = 0; i < NUM_GPIO_PINS; i++) {
		rc = gpiod_line_request_output(bulk.lines[i], "hvac", 1);
		xassert(!rc, return errno, "%d", errno);
	}

	return 0;
}

void gpio_cleanup(void)
{
	gpiod_line_release_bulk(&bulk);
	gpiod_chip_close(chip);
}

int modbus_init(void)
{
	int rc;

	mb = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
	xassert(mb, return errno, "%d", errno);

	rc = modbus_connect(mb);
	xassert(!rc, return errno, "%d", errno);

	return 0;
}

void modbus_cleanup(void)
{
	modbus_close(mb);
	modbus_free(mb);
}

void sensors_print(struct sensor_data *sd)
{
	xprintf(SD_DEBUG
		"T1=%.01f T2=%.01f H1=%.01f H2=%.01f AQ=%d\n",
		sd->temp1 / 10.0, sd->temp2 / 10.0,
		sd->hum1 / 10.0, sd->hum2 / 10.0,
		sd->aq);
}

int sensors_once(void)
{
	struct sensor_data sd;
	int rc;

	rc = modbus_init();
	xassert(!rc, return rc, "%d", rc);

	sensor_read(mb, &sd);
	sensors_print(&sd);

	modbus_cleanup();

	return 0;
}

int loop_1_sec(void)
{
	static enum {STATE_OFF, STATE_ON} state = STATE_OFF;
	static int sens_cnt, sens_fail, hum_cnt, hum_duty;
	static int old_furnace_mode = FURNACE_OFF;
	static int furnace_holdoff;

	struct sensor_data sd = {.valid = 0};

	if (sens_cnt) {
		sens_cnt = (sens_cnt + 1) % 5;
		sd = sd_snap;
	} else if (sensor_read(mb, &sd)) {
		if (sens_fail++ >= 30) {
			xprintf(SD_ERR, "Sensor failure\n");
			return 1;
		}
	} else {
		sens_fail = 0;
		sensors_print(&sd);
		pthread_mutex_lock(&sd_mutex);
		sd_snap = sd;
		pthread_mutex_unlock(&sd_mutex);
		sens_cnt++;
	}

	pthread_mutex_lock(&rd_mutex);

	if (rd_inst.furnace_mode != old_furnace_mode) {
		xprintf(SD_NOTICE "Furnace mode: %s\n",
			rd_furnace_map[rd_inst.furnace_mode]);
		furnace_holdoff = 5;
		state = STATE_OFF;
		old_furnace_mode = rd_inst.furnace_mode;
	}

	switch (rd_inst.furnace_mode) {
	case FURNACE_OFF:
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_BLOW], 1);
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 1);
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 1);
		break;
	case FURNACE_FAN:
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_BLOW], 0);
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 1);
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 1);
		break;
	case FURNACE_HEAT:
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_BLOW], 0);
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 1);
		if (furnace_holdoff) {
			furnace_holdoff--;
			break;
		}
		if (!sd.valid)
			break;
		if (sd.temp_avg >= rd_inst.temp_sp_heat + rd_inst.temp_thres &&
		    state == STATE_ON) {
			xprintf(SD_NOTICE "HEAT OFF\n");
			gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 1);
			state = STATE_OFF;
			break;
		}
		if (sd.temp_avg <= rd_inst.temp_sp_heat - rd_inst.temp_thres &&
		    state == STATE_OFF) {
			xprintf(SD_NOTICE "HEAT ON\n");
			gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 0);
			state = STATE_ON;
		}
		break;
	case FURNACE_COOL:
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_BLOW], 0);
		gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 1);
		if (furnace_holdoff) {
			furnace_holdoff--;
			break;
		}
		if (!sd.valid)
			break;
		if (sd.temp_avg >= rd_inst.temp_sp_cool + rd_inst.temp_thres &&
		    state == STATE_OFF) {
			xprintf(SD_NOTICE "COOL ON\n");
			gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 0);
			state = STATE_ON;
			break;
		}
		if (sd.temp_avg <= rd_inst.temp_sp_cool - rd_inst.temp_thres &&
		    state == STATE_ON) {
			xprintf(SD_NOTICE "COOL OFF\n");
			gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 1);
			state = STATE_OFF;
		}
		break;
	}

	if (rd_inst.furnace_mode == FURNACE_HEAT) {
		if (state == STATE_ON) {
			gpiod_line_set_value(bulk.lines[GPIO_HUM_FAN], 1);
			hum_duty = 20;
		} else {
			gpiod_line_set_value(bulk.lines[GPIO_HUM_FAN], 0);
			hum_duty = 10;
		}

		if (hum_cnt == 0)
			gpiod_line_set_value(bulk.lines[GPIO_HUM_VALVE], 0);
		else if (hum_cnt >= hum_duty)
			gpiod_line_set_value(bulk.lines[GPIO_HUM_VALVE], 1);
		hum_cnt = (hum_cnt + 1) % 30;
	} else {
		gpiod_line_set_value(bulk.lines[GPIO_HUM_FAN], 1);
		gpiod_line_set_value(bulk.lines[GPIO_HUM_VALVE], 1);
		hum_cnt = 0;
	}

	if (rd_inst.sync == 1) {
		nvram_write();
		rd_inst.sync = 0;
	} else if (rd_inst.sync)
		rd_inst.sync--;

	pthread_mutex_unlock(&rd_mutex);

	return 0;
}

long long cv_read_buf(struct mg_connection *conn, void *buf, long long blen)
{
	const struct mg_request_info *ri = mg_get_request_info(conn);
	long long tlen, rlen;

	if (ri->content_length >= 0)
		blen = MIN(blen, ri->content_length);

	for (tlen = 0; tlen < blen; tlen += rlen) {
		rlen = mg_read(conn, buf + tlen, blen - tlen);
		if (rlen <= 0)
			return tlen;
	}

	return tlen;
}

int cv_hdlr_sensor_data_get(struct mg_connection *conn, void *cbdata)
{
	struct sensor_data sd;
	cJSON *rsp_json = cJSON_CreateObject();
	char *rsp_str;
	unsigned long len;

	pthread_mutex_lock(&sd_mutex);
	sd = sd_snap;
	pthread_mutex_unlock(&sd_mutex);

	cJSON_AddItemToObject(rsp_json, "curr_temp", cJSON_CreateNumber(sd.temp_avg / 10.0));
	cJSON_AddItemToObject(rsp_json, "curr_hum", cJSON_CreateNumber(sd.hum_avg / 10.0));

	rsp_str = cJSON_Print(rsp_json);
	cJSON_Delete(rsp_json);

	len = strlen(rsp_str);
	mg_send_http_ok(conn, "application/json", len);
	mg_write(conn, rsp_str, len);
	free(rsp_str);

	return 200;
}

const mg_request_handler cv_hmap_sensor_data[NUM_HTTP_METHODS] = {
	[HTTP_GET]	= cv_hdlr_sensor_data_get,
};

int cv_hdlr_run_data_get(struct mg_connection *conn, void *cbdata)
{
	struct run_data rd;
	cJSON *rsp_json = cJSON_CreateObject();
	char *rsp_str;
	unsigned long len;

	pthread_mutex_lock(&rd_mutex);
	rd = rd_inst;
	pthread_mutex_unlock(&rd_mutex);

	cJSON_AddItemToObject(rsp_json, "furnace_mode",
			      cJSON_CreateString(rd_furnace_map[rd.furnace_mode]));
	cJSON_AddItemToObject(rsp_json, "temp_sp_heat",
			      cJSON_CreateNumber(rd.temp_sp_heat / 10.0));
	cJSON_AddItemToObject(rsp_json, "temp_sp_cool",
			      cJSON_CreateNumber(rd.temp_sp_cool / 10.0));
	cJSON_AddItemToObject(rsp_json, "hum_sp",
			      cJSON_CreateNumber(rd.hum_sp / 10.0));

	rsp_str = cJSON_Print(rsp_json);
	cJSON_Delete(rsp_json);

	len = strlen(rsp_str);
	mg_send_http_ok(conn, "application/json", len);
	mg_write(conn, rsp_str, len);
	free(rsp_str);

	return 200;
}

int cv_hdlr_run_data_post(struct mg_connection *conn, void *cbdata)
{
	const char *ctype = mg_get_header(conn, "content-type");
	char buf[2048];
	long long len;
	cJSON *req;
	int idx, chg = 0;
	double val;

	if (ctype && strcmp(ctype, "application/json")) {
		mg_send_http_error(conn, 400, "Invalid content type");
		return 400;
	}

	len = cv_read_buf(conn, buf, sizeof(buf));
	req = cJSON_ParseWithLength(buf, len);

	if (!req) {
		const char *err = cJSON_GetErrorPtr();
		mg_send_http_error(conn, 400, "Parse error at offset %zu",
				   err - buf);
		return 400;
	}

	pthread_mutex_lock(&rd_mutex);
	if ((idx = json_map_string(req, "furnace_mode", NULL, rd_furnace_map)) >= 0) {
		chg |= rd_inst.furnace_mode != idx;
		rd_inst.furnace_mode = idx;
	}
	if (!isnan(val = json_get_number(req, "temp_sp_heat",
	    TEMP_SP_HEAT_MIN/10.0, TEMP_SP_HEAT_MAX/10.0, NAN))) {
		int x = val * 10.0;
		chg |= rd_inst.temp_sp_heat != x;
		rd_inst.temp_sp_heat = x;
	}
	if (!isnan(val = json_get_number(req, "temp_sp_cool",
	    TEMP_SP_COOL_MIN/10.0, TEMP_SP_COOL_MAX/10.0, NAN))) {
		int x = val * 10.0;
		chg |= rd_inst.temp_sp_cool != x;
		rd_inst.temp_sp_cool = x;
	}
	if (!isnan(val = json_get_number(req, "hum_sp",
	    HUM_SP_MIN/10.0, HUM_SP_MAX/10.0, NAN))) {
		int x = val * 10.0;
		chg |= rd_inst.hum_sp != x;
		rd_inst.hum_sp = x;
	}
	if (chg)
		rd_inst.sync = 2;
	pthread_mutex_unlock(&rd_mutex);

	cJSON_Delete(req);

	mg_send_http_ok(conn, NULL, 0);

	return 200;
}

const mg_request_handler cv_hmap_run_data[NUM_HTTP_METHODS] = {
	[HTTP_GET]	= cv_hdlr_run_data_get,
	[HTTP_POST]	= cv_hdlr_run_data_post,
};

int cv_hdlr_api(struct mg_connection *conn, void *cbdata)
{
	const struct mg_request_info *ri = mg_get_request_info(conn);
	int method_idx = map_find(http_method_map, ri->request_method);
	mg_request_handler *hmap = cbdata, hdlr = NULL;

	if (method_idx >= 0)
		hdlr = hmap[method_idx];

	if (hdlr)
		return hdlr(conn, cbdata);

	mg_send_http_error(conn, 405, "%s", "");

	return 405;
}

int worker(void)
{
	static const struct itimerspec its = {
		.it_interval = {1, 0},
		.it_value = {1, 0},
	};
	static const struct mg_callbacks cv_cbk = {
	};

	char http_port[6];
	const char *cv_opt[] = {
		"document_root", cfg_get_string("document_root", "web"),
		"listening_ports", http_port,
		NULL
	};

	int i, rc, tfd;
	struct mg_context *cv_ctx;

	rc = nvram_read();
	if (rc == ENOENT)
		rc = nvram_write();
	xassert(!rc, return rc, "%d", rc);

	rc = gpio_init();
	xassert(!rc, return rc, "%d", rc);
	rc = modbus_init();
	xassert(!rc, return rc, "%d", rc);

	mg_init_library(0);
	snprintf(http_port, sizeof(http_port), "%d",
		(int)cfg_get_number("http_port", 1, 65535, 8080));
	cv_ctx = mg_start(&cv_cbk, NULL, cv_opt);
	mg_set_request_handler(cv_ctx, "/api/rundata", cv_hdlr_api, (void *)cv_hmap_run_data);
	mg_set_request_handler(cv_ctx, "/api/sensordata", cv_hdlr_api, (void *)cv_hmap_sensor_data);

	tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
	xassert(tfd != -1, goto out_stop, "%d", errno);

	rc = timerfd_settime(tfd, 0, &its, NULL);
	xassert(rc != -1, goto out_clo_tfd, "%d", errno);

	while (keep_going) {
		uint64_t overruns = 0;

		if (loop_1_sec())
			break;

		do {
			rc = read(tfd, &overruns, sizeof(overruns));
		} while (rc == -1 && errno == EINTR && keep_going);

		if (!keep_going)
			break;

		xassert(rc == sizeof(overruns), continue, "%d %d", rc, errno);

		if (overruns > 1)
			xprerrf(SD_ERR "Timer overrun: %"PRIu64"\n", overruns);
	}

	xprintf(SD_INFO "Shutting down...\n");
out_clo_tfd:
	close(tfd);
out_stop:
	mg_stop(cv_ctx);
	mg_exit_library();

	/*
	 * Handle pending rd_inst sync, in case rd_inst has been modified after
	 * we got the stop signal or shortly before. At this point all extra
	 * threads (civetweb) are stopped, so no locking is needed.
	 */
	if (rd_inst.sync)
		nvram_write();

	/*
	 * Turn the furnace off. GPIO pins keep their state, and we must make
	 * sure we don't leave heating or cooling running.
	 */
	for (i = 0; i < NUM_GPIO_PINS; i++)
		gpiod_line_set_value(bulk.lines[i], 1);

	gpio_cleanup();
	modbus_cleanup();

	return 0;
}

int parse_config(const char *path)
{
	int l = 0, c = 0;
	int rc = json_read(path, &config, &l, &c);

	if (rc == EILSEQ) {
		xprerrf("%s: Parse error at line %d column %d\n", path, l, c);
	} else if (rc) {
		xprerrf("%s: %s\n", path, strerror(rc));
	}

	return rc;
}

void global_cleanup(void)
{
	cJSON_Delete(config);
}

int main(int argc, char **argv)
{
	static const struct sigaction act = {.sa_handler = sig_hdlr};

	int opt, rc;
	int fg = 0, fail = 0, help = 0;
	pid_t wpid_ret;
	int wstatus = 0;

	atexit(global_cleanup);

	while ((opt = getopt(argc, argv, "c:fh:s")) != -1) {
		switch (opt) {
		case 'c':
			if (parse_config(optarg))
				fail = 1;
			break;
		case 'f':
			fg = 1;
			break;
		case 'h':
			help = 1;
			break;
		case 's':
			return sensors_once() ? EXIT_FAILURE : EXIT_SUCCESS;
		default:
			fail = 1;
		}
	}

	if (help || fail) {
		fprintf(stderr,
			"Usage: %s [options]\n"
			"Options:\n"
			"  -c file   Read configuration from file\n"
			"  -f        Foreground mode\n"
			"  -h        Show this help\n"
			"  -m mode   Active mode: heat or cool\n"
			"  -s        Read all sensors once and exit\n",
			argv[0]);
		return fail ? EXIT_FAILURE : EXIT_SUCCESS;
	}

	sigaction(SIGINT, &act, NULL);
	sigaction(SIGQUIT, &act, NULL);
	sigaction(SIGTERM, &act, NULL);
	signal(SIGUSR1, SIG_IGN);
	signal(SIGUSR2, SIG_IGN);

	if (!fg) {
		child_pid = fork();
		xassert(child_pid != -1, return EXIT_FAILURE, "%d", errno);
	}

	if (!child_pid) {
		rc = worker();
		return rc ? EXIT_FAILURE : EXIT_SUCCESS;
	}

	/* Supervisor code follows... */

	xprintf(SD_INFO "Started worker process %d\n", child_pid);

	/*
	 * In case we got a signal before we put the child process pid into the
	 * child_pid variable, make sure we send a signal to the child. It
	 * doesn't hurt if we send the signal twice.
	 */
	if (!keep_going)
		kill(child_pid, SIGTERM);

	wpid_ret = TEMP_FAILURE_RETRY(waitpid(child_pid, &wstatus, 0));
	xassert(wpid_ret > 0, return EXIT_FAILURE, "%d", errno);
	xassert(wpid_ret == child_pid, return EXIT_FAILURE);

	if (!WIFEXITED(wstatus)) {
		xprintf(SD_ERR "Worker terminated abnormally... cleaning up\n");
		rc = gpio_init();
		xassert(!rc, return EXIT_FAILURE, "%d", rc);
		/*
		 * No explicit pin control. Calling gpio_init() is enough to put
		 * all output pins in a clean state. See function comment.
		 */
		gpio_cleanup();
	}

	return EXIT_SUCCESS;
}
