#define _GNU_SOURCE

#include <errno.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <gpiod.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/timerfd.h>
#include <modbus/modbus.h>
#include <civetweb.h>

#include "common.h"
#include "json.h"
#include "telemetry.h"

enum furnace_mode {
	FURNACE_OFF,
	FURNACE_FAN,
	FURNACE_HEAT,
	FURNACE_COOL,
	FURNACE_MAX = FURNACE_COOL
};

enum erv_mode {
	ERV_OFF,
	ERV_RECIRC,	// Recirculate
	ERV_I20MH,	// Intermittent - 20 min/hour
	ERV_I30MH,	// Intermittent - 30 min/hour
	ERV_I40MH,	// Intermittent - 40 min/hour
	ERV_LOW,	// Always on, low speed
	ERV_HIGH,	// Always on, high speed
	ERV_MAX = ERV_HIGH
};

enum std_on_off {
	STD_OFF,
	STD_ON,
};

/* Running mode configuration/settings */
// TODO: better name
struct run_data {
	enum furnace_mode furnace_mode;
	enum std_on_off humid_mode;
	enum erv_mode erv_mode;
	int temp_sp_heat;
	int temp_sp_cool;
	int temp_thres;
	int humid_sp;
	// sync to disk delay counter
	int sync;
};

/* Discrete output control data */
// TODO: better name
struct ctrl_data {
	enum std_on_off furnace_blow;
	enum std_on_off furnace_heat;
	enum std_on_off furnace_cool;
	enum std_on_off humid_d_close;
	enum std_on_off humid_d_open;
	enum std_on_off humid_fan;
	enum std_on_off humid_valve;
	enum std_on_off erv_off;
	enum std_on_off erv_recirc;
	enum std_on_off erv_low;
	enum std_on_off erv_high;
};

/* Sensor data - raw and computed */
struct sensor_data {
	// AQ-N-LCD
	int temp1;
	int humid1;
	int aq;
	// XDUCER-D-TH
	int temp2;
	int humid2;
	// calculated
	int temp_avg;
	int humid_avg;
	int valid;
};

enum gpio_pins {
	GPIO_FURNACE_BLOW,
	GPIO_FURNACE_HEAT,
	GPIO_FURNACE_COOL,
	GPIO_HUMID_D_CLOSE,
	GPIO_HUMID_D_OPEN,
	GPIO_HUMID_FAN,
	GPIO_HUMID_VALVE,
	GPIO_ERV_OFF,
	GPIO_ERV_RECIRC,
	GPIO_ERV_LOW,
	GPIO_ERV_HIGH,
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
#define HUMID_SP_MIN		100
#define HUMID_SP_MAX		500

const char * const rd_furnace_map[] = {
	[FURNACE_OFF]	= "off",
	[FURNACE_FAN]	= "fan",
	[FURNACE_HEAT]	= "heat",
	[FURNACE_COOL]	= "cool",
};

const char * const rd_erv_map[] = {
	[ERV_OFF]	= "off",
	[ERV_RECIRC]	= "recirc",
	[ERV_I20MH]	= "i20mh",
	[ERV_I30MH]	= "i30mh",
	[ERV_I40MH]	= "i40mh",
	[ERV_LOW]	= "low",
	[ERV_HIGH]	= "high",
};

const char * const std_on_off_map[] = {
	[STD_OFF]	= "off",
	[STD_ON]	= "on",
};

const char * const http_method_map[] = {
	[HTTP_GET]	= "GET",
	[HTTP_POST]	= "POST",
};

const struct {
	int run_time_s;
	int off_time_s;
} erv_int_map[ERV_MAX] = {
	[ERV_I20MH] = {20 * 60, 40 * 60},
	[ERV_I30MH] = {30 * 60, 30 * 60},
	[ERV_I40MH] = {40 * 60, 20 * 60},
};

pthread_mutex_t oestream_mutex = PTHREAD_MUTEX_INITIALIZER;
cJSON *config;
cJSON *telemetry_cfg;

unsigned int gpio_pin_map[NUM_GPIO_PINS] = GPIO_MAP_INITIALIZER;
const int gpio_def_val[NUM_GPIO_PINS] = {[0 ... NUM_GPIO_PINS - 1] = 1};

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
	.erv_mode = ERV_OFF,
	.temp_sp_heat = 220,	// 22.0 C
	.temp_sp_cool = 250,	// 25.0 C
	.temp_thres = 5,	// 0.5 C
	.humid_sp = 350,	// 35.0 %
};
struct ctrl_data cd_inst = {
	.furnace_blow = STD_OFF,
	.furnace_heat = STD_OFF,
	.furnace_cool = STD_OFF,
	.humid_d_close = STD_OFF,
	.humid_d_open = STD_OFF,
	.humid_fan = STD_OFF,
	.humid_valve = STD_OFF,
	.erv_off = STD_ON,
	.erv_recirc = STD_OFF,
	.erv_low = STD_OFF,
	.erv_high = STD_OFF,
};
pthread_mutex_t rd_mutex = PTHREAD_MUTEX_INITIALIZER;
/* end of run data */

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
	rd_inst.humid_mode = json_get_number(json, "humid_mode",
		0, STD_ON, rd_inst.humid_mode);
	rd_inst.erv_mode = json_get_number(json, "erv_mode",
		0, ERV_MAX, rd_inst.erv_mode);
	rd_inst.temp_sp_heat = json_get_number(json, "temp_sp_heat",
		TEMP_SP_HEAT_MIN, TEMP_SP_HEAT_MAX, rd_inst.temp_sp_heat);
	rd_inst.temp_sp_cool = json_get_number(json, "temp_sp_cool",
		TEMP_SP_COOL_MIN, TEMP_SP_COOL_MAX, rd_inst.temp_sp_cool);
	rd_inst.humid_sp = json_get_number(json, "humid_sp",
		HUMID_SP_MIN, HUMID_SP_MAX, rd_inst.humid_sp);

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
	cJSON_AddItemToObject(json, "humid_mode", cJSON_CreateNumber(rd_inst.humid_mode));
	cJSON_AddItemToObject(json, "erv_mode", cJSON_CreateNumber(rd_inst.erv_mode));
	cJSON_AddItemToObject(json, "temp_sp_heat", cJSON_CreateNumber(rd_inst.temp_sp_heat));
	cJSON_AddItemToObject(json, "temp_sp_cool", cJSON_CreateNumber(rd_inst.temp_sp_cool));
	cJSON_AddItemToObject(json, "humid_sp", cJSON_CreateNumber(rd_inst.humid_sp));

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
	rc = modbus_read_registers(mb, 0, 2, reg);
	xassert(rc != -1, ret = errno, "%d", errno);
	data->temp1 = reg[1];
	data->humid1 = reg[0];

read2:
	rc = modbus_set_slave(mb, 2);
	xassert(!rc, return errno, "%d", errno);

	usleep(200000);
	rc = modbus_read_registers(mb, 34, 2, reg);
	xassert(rc != -1, ret = errno, "%d", errno);
	data->temp2 = reg[0];
	data->humid2 = reg[1];

	if (ret)
		return ret;

#if 0
	data->temp_avg = (data->temp1 + 3 * data->temp2) / 4;
	data->humid_avg = (data->humid1 + 3 * data->humid2) / 4;
#else
	data->temp_avg = data->temp1;
	data->humid_avg = data->humid1;
#endif
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
 * @note gpiod_line_request_bulk_output() additionally sets the pin state.
 *       The side effect is that we initialize the output pins in a clean
 *       state. For example, the furnace is turned off by default.
 */
int gpio_init(void)
{
	int rc;

	chip = gpiod_chip_open_by_label(GPIO_CHIP_LABEL);
	xassert(chip, return errno, "%d", errno);

	rc = gpiod_chip_get_lines(chip, gpio_pin_map, NUM_GPIO_PINS, &bulk);
	xassert(!rc, return errno, "%d", errno);

	rc = gpiod_line_request_bulk_output(&bulk, "hvac", gpio_def_val);
	xassert(!rc, return errno, "%d", errno);

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

	mb = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
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
		sd->humid1 / 10.0, sd->humid2 / 10.0,
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

/*
 * There is no locking around cd_inst because cd_inst is modified only in
 * loop_1_sec(), which always runs in the same thread, and we are called only
 * from that function.
 */
void gpio_state_sync(void)
{
	const int values[NUM_GPIO_PINS] = {
		[GPIO_FURNACE_BLOW]	= !cd_inst.furnace_blow,
		[GPIO_FURNACE_HEAT]	= !cd_inst.furnace_heat,
		[GPIO_FURNACE_COOL]	= !cd_inst.furnace_cool,
		[GPIO_HUMID_D_CLOSE]	= !cd_inst.humid_d_close,
		[GPIO_HUMID_D_OPEN]	= !cd_inst.humid_d_open,
		[GPIO_HUMID_FAN]	= !cd_inst.humid_fan,
		[GPIO_HUMID_VALVE]	= !cd_inst.humid_valve,
		[GPIO_ERV_OFF]		= !cd_inst.erv_off,
		[GPIO_ERV_RECIRC]	= !cd_inst.erv_recirc,
		[GPIO_ERV_LOW]		= !cd_inst.erv_low,
		[GPIO_ERV_HIGH]		= !cd_inst.erv_high,
	};

	int rc = gpiod_line_set_value_bulk(&bulk, values);

	xassert(!rc, (void)0, "%d", errno);
}

static int telemetry_prep_send(struct timeval tv,
				const struct run_data *rd,
				const struct ctrl_data *cd,
				const struct sensor_data *sd)
{
	const struct telemetry_data td = {
		.tv = tv,

		.run = {
			.furnace_mode = rd->furnace_mode,
			.humid_mode = rd->humid_mode,
			.erv_mode = rd->erv_mode,
			.temp_sp_heat = rd->temp_sp_heat / 10.0f,
			.temp_sp_cool = rd->temp_sp_cool / 10.0f,
			.temp_thres = rd->temp_thres / 10.0f,
			.humid_sp = rd->humid_sp / 10.0f,
		},

		.ctrl = {
			.furnace_blow = cd->furnace_blow,
			.furnace_heat = cd->furnace_heat,
			.furnace_cool = cd->furnace_cool,
			.humid_d_close = cd->humid_d_close,
			.humid_d_open = cd->humid_d_open,
			.humid_fan = cd->humid_fan,
			.humid_valve = cd->humid_valve,
			.erv_off = cd->erv_off,
			.erv_recirc = cd->erv_recirc,
			.erv_low = cd->erv_low,
			.erv_high = cd->erv_high,
		},

		.sens = {
			.temp1 = sd->temp1 / 10.0f,
			.humid1 = sd->humid1 / 10.0f,
			.temp2 = sd->temp2 / 10.0f,
			.humid2 = sd->humid2 / 10.0f,
		},
	};

	return telemetry_send(&td);
}

int loop_1_sec(void)
{
	static enum std_on_off heat_cool_state = STD_OFF;
	static int sens_cnt, sens_fail, humid_cnt, humid_duty;
	static int old_furnace_mode = FURNACE_OFF;
	static int old_erv_mode = ERV_OFF;
	static int old_humid_mode = INT_MAX;
	static int furnace_holdoff, humid_holdoff;
	static enum std_on_off erv_state;
	static int erv_timer;

	struct sensor_data sd = {.valid = 0};
	struct run_data rd_snap;
	struct timeval tv;

	gettimeofday(&tv, NULL);

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
		heat_cool_state = STD_OFF;
		old_furnace_mode = rd_inst.furnace_mode;
	}

	switch (rd_inst.furnace_mode) {
	case FURNACE_OFF:
		cd_inst.furnace_blow = STD_OFF;
		cd_inst.furnace_heat = STD_OFF;
		cd_inst.furnace_cool = STD_OFF;
		break;
	case FURNACE_FAN:
		cd_inst.furnace_blow = STD_ON;
		cd_inst.furnace_heat = STD_OFF;
		cd_inst.furnace_cool = STD_OFF;
		break;
	case FURNACE_HEAT:
		cd_inst.furnace_blow = STD_ON;
		cd_inst.furnace_cool = STD_OFF;
		if (furnace_holdoff) {
			furnace_holdoff--;
			break;
		}
		if (!sd.valid)
			break;
		if (sd.temp_avg >= rd_inst.temp_sp_heat + rd_inst.temp_thres &&
		    heat_cool_state == STD_ON) {
			xprintf(SD_NOTICE "HEAT OFF\n");
			cd_inst.furnace_heat = STD_OFF;
			heat_cool_state = STD_OFF;
			break;
		}
		if (sd.temp_avg <= rd_inst.temp_sp_heat - rd_inst.temp_thres &&
		    heat_cool_state == STD_OFF) {
			xprintf(SD_NOTICE "HEAT ON\n");
			cd_inst.furnace_heat = STD_ON;
			heat_cool_state = STD_ON;
		}
		break;
	case FURNACE_COOL:
		cd_inst.furnace_blow = STD_ON;
		cd_inst.furnace_heat = STD_OFF;
		if (furnace_holdoff) {
			furnace_holdoff--;
			break;
		}
		if (!sd.valid)
			break;
		if (sd.temp_avg >= rd_inst.temp_sp_cool + rd_inst.temp_thres &&
		    heat_cool_state == STD_OFF) {
			xprintf(SD_NOTICE "COOL ON\n");
			cd_inst.furnace_cool = STD_ON;
			heat_cool_state = STD_ON;
			break;
		}
		if (sd.temp_avg <= rd_inst.temp_sp_cool - rd_inst.temp_thres &&
		    heat_cool_state == STD_ON) {
			xprintf(SD_NOTICE "COOL OFF\n");
			cd_inst.furnace_cool = STD_OFF;
			heat_cool_state = STD_OFF;
		}
		break;
	}

	if (rd_inst.erv_mode != old_erv_mode) {
		xprintf(SD_NOTICE "ERV mode: %s\n",
			rd_erv_map[rd_inst.erv_mode]);
		erv_state = STD_ON;
		erv_timer = 0;
		old_erv_mode = rd_inst.erv_mode;
	}


	switch (rd_inst.erv_mode) {
	case ERV_OFF:
		cd_inst.erv_off = STD_ON;
		cd_inst.erv_recirc = STD_OFF;
		cd_inst.erv_low = STD_OFF;
		cd_inst.erv_high = STD_OFF;
		break;
	case ERV_RECIRC:
		cd_inst.erv_off = STD_OFF;
		cd_inst.erv_recirc = STD_ON;
		cd_inst.erv_low = STD_OFF;
		cd_inst.erv_high = STD_OFF;
		break;
	case ERV_I20MH:
	case ERV_I30MH:
	case ERV_I40MH:
		if (erv_timer > 0)
			erv_timer--;
		else {
			erv_state = !erv_state;
			erv_timer = erv_state ?
				    erv_int_map[rd_inst.erv_mode].run_time_s:
				    erv_int_map[rd_inst.erv_mode].off_time_s;
		}
		cd_inst.erv_off = STD_OFF;
		cd_inst.erv_recirc = STD_OFF;
		cd_inst.erv_low = erv_state;
		cd_inst.erv_high = STD_OFF;
		break;
	case ERV_LOW:
		cd_inst.erv_off = STD_OFF;
		cd_inst.erv_recirc = STD_OFF;
		cd_inst.erv_low = STD_ON;
		cd_inst.erv_high = STD_OFF;
		break;
	case ERV_HIGH:
		cd_inst.erv_off = STD_OFF;
		cd_inst.erv_recirc = STD_OFF;
		cd_inst.erv_low = STD_OFF;
		cd_inst.erv_high = STD_ON;
		break;
	}

	if (rd_inst.humid_mode != old_humid_mode && !humid_holdoff) {
		xprintf(SD_NOTICE "Humidifier mode: %s\n",
			std_on_off_map[rd_inst.humid_mode]);
		cd_inst.humid_d_close = rd_inst.humid_mode == STD_OFF;
		cd_inst.humid_d_open = rd_inst.humid_mode == STD_ON;
		humid_holdoff = 10;
		old_humid_mode = rd_inst.humid_mode;
	}

	if (rd_inst.humid_mode == STD_ON && rd_inst.furnace_mode != FURNACE_OFF &&
	    !humid_holdoff) {
		if (heat_cool_state == STD_ON) {
			cd_inst.humid_fan = STD_OFF;
			humid_duty = rd_inst.furnace_mode == FURNACE_HEAT ? 20 : 10;
		} else {
			cd_inst.humid_fan = STD_ON;
			humid_duty = 10;
		}

		if (humid_cnt == 0)
			cd_inst.humid_valve = STD_ON;
		else if (humid_cnt >= humid_duty)
			cd_inst.humid_valve = STD_OFF;
		humid_cnt = (humid_cnt + 1) % 30;
	} else {
		cd_inst.humid_fan = STD_OFF;
		cd_inst.humid_valve = STD_OFF;
		humid_cnt = 0;
	}

	if (humid_holdoff)
		humid_holdoff--;
	else {
		cd_inst.humid_d_close = STD_OFF;
		cd_inst.humid_d_open = STD_OFF;
	}

	if (rd_inst.sync == 1) {
		nvram_write();
		rd_inst.sync = 0;
	} else if (rd_inst.sync)
		rd_inst.sync--;

	rd_snap = rd_inst;
	pthread_mutex_unlock(&rd_mutex);

	/*
	 * Sync the GPIO pin state outside the critical region. There is no
	 * other writer and we always run in the same thread, so the data is
	 * guaranteed to be consistent outside the critical region. On the
	 * other hand, the GPIO pins are on an IO expander that is connected
	 * over I2C, so setting the pin state takes time. Avoid holding the
	 * mutex when we don't really need it.
	 */
	gpio_state_sync();

	if (telemetry_cfg)
		telemetry_prep_send(tv, &rd_snap, &cd_inst, &sd);

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

void cv_write_json(struct mg_connection *conn, cJSON *rsp_json)
{
	char *rsp_str;
	unsigned long len;

	rsp_str = cJSON_Print(rsp_json);
	cJSON_Delete(rsp_json);

	len = strlen(rsp_str);
	mg_send_http_ok(conn, "application/json", len);
	mg_write(conn, rsp_str, len);
	free(rsp_str);
}

int cv_hdlr_sensor_data_get(struct mg_connection *conn, void *cbdata)
{
	struct sensor_data sd;
	cJSON *rsp = cJSON_CreateObject();

	pthread_mutex_lock(&sd_mutex);
	sd = sd_snap;
	pthread_mutex_unlock(&sd_mutex);

	cJSON_AddItemToObject(rsp, "temp_avg", cJSON_CreateNumber(sd.temp_avg / 10.0));
	cJSON_AddItemToObject(rsp, "humid_avg", cJSON_CreateNumber(sd.humid_avg / 10.0));

	cv_write_json(conn, rsp);

	return 200;
}

const mg_request_handler cv_hmap_sensor_data[NUM_HTTP_METHODS] = {
	[HTTP_GET]	= cv_hdlr_sensor_data_get,
};

int cv_hdlr_ctrl_data_get(struct mg_connection *conn, void *cbdata)
{
	struct ctrl_data cd;
	cJSON *rsp = cJSON_CreateObject();

	pthread_mutex_lock(&rd_mutex);
	cd = cd_inst;
	pthread_mutex_unlock(&rd_mutex);

	cJSON_AddItemToObject(rsp, "furnace_blow",
			      cJSON_CreateString(std_on_off_map[cd.furnace_blow]));
	cJSON_AddItemToObject(rsp, "furnace_heat",
			      cJSON_CreateString(std_on_off_map[cd.furnace_heat]));
	cJSON_AddItemToObject(rsp, "furnace_cool",
			      cJSON_CreateString(std_on_off_map[cd.furnace_cool]));
	cJSON_AddItemToObject(rsp, "humid_d_close",
			      cJSON_CreateString(std_on_off_map[cd.humid_d_close]));
	cJSON_AddItemToObject(rsp, "humid_d_open",
			      cJSON_CreateString(std_on_off_map[cd.humid_d_open]));
	cJSON_AddItemToObject(rsp, "humid_fan",
			      cJSON_CreateString(std_on_off_map[cd.humid_fan]));
	cJSON_AddItemToObject(rsp, "humid_valve",
			      cJSON_CreateString(std_on_off_map[cd.humid_valve]));
	cJSON_AddItemToObject(rsp, "erv_off",
			      cJSON_CreateString(std_on_off_map[cd.erv_off]));
	cJSON_AddItemToObject(rsp, "erv_recirc",
			      cJSON_CreateString(std_on_off_map[cd.erv_recirc]));
	cJSON_AddItemToObject(rsp, "erv_low",
			      cJSON_CreateString(std_on_off_map[cd.erv_low]));
	cJSON_AddItemToObject(rsp, "erv_high",
			      cJSON_CreateString(std_on_off_map[cd.erv_high]));

	cv_write_json(conn, rsp);

	return 200;
}

const mg_request_handler cv_hmap_ctrl_data[NUM_HTTP_METHODS] = {
	[HTTP_GET]	= cv_hdlr_ctrl_data_get,
};

int cv_hdlr_run_data_get(struct mg_connection *conn, void *cbdata)
{
	struct run_data rd;
	cJSON *rsp = cJSON_CreateObject();

	pthread_mutex_lock(&rd_mutex);
	rd = rd_inst;
	pthread_mutex_unlock(&rd_mutex);

	cJSON_AddItemToObject(rsp, "furnace_mode",
			      cJSON_CreateString(rd_furnace_map[rd.furnace_mode]));
	cJSON_AddItemToObject(rsp, "erv_mode",
			      cJSON_CreateString(rd_erv_map[rd.erv_mode]));
	cJSON_AddItemToObject(rsp, "humid_mode",
			      cJSON_CreateString(std_on_off_map[rd.humid_mode]));
	cJSON_AddItemToObject(rsp, "temp_sp_heat",
			      cJSON_CreateNumber(rd.temp_sp_heat / 10.0));
	cJSON_AddItemToObject(rsp, "temp_sp_cool",
			      cJSON_CreateNumber(rd.temp_sp_cool / 10.0));
	cJSON_AddItemToObject(rsp, "humid_sp",
			      cJSON_CreateNumber(rd.humid_sp / 10.0));

	cv_write_json(conn, rsp);

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
	if ((idx = json_map_string(req, "erv_mode", NULL, rd_erv_map)) >= 0) {
		chg |= rd_inst.erv_mode != idx;
		rd_inst.erv_mode = idx;
	}
	if ((idx = json_map_string(req, "humid_mode", NULL, std_on_off_map)) >= 0) {
		chg |= rd_inst.humid_mode != idx;
		rd_inst.humid_mode = idx;
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
	if (!isnan(val = json_get_number(req, "humid_sp",
	    HUMID_SP_MIN/10.0, HUMID_SP_MAX/10.0, NAN))) {
		int x = val * 10.0;
		chg |= rd_inst.humid_sp != x;
		rd_inst.humid_sp = x;
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
	static const char * const realm = "api";
	const struct mg_request_info *ri = mg_get_request_info(conn);
	int method_idx = map_find(http_method_map, ri->request_method);
	mg_request_handler *hmap = cbdata, hdlr = NULL;
	const char *passfile = cfg_get_string("http_passwd", "htpasswd");

	if (mg_check_digest_access_authentication(conn, realm, passfile) <= 0) {
		/* No valid authorization */
		mg_send_digest_access_authentication_request(conn, realm);
		return 401;
	}

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

	int rc, tfd;
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
	mg_set_request_handler(cv_ctx, "/api/ctrldata", cv_hdlr_api, (void *)cv_hmap_ctrl_data);
	mg_set_request_handler(cv_ctx, "/api/sensordata", cv_hdlr_api, (void *)cv_hmap_sensor_data);

	tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
	xassert(tfd != -1, goto out_stop, "%d", errno);

	rc = timerfd_settime(tfd, 0, &its, NULL);
	xassert(rc != -1, goto out_clo_tfd, "%d", errno);

	if (telemetry_cfg) {
		rc = telemetry_init(telemetry_cfg);
		xassert(!rc, goto out_clo_tfd, "%d", rc);
	}

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

	if (telemetry_cfg)
		telemetry_exit();

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
	rc = gpiod_line_set_value_bulk(&bulk, gpio_def_val);
	xassert(!rc, (void)0, "%d", errno);

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
	} else {
		telemetry_cfg = cJSON_GetObjectItem(config, "telemetry");
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
