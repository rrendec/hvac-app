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
#include "gsdata.h"
#include "json.h"
#include "telemetry.h"
#include "memcache.h"

__thread volatile int __canceled;

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
#define FURNACE_HOLDOFF_S	5
#define ERV_HOLDOFF_S		600
#define EXT_STALE_S		3600

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
	[ERV_AUTO]	= "auto",
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

const struct {
	int temp_offset;
	enum erv_mode erv_mode;
} erv_temp_map[] = {
	{-450,	ERV_I20MH},
	{-360,	ERV_I30MH},
	{-270,	ERV_I40MH},
	{-180,	ERV_LOW},
	{-90,	ERV_HIGH},
	{20,	ERV_LOW},
	{40,	ERV_I40MH},
	{60,	ERV_I30MH},
	{80,	ERV_I20MH},
	{100,	ERV_OFF}
};

#define ERV_SPEED_OFF	0
#define ERV_SPEED_LOW	1
#define ERV_SPEED_HIGH  2
#define ERV_SPEED_MASK	3
#define ERV_DAMPER_OPEN	4
#define ERV_INTERM	8

const int erv_mode_flags[] = {
	[ERV_OFF]	= ERV_SPEED_OFF,
	[ERV_RECIRC]	= ERV_SPEED_HIGH,
	[ERV_I20MH]	= ERV_SPEED_LOW | ERV_DAMPER_OPEN | ERV_INTERM,
	[ERV_I30MH]	= ERV_SPEED_LOW | ERV_DAMPER_OPEN | ERV_INTERM,
	[ERV_I40MH]	= ERV_SPEED_LOW | ERV_DAMPER_OPEN | ERV_INTERM,
	[ERV_LOW]	= ERV_SPEED_LOW | ERV_DAMPER_OPEN,
	[ERV_HIGH]	= ERV_SPEED_HIGH | ERV_DAMPER_OPEN,
	[ERV_AUTO]	= 0, /* not a real mode */
};

pthread_mutex_t oestream_mutex = PTHREAD_MUTEX_INITIALIZER;
cJSON *config;
cJSON *telemetry_cfg;
cJSON *memcache_cfg;

unsigned int gpio_pin_map[NUM_GPIO_PINS] = GPIO_MAP_INITIALIZER;
const int gpio_def_val[NUM_GPIO_PINS] = {[0 ... NUM_GPIO_PINS - 1] = 1};

volatile int keep_going = 1;
volatile pid_t child_pid;

/* hw handles below */
struct gpiod_chip *chip;
struct gpiod_line_bulk bulk;
modbus_t *mb;
/* end of hw handles */

static enum std_on_off heat_cool_state = STD_OFF;

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
 * (such as civetweb) are started. In this case, rd is &gs_rd, and no
 * synchronization is required because there are no other writers (yet).
 * Furthermore, we can use the default values from the gs_rd initializer to
 * avoid duplicating the default values.
 */
int nvram_read(struct run_data *rd)
{
	cJSON *json = NULL;
	int rc;

	rc = json_read(nvram_path(), &json, NULL, NULL);
	if (rc)
		return rc;

	rd->furnace_mode = json_get_number(json, "furnace_mode",
		0, FURNACE_MAX, rd->furnace_mode);
	rd->humid_mode = json_get_number(json, "humid_mode",
		0, STD_ON, rd->humid_mode);
	rd->erv_mode = json_get_number(json, "erv_mode",
		0, ERV_MAX, rd->erv_mode);
	rd->temp_sp_heat = json_get_number(json, "temp_sp_heat",
		TEMP_SP_HEAT_MIN, TEMP_SP_HEAT_MAX, rd->temp_sp_heat);
	rd->temp_sp_cool = json_get_number(json, "temp_sp_cool",
		TEMP_SP_COOL_MIN, TEMP_SP_COOL_MAX, rd->temp_sp_cool);
	rd->humid_sp = json_get_number(json, "humid_sp",
		HUMID_SP_MIN, HUMID_SP_MAX, rd->humid_sp);

	cJSON_Delete(json);

	return 0;
}

/*
 * This function assumes *rd does not change. It is called either:
 *   - during initialization, before other threads (such as civetweb) are
 *     started, and in this case rd is &gs_rd and no synchronization is
 *     needed because there are no other writers; or
 *   - in the delay loop, and in this case a copy of gs_rd is passed.
 */
int nvram_write(const struct run_data *rd)
{
	cJSON *json = cJSON_CreateObject();
	const char *cfg_path;
	char *tmp_path = NULL;
	int rc = ENOMEM;

	xprintf(SD_DEBUG "Sync run mode data to non-volatile storage\n");

	cJSON_AddItemToObject(json, "furnace_mode", cJSON_CreateNumber(rd->furnace_mode));
	cJSON_AddItemToObject(json, "humid_mode", cJSON_CreateNumber(rd->humid_mode));
	cJSON_AddItemToObject(json, "erv_mode", cJSON_CreateNumber(rd->erv_mode));
	cJSON_AddItemToObject(json, "temp_sp_heat", cJSON_CreateNumber(rd->temp_sp_heat));
	cJSON_AddItemToObject(json, "temp_sp_cool", cJSON_CreateNumber(rd->temp_sp_cool));
	cJSON_AddItemToObject(json, "humid_sp", cJSON_CreateNumber(rd->humid_sp));

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

void term_sig_hdlr(int signal)
{
	keep_going = 0;

	if (child_pid > 0)
		kill(child_pid, SIGTERM);
}

void usr2_sig_hdlr(int signal)
{
	__canceled = 1;
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
 * There is no locking around gs_cd because gs_cd is modified only in
 * loop_1_sec(), which always runs in the same thread, and we are called only
 * from that function.
 */
void gpio_state_sync(void)
{
	const int values[NUM_GPIO_PINS] = {
		[GPIO_FURNACE_BLOW]	= !gs_cd.furnace_blow,
		[GPIO_FURNACE_HEAT]	= !gs_cd.furnace_heat,
		[GPIO_FURNACE_COOL]	= !gs_cd.furnace_cool,
		[GPIO_HUMID_D_CLOSE]	= !gs_cd.humid_d_close,
		[GPIO_HUMID_D_OPEN]	= !gs_cd.humid_d_open,
		[GPIO_HUMID_FAN]	= !gs_cd.humid_fan,
		[GPIO_HUMID_VALVE]	= !gs_cd.humid_valve,
		[GPIO_ERV_OFF]		= !gs_cd.erv_off,
		[GPIO_ERV_RECIRC]	= !gs_cd.erv_recirc,
		[GPIO_ERV_LOW]		= !gs_cd.erv_low,
		[GPIO_ERV_HIGH]		= !gs_cd.erv_high,
	};

	int rc = gpiod_line_set_value_bulk(&bulk, values);

	xassert(!rc, NOOP, "%d", errno);
}

static int telemetry_prep_send(struct timeval tv,
				const struct run_data *rd,
				const struct ctrl_data *cd,
				const struct sensor_data *sd)
{
	struct telemetry_data td = {
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
			.temp1 = NAN,
			.humid1 = NAN,
			.temp2 = NAN,
			.humid2 = NAN,
		},
	};

	if (sd->valid) {
		td.sens.temp1 = sd->temp1 / 10.0f;
		td.sens.humid1 = sd->humid1 / 10.0f;
		td.sens.temp2 = sd->temp2 / 10.0f;
		td.sens.humid2 = sd->humid2 / 10.0f;
	}

	return telemetry_send(&td);
}

/*
 * Update furnace state. Called with gs_mutex locked.
 */
static void furnace_update(const struct sensor_data *sd)
{
	static int old_furnace_mode = FURNACE_OFF;
	static int furnace_holdoff;

	if (gs_rd.furnace_mode != old_furnace_mode) {
		xprintf(SD_NOTICE "Furnace mode: %s\n",
			rd_furnace_map[gs_rd.furnace_mode]);
		furnace_holdoff = FURNACE_HOLDOFF_S;
		heat_cool_state = STD_OFF;
		old_furnace_mode = gs_rd.furnace_mode;
	}

	switch (gs_rd.furnace_mode) {
	case FURNACE_OFF:
		gs_cd.furnace_blow = STD_OFF;
		gs_cd.furnace_heat = STD_OFF;
		gs_cd.furnace_cool = STD_OFF;
		break;
	case FURNACE_FAN:
		gs_cd.furnace_blow = STD_ON;
		gs_cd.furnace_heat = STD_OFF;
		gs_cd.furnace_cool = STD_OFF;
		break;
	case FURNACE_HEAT:
		gs_cd.furnace_blow = STD_ON;
		gs_cd.furnace_cool = STD_OFF;
		if (furnace_holdoff) {
			furnace_holdoff--;
			break;
		}
		if (!sd->valid)
			break;
		if (sd->temp_avg >= gs_rd.temp_sp_heat + gs_rd.temp_thres &&
		    heat_cool_state == STD_ON) {
			xprintf(SD_NOTICE "HEAT OFF\n");
			gs_cd.furnace_heat = STD_OFF;
			heat_cool_state = STD_OFF;
			break;
		}
		if (sd->temp_avg <= gs_rd.temp_sp_heat - gs_rd.temp_thres &&
		    heat_cool_state == STD_OFF) {
			xprintf(SD_NOTICE "HEAT ON\n");
			gs_cd.furnace_heat = STD_ON;
			heat_cool_state = STD_ON;
		}
		break;
	case FURNACE_COOL:
		gs_cd.furnace_blow = STD_ON;
		gs_cd.furnace_heat = STD_OFF;
		if (furnace_holdoff) {
			furnace_holdoff--;
			break;
		}
		if (!sd->valid)
			break;
		if (sd->temp_avg >= gs_rd.temp_sp_cool + gs_rd.temp_thres &&
		    heat_cool_state == STD_OFF) {
			xprintf(SD_NOTICE "COOL ON\n");
			gs_cd.furnace_cool = STD_ON;
			heat_cool_state = STD_ON;
			break;
		}
		if (sd->temp_avg <= gs_rd.temp_sp_cool - gs_rd.temp_thres &&
		    heat_cool_state == STD_ON) {
			xprintf(SD_NOTICE "COOL OFF\n");
			gs_cd.furnace_cool = STD_OFF;
			heat_cool_state = STD_OFF;
		}
		break;
	}
}

/*
 * Determine the optimal ERV mode based on the outdoor temperature
 *
 * Since we have no control over how the outdoor temperature varies, hysteresis
 * around the threshold values in erv_temp_map[] is simulated by adding an
 * offset. The absolute value of the offset is fixed, and the sign depends on
 * the outdoor temperature trend.
 *
 * The outdoor temperature trend is determined by calculating a weighted average
 * of the outdoor temperature samples. If the current average value is higher
 * than the previous average value, the outdoor temperature has gone up. If the
 * current value is lower than the previous value, the temperature has gone
 * down. The purpose of the weighted average is to eliminate small oscillations
 * of the outdoor temperature, for example due to measurement errors.
 *
 * The weighted average is used only to determine the outdoor temperature trend.
 * The current value of the outdoor temperature (the most recent sample) is
 * still used to determine the optimal ERV mode based on erv_temp_map[]. The
 * reason is that the weighted average tends to converge slower.
 *
 * Fixed-point arithmetic with a precision factor of k is used for the weighted
 * average. Using integer arithmetic alone can lead to weird situations when the
 * average converges to a value that is way of to the actual value of the
 * temperature samples, if the temperature is almost constant. For example, if
 * the average is 3.8 and the temperature samples are 3.3 (note that fixed-point
 * arithmetic with a precision of decimal 0.1 is used, and the actual values in
 * the program are 38 and 33 respectively) and a weight of 9:1 is used, the
 * formula to calculate a new value of avg from the existing value and the
 * temperature sample is:
 *     avg = (avg * 9 + sample + 5) / 10
 * where avg = 38 and sample = 33. The extra +5 is there to round the integer
 * division. So, avg = (38 * 9 + 33 + 5) / 10 = (38 * 9 + 38) / 10 =
 * 38 * 10 / 10 = 38.
 */
static enum erv_mode erv_auto_temp(const struct sensor_data *_sd,
				   struct timeval now)
{
	static bool init;
	static int avg;
	static enum erv_mode last_mode;
	static time_t last_dt;
	static struct sensor_data sd;

	/* Fixed-point arithmetic precision for average calculation */
	const int k = 0x10000;
	/* Absolute value of temperature threshold, in units of 0.1 C */
	const int th_abs = 5;

	int thres, ref, i;
	enum erv_mode curr_mode;

	if (_sd->valid)
		sd = *_sd;

	if (now.tv_sec - gs_ed.dt > EXT_STALE_S) {
		init = false;
		return ERV_I30MH;
	}

	if (init && gs_ed.dt == last_dt)
		return last_mode;

	/*
	 * Assignments from _sd to sd are already validated, but there is still
	 * the case when we have never seen valid data in _sd.
	 */
	if (!sd.valid)
		return ERV_I30MH;

	if (init) {
		int anew = rdivi(avg * 7 + gs_ed.temp * k, 8);

		thres = th_abs * sgni(anew - avg);
		avg = anew;
	} else {
		avg = gs_ed.temp * k;
		thres = 0;
		init = true;
	}

	ref = sd.temp_avg;
	if (gs_rd.furnace_mode == FURNACE_HEAT)
		ref = gs_rd.temp_sp_heat;
	if (gs_rd.furnace_mode == FURNACE_COOL)
		ref = gs_rd.temp_sp_cool;

	curr_mode = ERV_OFF;
	for (i = 0; i < ARRAY_SIZE(erv_temp_map); i++)
		if (gs_ed.temp >= ref + erv_temp_map[i].temp_offset + thres)
			curr_mode = erv_temp_map[i].erv_mode;
		else
			break;

	last_dt = gs_ed.dt;
	last_mode = curr_mode;

	return curr_mode;
}

static enum erv_mode erv_auto_aqi(struct timeval now)
{
	// TODO
	return ERV_HIGH;
}

/*
 * Allow smooth transitions between intermittent states. Instead of resetting
 * the timer and turning the ERV off immediately, let the current cycle finish
 * and transition to the new delay values starting with the next cycle.
 */
static bool erv_interm_fallthrough(enum erv_mode em_old, enum erv_mode em_new)
{
	const int fl_old = erv_mode_flags[em_old];
	const int fl_new = erv_mode_flags[em_new];

	return (fl_old & ERV_INTERM) && (fl_new & ERV_INTERM);
}

/*
 * Determine if we need to wait for the holdoff timer when we transition
 * between two ERV modes. The assumption is that em_new != em_old.
 */
static bool erv_holdoff(enum erv_mode em_new, enum erv_mode em_old,
			int erv_holdoff_tmr, enum std_on_off erv_state)
{
	int fl_old = erv_mode_flags[em_old];
	int fl_new = erv_mode_flags[em_new];

	if (!erv_holdoff_tmr || erv_interm_fallthrough(em_old, em_new))
		return false;

	/* For intermittent modes, update the old state if the ERV is off */
	if ((fl_old & ERV_INTERM) && erv_state == STD_OFF)
		fl_old &= !ERV_SPEED_MASK & !ERV_DAMPER_OPEN;

	/* For intermittent modes, the new state starts with an off cycle */
	if ((fl_new & ERV_INTERM))
		fl_new &= !ERV_SPEED_MASK & !ERV_DAMPER_OPEN;

	return (fl_new & ERV_SPEED_MASK) != (fl_old & ERV_SPEED_MASK) ||
	       (fl_new & ERV_DAMPER_OPEN) != (fl_old & ERV_DAMPER_OPEN);
}

/*
 * Update ERV state. Called with gs_mutex locked.
 */
static void erv_update(const struct sensor_data *sd, struct timeval now)
{
	static enum erv_mode set_mode = ERV_OFF;
	static enum erv_mode old_mode = ERV_OFF;
	static enum std_on_off erv_state;
	static int erv_interm_tmr;
	static int erv_holdoff_tmr;

	enum erv_mode act_mode;

	if (gs_rd.erv_mode != set_mode) {
		xprintf(SD_NOTICE "ERV set mode: %s\n",
			rd_erv_map[gs_rd.erv_mode]);
		set_mode = gs_rd.erv_mode;
	}

	if (gs_rd.erv_mode == ERV_AUTO) {
		enum erv_mode m_temp = erv_auto_temp(sd, now);
		enum erv_mode m_aqi = erv_auto_aqi(now);
		act_mode = MIN(m_temp, m_aqi);
	} else
		act_mode = gs_rd.erv_mode;

	if (erv_holdoff_tmr)
		erv_holdoff_tmr--;

	if (act_mode != old_mode) {
		if (erv_holdoff(act_mode, old_mode, erv_holdoff_tmr, erv_state))
			act_mode = old_mode;
		else {
			xprintf(SD_NOTICE "ERV actual mode: %s\n",
				rd_erv_map[act_mode]);
			if (!erv_interm_fallthrough(act_mode, old_mode)) {
				erv_state = STD_ON;
				erv_interm_tmr = 0;
				erv_holdoff_tmr = ERV_HOLDOFF_S;
			}
			old_mode = act_mode;
		}
	}

	switch (act_mode) {
	case ERV_OFF:
		gs_cd.erv_off = STD_ON;
		gs_cd.erv_recirc = STD_OFF;
		gs_cd.erv_low = STD_OFF;
		gs_cd.erv_high = STD_OFF;
		break;
	case ERV_RECIRC:
		gs_cd.erv_off = STD_OFF;
		gs_cd.erv_recirc = STD_ON;
		gs_cd.erv_low = STD_OFF;
		gs_cd.erv_high = STD_OFF;
		break;
	case ERV_I20MH:
	case ERV_I30MH:
	case ERV_I40MH:
		if (erv_interm_tmr > 0)
			erv_interm_tmr--;
		else {
			erv_state = !erv_state;
			erv_interm_tmr = erv_state ?
					 erv_int_map[act_mode].run_time_s:
					 erv_int_map[act_mode].off_time_s;
			erv_holdoff_tmr = ERV_HOLDOFF_S;
		}
		gs_cd.erv_off = !erv_state;
		gs_cd.erv_recirc = STD_OFF;
		gs_cd.erv_low = erv_state;
		gs_cd.erv_high = STD_OFF;
		break;
	case ERV_LOW:
		gs_cd.erv_off = STD_OFF;
		gs_cd.erv_recirc = STD_OFF;
		gs_cd.erv_low = STD_ON;
		gs_cd.erv_high = STD_OFF;
		break;
	case ERV_HIGH:
		gs_cd.erv_off = STD_OFF;
		gs_cd.erv_recirc = STD_OFF;
		gs_cd.erv_low = STD_OFF;
		gs_cd.erv_high = STD_ON;
		break;
	case ERV_AUTO:
		xassert(0, NOOP);
	}
}

/*
 * Update humidifier state. Called with gs_mutex locked.
 */
static void humid_update(void)
{
	static int humid_cnt, humid_duty;
	static int old_humid_mode = INT_MAX;
	static int humid_holdoff;

	if (gs_rd.humid_mode != old_humid_mode && !humid_holdoff) {
		xprintf(SD_NOTICE "Humidifier mode: %s\n",
			std_on_off_map[gs_rd.humid_mode]);
		gs_cd.humid_d_close = gs_rd.humid_mode == STD_OFF;
		gs_cd.humid_d_open = gs_rd.humid_mode == STD_ON;
		humid_holdoff = 10;
		old_humid_mode = gs_rd.humid_mode;
	}

	if (gs_rd.humid_mode == STD_ON && gs_rd.furnace_mode != FURNACE_OFF &&
	    !humid_holdoff) {
		if (heat_cool_state == STD_ON) {
			gs_cd.humid_fan = STD_OFF;
			humid_duty = gs_rd.furnace_mode == FURNACE_HEAT ? 20 : 10;
		} else {
			gs_cd.humid_fan = STD_ON;
			humid_duty = 10;
		}

		if (humid_cnt == 0)
			gs_cd.humid_valve = STD_ON;
		else if (humid_cnt >= humid_duty)
			gs_cd.humid_valve = STD_OFF;
		humid_cnt = (humid_cnt + 1) % 30;
	} else {
		gs_cd.humid_fan = STD_OFF;
		gs_cd.humid_valve = STD_OFF;
		humid_cnt = 0;
	}

	if (humid_holdoff)
		humid_holdoff--;
	else {
		gs_cd.humid_d_close = STD_OFF;
		gs_cd.humid_d_open = STD_OFF;
	}
}

int loop_1_sec(void)
{
	static int sens_cnt, sens_fail;

	struct sensor_data sd = {.valid = 0};
	struct run_data rd;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	if (sens_cnt) {
		sens_cnt = (sens_cnt + 1) % 5;
		sd = gs_sd;
	} else if (sensor_read(mb, &sd)) {
		if (sens_fail++ >= 30) {
			xprintf(SD_ERR "Sensor failure\n");
			return 1;
		}
	} else {
		sens_fail = 0;
		sensors_print(&sd);
		pthread_mutex_lock(&gs_mutex);
		gs_sd = sd;
		pthread_mutex_unlock(&gs_mutex);
		sens_cnt++;
	}

	pthread_mutex_lock(&gs_mutex);

	furnace_update(&sd);
	erv_update(&sd, tv);
	humid_update();

	rd = gs_rd;

	if (gs_rd.sync)
		gs_rd.sync--;

	pthread_mutex_unlock(&gs_mutex);

	if (rd.sync == 1)
		nvram_write(&rd);

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
		telemetry_prep_send(tv, &rd, &gs_cd, &sd);

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

	pthread_mutex_lock(&gs_mutex);
	sd = gs_sd;
	pthread_mutex_unlock(&gs_mutex);

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

	pthread_mutex_lock(&gs_mutex);
	cd = gs_cd;
	pthread_mutex_unlock(&gs_mutex);

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

	pthread_mutex_lock(&gs_mutex);
	rd = gs_rd;
	pthread_mutex_unlock(&gs_mutex);

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

	pthread_mutex_lock(&gs_mutex);
	if ((idx = json_map_string(req, "furnace_mode", NULL, rd_furnace_map)) >= 0) {
		chg |= gs_rd.furnace_mode != idx;
		gs_rd.furnace_mode = idx;
	}
	if ((idx = json_map_string(req, "erv_mode", NULL, rd_erv_map)) >= 0) {
		chg |= gs_rd.erv_mode != idx;
		gs_rd.erv_mode = idx;
	}
	if ((idx = json_map_string(req, "humid_mode", NULL, std_on_off_map)) >= 0) {
		chg |= gs_rd.humid_mode != idx;
		gs_rd.humid_mode = idx;
	}
	if (!isnan(val = json_get_number(req, "temp_sp_heat",
	    TEMP_SP_HEAT_MIN/10.0, TEMP_SP_HEAT_MAX/10.0, NAN))) {
		int x = val * 10.0;
		chg |= gs_rd.temp_sp_heat != x;
		gs_rd.temp_sp_heat = x;
	}
	if (!isnan(val = json_get_number(req, "temp_sp_cool",
	    TEMP_SP_COOL_MIN/10.0, TEMP_SP_COOL_MAX/10.0, NAN))) {
		int x = val * 10.0;
		chg |= gs_rd.temp_sp_cool != x;
		gs_rd.temp_sp_cool = x;
	}
	if (!isnan(val = json_get_number(req, "humid_sp",
	    HUMID_SP_MIN/10.0, HUMID_SP_MAX/10.0, NAN))) {
		int x = val * 10.0;
		chg |= gs_rd.humid_sp != x;
		gs_rd.humid_sp = x;
	}
	if (chg)
		gs_rd.sync = 2;
	pthread_mutex_unlock(&gs_mutex);

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
		"num_threads", "10",
		NULL
	};

	int rc, tfd;
	struct mg_context *cv_ctx;

	rc = nvram_read(&gs_rd);
	if (rc == ENOENT)
		rc = nvram_write(&gs_rd);
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

	if (memcache_cfg) {
		rc = memcache_init(memcache_cfg);
		xassert(!rc, goto out_tlm_exit, "%d", rc);
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

	if (memcache_cfg)
		memcache_exit();

out_tlm_exit:
	if (telemetry_cfg)
		telemetry_exit();
out_clo_tfd:
	close(tfd);
out_stop:
	mg_stop(cv_ctx);
	mg_exit_library();

	/*
	 * Handle pending gs_rd sync, in case gs_rd has been modified after
	 * we got the stop signal or shortly before. At this point all extra
	 * threads (civetweb) are stopped, so no locking is needed.
	 */
	if (gs_rd.sync)
		nvram_write(&gs_rd);

	/*
	 * Turn the furnace off. GPIO pins keep their state, and we must make
	 * sure we don't leave heating or cooling running.
	 */
	rc = gpiod_line_set_value_bulk(&bulk, gpio_def_val);
	xassert(!rc, NOOP, "%d", errno);

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
		memcache_cfg = cJSON_GetObjectItem(config, "memcache");
	}

	return rc;
}

void global_cleanup(void)
{
	cJSON_Delete(config);
}

int main(int argc, char **argv)
{
	static const struct sigaction term_sa = {.sa_handler = term_sig_hdlr};
	static const struct sigaction usr2_sa = {.sa_handler = usr2_sig_hdlr};

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

	sigaction(SIGINT, &term_sa, NULL);
	sigaction(SIGQUIT, &term_sa, NULL);
	sigaction(SIGTERM, &term_sa, NULL);
	signal(SIGUSR1, SIG_IGN);
	sigaction(SIGUSR2, &usr2_sa, NULL);

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
