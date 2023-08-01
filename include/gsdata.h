#ifndef __HVAC_GSDATA_H__
#define __HVAC_GSDATA_H__

/* Global shared/state types and data */

#include <pthread.h>

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

struct ext_data {
	int temp;
	int humid;
	int pres;
	int aqi;
	time_t dt;
};

extern struct sensor_data sd_inst;
extern pthread_mutex_t sd_mutex;

extern struct run_data rd_inst;
extern struct ctrl_data cd_inst;
extern struct ext_data ed_inst;
extern pthread_mutex_t rd_mutex;
#endif
