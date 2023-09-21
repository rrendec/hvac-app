#include "gsdata.h"

/* Global sensor data */
struct sensor_data gs_sd;

/* Global run data */
struct run_data gs_rd = {
	.furnace_mode = FURNACE_OFF,
	.erv_mode = ERV_OFF,
	.temp_sp_heat = 220,	// 22.0 C
	.temp_sp_cool = 250,	// 25.0 C
	.temp_thres = 5,	// 0.5 C
	.humid_sp = 350,	// 35.0 %
};

/* Global control data */
struct ctrl_data gs_cd = {
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

/* Global external data */
struct ext_data gs_ed;

/* Global data mutex */
pthread_mutex_t gs_mutex = PTHREAD_MUTEX_INITIALIZER;
