#include "gsdata.h"

/* sensor data below */
struct sensor_data sd_inst;
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
