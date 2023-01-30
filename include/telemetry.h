#ifndef __HVAC_TELEMETRY_H__
#define __HVAC_TELEMETRY_H__

#include <sys/time.h>

#include "json.h"

struct telemetry_data {
	struct timeval tv;

	struct {
		int furnace_mode;
		int humid_mode;
		int erv_mode;
		float temp_sp_heat;
		float temp_sp_cool;
		float temp_thres;
		float humid_sp;
	} run;

	struct {
		unsigned int furnace_blow:1;
		unsigned int furnace_heat:1;
		unsigned int furnace_cool:1;
		unsigned int humid_d_close:1;
		unsigned int humid_d_open:1;
		unsigned int humid_fan:1;
		unsigned int humid_valve:1;
		unsigned int erv_off:1;
		unsigned int erv_recirc:1;
		unsigned int erv_low:1;
		unsigned int erv_high:1;
	} ctrl;

	struct {
		float temp1;
		float humid1;
		float temp2;
		float humid2;
	} sens;
};

int telemetry_init(cJSON *cfg);
int telemetry_exit(void);
int telemetry_send(const struct telemetry_data *data);

#endif
