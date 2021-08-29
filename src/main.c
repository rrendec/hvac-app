#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <modbus/modbus.h>

#define prerr(...) fprintf(stderr, __VA_ARGS__)

#define __xassert(cond, fallback, msg, ...)					\
	do {									\
		if (!(cond)) {							\
			prerr("Assertion '%s' failed in %s() [%s:%d]" msg "\n", \
				#cond, __func__, __FILE__, __LINE__,		\
				##__VA_ARGS__);					\
			fallback;						\
		}								\
	} while (0)

#define xassert(cond, fallback, msg...) __xassert(cond, fallback, msg)

struct sensor_data {
	// AQ-N-LCD
	int temp1;
	int hum1;
	int aq;
	int temp_sp;
	int hum_sp;
	// XDUCER-D-TH
	int temp2;
	int hum2;
};

int sensor_read(modbus_t *mb, struct sensor_data *data)
{
	uint16_t reg[2];
	struct sensor_data tmp;
	int rc;

	// Note: Delay required between consecutive Modbus transactions with
	//       the same sensor, or else the sensor will ignore the second
	//       transaction. Experimentally, 5 ms is too little and 8 ms is
	//       okay. Use 10 ms just to be on the safe side.

	rc = modbus_set_slave(mb, 1);
	xassert(!rc, return errno, "%d", errno);

	usleep(10000);
	rc = modbus_read_registers(mb, 136, 2, reg);
	xassert(rc != -1, return errno, "%d", errno);
	tmp.temp1 = reg[0];
	tmp.hum1 = reg[1];

	usleep(10000);
	rc = modbus_read_registers(mb, 184, 1, reg);
	xassert(rc != -1, return errno, "%d", errno);
	tmp.aq = reg[0];

	usleep(10000);
	rc = modbus_read_registers(mb, 167, 2, reg);
	xassert(rc != -1, return errno, "%d", errno);
	tmp.temp_sp = reg[0];
	tmp.hum_sp = reg[1];

	rc = modbus_set_slave(mb, 2);
	xassert(!rc, return errno, "%d", errno);

	usleep(10000);
	rc = modbus_read_registers(mb, 34, 2, reg);
	xassert(rc != -1, return errno, "%d", errno);
	tmp.temp2 = reg[0] + 9; // FIXME: hard-coded calibration
	tmp.hum2 = reg[1] - 44; // FIXME: hard-coded calibration

	*data = tmp;
	return 0;
}

void sensor_print(const struct sensor_data *data)
{
	printf("Temp1\t%.01f\n", data->temp1 / 10.0);
	printf("Temp2\t%.01f\n", data->temp2 / 10.0);
	printf("Hum1\t%.01f\n", data->hum1 / 10.0);
	printf("Hum2\t%.01f\n", data->hum2 / 10.0);
	printf("AQ\t%d\n", data->aq);
	putchar('\n');
	printf("Temp SP\t%.01f\n", data->temp_sp / 10.0);
	printf("Hum SP\t%.01f\n", data->hum_sp / 10.0);
	fflush(stdout);
}

int main(void)
{
	modbus_t *mb;
	int rc;
	struct sensor_data sd;

	mb = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
	assert(mb);

	rc = modbus_connect(mb);
	assert(!rc);

	rc = sensor_read(mb, &sd);
	if (!rc)
		sensor_print(&sd);

	modbus_close(mb);
	modbus_free(mb);

	return 0;
}
