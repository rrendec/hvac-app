#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <gpiod.h>
#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <modbus/modbus.h>

#include "common.h"

enum {
	MODE_HEAT,
	MODE_COOL,
} mode;

int thres = 5; // 0.5 C

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

enum gpio_pins {
	GPIO_FURNACE_BLOW,
	GPIO_FURNACE_HEAT,
	GPIO_FURNACE_COOL,
	NUM_GPIO_PINS
};

unsigned int gpio_pin_map[NUM_GPIO_PINS] = {
	[GPIO_FURNACE_BLOW] = 17, // GPIO_GEN0
	[GPIO_FURNACE_HEAT] = 18, // GPIO_GEN1
	[GPIO_FURNACE_COOL] = 27, // GPIO_GEN2
};

volatile int keep_going = 1;

int sensor_read(modbus_t *mb, struct sensor_data *data)
{
	uint16_t reg[2];
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
	data->temp1 = reg[0];
	data->hum1 = reg[1];

	usleep(10000);
	rc = modbus_read_registers(mb, 184, 1, reg);
	xassert(rc != -1, return errno, "%d", errno);
	data->aq = reg[0];

	usleep(10000);
	rc = modbus_read_registers(mb, 167, 2, reg);
	xassert(rc != -1, return errno, "%d", errno);
	data->temp_sp = reg[0];
	data->hum_sp = reg[1];

	rc = modbus_set_slave(mb, 2);
	xassert(!rc, return errno, "%d", errno);

	usleep(10000);
	rc = modbus_read_registers(mb, 34, 2, reg);
	xassert(rc != -1, return errno, "%d", errno);
	data->temp2 = reg[0] + 9; // FIXME: hard-coded calibration
	data->hum2 = reg[1] - 44; // FIXME: hard-coded calibration

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
}

void sig_hdlr(int signal)
{
	keep_going = 0;
}

int main(int argc, char **argv)
{
	static const struct sigaction act = {.sa_handler = sig_hdlr};

	struct gpiod_chip *chip;
	struct gpiod_line_bulk bulk;
	modbus_t *mb;
	int i, rc;
	struct sensor_data sd;
	enum {STATE_OFF, STATE_ON} state = STATE_OFF;

	do {
		if (argc > 1) {
			if (!strcmp(argv[1], "heat")) {
				mode = MODE_HEAT;
				break;
			} else if (!strcmp(argv[1], "cool")) {
				mode = MODE_COOL;
				break;
			}
		}

		fprintf(stderr, "Usage: %s <heat|cool>\n", argv[0]);
		return EXIT_FAILURE;
	} while (0);

	sigaction(SIGINT, &act, NULL);
	sigaction(SIGQUIT, &act, NULL);
	sigaction(SIGTERM, &act, NULL);
	signal(SIGUSR1, SIG_IGN);
	signal(SIGUSR2, SIG_IGN);

	chip = gpiod_chip_open_by_name("gpiochip0");
	assert(chip);

	rc = gpiod_chip_get_lines(chip, gpio_pin_map, NUM_GPIO_PINS, &bulk);
	assert(!rc);

	for (i = 0; i < NUM_GPIO_PINS; i++) {
		rc = gpiod_line_request_output(bulk.lines[i], "hvac", 1);
		assert(!rc);
	}

	mb = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
	assert(mb);

	rc = modbus_connect(mb);
	assert(!rc);

	// Blower on
	gpiod_line_set_value(bulk.lines[GPIO_FURNACE_BLOW], 0);

	// In case blower was off, give it time to get to nominal speed
	sleep(2);

	while (keep_going) {
		struct timeval tv;
		struct tm *tm;
		int temp;

		gettimeofday(&tv, NULL);
		tm = localtime(&tv.tv_sec);
		printf("\n\n%s\n", asctime(tm));
		fflush(stdout);

		rc = sensor_read(mb, &sd);
		if (rc) {
			sleep(1);
			continue;
		}

		temp = (sd.temp1 + 3 * sd.temp2) / 4;

		sensor_print(&sd);
		fflush(stdout);

		switch (mode) {
		case MODE_HEAT:
			if (temp > sd.temp_sp + thres && state == STATE_ON) {
				printf("\n* HEAT OFF\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 1);
				state = STATE_OFF;
				break;
			}
			if (temp < sd.temp_sp - thres && state == STATE_OFF) {
				printf("\n* HEAT ON\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 0);
				state = STATE_ON;
			}
			break;
		case MODE_COOL:
			if (temp > sd.temp_sp + thres && state == STATE_OFF) {
				printf("\n* COOL ON\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 0);
				state = STATE_ON;
				break;
			}
			if (temp < sd.temp_sp - thres && state == STATE_ON) {
				printf("\n* COOL OFF\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 1);
				state = STATE_OFF;
			}
			break;
		}

		fflush(stdout);
		sleep(5);
	}

	printf("\nShutting down...\n");
	fflush(stdout);

	modbus_close(mb);
	modbus_free(mb);

	for (i = 0; i < NUM_GPIO_PINS; i++)
		gpiod_line_set_value(bulk.lines[i], 1);

	gpiod_line_release_bulk(&bulk);
	gpiod_chip_close(chip);

	return EXIT_SUCCESS;
}
