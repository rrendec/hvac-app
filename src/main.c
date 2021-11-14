#define _GNU_SOURCE

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <gpiod.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
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
volatile pid_t child_pid;

/* hw handles below */
struct gpiod_chip *chip;
struct gpiod_line_bulk bulk;
modbus_t *mb;
/* end of hw handles */

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
		"T1=%.01f T2=%.01f H1=%.01f H2=%.01f AQ=%d TP=%.01f HP=%.01f\n",
		sd->temp1 / 10.0, sd->temp2 / 10.0,
		sd->hum1 / 10.0, sd->hum2 / 10.0,
		sd->aq, sd->temp_sp / 10.0, sd->hum_sp / 10.0);
}

int sensors_once(void)
{
	struct sensor_data sd;
	int rc;

	rc = modbus_init();
	xassert(!rc, return rc, "%d", rc);

	rc = sensor_read(mb, &sd);
	xassert(!rc, return rc, "%d", rc);

	sensors_print(&sd);
	modbus_cleanup();

	return 0;
}

int worker(void)
{
	struct sensor_data sd;
	enum {STATE_OFF, STATE_ON} state = STATE_OFF;
	int i, rc;

	rc = gpio_init();
	xassert(!rc, return rc, "%d", rc);
	rc = modbus_init();
	xassert(!rc, return rc, "%d", rc);

	// Blower on
	gpiod_line_set_value(bulk.lines[GPIO_FURNACE_BLOW], 0);

	// In case blower was off, give it time to get to nominal speed
	sleep(2);

	while (keep_going) {
		int temp;

		rc = sensor_read(mb, &sd);
		if (rc) {
			sleep(1);
			continue;
		}

		sensors_print(&sd);
		temp = (sd.temp1 + 3 * sd.temp2) / 4;

		switch (mode) {
		case MODE_HEAT:
			if (temp > sd.temp_sp + thres && state == STATE_ON) {
				xprintf(SD_NOTICE "HEAT OFF\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 1);
				state = STATE_OFF;
				break;
			}
			if (temp < sd.temp_sp - thres && state == STATE_OFF) {
				xprintf(SD_NOTICE "HEAT ON\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_HEAT], 0);
				state = STATE_ON;
			}
			break;
		case MODE_COOL:
			if (temp > sd.temp_sp + thres && state == STATE_OFF) {
				xprintf(SD_NOTICE "COOL ON\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 0);
				state = STATE_ON;
				break;
			}
			if (temp < sd.temp_sp - thres && state == STATE_ON) {
				xprintf(SD_NOTICE "COOL OFF\n");
				gpiod_line_set_value(bulk.lines[GPIO_FURNACE_COOL], 1);
				state = STATE_OFF;
			}
			break;
		}

		sleep(5);
	}

	xprintf(SD_INFO "Shutting down...\n");

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

int main(int argc, char **argv)
{
	static const struct sigaction act = {.sa_handler = sig_hdlr};

	int opt, rc;
	int fg = 0, fail = 0;
	pid_t wpid_ret;
	int wstatus = 0;

	while ((opt = getopt(argc, argv, "fm:s")) != -1) {
		switch (opt) {
		case 'f':
			fg = 1;
			break;
		case 'm':
			if (!strcmp(optarg, "heat")) {
				mode = MODE_HEAT;
			} else if (!strcmp(optarg, "cool")) {
				mode = MODE_COOL;
			} else
				fail = 1;
			break;
		case 's':
			return sensors_once() ? EXIT_FAILURE : EXIT_SUCCESS;
		default:
			fail = 1;
		}
	}

	if (fail) {
		fprintf(stderr, "Usage: %s [-f] [-m mode]\n", argv[0]);
		return EXIT_FAILURE;
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
