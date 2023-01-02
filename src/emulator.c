#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <gpiod.h>
#include <modbus/modbus.h>

#include "common.h"

/* {{{ gpio */

struct gpiod_line;

#define gpio_to_line(chip, offset) ((struct gpiod_line *)			\
	(((unsigned long)(chip) << 16) | ((offset) & 0xffff)))
#define gpio_line_chip(line)							\
	((unsigned int)((unsigned long)(line) >> 16))
#define gpio_line_offset(line)							\
	((unsigned int)((unsigned long)(line) & 0xffff))

static const char * const gpio_path = "emu/gpio.txt";

struct gpiod_chip *gpiod_chip_open_by_label(const char *label)
{
	return (struct gpiod_chip *)1UL;
}

int gpiod_chip_get_lines(struct gpiod_chip *chip,
			 unsigned int *offsets, unsigned int num_offsets,
			 struct gpiod_line_bulk *bulk)
{
	unsigned int i;

	for (i = 0; i < num_offsets; i++)
		bulk->lines[i] = gpio_to_line(chip, offsets[i]);

	bulk->num_lines = num_offsets;

	return 0;
}

int gpiod_line_request_output(struct gpiod_line *line,
			      const char *consumer, int default_val)
{
	gpiod_line_set_value(line, default_val);

	return 0;
}

int gpiod_line_request_bulk_output(struct gpiod_line_bulk *bulk,
				   const char *consumer,
				   const int *default_vals)
{
	int i, rc;

	for (i = 0; i < bulk->num_lines; i++) {
		rc = gpiod_line_request_output(bulk->lines[i], consumer,
					       default_vals[i]);
		if (rc)
			return rc;
	}

	return 0;
}

int gpiod_line_set_value(struct gpiod_line *line, int value)
{
	char tmp[PATH_MAX];
	char buf[120], *p;
	unsigned int chip = gpio_line_chip(line);
	unsigned int offset = gpio_line_offset(line);
	unsigned int xchip, xoffset, xvalue;
	int fd = -1, append = 1, ret;
	FILE *in = NULL, *out = NULL;

	in = fopen(gpio_path, "r");
	xassert(in, return errno, "%d", errno);

	snprintf(tmp, sizeof(tmp), "%s.XXXXXX", gpio_path);
	fd = mkstemp(tmp);
	xassert(fd != -1, goto out_close, "%d", errno);

	out = fdopen(fd, "r+");
	xassert(out, goto out_close, "%d", errno);

	while (fgets(buf, sizeof(buf), in)) {
		p = strchr(buf, '#');

		if (p)
			*p = '\0';

		if (sscanf(buf, "%u %u %u", &xchip, &xoffset, &xvalue) == 3 &&
			xchip == chip && xoffset == offset) {
			fprintf(out, "%u %u %u\n", chip, offset, value);
			append = 0;
			continue;
		}

		if (p)
			*p = '#';

		fputs(buf, out);
	}

	if (append)
		fprintf(out, "%u %u %u\n", chip, offset, value);

	fclose(in);
	fclose(out);

	ret = rename(tmp, gpio_path);
	xassert(!ret, goto out_unlink, "%d", errno);

	return ret;

out_close:
	ret = errno;
	fclose(in);

	if (out)
		fclose(out);
	else
		close(fd);

out_unlink:
	if (fd != -1)
		unlink(tmp);

	return ret;
}

int gpiod_line_set_value_bulk(struct gpiod_line_bulk *bulk, const int *values)
{
	int i, rc;

	for (i = 0; i < bulk->num_lines; i++) {
		rc = gpiod_line_set_value(bulk->lines[i], values[i]);
		if (rc)
			return rc;
	}

	return 0;
}

void gpiod_line_release_bulk(struct gpiod_line_bulk *bulk)
{
}

void gpiod_chip_close(struct gpiod_chip *chip)
{
}

/* }}} gpio */

/* {{{ modbus */

struct _modbus {
	int slave;
};

static const char * const modbus_path = "emu/modbus.txt";

modbus_t* modbus_new_rtu(const char *device, int baud, char parity,
			 int data_bit, int stop_bit)
{
	return calloc(1, sizeof(modbus_t));
}

int modbus_connect(modbus_t *ctx)
{
	return 0;
}

int modbus_set_slave(modbus_t* ctx, int slave)
{
	ctx->slave = slave;

	return 0;
}

static int modbus_read_one(int slave, int addr, uint16_t *dest)
{
	char buf[120], *p;
	int xslave, xaddr, xvalue;
	int ret = EIO;
	FILE *in = fopen(modbus_path, "r");

	xassert(in, return errno, "%d", errno);

	while (fgets(buf, sizeof(buf), in)) {
		p = strchr(buf, '#');

		if (p)
			*p = '\0';

		if (sscanf(buf, "%d %d %d", &xslave, &xaddr, &xvalue) != 3 ||
		    xslave != slave || xaddr != addr)
			continue;

		if (xvalue < 0) {
			ret = -xvalue;
		} else {
			*dest = xvalue;
			ret = 0;
		}

		break;
	}

	fclose(in);

	return ret;
}

int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest)
{
	int rc;

	while (nb-- > 0) {
		rc = modbus_read_one(ctx->slave, addr++, dest++);
		if (rc) {
			errno = rc;
			return -1;
		}
	}

	return 0;
}

void modbus_close(modbus_t *ctx)
{
}

void modbus_free(modbus_t *ctx)
{
	free(ctx);
}

/* }}} modbus */
