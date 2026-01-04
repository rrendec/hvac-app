#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <gpiod.h>
#include <modbus/modbus.h>

#include "common.h"

/* {{{ gpio */

/*
 * This is not a complete or true emulation. Multiple assumptions are made about
 * implementation details in gpio_init() and gpio_state_sync() (in src/main.c).
 *   - The initialization is done in a linear sequence, one chip at a time.
 *   - All lines are active low, and the initial value for output lines is
 *     "inactive". The data in struct gpiod_line_settings is ignored.
 *   - Only one tuple of line settings object and pin array is added per each
 *     line configuration object (gpiod_line_config_add_line_settings() is
 *     called only once for each line configuration object).
 */

#define MAX_REQUESTS	3
#define MAX_LINES	32

#define DEF_DUMMY_FN_INT(fn, ...) int fn(__VA_ARGS__)				\
	{									\
		return 0;							\
	}

#define DEF_DUMMY_FN_VOID(fn, ...) void fn(__VA_ARGS__) {}

#define DEF_DUMMY_ALLOC(type, fn, ...) type fn(__VA_ARGS__)			\
	{									\
		return (type)1UL;						\
	}

#define DEF_DUMMY_FREE(type, fn) DEF_DUMMY_FN_VOID(fn, type)

static const char * const gpio_path = "emu/gpio.txt";
static struct {
	unsigned int chip_idx;
	unsigned int offsets[MAX_LINES];
	unsigned int num_offsets;
	enum gpiod_line_direction direction;
} req_map[MAX_REQUESTS];
static unsigned long dummy_chip_idx;
static unsigned long dummy_request_idx;

struct gpiod_chip *gpiod_chip_open_by_label(const char *label)
{
	return (void *)++dummy_chip_idx;
}
DEF_DUMMY_FREE(struct gpiod_chip *, gpiod_chip_close)

DEF_DUMMY_ALLOC(struct gpiod_chip_info *, gpiod_chip_get_info, struct gpiod_chip *chip)
DEF_DUMMY_FREE(struct gpiod_chip_info *, gpiod_chip_info_free)

DEF_DUMMY_ALLOC(struct gpiod_line_settings *, gpiod_line_settings_new, void)
DEF_DUMMY_FREE(struct gpiod_line_settings *, gpiod_line_settings_free)
DEF_DUMMY_FN_VOID(gpiod_line_settings_reset, struct gpiod_line_settings *settings)

DEF_DUMMY_ALLOC(struct gpiod_line_config *, gpiod_line_config_new, void)
DEF_DUMMY_FREE(struct gpiod_line_config *, gpiod_line_config_free)
DEF_DUMMY_FN_VOID(gpiod_line_config_reset, struct gpiod_line_config *config)

DEF_DUMMY_ALLOC(struct gpiod_request_config *, gpiod_request_config_new, void)
DEF_DUMMY_FREE(struct gpiod_request_config *, gpiod_request_config_free)

int gpiod_line_settings_set_direction(struct gpiod_line_settings *settings,
				      enum gpiod_line_direction direction)
{
	req_map[dummy_request_idx].direction = direction;
	return 0;
}
DEF_DUMMY_FN_VOID(gpiod_line_settings_set_active_low,
		  struct gpiod_line_settings *settings, bool active_low)
DEF_DUMMY_FN_INT(gpiod_line_settings_set_output_value,
		 struct gpiod_line_settings *settings, enum gpiod_line_value value)

int gpiod_line_config_add_line_settings(struct gpiod_line_config *config,
	const unsigned int *offsets, size_t num_offsets, struct gpiod_line_settings *settings)
{
	if (num_offsets > MAX_LINES) {
		errno = EINVAL;
		return -1;
	}

	req_map[dummy_request_idx].chip_idx = dummy_chip_idx;
	memcpy(req_map[dummy_request_idx].offsets, offsets, num_offsets * sizeof(unsigned int));
	req_map[dummy_request_idx].num_offsets = num_offsets;

	return 0;
}

DEF_DUMMY_FN_VOID(gpiod_request_config_set_consumer,
	struct gpiod_request_config *config, const char *consumer)

struct gpiod_line_request *gpiod_chip_request_lines(struct gpiod_chip *chip,
	struct gpiod_request_config *req_cfg, struct gpiod_line_config *line_cfg)
{
	unsigned long req_idx = dummy_request_idx;

	/* sanity check */
	if ((unsigned long)chip != dummy_chip_idx) {
		errno = EINVAL;
		return NULL;
	}

	/*
	 * Increment the request idx counter. At the cost of wasting the last
	 * req_map slot, we check the bounds here to avoid checking it in each
	 * setter function that stores settings for the *next* request.
	 */
	if (++dummy_request_idx >= MAX_REQUESTS) {
		errno = ENOMEM;
		return NULL;
	}

	/* set initial state for output pins */
	if (req_map[req_idx].direction == GPIOD_LINE_DIRECTION_OUTPUT) {
		enum gpiod_line_value values[MAX_LINES] = {};
		if (gpiod_line_request_set_values((void *)(req_idx + 1), values))
			return NULL;
	}

	return (void *)dummy_request_idx;
}

DEF_DUMMY_FREE(struct gpiod_line_request *request, gpiod_line_request_release)

static int find_offset(const typeof(req_map[0]) *req, unsigned int offset)
{
	int idx;

	for (idx = 0; idx < req->num_offsets; idx++)
		if (req->offsets[idx] == offset)
			return idx;

	return -1;
}

int gpiod_line_request_set_values(struct gpiod_line_request *request,
				  const enum gpiod_line_value *values)
{
	char tmp[PATH_MAX];
	char buf[120];
	unsigned long req_idx = (unsigned long)request - 1;
	const typeof(req_map[0]) *req = &req_map[req_idx];
	unsigned int xchip, xoffset, xvalue;
	int fd = -1, idx, ret, len;
	FILE *in = NULL, *out = NULL;
	bool found[MAX_LINES] = {};

	if (req_idx >= dummy_request_idx) {
		errno = EINVAL;
		return -1;
	}

	if (req->direction != GPIOD_LINE_DIRECTION_OUTPUT) {
		errno = EINVAL;
		return -1;
	}

	in = fopen(gpio_path, "r");
	xassert(in, return errno, "%d", errno);

	snprintf(tmp, sizeof(tmp), "%s.XXXXXX", gpio_path);
	fd = mkstemp(tmp);
	xassert(fd != -1, goto out_close, "%d", errno);

	out = fdopen(fd, "r+");
	xassert(out, goto out_close, "%d", errno);

	while (fgets(buf, sizeof(buf), in)) {
		if (sscanf(buf, "%u %u %u%n", &xchip, &xoffset, &xvalue, &len) == 3 &&
			xchip == req->chip_idx && (idx = find_offset(req, xoffset)) >= 0) {
			fprintf(out, "%u %u %u%s", xchip, xoffset, !values[idx], &buf[len]);
			found[idx] = true;
		} else
			fputs(buf, out);
	}

	for (idx = 0; idx < req->num_offsets; idx++)
		if (!found[idx])
			fprintf(out, "%u %u %u\n", req->chip_idx,
				req->offsets[idx], !values[idx]);

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
	char buf[120];
	int xslave, xaddr, xvalue;
	int ret = EIO;
	FILE *in = fopen(modbus_path, "r");

	xassert(in, return errno, "%d", errno);

	while (fgets(buf, sizeof(buf), in)) {
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
