#ifndef __HVAC_COMMON_H__
#define __HVAC_COMMON_H__

#include <errno.h>
#include <systemd/sd-daemon.h>

#define xprintf(...) do {							\
	printf(__VA_ARGS__);							\
	fflush(stdout);								\
} while (0)

#define xfprintf(stream, ...) do {						\
	FILE *__stream = (stream);						\
	fprintf(__stream, __VA_ARGS__);						\
	fflush(__stream);							\
} while (0)

#define xprerrf(...) xfprintf(stderr, __VA_ARGS__)

#define first_arg(arg, ...) arg
#define shift_arg(arg, ...) __VA_OPT__(,) __VA_ARGS__

#define xassert(condition, fallback, ...) do {					\
	if (!(condition)) {							\
		int __errno = errno;						\
		xprerrf(SD_ERR "Assertion '%s' failed in %s() [%s:%d]"		\
			__VA_OPT__(": " first_arg(__VA_ARGS__)) "\n",		\
			#condition, __func__, __FILE__, __LINE__		\
			__VA_OPT__(shift_arg(__VA_ARGS__)));			\
		errno = __errno;						\
		fallback;							\
	}									\
} while (0)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#endif
