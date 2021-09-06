#ifndef __HVAC_COMMON_H__
#define __HVAC_COMMON_H__

#include <errno.h>

#define prerr(...) do {								\
	int __errno = errno;							\
	fprintf(stderr, __VA_ARGS__);						\
	fflush(stderr);								\
	errno = __errno;							\
} while (0)

#define first_arg(arg, ...) arg
#define shift_arg(arg, ...) __VA_OPT__(,) __VA_ARGS__

#define xassert(condition, fallback, ...) do {					\
	if (!(condition)) {							\
		prerr("Assertion '%s' failed in %s() [%s:%d]"			\
			__VA_OPT__(": " first_arg(__VA_ARGS__)) "\n",		\
			#condition, __func__, __FILE__, __LINE__		\
			__VA_OPT__(shift_arg(__VA_ARGS__)));			\
		fallback;							\
	}									\
} while (0)

#endif
