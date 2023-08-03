#ifndef __HVAC_COMMON_H__
#define __HVAC_COMMON_H__

#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <systemd/sd-daemon.h>

extern pthread_mutex_t oestream_mutex;

#define NOOP ((void)0)

#define xfprintf(stream, ...) do {						\
	int __errno = errno;							\
	FILE *__stream = (stream);						\
	pthread_mutex_lock(&oestream_mutex);					\
	fprintf(__stream, __VA_ARGS__);						\
	fflush(__stream);							\
	pthread_mutex_unlock(&oestream_mutex);					\
	errno = __errno;							\
} while (0)

#define xprintf(...) xfprintf(stdout, __VA_ARGS__)
#define xprerrf(...) xfprintf(stderr, __VA_ARGS__)

#define first_arg(arg, ...) arg
#define shift_arg(arg, ...) __VA_OPT__(,) __VA_ARGS__

#define xassert(condition, fallback, ...) ({					\
	if (!(condition)) {							\
		xprerrf(SD_ERR "Assertion '%s' failed in %s() [%s:%d]"		\
			__VA_OPT__(": " first_arg(__VA_ARGS__)) "\n",		\
			#condition, __func__, __FILE__, __LINE__		\
			__VA_OPT__(shift_arg(__VA_ARGS__)));			\
		fallback;							\
	}									\
})

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

extern __thread volatile int __canceled;

#define CHECK_CANCELED(fallback) ({						\
	if (__canceled) {							\
		fallback;							\
	}									\
})

#define __RETRY_NC(expression, fallback, ...) ({				\
	long int __rc;								\
	do {									\
		__rc = (long int)(expression);					\
		CHECK_CANCELED(fallback);					\
	} while (__rc == -1L && errno == EINTR);				\
	__rc;									\
})

#define RETRY_NC(expression, fallback...)					\
	__RETRY_NC(expression, ##fallback, break)

#endif
