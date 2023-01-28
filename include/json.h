#ifndef __HVAC_JSON_H__
#define __HVAC_JSON_H__

#include <cjson/cJSON.h>

int _map_find(const char * const *map, int size, const char *key);
#define map_find(map, key) _map_find(map, ARRAY_SIZE(map), key)

const char *json_get_string(cJSON *json, const char *key, const char *_default);

#define json_map_string(json, key, _default, map) ({				\
	const char *str = json_get_string(json, key, _default);			\
	str ? map_find(map, str) : -1;						\
})

double json_get_number(cJSON *json, const char *key,
		       double min, double max, double _default);

int json_read(const char *path, cJSON **json, int *errl, int *errc);
int json_write(const char *path, cJSON *json);

#endif
