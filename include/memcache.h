#ifndef __HVAC_MEMCACHE_H__
#define __HVAC_MEMCACHE_H__

#include "json.h"

#define MEMCACHE_MAX_KEY	250
#define MEMCACHE_MAX_ITEM	65280
#define MEMCACHE_MAX_CMD	(MEMCACHE_MAX_KEY + 80)

int memcache_init(cJSON *cfg);
int memcache_exit(void);

#endif
