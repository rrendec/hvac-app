#ifndef __HVAC_GPIOUTIL_H__
#define __HVAC_GPIOUTIL_H__

struct gpiod_chip *gpiod_chip_open_by_label(const char *label);

#endif
