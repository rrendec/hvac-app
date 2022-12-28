#ifndef __HVAC_BOARD_RPI_HVAC_H__
#define __HVAC_BOARD_RPI_HVAC_H__

#define GPIO_CHIP_LABEL "pcf8575"

#define GPIO_MAP_INITIALIZER {							\
	[GPIO_FURNACE_BLOW]	= 0,	/* P00 */				\
	[GPIO_FURNACE_HEAT]	= 1,	/* P01 */				\
	[GPIO_FURNACE_COOL]	= 2,	/* P02 */				\
	[GPIO_HUMID_D_CLOSE]	= 4,	/* P04 */				\
	[GPIO_HUMID_D_OPEN]	= 5,	/* P05 */				\
	[GPIO_HUMID_FAN]	= 6,	/* P06 */				\
	[GPIO_HUMID_VALVE]	= 7,	/* P07 */				\
}

#endif
