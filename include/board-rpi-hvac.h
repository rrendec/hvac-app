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
	[GPIO_ERV_OFF]		= 8,	/* P10 */				\
	[GPIO_ERV_RECIRC]	= 9,	/* P11 */				\
	[GPIO_ERV_LOW]		= 10,	/* P12 */				\
	[GPIO_ERV_HIGH]		= 11,	/* P13 */				\
}

#endif
