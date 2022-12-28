#ifndef __HVAC_BOARD_RPI_BARE_H__
#define __HVAC_BOARD_RPI_BARE_H__

#define GPIO_CHIP_LABEL "pinctrl-bcm2835"

#define GPIO_MAP_INITIALIZER {							\
	[GPIO_FURNACE_BLOW]	= 17,	/* GPIO_GEN0 */				\
	[GPIO_FURNACE_HEAT]	= 18,	/* GPIO_GEN1 */				\
	[GPIO_FURNACE_COOL]	= 27,	/* GPIO_GEN2 */				\
	[GPIO_HUMID_D_CLOSE]	= 23,	/* GPIO_GEN4 */				\
	[GPIO_HUMID_D_OPEN]	= 24,	/* GPIO_GEN5 */				\
	[GPIO_HUMID_FAN]	= 25,	/* GPIO_GEN6 */				\
	[GPIO_HUMID_VALVE]	= 4,	/* GPCLK0    */				\
}

#endif
