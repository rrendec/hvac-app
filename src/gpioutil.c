#include <dirent.h>
#include <unistd.h>
#include <string.h>
#include <gpiod.h>
#include <sys/types.h>
#include <sys/syscall.h>

#include "common.h"

struct gpiod_chip *gpiod_chip_open_by_label(const char *label)
{
	DIR *dp;
	struct dirent *entry;
	struct gpiod_chip *chip = NULL;

	dp = opendir("/dev");
	xassert(dp, return NULL, "%d", errno);

	while ((entry = readdir(dp))) {
		char path[20];
		struct gpiod_chip_info *info;

		if (STATIC_STRNCMP(entry->d_name, "gpiochip"))
			continue;

		__ccignore(format-truncation,
			   snprintf(path, sizeof(path), "/dev/%s", entry->d_name));
		chip = gpiod_chip_open(path);
		xassert(chip, continue, "%d", errno);

		info = gpiod_chip_get_info(chip);
		if (info) {
			const char *xlabel = gpiod_chip_info_get_label(info);
			bool match = xlabel && !strcmp(xlabel, label);

			gpiod_chip_info_free(info);
			if (match)
				break;
		}

		gpiod_chip_close(chip);
		chip = NULL;
	}

	closedir(dp);

	return chip;
}
