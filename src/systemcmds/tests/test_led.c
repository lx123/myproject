#include <nuttx/config.h>
#include <px4_posix.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <arch/board/board.h>

#include <drivers/drv_led.h>

#include "tests.h"

int test_led(int argc, char *argv[])
{
	int		fd;
	int		ret = 0;

	fd = px4_open(LED0_DEVICE_PATH, 0);

	if (fd < 0) {
		printf("\tLED: open fail\n");
		return ERROR;
	}

	if (px4_ioctl(fd, LED_ON, LED_AMBER)) {  //只有琥珀色的灯

		printf("\tLED: ioctl fail\n"); //操作失败
		return ERROR;
	}

	/* let them blink for fun */

	int i;
	uint8_t ledon = 1;

	for (i = 0; i < 10; i++) { //闪烁5次
		if (ledon) {
			px4_ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			px4_ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon; //取反
		usleep(60000);
	}

	/* Go back to default */
	px4_ioctl(fd, LED_OFF, LED_AMBER);

	printf("\t LED test completed, no errors.\n");

	return ret;
}


