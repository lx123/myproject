/**
 * @file led.cpp
 *
 * LED driver.
 */

#include <board_config.h>
#include <drivers/device/device_nuttx.h>
#include <drivers/drv_led.h>
#include <stdio.h>

__BEGIN_DECLS
extern void led_init();
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

class LED : device::CDev
{
public:
	LED();
	virtual ~LED();

	virtual int init();
	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);
};

LED::LED() :
		CDev("led", LED0_DEVICE_PATH)
{
	init();
}

LED::~LED()
{
}

int
LED::init()
{
//	DEVICE_DEBUG("LED::init");
	CDev::init();
	led_init();
	return 0;
}

int
LED::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int result = OK;

	switch (cmd) {
	case LED_ON:
		led_on(arg);
		break;

	case LED_OFF:
		led_off(arg);
		break;

	case LED_TOGGLE:
		led_toggle(arg);
		break;

	default:
		result = CDev::ioctl(filp, cmd, arg);
	}
	return result;
}

namespace
{
LED *gLED;
}

void
drv_led_start(void)
{
	if(gLED == nullptr) {
		gLED = new LED;
	}

	if(gLED != nullptr){
		gLED->init();
	}
}



