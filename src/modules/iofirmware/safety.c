
/**
 * @file safety.c
 * Safety button logic.  安全开关逻辑
 */

#include <nuttx/config.h>

#include <stdbool.h>

#include <drivers/drv_hrt.h>

#include "io.h"

static struct hrt_call arming_call;    //解锁回调
static struct hrt_call failsafe_call;  //失效保护回调

/*
 * Count the number of times in a row that we see the arming button
 * held down.
 */
static unsigned counter = 0;

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 		0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

static unsigned blink_counter = 0;

/*
 * IMPORTANT: The arming state machine critically
 * 	      depends on using the same threshold
 *            for arming and disarming. Since disarming
 *            is quite deadly for the system, a similar
 *            length can be justified.
 */
#define ARM_COUNTER_THRESHOLD	10  //解锁计数阈值

static bool safety_button_pressed;  //安全开关按下状态标志位

static void safety_check_button(void *arg);  //检查安全开关
static void failsafe_blink(void *arg);  //失效保护

void
safety_init(void)
{
    /* arrange for the button handler to be called at 10Hz */  //1s钟调用10次
	hrt_call_every(&arming_call, 1000, 100000, safety_check_button, NULL);
}

void
failsafe_led_init(void)  //失效保护led
{
	/* arrange for the failsafe blinker to be called at 8Hz */
	hrt_call_every(&failsafe_call, 1000, 125000, failsafe_blink, NULL);
}

static void
safety_check_button(void *arg)
{
	/*
	 * Debounce the safety button, change state if it has been held for long enough.
     *去开关抖动，如果开关按住足够的时间，状态才改变
	 */
    safety_button_pressed = BUTTON_SAFETY;//读取开关状态

	/*
	 * Keep pressed for a while to arm.
     *保持按住状态解锁
	 * Note that the counting sequence has to be same length
	 * for arming / disarming in order to end up as proper
	 * state machine, keep ARM_COUNTER_THRESHOLD the same
	 * length in all cases of the if/else struct below.
     * 加锁和解锁的计数序列长度一样，为了结束适当的状态机
	 */
	if (safety_button_pressed && !(r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
        (r_setup_arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK)) { //按钮按下，并且接收状态，安全关闭

        if (counter < ARM_COUNTER_THRESHOLD) {//如果计数小于计数阈值
			counter++;

		} else if (counter == ARM_COUNTER_THRESHOLD) {
            /* switch to armed state *///转换到解锁状态
			r_status_flags |= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
			counter++;
		}

	} else if (safety_button_pressed && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)) {

		if (counter < ARM_COUNTER_THRESHOLD) {
			counter++;

		} else if (counter == ARM_COUNTER_THRESHOLD) {
			/* change to disarmed state and notify the FMU */
			r_status_flags &= ~PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
			counter++;
		}

	} else {
		counter = 0;
	}

	/* Select the appropriate LED flash pattern depending on the current IO/FMU arm state */
	uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

	if (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
		if (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED) {
			pattern = LED_PATTERN_IO_FMU_ARMED;

		} else {
			pattern = LED_PATTERN_IO_ARMED;
		}

	} else if (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED) {
		pattern = LED_PATTERN_FMU_ARMED;

	} else if (r_setup_arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK) {
		pattern = LED_PATTERN_FMU_OK_TO_ARM;

	}

	/* Turn the LED on if we have a 1 at the current bit position */
    LED_SAFETY(pattern & (1 << blink_counter++)); //安全开关上的led

	if (blink_counter > 15) {
		blink_counter = 0;
	}
}

static void
failsafe_blink(void *arg)  //失效保护闪灯
{
	/* indicate that a serious initialisation error occured */
	if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)) {
        LED_AMBER(true); //琥珀色灯
		return;
	}

	static bool failsafe = false;

	/* blink the failsafe LED if we don't have FMU input */
	if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {
		failsafe = !failsafe;

	} else {
		failsafe = false;
	}

	LED_AMBER(failsafe);  //依次按编码亮灯
}
