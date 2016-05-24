/**
 * @file drv_hrt.c
 *
 * High-resolution timer callouts and timekeeping.   高精度定时器回调和时间保持
 *
 * This can use any general or advanced STM32 timer.  可以用在任何高级定时器
 *
 * Note that really, this could use systick too, but that's   实际上，这个也可以使用systic，但是systic被Nuttx使用，修改它会有些问题
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX STM32 driver per se; rather, we
 * claim the timer and then drive it directly.  //我们直接驱动它
 */


#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32.h"


#ifdef HRT_TIMER

/* HRT configuration */
#if   HRT_TIMER == 1
# define HRT_TIMER_BASE		STM32_TIM1_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1CC
# define HRT_TIMER_CLOCK	STM32_APB2_TIM1_CLKIN
# if CONFIG_STM32_TIM1
#  error must not set CONFIG_STM32_TIM1=y and HRT_TIMER=1
# endif
#elif HRT_TIMER == 2
# define HRT_TIMER_BASE		STM32_TIM2_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB1ENR_TIM2EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM2
# define HRT_TIMER_CLOCK	STM32_APB1_TIM2_CLKIN
# if CONFIG_STM32_TIM2
#  error must not set CONFIG_STM32_TIM2=y and HRT_TIMER=2
# endif
#elif HRT_TIMER == 3
# define HRT_TIMER_BASE		STM32_TIM3_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB1ENR_TIM3EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM3
# define HRT_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN
# if CONFIG_STM32_TIM3
#  error must not set CONFIG_STM32_TIM3=y and HRT_TIMER=3
# endif
#elif HRT_TIMER == 4
# define HRT_TIMER_BASE		STM32_TIM4_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB1ENR_TIM4EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM4
# define HRT_TIMER_CLOCK	STM32_APB1_TIM4_CLKIN
# if CONFIG_STM32_TIM4
#  error must not set CONFIG_STM32_TIM4=y and HRT_TIMER=4
# endif
#elif HRT_TIMER == 5
# define HRT_TIMER_BASE		STM32_TIM5_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB1ENR_TIM5EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM5
# define HRT_TIMER_CLOCK	STM32_APB1_TIM5_CLKIN
# if CONFIG_STM32_TIM5
#  error must not set CONFIG_STM32_TIM5=y and HRT_TIMER=5
# endif
#elif HRT_TIMER == 8
# define HRT_TIMER_BASE		STM32_TIM8_BASE   //基地址
# define HRT_TIMER_POWER_REG	STM32_RCC_APB2ENR   //时钟使能
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN   //定时器使能
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM8CC        //中断矢量
# define HRT_TIMER_CLOCK	STM32_APB2_TIM8_CLKIN   //时钟
# if CONFIG_STM32_TIM8
#  error must not set CONFIG_STM32_TIM8=y and HRT_TIMER=8
# endif
#elif HRT_TIMER == 9
# define HRT_TIMER_BASE		STM32_TIM9_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM9EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1BRK
# define HRT_TIMER_CLOCK	STM32_APB2_TIM9_CLKIN
# if CONFIG_STM32_TIM9
#  error must not set CONFIG_STM32_TIM9=y and HRT_TIMER=9
# endif
#elif HRT_TIMER == 10
# define HRT_TIMER_BASE		STM32_TIM10_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM10EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1UP
# define HRT_TIMER_CLOCK	STM32_APB2_TIM10_CLKIN
# if CONFIG_STM32_TIM10
#  error must not set CONFIG_STM32_TIM11=y and HRT_TIMER=10
# endif
#elif HRT_TIMER == 11
# define HRT_TIMER_BASE		STM32_TIM11_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM11EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1TRGCOM
# define HRT_TIMER_CLOCK	STM32_APB2_TIM11_CLKIN
# if CONFIG_STM32_TIM11
#  error must not set CONFIG_STM32_TIM11=y and HRT_TIMER=11
# endif
#else
# error HRT_TIMER must be a value between 1 and 11
#endif

/*
 * HRT clock must be a multiple of 1MHz greater than 1MHz
 */
#if (HRT_TIMER_CLOCK % 1000000) != 0
# error HRT_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if HRT_TIMER_CLOCK <= 1000000
# error HRT_TIMER_CLOCK must be greater than 1MHz
#endif

/**
 * Minimum/maximum deadlines.  最大和最小死区时间
 *
 * These are suitable for use with a 16-bit timer/counter clocked
 * at 1MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define HRT_INTERVAL_MIN	50       //最小间隔
#define HRT_INTERVAL_MAX	50000    //最大间隔

/*
 * Period of the free-running counter, in microseconds.
 */
#define HRT_COUNTER_PERIOD	65536  //计数周期

/*
 * Scaling factor(s) for the free-running counter; convert an input  //自由计数的缩放因子，在1ms内转换一个输入到计数
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * Timer register accessors  可访问的定时器寄存器
 */
#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions HRT子功能需要的特殊功能寄存器和位
 */
/* FIXME! There is an interaction in the CCMR registers that prevents using Chan 1 as the timer and chan 2 as the PPM*/  //通道2作为ppm
#if HRT_TIMER_CHANNEL == 1   //定时器通道号
# define rCCR_HRT	rCCR1			/* compare register for HRT */   //比较寄存器
# define DIER_HRT	GTIM_DIER_CC1IE		/* interrupt enable for HRT */  //中断使能
# define SR_INT_HRT	GTIM_SR_CC1IF		/* interrupt status for HRT */  //中断状态
#elif HRT_TIMER_CHANNEL == 2
# define rCCR_HRT	rCCR2			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC2IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC2IF		/* interrupt status for HRT */
#elif HRT_TIMER_CHANNEL == 3
# define rCCR_HRT	rCCR3			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC3IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC3IF		/* interrupt status for HRT */
#elif HRT_TIMER_CHANNEL == 4
# define rCCR_HRT	rCCR4			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC4IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC4IF		/* interrupt status for HRT */
#else
# error HRT_TIMER_CHANNEL must be a value between 1 and 4
#endif

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;  //回调函数队列入口

/* latency baseline (last compare value applied) */
static uint16_t			latency_baseline;  //延迟基准

/* timer count at interrupt (for latency purposes) */
static uint16_t			latency_actual;  //中断中定时计数器，对于延迟目的

/* latency histogram */  //延迟直方图
#define LATENCY_BUCKET_COUNT 8   //直方图数量
__EXPORT const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
__EXPORT const uint16_t	latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t		latency_counters[LATENCY_BUCKET_COUNT + 1];


/* timer-specific functions */  //定时器特殊功能
static void		hrt_tim_init(void);   //定时器初始化
static int		hrt_tim_isr(int irq, void *context); //定时器中断
static void		hrt_latency_update(void);    //延迟更新

/* callout list manipulation */  //回调函数列表处理
static void		hrt_call_internal(struct hrt_call *entry,
		hrt_abstime deadline,
		hrt_abstime interval,
		hrt_callout callout,
		void *arg);
static void		hrt_call_enter(struct hrt_call *entry);  //回调函数入口
static void		hrt_call_reschedule(void);    //回调函数重新计划
static void		hrt_call_invoke(void);     //回调函数抢占

/*
 * Specific registers and bits used by PPM sub-functions  特殊寄存器和位使用，PPM子系统
 */
#ifdef HRT_PPM_CHANNEL
/*
 * If the timer hardware doesn't support GTIM_CCER_CCxNP, then we will work around it.
 *
 * Note that we assume that M3 means STM32F1 (since we don't really care about the F2).
 */
# ifdef CONFIG_ARCH_CORTEXM3
#  undef GTIM_CCER_CC1NP
#  undef GTIM_CCER_CC2NP
#  undef GTIM_CCER_CC3NP
#  undef GTIM_CCER_CC4NP
#  define GTIM_CCER_CC1NP 0
#  define GTIM_CCER_CC2NP 0
#  define GTIM_CCER_CC3NP 0
#  define GTIM_CCER_CC4NP 0
#  define PPM_EDGE_FLIP
# endif
/* FIXME! There is an interaction in the CCMR registers that prevents using Chan 1 as the timer and chan 2 as the PPM*/
# if HRT_PPM_CHANNEL == 1
#  define rCCR_PPM	rCCR1			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC1IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC1IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC1OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	1			/* not on TI1/TI2 */
#  define CCMR2_PPM	0			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP) /* CC1, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC1P
# elif HRT_PPM_CHANNEL == 2
#  define rCCR_PPM	rCCR2			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC2IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC2IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC2OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	2			/* not on TI1/TI2 */
#  define CCMR2_PPM	0			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC2E | GTIM_CCER_CC2P | GTIM_CCER_CC2NP) /* CC2, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC2P
# elif HRT_PPM_CHANNEL == 3
#  define rCCR_PPM	rCCR3			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC3IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC3IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC3OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0			/* not on TI1/TI2 */
#  define CCMR2_PPM	1			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC3E | GTIM_CCER_CC3P | GTIM_CCER_CC3NP) /* CC3, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC3P
# elif HRT_PPM_CHANNEL == 4
#  define rCCR_PPM	rCCR4			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC4IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC4IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC4OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0			/* not on TI1/TI2 */
#  define CCMR2_PPM	2			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC4E | GTIM_CCER_CC4P | GTIM_CCER_CC4NP) /* CC4, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC4P
# else
#  error HRT_PPM_CHANNEL must be a value between 1 and 4
# endif

/*
 * PPM decoder tuning parameters
 */
# define PPM_MIN_PULSE_WIDTH	200		/**< minimum width of a valid first pulse */
# define PPM_MAX_PULSE_WIDTH	600		/**< maximum width of a valid first pulse */
# define PPM_MIN_CHANNEL_VALUE	800		/**< shortest valid channel signal */
# define PPM_MAX_CHANNEL_VALUE	2200		/**< longest valid channel signal */
# define PPM_MIN_START		2300		/**< shortest valid start gap (only 2nd part of pulse) */

/* decoded PPM buffer */
#define PPM_MIN_CHANNELS	5
#define PPM_MAX_CHANNELS	20

/** Number of same-sized frames required to 'lock' */
#define PPM_CHANNEL_LOCK	4		/**< should be less than the input timeout */

__EXPORT uint16_t ppm_buffer[PPM_MAX_CHANNELS];
__EXPORT uint16_t ppm_frame_length = 0;
__EXPORT unsigned ppm_decoded_channels = 0;
__EXPORT uint64_t ppm_last_valid_decode = 0;

#define PPM_DEBUG 0

#if PPM_DEBUG
/* PPM edge history */
__EXPORT uint16_t ppm_edge_history[32];
unsigned ppm_edge_next;

/* PPM pulse history */
__EXPORT uint16_t ppm_pulse_history[32];
unsigned ppm_pulse_next;
#endif

static uint16_t ppm_temp_buffer[PPM_MAX_CHANNELS];

/** PPM decoder state machine */
struct {
	uint16_t	last_edge;	/**< last capture time */
	uint16_t	last_mark;	/**< last significant edge */
	uint16_t	frame_start;	/**< the frame width */
	unsigned	next_channel;	/**< next channel index */
	enum {
		UNSYNCH = 0,
		ARM,
		ACTIVE,
		INACTIVE
	} phase;
} ppm;

static void	hrt_ppm_decode(uint32_t status);

#else
/* disable the PPM configuration */  //禁止ppm配置
# define rCCR_PPM	0
# define DIER_PPM	0
# define SR_INT_PPM	0
# define SR_OVF_PPM	0
# define CCMR1_PPM	0
# define CCMR2_PPM	0
# define CCER_PPM	0
#endif /* HRT_PPM_CHANNEL */

/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 * 我们希望拥有一个精简的stm32通用定时器
 */
static void
hrt_tim_init(void)
{
    /* claim our interrupt vector *///请求中断向量
    irq_attach(HRT_TIMER_VECTOR, hrt_tim_isr);//中断绑定，timer8，第二个实参为回调函数//中断回调

    /* clock/power on our timer *///时钟和电源寄存器，打开定时器开关
	modifyreg32(HRT_TIMER_POWER_REG, 0, HRT_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_HRT | DIER_PPM;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PPM;
	rCCMR2 = CCMR2_PPM;
	rCCER = CCER_PPM;
	rDCR = 0;

	/* configure the timer to free-run at 1MHz */ //配置定时器运行在1MHz
	rPSC = (HRT_TIMER_CLOCK / 1000000) - 1;	/* this really only works for whole-MHz clocks */

	/* run the full span of the counter */
	rARR = 0xffff;

	/* set an initial capture a little ways off */
	rCCR_HRT = 1000;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

    /* enable interrupts *///使能中断
	up_enable_irq(HRT_TIMER_VECTOR);  //使能定时器中断
}

#ifdef HRT_PPM_CHANNEL
/**
 * Handle the PPM decoder state machine.
 */
static void
hrt_ppm_decode(uint32_t status)
{
	uint16_t count = rCCR_PPM;
	uint16_t width;
	uint16_t interval;
	unsigned i;

	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PPM) {
		goto error;
	}

	/* how long since the last edge? - this handles counter wrapping implicitely. */
	width = count - ppm.last_edge;

#if PPM_DEBUG
	ppm_edge_history[ppm_edge_next++] = width;

	if (ppm_edge_next >= 32) {
		ppm_edge_next = 0;
	}

#endif

	/*
	 * if this looks like a start pulse, then push the last set of values
	 * and reset the state machine
	 */
	if (width >= PPM_MIN_START) {

		/*
		 * If the number of channels changes unexpectedly, we don't want
		 * to just immediately jump on the new count as it may be a result
		 * of noise or dropped edges.  Instead, take a few frames to settle.
		 */
		if (ppm.next_channel != ppm_decoded_channels) {
			static unsigned new_channel_count;
			static unsigned new_channel_holdoff;

			if (new_channel_count != ppm.next_channel) {
				/* start the lock counter for the new channel count */
				new_channel_count = ppm.next_channel;
				new_channel_holdoff = PPM_CHANNEL_LOCK;

			} else if (new_channel_holdoff > 0) {
				/* this frame matched the last one, decrement the lock counter */
				new_channel_holdoff--;

			} else {
				/* we have seen PPM_CHANNEL_LOCK frames with the new count, accept it */
				ppm_decoded_channels = new_channel_count;
				new_channel_count = 0;
			}

		} else {
			/* frame channel count matches expected, let's use it */
			if (ppm.next_channel > PPM_MIN_CHANNELS) {
				for (i = 0; i < ppm.next_channel; i++) {
					ppm_buffer[i] = ppm_temp_buffer[i];
				}

				ppm_last_valid_decode = hrt_absolute_time();

			}
		}

		/* reset for the next frame */
		ppm.next_channel = 0;

		/* next edge is the reference for the first channel */
		ppm.phase = ARM;

		ppm.last_edge = count;
		return;
	}

	switch (ppm.phase) {
	case UNSYNCH:
		/* we are waiting for a start pulse - nothing useful to do here */
		break;

	case ARM:

		/* we expect a pulse giving us the first mark */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too short or too long */
		}

		/* record the mark timing, expect an inactive edge */
		ppm.last_mark = ppm.last_edge;

		/* frame length is everything including the start gap */
		ppm_frame_length = (uint16_t)(ppm.last_edge - ppm.frame_start);
		ppm.frame_start = ppm.last_edge;
		ppm.phase = ACTIVE;
		break;

	case INACTIVE:

		/* we expect a short pulse */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too short or too long */
		}

		/* this edge is not interesting, but now we are ready for the next mark */
		ppm.phase = ACTIVE;
		break;

	case ACTIVE:
		/* determine the interval from the last mark */
		interval = count - ppm.last_mark;
		ppm.last_mark = count;

#if PPM_DEBUG
		ppm_pulse_history[ppm_pulse_next++] = interval;

		if (ppm_pulse_next >= 32) {
			ppm_pulse_next = 0;
		}

#endif

		/* if the mark-mark timing is out of bounds, abandon the frame */
		if ((interval < PPM_MIN_CHANNEL_VALUE) || (interval > PPM_MAX_CHANNEL_VALUE)) {
			goto error;
		}

		/* if we have room to store the value, do so */
		if (ppm.next_channel < PPM_MAX_CHANNELS) {
			ppm_temp_buffer[ppm.next_channel++] = interval;
		}

		ppm.phase = INACTIVE;
		break;

	}

	ppm.last_edge = count;
	return;

	/* the state machine is corrupted; reset it */

error:
	/* we don't like the state of the decoder, reset it and try again */
	ppm.phase = UNSYNCH;
	ppm_decoded_channels = 0;

}
#endif /* HRT_PPM_CHANNEL */

/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context)  //定时器中断回调函数
{
	uint32_t status;

	/* grab the timer for latency tracking purposes */
	latency_actual = rCNT;

	/* copy interrupt status */
	status = rSR;

	/* ack the interrupts we just read */
	rSR = ~status;

#ifdef HRT_PPM_CHANNEL

	/* was this a PPM edge? */
	if (status & (SR_INT_PPM | SR_OVF_PPM)) {
		/* if required, flip edge sensitivity */
# ifdef PPM_EDGE_FLIP
		rCCER ^= CCER_PPM_FLIP;
# endif

		hrt_ppm_decode(status);
	}

#endif

	/* was this a timer tick? */
	if (status & SR_INT_HRT) {

        /* do latency calculations */  //延时时间计算
		hrt_latency_update();

		/* run any callouts that have met their deadline */
		hrt_call_invoke();

		/* and schedule the next interrupt */
		hrt_call_reschedule();
	}

	return OK;
}

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime
hrt_absolute_time(void)  //获取绝对时间
{
	hrt_abstime	abstime;
	uint32_t	count;
	irqstate_t	flags;

	/*
	 * Counter state.  Marked volatile as they may change
	 * inside this routine but outside the irqsave/restore
	 * pair.  Discourage the compiler from moving loads/stores
	 * to these outside of the protected range.
	 */
	static volatile hrt_abstime base_time;
	static volatile uint32_t last_count;

	/* prevent re-entry */
	flags = up_irq_save();

	/* get the current counter value */
	count = rCNT;

	/*
	 * Determine whether the counter has wrapped since the
	 * last time we're called.
	 *
	 * This simple test is sufficient due to the guarantee that
	 * we are always called at least once per counter period.
	 */
	if (count < last_count) {
		base_time += HRT_COUNTER_PERIOD;
	}

	/* save the count for next time */
	last_count = count;

	/* compute the current time */
	abstime = HRT_COUNTER_SCALE(base_time + count);

	up_irq_restore(flags);

	return abstime;
}

/**
 * Convert a timespec to absolute time  转换到绝对时间  us
 */
hrt_abstime
ts_to_abstime(struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;   //s转换到us
	result += ts->tv_nsec / 1000;  //ns转换到us

	return result;
}

/**
 * Convert absolute time to a timespec.  //绝对时间到标准时间的转换
 */
void
abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / 1000000;
	abstime -= ts->tv_sec * 1000000;
	ts->tv_nsec = abstime * 1000;
}

/**
 * Compare a time value with the current time.  与当前时间进行比较
 */
hrt_abstime
hrt_elapsed_time(const volatile hrt_abstime *then)
{
	irqstate_t flags = up_irq_save();  //保存中断状态

	hrt_abstime delta = hrt_absolute_time() - *then;  //当前时间减去指定时间

	up_irq_restore(flags);  //恢复中断状态

	return delta;
}

/**
 * Store the absolute time in an interrupt-safe fashion  在中断保护状态下存储一个绝对时间
 */
hrt_abstime
hrt_store_absolute_time(volatile hrt_abstime *now)
{
	irqstate_t flags = up_irq_save();

	hrt_abstime ts = hrt_absolute_time();

	up_irq_restore(flags);

	return ts;
}

/**
 * Initialise the high-resolution timing module.
 * 初始化高分辨率的时间模块
 */
void
hrt_init(void)
{
    sq_init(&callout_queue);//队列初始化
	hrt_tim_init();   //定时计数器1初始化，并注册中断

#ifdef HRT_PPM_CHANNEL
	/* configure the PPM input pin */
	stm32_configgpio(GPIO_PPM_IN);
#endif
}

/**
 * Call callout(arg) after interval has elapsed.  在一次调用后 时间间隔过去了
 */
void
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,  //重新计算延迟时间
			  0,
			  callout,
			  arg);
}

/**
 * Call callout(arg) at calltime.  在调用时间到达是调用回调函数
 */
void
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.  周期性调用回调函数
 */
void
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

//最后都调用的函数
static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = up_irq_save();

	/* if the entry is currently queued, remove it */  //如果入口已经在队列中，移除它
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
	if (entry->deadline != 0) { //如果死区时间不等于0
		sq_rem(&entry->link, &callout_queue);  //删除队列
	}

	entry->deadline = deadline;  //死区时间
	entry->period = interval;    //间隔
	entry->callout = callout;   //回调
	entry->arg = arg;

	hrt_call_enter(entry);   //进入回调

	up_irq_restore(flags);    //恢复中断标志
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);  //如果返回true，回调函数被挑勇，并中回调队列中删除
}

/**
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)  //中回调函列表中删除入口
{
	irqstate_t flags = up_irq_save();

	sq_rem(&entry->link, &callout_queue);  //队列中删除
	entry->deadline = 0;  //死区时间清零

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;  //周期清零

	up_irq_restore(flags);
}

static void
hrt_call_enter(struct hrt_call *entry)  //回调入口
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		//lldbg("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				//lldbg("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	//lldbg("scheduled\n");
}

static void
hrt_call_invoke(void)   //回调调用
{
	struct hrt_call	*call;
	hrt_abstime deadline;  //死区

	while (true) {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();  //获得当前绝对时间

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		//lldbg("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			//lldbg("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}
}

/**
 * Reschedule the next timer interrupt. 重新安排下一个时间中断
 *
 * This routine must be called with interrupts disabled.
 */
static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();  //获得当前绝对时间
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 16 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != NULL) {
		//lldbg("entry in queue\n");
		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			//lldbg("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			//lldbg("due soon\n");
			deadline = next->deadline;
		}
	}

	//lldbg("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */
	rCCR_HRT = latency_baseline = deadline & 0xffff;
}

static void
hrt_latency_update(void)
{
	uint16_t latency = latency_actual - latency_baseline;
	unsigned	index;

	/* bounded buckets */
	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	/* catch-all at the end */
	latency_counters[index]++;
}

void
hrt_call_init(struct hrt_call *entry)  //调用初始化
{
	memset(entry, 0, sizeof(*entry));
}

void
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)   //调用延迟
{
	entry->deadline = hrt_absolute_time() + delay;
}

#endif /* HRT_TIMER */











