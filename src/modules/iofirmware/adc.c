
/**
 * @file adc.c
 *
 * Simple ADC support for PX4IO on STM32.
 */
#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <arch/stm32/chip.h>
#include <stm32.h>

#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

#define DEBUG
#include "io.h"

/*
 * Register accessors.  可访问寄存器
 * For now, no reason not to just use ADC1.
 */
#define REG(_reg)	(*(volatile uint32_t *)(STM32_ADC1_BASE + _reg))  //adc1寄存器访问

#define rSR		REG(STM32_ADC_SR_OFFSET)
#define rCR1		REG(STM32_ADC_CR1_OFFSET)
#define rCR2		REG(STM32_ADC_CR2_OFFSET)
#define rSMPR1		REG(STM32_ADC_SMPR1_OFFSET)
#define rSMPR2		REG(STM32_ADC_SMPR2_OFFSET)
#define rJOFR1		REG(STM32_ADC_JOFR1_OFFSET)
#define rJOFR2		REG(STM32_ADC_JOFR2_OFFSET)
#define rJOFR3		REG(STM32_ADC_JOFR3_OFFSET)
#define rJOFR4		REG(STM32_ADC_JOFR4_OFFSET)
#define rHTR		REG(STM32_ADC_HTR_OFFSET)
#define rLTR		REG(STM32_ADC_LTR_OFFSET)
#define rSQR1		REG(STM32_ADC_SQR1_OFFSET)
#define rSQR2		REG(STM32_ADC_SQR2_OFFSET)
#define rSQR3		REG(STM32_ADC_SQR3_OFFSET)
#define rJSQR		REG(STM32_ADC_JSQR_OFFSET)
#define rJDR1		REG(STM32_ADC_JDR1_OFFSET)
#define rJDR2		REG(STM32_ADC_JDR2_OFFSET)
#define rJDR3		REG(STM32_ADC_JDR3_OFFSET)
#define rJDR4		REG(STM32_ADC_JDR4_OFFSET)
#define rDR		REG(STM32_ADC_DR_OFFSET)

perf_counter_t		adc_perf;  //性能计数

int
adc_init(void)
{
	adc_perf = perf_alloc(PC_ELAPSED, "adc");  //计算adc花费时间

	/* put the ADC into power-down mode */
	rCR2 &= ~ADC_CR2_ADON;  //关闭adc转换
	up_udelay(10);

	/* bring the ADC out of power-down mode */
	rCR2 |= ADC_CR2_ADON;  //打开转换
	up_udelay(10);

	/* do calibration if supported */  //如果需要进行校准
#ifdef ADC_CR2_CAL
	rCR2 |= ADC_CR2_RSTCAL;
	up_udelay(1);

	if (rCR2 & ADC_CR2_RSTCAL) {
		return -1;
	}

	rCR2 |= ADC_CR2_CAL;
	up_udelay(100);

	if (rCR2 & ADC_CR2_CAL) {
		return -1;
	}

#endif

	/*
	 * Configure sampling time.  设置采样时间
	 *
	 * For electrical protection reasons, we want to be able to have
	 * 10K in series with ADC inputs that leave the board. At 12MHz this
	 * means we need 28.5 cycles of sampling time (per table 43 in the
	 * datasheet).  由于电气保护原因，需要10k串联电阻到输入口，
	 */
	rSMPR1 = 0b00000000011011011011011011011011;
	rSMPR2 = 0b00011011011011011011011011011011;

	rCR2 |=	ADC_CR2_TSVREFE;		/* enable the temperature sensor / Vrefint channel */  //使能温度传感器

	/* configure for a single-channel sequence */  //配置单通道队列
	rSQR1 = 0;
	rSQR2 = 0;
	rSQR3 = 0;	/* will be updated with the channel at conversion time */

	return 0;
}

/*
  return one measurement, or 0xffff on error  返回一个测量值
 */
uint16_t
adc_measure(unsigned channel)
{

	perf_begin(adc_perf);  //测量一次转换的时间花费

	/* clear any previous EOC */
	rSR = 0;
	(void)rDR;

	/* run a single conversion right now - should take about 60 cycles (a few microseconds) max */
	rSQR3 = channel;
	rCR2 |= ADC_CR2_ADON;

	/* wait for the conversion to complete */
	hrt_abstime now = hrt_absolute_time();

	while (!(rSR & ADC_SR_EOC)) {

		/* never spin forever - this will give a bogus result though */
		if (hrt_elapsed_time(&now) > 100) {
			perf_end(adc_perf);
			return 0xffff;
		}
	}

	/* read the result and clear EOC */
	uint16_t result = rDR;  //读取adc值
	rSR = 0;

	perf_end(adc_perf);
	return result;
}
