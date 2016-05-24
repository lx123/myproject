/**
 * @file serial.c
 *
 * Serial communication for the PX4IO module.
 */

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

/* XXX might be able to prune these */
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>
#include <stm32.h>
#include <systemlib/perf_counter.h>

//#define DEBUG
#include "io.h"

static perf_counter_t	pc_txns;
static perf_counter_t	pc_errors;
static perf_counter_t	pc_ore;
static perf_counter_t	pc_fe;
static perf_counter_t	pc_ne;
static perf_counter_t	pc_idle;
static perf_counter_t	pc_badidle;
static perf_counter_t	pc_regerr;
static perf_counter_t	pc_crcerr;

static void		rx_handle_packet(void);
static void		rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static DMA_HANDLE	tx_dma;
static DMA_HANDLE	rx_dma;

static int		serial_interrupt(int irq, void *context);
static void		dma_reset(void);

static struct IOPacket	dma_packet;

/* serial register accessors */
#define REG(_x)		(*(volatile uint32_t *)(PX4FMU_SERIAL_BASE + _x))
#define rSR		REG(STM32_USART_SR_OFFSET)
#define rDR		REG(STM32_USART_DR_OFFSET)
#define rBRR		REG(STM32_USART_BRR_OFFSET)
#define rCR1		REG(STM32_USART_CR1_OFFSET)
#define rCR2		REG(STM32_USART_CR2_OFFSET)
#define rCR3		REG(STM32_USART_CR3_OFFSET)
#define rGTPR		REG(STM32_USART_GTPR_OFFSET)

void
interface_init(void)  //与fmu通讯接口
{
	pc_txns = perf_alloc(PC_ELAPSED, "txns"); //发送 消耗时间
	pc_errors = perf_alloc(PC_COUNT, "errors");  //错误计数
	pc_ore = perf_alloc(PC_COUNT, "overrun");
	pc_fe = perf_alloc(PC_COUNT, "framing");
	pc_ne = perf_alloc(PC_COUNT, "noise");
	pc_idle = perf_alloc(PC_COUNT, "idle");
	pc_badidle = perf_alloc(PC_COUNT, "badidle");
	pc_regerr = perf_alloc(PC_COUNT, "regerr");
	pc_crcerr = perf_alloc(PC_COUNT, "crcerr");

	/* allocate DMA */
	tx_dma = stm32_dmachannel(PX4FMU_SERIAL_TX_DMA);  //发送和接收dma通道，串口2
	rx_dma = stm32_dmachannel(PX4FMU_SERIAL_RX_DMA);

	/* configure pins for serial use */
	stm32_configgpio(PX4FMU_SERIAL_TX_GPIO);
	stm32_configgpio(PX4FMU_SERIAL_RX_GPIO);

	/* reset and configure the UART */
	rCR1 = 0;
	rCR2 = 0;
	rCR3 = 0;

	/* clear status/errors */
	(void)rSR;
	(void)rDR;

	/* configure line speed */  //配置波特率
	uint32_t usartdiv32 = PX4FMU_SERIAL_CLOCK / (PX4FMU_SERIAL_BITRATE / 2);
	uint32_t mantissa = usartdiv32 >> 5;
	uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
	rBRR = (mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT);

	/* connect our interrupt */
	irq_attach(PX4FMU_SERIAL_VECTOR, serial_interrupt);  //中断绑定
	up_enable_irq(PX4FMU_SERIAL_VECTOR);  //使能串口中断

	/* enable UART and error/idle interrupts */ //中断处理错误和空闲
	rCR3 = USART_CR3_EIE;  //错误中断使能
	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;  //接收使能，发送使能，uart使能IDLE中断

#if 0	/* keep this for signal integrity testing */

	for (;;) {
		while (!(rSR & USART_SR_TXE))
			;

		rDR = 0xfa;

		while (!(rSR & USART_SR_TXE))
			;

		rDR = 0xa0;
	}

#endif

	/* configure RX DMA and return to listening state */  //配置接收dma，并返回监听状态
	dma_reset();  //数据获取，就是靠dma

	debug("serial init");
}

static void
rx_handle_packet(void)   //处理接收到的包
{
	/* check packet CRC */
	uint8_t crc = dma_packet.crc;  //取出包的crc校验值
	dma_packet.crc = 0;  //将包的crc清零，以计算包的crc

	if (crc != crc_packet(&dma_packet)) {  //crc校验错误
		perf_count(pc_crcerr);

		/* send a CRC error reply */  //发送crc校验错误应答
		dma_packet.count_code = PKT_CODE_CORRUPT;
		dma_packet.page = 0xff;
		dma_packet.offset = 0xff;

		return;
	}

	if (PKT_CODE(dma_packet) == PKT_CODE_WRITE) {  //代码写入

		/* it's a blind write - pass it on */  //是一个瞎写，让它通过
		if (registers_set(dma_packet.page, dma_packet.offset, &dma_packet.regs[0], PKT_COUNT(dma_packet))) {
			perf_count(pc_regerr);  //寄存器错误计数
			dma_packet.count_code = PKT_CODE_ERROR; //寄存器操作错误

		} else {
			dma_packet.count_code = PKT_CODE_SUCCESS;  //寄存器操作成功
		}

		return;
	}

	if (PKT_CODE(dma_packet) == PKT_CODE_READ) { //操作代码为读取

		/* it's a read - get register pointer for reply */  //读取操作，获取寄存器指针进行应答
		unsigned count;
		uint16_t *registers;

		if (registers_get(dma_packet.page, dma_packet.offset, &registers, &count) < 0) {  //返回-1表示出错
			perf_count(pc_regerr); //寄存器错误计数
			dma_packet.count_code = PKT_CODE_ERROR;  //寄存器错误操作应答

		} else {  //寄存器读取成功
			/* constrain reply to requested size */  //限制应答到请求的大小
			if (count > PKT_MAX_REGS) {  //如果数量大于最大寄存器数
				count = PKT_MAX_REGS;
			}

			if (count > PKT_COUNT(dma_packet)) {  //代码指令数量部分解析
				count = PKT_COUNT(dma_packet);
			}

			/* copy reply registers into DMA buffer */  //复制应答寄存器到dma缓冲器
			memcpy((void *)&dma_packet.regs[0], registers, count * 2);
			dma_packet.count_code = count | PKT_CODE_SUCCESS;
		}

		return;
	}

	/* send a bad-packet error reply */
	dma_packet.count_code = PKT_CODE_CORRUPT;
	dma_packet.page = 0xff;
	dma_packet.offset = 0xfe;
}

static void
rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)  //接收dma回调函数
{
	/*
	 * We are here because DMA completed, or UART reception stopped and  到了这里是因为dma完成或者uart接收停止
	 * we think we have a packet in the buffer. 我们认为有个包在缓冲区
	 */
	perf_begin(pc_txns);

	/* disable UART DMA */  //失能uart dma
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);  //发送和接收

	/* handle the received packet */
	rx_handle_packet();  //处理接收到的包

	/* re-set DMA for reception first, so we are ready to receive before we start sending */
	dma_reset();  //每次都复位dma

	/* send the reply to the just-processed request */  //发送应答，仅仅处理应答
	dma_packet.crc = 0;
	dma_packet.crc = crc_packet(&dma_packet);  //对包进行crc校验
	stm32_dmasetup(  //dma发送设置
		tx_dma,
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,
		PKT_SIZE(dma_packet),
		DMA_CCR_DIR		|
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS);
	stm32_dmastart(tx_dma, NULL, NULL, false);  //发送dma
	rCR3 |= USART_CR3_DMAT;

	perf_end(pc_txns);
}

static int
serial_interrupt(int irq, void *context)
{
	static bool abort_on_idle = false;

	uint32_t sr = rSR;	/* get UART status register */
	(void)rDR;		/* required to clear any of the interrupt status that brought us here */

	if (sr & (USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
		  USART_SR_NE |		/* noise error - we have lost a byte due to noise */
		  USART_SR_FE)) {		/* framing error - start/stop bit lost or line break */

		perf_count(pc_errors);

		if (sr & USART_SR_ORE) {
			perf_count(pc_ore);
		}

		if (sr & USART_SR_NE) {
			perf_count(pc_ne);
		}

		if (sr & USART_SR_FE) {
			perf_count(pc_fe);
		}

		/* send a line break - this will abort transmission/reception on the other end */
		rCR1 |= USART_CR1_SBK;

		/* when the line goes idle, abort rather than look at the packet */
		abort_on_idle = true;
	}

	if (sr & USART_SR_IDLE) {

		/*
		 * If we saw an error, don't bother looking at the packet - it should have
		 * been aborted by the sender and will definitely be bad. Get the DMA reconfigured
		 * ready for their retry.
		 */
		if (abort_on_idle) {

			abort_on_idle = false;
			dma_reset();
			return 0;
		}

		/*
		 * The sender has stopped sending - this is probably the end of a packet.
		 * Check the received length against the length in the header to see if
		 * we have something that looks like a packet.
		 */
		unsigned length = sizeof(dma_packet) - stm32_dmaresidual(rx_dma);

		if ((length < 1) || (length < PKT_SIZE(dma_packet))) {

			/* it was too short - possibly truncated */
			perf_count(pc_badidle);
			dma_reset();
			return 0;
		}

		/*
		 * Looks like we received a packet. Stop the DMA and go process the
		 * packet.
		 */
		perf_count(pc_idle);
		stm32_dmastop(rx_dma);
		rx_dma_callback(rx_dma, DMA_STATUS_TCIF, NULL);
	}

	return 0;
}

static void
dma_reset(void)
{
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
	(void)rSR;
	(void)rDR;
	(void)rDR;

	/* kill any pending DMA */
	stm32_dmastop(tx_dma);  //停止所有的挂起dma
	stm32_dmastop(rx_dma);

	/* reset the RX side */  //复位接收端
	stm32_dmasetup(  //dma设置
		rx_dma,  //句柄
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,  //dma数据包
		sizeof(dma_packet),
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS     |
		DMA_CCR_PRIVERYHI);

	/* start receive DMA ready for the next packet */
	stm32_dmastart(rx_dma, rx_dma_callback, NULL, false);  //开始dma
	rCR3 |= USART_CR3_DMAR;
}

