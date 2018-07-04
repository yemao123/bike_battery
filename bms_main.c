#include <hal.h>
#include <cpuusg.h>
#include <string.h>
#include <utils.h>
#include <stdarg.h>
#include <stdlib.h>

typedef struct _IO_INIT
{
	GPIO_TypeDef * port;
	uint16_t pin;
	uint16_t af;
	uint16_t mode;
	uint16_t flags;
}IO_INIT;

#define DEF_IO_INIT(name, mode, af, flags)	{name##_PORT, name##_PIN, af, mode, flags}

#define IO_TYPE_AF				0
#define IO_TYPE_IN_FLOATING	1
#define IO_TYPE_IN_PULL_HI		2
#define IO_TYPE_IN_PULL_LO		3
#define IO_TYPE_OUT_PP			4
#define IO_TYPE_OUT_OD			5
#define IO_TYPE_AN				6

#define IO_FLAG_IN_INT_RF		1
#define IO_FLAG_IN_INT_R		2
#define IO_FLAG_IN_INT_F		3

#define IO_FLAG_OUT_LO			0
#define IO_FLAG_OUT_HI			1

static IO_INIT io_init[] = {
	DEF_IO_INIT(UPS_UART_TXD, IO_TYPE_AF, GPIO_AF_UART4, 0),
	DEF_IO_INIT(UPS_UART_RXD, IO_TYPE_AF, GPIO_AF_UART4, 0),

	DEF_IO_INIT(TEST_UART_TXD, IO_TYPE_AF, GPIO_AF_USART2, 0),
	DEF_IO_INIT(TEST_UART_RXD, IO_TYPE_AF, GPIO_AF_USART2, 0),

	DEF_IO_INIT(BATT1_UART_TXD, IO_TYPE_AF, GPIO_AF_USART1, 0),
	DEF_IO_INIT(BATT1_UART_RXD, IO_TYPE_AF, GPIO_AF_USART1, 0),

	DEF_IO_INIT(BATT2_UART_TXD, IO_TYPE_AF, GPIO_AF_UART5, 0),
	DEF_IO_INIT(BATT2_UART_RXD, IO_TYPE_AF, GPIO_AF_UART5, 0),

	DEF_IO_INIT(BATT3_UART_TXD, IO_TYPE_AF, GPIO_AF_USART6, 0),
	DEF_IO_INIT(BATT3_UART_RXD, IO_TYPE_AF, GPIO_AF_USART6, 0),

	DEF_IO_INIT(BATT4_UART_TXD, IO_TYPE_AF, GPIO_AF_USART3, 0),
	DEF_IO_INIT(BATT4_UART_RXD, IO_TYPE_AF, GPIO_AF_USART3, 0),


	DEF_IO_INIT(LED_SPI_CLK, IO_TYPE_AF, GPIO_AF_SPI1, 0),
	DEF_IO_INIT(LED_SPI_MOSI, IO_TYPE_AF, GPIO_AF_SPI1, 0),

	DEF_IO_INIT(BATT1_INT, IO_TYPE_IN_PULL_HI, 0, 0),
	DEF_IO_INIT(BATT2_INT, IO_TYPE_IN_PULL_HI, 0, 0),
	DEF_IO_INIT(BATT3_INT, IO_TYPE_IN_PULL_HI, 0, 0),
	DEF_IO_INIT(BATT4_INT, IO_TYPE_IN_PULL_HI, 0, 0),

	DEF_IO_INIT(BATT1_RST, IO_TYPE_OUT_PP, 0, 1),
	DEF_IO_INIT(BATT2_RST, IO_TYPE_OUT_PP, 0, 1),
	DEF_IO_INIT(BATT3_RST, IO_TYPE_OUT_PP, 0, 1),
	DEF_IO_INIT(BATT4_RST, IO_TYPE_OUT_PP, 0, 1),

	DEF_IO_INIT(BATT1_OUTE, IO_TYPE_OUT_PP, 0, 0),
	DEF_IO_INIT(BATT2_OUTE, IO_TYPE_OUT_PP, 0, 0),
	DEF_IO_INIT(BATT3_OUTE, IO_TYPE_OUT_PP, 0, 0),
	DEF_IO_INIT(BATT4_OUTE, IO_TYPE_OUT_PP, 0, 0),

	
	DEF_IO_INIT(DETECT_12V, IO_TYPE_IN_FLOATING, 0, 0),
	DEF_IO_INIT(DETECT_SPEED, IO_TYPE_IN_FLOATING, 0, 0),
	

	DEF_IO_INIT(LOCK_QH, IO_TYPE_IN_FLOATING, 0, 0),
	DEF_IO_INIT(LOCK_QG, IO_TYPE_IN_FLOATING, 0, 0),
	DEF_IO_INIT(LOCK_OUTA, IO_TYPE_OUT_PP, 0, 0),
	DEF_IO_INIT(LOCK_CLK, IO_TYPE_OUT_PP, 0, 1),
	DEF_IO_INIT(LOCK_CLR, IO_TYPE_OUT_PP, 0, 1),
};

static void EXTI_int_init(const IO_INIT *io, int source)
{
	EXTI_InitTypeDef exti_init;
	EXTI_StructInit(&exti_init);
	exti_init.EXTI_Line = source;
	exti_init.EXTI_LineCmd = ENABLE;
	switch(io->flags) {
		case IO_FLAG_IN_INT_RF:
			exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			break;
		case IO_FLAG_IN_INT_R:
			exti_init.EXTI_Trigger = EXTI_Trigger_Rising;
			break;
		case IO_FLAG_IN_INT_F:
			exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
			break;
	}
	EXTI_Init(&exti_init);
}

static void _GPIO_init(const IO_INIT *io, int n_io)
{
	GPIO_InitTypeDef gpio_init;
	
	int i, source;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	for(i=0; i<n_io; i++) {
		GPIO_StructInit(&gpio_init);
		gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_init.GPIO_Pin = io[i].pin;

		for(source=0; source < 16; source ++) {
			if(io[i].pin & (1<<source))
				break;
		}
		
		switch(io[i].mode) {
		case IO_TYPE_AF:
			gpio_init.GPIO_Mode = GPIO_Mode_AF;	
			GPIO_PinAFConfig(io[i].port, source, io[i].af);
			if(io[i].flags)
				gpio_init.GPIO_PuPd = (GPIOPuPd_TypeDef)io[i].flags;
			break;
		case IO_TYPE_IN_FLOATING:
			gpio_init.GPIO_Mode = GPIO_Mode_IN;
			gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
			if(io[i].flags) {
				EXTI_int_init(&io[i], source);
			}
			break;
		case IO_TYPE_IN_PULL_HI:
			gpio_init.GPIO_Mode = GPIO_Mode_IN;
			gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
			if(io[i].flags) {
				EXTI_int_init(&io[i], source);
			}
			break;
		case IO_TYPE_IN_PULL_LO:
			gpio_init.GPIO_Mode = GPIO_Mode_IN;
			gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
			if(io[i].flags) {
				EXTI_int_init(&io[i], source);
			}
			break;
		case IO_TYPE_OUT_PP:
			gpio_init.GPIO_Mode = GPIO_Mode_OUT;
			gpio_init.GPIO_OType= GPIO_OType_PP;
			if(io[i].flags)
				GPIO_SetBits(io[i].port, io[i].pin);
			else
				GPIO_ResetBits(io[i].port, io[i].pin);
			break;
		case IO_TYPE_OUT_OD:
			gpio_init.GPIO_Mode = GPIO_Mode_OUT;
			gpio_init.GPIO_OType= GPIO_OType_OD;
			if(io[i].flags)
				GPIO_SetBits(io[i].port, io[i].pin);
			else
				GPIO_ResetBits(io[i].port, io[i].pin);
			break;
		case IO_TYPE_AN:
			gpio_init.GPIO_Mode = GPIO_Mode_AN;
			break;
		}
		GPIO_Init(io[i].port, &gpio_init);
	}
}


static void GPIO_init()
{
	_GPIO_init(io_init, sizeof(io_init)/sizeof(io_init[0]));
}

static void clk_init(void)
{
	// clk is initialized in bootloader
#if 0
	RCC_DeInit();

	RCC_PLLCmd(DISABLE);
	RCC_HSEConfig(RCC_HSE_ON);

	while( RCC_WaitForHSEStartUp() != SUCCESS );

	/*
	** PLLM: PLL_VCO input clock = (HSE_VALUE or HSI_VALUE / PLL_M),
	** PLLN: PLL_VCO output clock = (PLL_VCO input clock) * PLL_N,
	** PLLP: System Clock = (PLL_VCO output clock)/PLL_P ,
	** 8MHz OSC,  1MHz VCO input, 366MHz VCO output, 366/2=MHz Sysclk, 366/7=48MHz USB clock
	*/
	RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
	//RCC_PLLConfig(RCC_PLLSource_HSE, 12, 336, 2, 7);		// Sam Su

	//RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);	//??? cannot set to PLLCLK ???
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 	// Sam Su

	// config AHB
	RCC_HCLKConfig(RCC_HCLK_Div1); 

	//config APB1 div4  168/4=42MHz
	RCC_PCLK1Config(RCC_HCLK_Div4);

	//config APB2 div2 168/2=84MHz
	RCC_PCLK2Config(RCC_HCLK_Div2);

	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);    //等待PLL时钟准备好

	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 	// Sam Su

	while(RCC_GetSYSCLKSource() != 0x08);        //0x08：PLL作为系统时钟
#endif
}

/*
** USART *************************
*/
void (*usart1_on_data)(unsigned char) = NULL;
void (*usart1_on_sent)(void) = NULL;
void (*usart1_on_error)(void) = NULL;

void Usart_Tc_Int_Enable(void)
{
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

void Usart_Tc_Int_Disable(void)
{
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
}

extern int vsnprintf(char *buf, size_t size, const char *fmt, va_list args);
extern void usart_puts(const char *msg);

int hal_trace(const signed char* fmt, ...)
{
	va_list args;
	int len;
	signed char buff[128];

	va_start(args, fmt);
	len = vsnprintf((char*)buff, sizeof(buff)-1, (char const*)fmt, args);
	va_end(args);

	buff[len] = 0;
	usart_puts((const char*)buff);
	return len;
}

void hal_puts(char *s)
{
	while(*s) {
		USART_SendData(USART2, *s++);
		while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
	}
}

#define RX_BUF_SIZE		128
static unsigned char __rx_buffer[RX_BUF_SIZE];
static unsigned short __rx_buffer_tail, __rx_buffer_head;

unsigned char __rx_buffer_get(void)
{
	unsigned char	data;
	hal_flags		flags;
	save_and_cli(flags);
	data = __rx_buffer[__rx_buffer_tail++];
	if(__rx_buffer_tail == RX_BUF_SIZE)
		__rx_buffer_tail = 0;
	restore_flags(flags);
	return data;
}

void __rx_buffer_put(unsigned char data)
{
	hal_flags	flags;
	save_and_cli(flags);
	__rx_buffer[__rx_buffer_head++] = data;
	if(__rx_buffer_head == RX_BUF_SIZE)
		__rx_buffer_head = 0;
	if(__rx_buffer_head == __rx_buffer_tail) {
		__rx_buffer_tail ++;
		if(__rx_buffer_tail == RX_BUF_SIZE)
			__rx_buffer_tail = 0;
	}
	restore_flags(flags);
}

void __rx_buffer_clear(void)
{
	__rx_buffer_head = __rx_buffer_tail = 0;
}

int __rx_buffer_empty(void)
{
	return (__rx_buffer_tail == __rx_buffer_head);
}

static void __uart_recv_data(unsigned char ch)
{
	__rx_buffer_put(ch);
}


#define TX_BUF_SIZE		1024
static unsigned char __tx_buffer[TX_BUF_SIZE];
static unsigned short __tx_buffer_head, __tx_buffer_tail;

static unsigned char __tx_buffer_get(void)
{
	unsigned char data;
	hal_flags flags;
	
	save_and_cli(flags);
	data = __tx_buffer[__tx_buffer_tail++];
	__tx_buffer_tail %= TX_BUF_SIZE;
	restore_flags(flags);
	return data;
}

static void __tx_buffer_put(unsigned char data)
{
	hal_flags flags;

	save_and_cli(flags);
	__tx_buffer[__tx_buffer_head++] = data;
	__tx_buffer_head %= TX_BUF_SIZE;
	restore_flags(flags);
}

static void __wait_tx_buffer_space()
{
	unsigned short head = __tx_buffer_head;
	unsigned long timeout = 1000;
	
	while(((head+1)%TX_BUF_SIZE) == __tx_buffer_tail) {
		if(--timeout == 0)
			break;
		__delay_x(2);
	}
}

static unsigned short __tx_buffer_get_space(void)
{
	
	if(__tx_buffer_tail > __tx_buffer_head)
		return __tx_buffer_tail - __tx_buffer_head - 1;
	else
		return TX_BUF_SIZE - (__tx_buffer_head - __tx_buffer_tail) - 1;
}

static unsigned short __tx_buffer_get_used(void)
{
	return TX_BUF_SIZE - __tx_buffer_get_space() + 1;
}

static void __tx_buffer_clear(void)
{
	__tx_buffer_tail = __tx_buffer_head = 0;
}

static int __tx_buffer_empty(void)
{
	return __tx_buffer_tail == __tx_buffer_head;
}

static void __uart_send_complete(void)
{
	if(!__tx_buffer_empty()) {
		char ch = __tx_buffer_get();
		USART_SendData(USART2, ch);
	}
	else {
		Usart_Tc_Int_Disable();
	}
}

static void uart_comm_init(void)
{
	__rx_buffer_clear();
	usart1_on_data = __uart_recv_data;
	usart1_on_sent = __uart_send_complete;
}

void USART2_IRQHandler(void)
{
	volatile unsigned short SR = USART2->SR;
	if(SR & (USART_FLAG_RXNE | USART_FLAG_PE | USART_FLAG_FE | USART_FLAG_ORE | USART_FLAG_NE)) {
		unsigned short ch = USART_ReceiveData(USART2);
		if(SR & (USART_FLAG_PE | USART_FLAG_FE | USART_FLAG_ORE | USART_FLAG_NE)) {
			if(usart1_on_error)
				usart1_on_error();
			goto nodata;
		}
		if((SR & USART_FLAG_RXNE) && usart1_on_data) {
			usart1_on_data((unsigned char)ch);
		}
nodata:
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
	if(SR & USART_FLAG_TXE) {
		if(usart1_on_sent)
			usart1_on_sent();
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
	}
}

static void usart_init(void)
{
     /* Use usart1 */
	USART_InitTypeDef usart_init;
	GPIO_InitTypeDef gpio_init;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	// RS232-TXD-T
	GPIO_SetBits(TEST_UART_TXD_PORT, TEST_UART_TXD_PIN);
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_Mode = GPIO_Mode_OUT;
	gpio_init.GPIO_Pin =  TEST_UART_TXD_PIN;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(TEST_UART_TXD_PORT, &gpio_init);
	_HI(TEST_UART_TXD);
	
	USART_Cmd(USART2, DISABLE);
	USART_DeInit(USART2);
	__delay_ms(200);

	GPIO_StructInit(&gpio_init);
	/* RS232 TX */
	gpio_init.GPIO_Pin =  TEST_UART_TXD_PIN;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TEST_UART_TXD_PORT, &gpio_init);

	/* RS232 RX */
	gpio_init.GPIO_Pin = TEST_UART_RXD_PIN ;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(TEST_UART_RXD_PORT, &gpio_init);

	GPIO_PinAFConfig(TEST_UART_TXD_PORT, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(TEST_UART_RXD_PORT, GPIO_PinSource3, GPIO_AF_USART2);

	USART_StructInit(&usart_init);
	usart_init.USART_BaudRate = 115200;
	usart_init.USART_Mode = USART_Mode_Tx |USART_Mode_Rx;	
		
	USART_Init(USART2, &usart_init);
	USART_ClearITPendingBit(USART2, USART_IT_TXE);
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_PE, ENABLE);
	USART_ITConfig(USART2, USART_IT_FE, ENABLE);
	USART_ITConfig(USART2, USART_IT_ORE, ENABLE);
	USART_ITConfig(USART2, USART_IT_NE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}


const static struct {
    u8 channel;
    u8 priority;    
}irqs[] = {
	{USART2_IRQn, 2},
	{USART1_IRQn, 2},
	{UART5_IRQn, 2},
	{USART6_IRQn, 2},
	{USART3_IRQn, 2},
	{SPI1_IRQn, 2},
};

typedef void( *intfunc )( void );
typedef union { intfunc __fun; void * __ptr; } intvec_elem;
extern const intvec_elem __vector_table[];

static void nvic_init(void)
{
	int i;
	NVIC_InitTypeDef irq_init;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, (unsigned long)__vector_table - NVIC_VectTab_FLASH);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);    

	for (i = 0; i < sizeof(irqs)/sizeof(*irqs); i++)
	{
		irq_init.NVIC_IRQChannel = irqs[i].channel;
		irq_init.NVIC_IRQChannelPreemptionPriority = irqs[i].priority;
		irq_init.NVIC_IRQChannelSubPriority = 0;
		irq_init.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&irq_init);
	}

}

unsigned long jiffies = 0;

void SysTick_Handler(void)
{
	jiffies+=10;
}

void get_tick_count(unsigned long *timestamp)
{
	*timestamp = jiffies;
}

static void systick_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	SysTick_Config(170000);	
}

void SPI_DMA_init(uint16_t *buf, int len)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
	
	/* init DMA2_Stream 0 */	
	DMA_InitStructure.DMA_Channel = DMA_Channel_3; 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buf;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = len;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);

	/* DMA2_Stream3 enable */
	DMA_Cmd(DMA2_Stream3, ENABLE);
}

void SPI_init()
{
	SPI_InitTypeDef  SPI_InitStructure;

	/*
	** SPI1
	*/
	SPI_DeInit(SPI1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);
	__delay_ms(50);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	/*!< Enable the sFLASH_SPI  */
	SPI_Cmd(SPI1, ENABLE);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

	SPI_TIModeCmd(SPI1, ENABLE);

	
}

void DMA_init()
{
	/* Enable the DMA2 Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
}

void EXTI_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT, ENABLE);
}

static void hw_init(void)
{
	clk_init();
	
	nvic_init();

	GPIO_init();

	EXTI_init();
	
	usart_init();

	DMA_init();
	
	systick_init();
	
	FLASH_Unlock();

	SPI_init();
}

/* */
void usart_putc(int c)
{
	__tx_buffer_put(c);
	Usart_Tc_Int_Enable();
}

void fputc(char ch)
{
	__wait_tx_buffer_space();
	usart_putc(ch);
}

void mdelay(int ms)
{
	__delay_ms(ms);
}

int recv_byte(int timeout)                      //!!!!!!!!!!!!
{
	int j;
	while(timeout--) {
		for(j=0; j<4000;   j++) {
			if(!__rx_buffer_empty())
				return __rx_buffer_get();
	       }
	}
	return -1;
	
}

int usart_get_byte()
{
	if(!__rx_buffer_empty())
		return __rx_buffer_get();
	return -1;
}

void usart_puts(const char* msg)
{
	while(*msg) {
		if ('\n' == *msg) {
			usart_putc('\r');
		}
		usart_putc(*msg);
		msg ++;
	}
}

/* Flash OPs */
#define FLASH_START_ADDRESS (0x08000000)
#define FLASH_PAGE_SIZE (0x800)
#define FLASH_PAGE_NUM  (256)

#define MAX_CMD_LENGTH 256
#define MAX_CMD_ITEM   16
struct cmdline {
	int count;
	const char* params[MAX_CMD_ITEM];
};

static void parse_cmdline (char* in, struct cmdline* cmd)
{
	char* p_in = in;

	cmd->count = 0;
	while (*p_in)
	{
		switch (*p_in)
		{
		case '\r':
		case '\n':
		case ' ':
		case '\t':
			p_in ++;
			continue;
		}

		if (0 == *p_in)
			break;
		cmd->params[cmd->count] = p_in;
		cmd->count ++;

		while (*p_in)
		{
			if ('\r' == *p_in ||'\n' == *p_in ||' '  == *p_in || '\t' == *p_in)
			{
				*p_in = 0;
				p_in ++;
				break;
			}
			p_in ++;
		}
	}	
}

static int get_cmd_buffer (char* buff, int len)
{
	char* p = buff;
	int ch;

	while (p - buff < len -1)
	{
		ch = recv_byte (20);
		if(ch >= 0) {
			if ('\r' == ch || '\n' == ch) {
				break;		
			}
			if('\b' == ch) {
				if(p > buff)
					p--;
			}
			else {
				*p ++ = ch;
			}
#if SECURITY_ON
#else
			usart_putc (ch);
#endif
		}
	}
	*p = 0;
	return p-buff;
}

void Program_DATA_To_Sector(void *Dst, void *DATA, int Csize)
{
	u32 DestBuf, *Data;
	volatile int Index;

	DestBuf = (u32)Dst;
        Data = (u32*)DATA;
	for (Index = 0; Index < Csize; Index+=4) {
		FLASH_ProgramWord(DestBuf, *Data);
		Data++;
                DestBuf += 4;
                
	}
}

static void  cmd_reset (struct cmdline* cmd)
{
	__cli();
	NVIC_SystemReset();
}

#define BYTES_PER_COLOR	3
#define LED_PATTERN_N  60

const u8 LED_pattern_data[][BYTES_PER_COLOR]={
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	//G,R,B
	{0,255,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//16
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//32
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//48
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//64
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//80
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//96
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//112
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},

	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
//128
};

#define LED_ARRAY_SIZE		(LED_PATTERN_N*BYTES_PER_COLOR*5*8/(sizeof(uint16_t)*8))

static uint16_t *LED_data;
static unsigned int LED_data_pos = 0, LED_data_end = 0;


void SPI1_IRQHandler()
{
	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET) {
		if(LED_data_pos < LED_data_end) {
			SPI_I2S_SendData(SPI1, LED_data[LED_data_pos++]);
		}
		else {
			SPI_I2S_SendData(SPI1, 0x8000);
			SPI_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
		}
	}
}

static void SPI1_SendData(uint16_t hword)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, hword);
}

static void set_bits(uint16_t *bits, int offset, int nbits)
{
	while(nbits --) {
		bits[offset/16] |= 1<<(15-(offset%16));
		offset ++;
	}
}

static int get_bit(uint16_t *bits, int offset)
{
	return (bits[offset/16] & (1<<(15-(offset%16)))) ? 1 : 0;
}

static int led_get_bits(uint16_t *bits, uint8_t *color, int n_color)
{
	int i, j, offset = 0;
	uint8_t c;
	int n_sh;

	n_sh = (n_color * BYTES_PER_COLOR * 8 * 5) / (sizeof(uint16_t) * 8);
	memset(bits, 0, n_sh * sizeof(uint16_t));
	for(j=0; j<n_color*BYTES_PER_COLOR; j++) {
		c = color[j];
		for(i=0; i<8; i++) {
			if(c & (1<<(7-i)))
				set_bits(bits, offset, 4);
			else
				set_bits(bits, offset, 1);
			offset += 5;
		}
	}
	//set_bits(bits, offset, 1);
	return offset;
}

static void  cmd_led (struct cmdline* cmd)
{
	int offset, i, j;
	static uint16_t led_spi_data[7][LED_ARRAY_SIZE];

	if(cmd->count == 2 &&  (strcmp(cmd->params[1], "init") == 0)) {
		
	}
	else if(cmd->count == 2 &&  (strcmp(cmd->params[1], "single") == 0)) {
		hal_trace("led single!!\n");
		__delay_ms(10);
		
		offset = led_get_bits(led_spi_data[0], (uint8_t*)LED_pattern_data, LED_PATTERN_N);
#if 0
		LED_data = led_spi_data[0];
		LED_data_pos = 0;
		LED_data_end = ((offset + 15) & (~15)) / (sizeof(uint16_t)*8);
		SPI_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);

		while(LED_data_pos < LED_data_end);
		__delay_us(100);
		SPI_I2S_SendData(SPI1, 0xF000);
#else
		SPI_DMA_init(led_spi_data[0], LED_ARRAY_SIZE);
		while(DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		DMA_Cmd(DMA2_Stream3, DISABLE);
		__delay_us(100);
#endif
	}
	else if(cmd->count == 2 &&  (strcmp(cmd->params[1], "black") == 0)) {
		unsigned char black[LED_PATTERN_N][3];
		hal_trace("led black!!\n");
		__delay_ms(10);

		memset(black, 0, sizeof(black));
		offset = led_get_bits(led_spi_data[0], (uint8_t*)black, LED_PATTERN_N);
#if 0
		LED_data = led_spi_data[0];
		LED_data_pos = 0;
		LED_data_end = ((offset + 15) & (~15)) / (sizeof(uint16_t)*8);
		SPI_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);

		while(LED_data_pos < LED_data_end);
		
#else
		SPI_DMA_init(led_spi_data[0], LED_ARRAY_SIZE);
		while(DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		DMA_Cmd(DMA2_Stream3, DISABLE);
		__delay_us(100);
#endif
	}
	else if((cmd->count == 2) && (strcmp(cmd->params[1], "reset") == 0)) {
		hal_trace("led reset!!\n");
		__delay_ms(10);

		LED_data = led_spi_data[0];
		memset(LED_data, 0, 8*sizeof(uint16_t));
		LED_data[0] = 0xff00;
		LED_data[7] = 0x00ff;
		LED_data_pos = 0;
		LED_data_end = 8;
		SPI_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
	}
	else if((cmd->count == 2) && (strcmp(cmd->params[1], "run") == 0)) {
		unsigned char single_led[LED_PATTERN_N][3];
		unsigned char colors[7][3] = {
				{0,255,0}, {255,0,0},{0,0,255},{0,255,255},{255,0,255},{255,255,0},{255,255,255},
		};
		
		hal_trace("led run!!\n");
		__delay_ms(10);

		
		while(1) {
			for(i=0; i<7; i++) {
				for(j=0; j<60; j++) {
					memset(single_led, 0, sizeof(single_led));
					single_led[j][0] = colors[i][0];
					single_led[j][1] = colors[i][1];
					single_led[j][2] = colors[i][2];
					led_get_bits(led_spi_data[j&1], (uint8_t*)single_led, LED_PATTERN_N);
					
					SPI_DMA_init(led_spi_data[j&1], LED_ARRAY_SIZE);
					while(DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					DMA_Cmd(DMA2_Stream3, DISABLE);

					__delay_ms(20);
				}
			}
		}
	}
}

extern void cmd_bat(int argc, const char **argv);
static void cmd_battery(struct cmdline* cmd)
{
	cmd_bat(cmd->count, cmd->params);
}

static void cmd_12v(struct cmdline* cmd)
{
	hal_trace("12v: %s\n", _IS_LO(DETECT_12V)?"on":"off");
}

int locker_lock(int lock)
{
	if(lock) {
		int i;
		for(i=0; i<8; i++) {
			if(i < 2) {
				_HI(LOCK_OUTA);
			}
			else {
				_LO(LOCK_OUTA);
			}
			__delay_us(1);
			_LO(LOCK_CLK);
			__delay_us(1);
			_HI(LOCK_CLK);
			__delay_x(100);
		}
		if(!_IS_HI(LOCK_QH) || !_IS_HI(LOCK_QG)) {
			return 0;
		}
		else {
			return 1;
		}
	}
	else {
		_LO(LOCK_CLR);
		__delay_us(1);
		_HI(LOCK_CLR);
		__delay_us(1);
		if(!_IS_LO(LOCK_QH) || !_IS_LO(LOCK_QG)) {
			return 0;
		}
		else {
			return 1;
		}
	}
}

static void cmd_lock(struct cmdline* cmd)
{
	if(cmd->count == 2) {
		if(strcmp("lock", cmd->params[1]) == 0) {
			int ret = locker_lock(1);
			hal_trace("Lock: lock %s!!\n", ret?"success":"fail");
		}
		else if(strcmp("clear", cmd->params[1]) == 0) {
			int ret = locker_lock(0);
			hal_trace("Lock: clear %s!!\n", ret?"success":"fail");
		}
		else if(strcmp("show", cmd->params[1]) == 0) {
			char *QH = _IS_LO(LOCK_QH)?"LOW":"HI";
			char *QG = _IS_LO(LOCK_QG)?"LOW":"HI";
			hal_trace("Lock: QH:%s, QG:%s\n",  QH, QG);
		}
		
	}
}


static void  cmd_help (struct cmdline* cmd);

const static struct _cmd_def{
	char* name;
	void (*proc) (struct cmdline*);
	char* help_msg;
} cmds[] = {
	{"help", cmd_help, "show this message"},
	{"reset", cmd_reset, "reset cpu"},
	{"led", cmd_led, "play led pattern"},
	{"bat", cmd_battery, "comm with battery"},
	{"12v", cmd_12v, "detect 12v on"},
	{"lock", cmd_lock, "car lock ctrl"},
};

static void  cmd_help (struct cmdline* cmd)
{
	int i;
	for (i = 0; i < sizeof(cmds)/sizeof(*cmds); i++) {
		hal_trace("%s : %s\n", cmds[i].name, cmds[i].help_msg);
	}
}

static void dispatch (struct cmdline* cmd)
{
	int i;
	if (!cmd->count)
		return;
	for (i = 0; i < sizeof(cmds)/sizeof(*cmds); i++)
	{
		if (!strcmp (cmds[i].name, cmd->params[0]))
		{
			usart_putc('\n');
			cmds[i].proc(cmd);
			return;
		}
	}
}

extern int decode_cmd_buffer(char *obuf, int len);

int main()
{
	static char cmd_buffer [MAX_CMD_LENGTH];
	struct cmdline cmd;
		
        __cli();
	
        hw_init();
	__sti();
	
	uart_comm_init();

	while (1) {
		usart_puts ("\nMach>");                         //TX
		get_cmd_buffer (cmd_buffer, MAX_CMD_LENGTH);    //RX
		parse_cmdline (cmd_buffer, &cmd);
		dispatch(&cmd);
	}
}

void USB_Istr(void)
{
}

