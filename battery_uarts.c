#include <hal.h>
#include <cpuusg.h>
#include <string.h>
#include <utils.h>
#include <stdarg.h>
#include <stdlib.h>

static USART_TypeDef *usarts[] = {USART1, UART5, USART6, USART3};               //USART1, UART5, USART6, USART3
#define USART_N	4       

void battery_usart_TC_Int_Enable(int idx)                                       //使能串口idx中断-发送据完成中断
{
	USART_ITConfig(usarts[idx], USART_IT_TXE, ENABLE);
}

void battery_usart_TC_Int_Disable(int idx)                                      //关闭串口idx中断
{
	USART_ITConfig(usarts[idx], USART_IT_TXE, DISABLE);
}


#define RX_BUF_SIZE		1024
static unsigned char __battery_rx_buffer[USART_N][RX_BUF_SIZE];                 //定义串口通信的RX缓冲区
static unsigned short __battery_rx_buffer_tail[USART_N], __battery_rx_buffer_head[USART_N];     //定义head和tail

static unsigned char __battery_rx_buffer_get(int idx)                           //RX缓冲区的获取
{
	unsigned char	data;
	hal_flags		flags;
	save_and_cli(flags);
	data = __battery_rx_buffer[idx][__battery_rx_buffer_tail[idx]++];
	if(__battery_rx_buffer_tail[idx] == RX_BUF_SIZE)
		__battery_rx_buffer_tail[idx] = 0;
	restore_flags(flags);
	return data;
}

static void __battery_rx_buffer_put(int idx, unsigned char data)                //将DATA放入RX缓冲区
{
	hal_flags	flags;
	save_and_cli(flags);
	__battery_rx_buffer[idx][__battery_rx_buffer_head[idx]++] = data;
	if(__battery_rx_buffer_head[idx] == RX_BUF_SIZE)
		__battery_rx_buffer_head[idx] = 0;
	if(__battery_rx_buffer_head[idx] == __battery_rx_buffer_tail[idx]) {
		__battery_rx_buffer_tail[idx] ++;
		if(__battery_rx_buffer_tail[idx] == RX_BUF_SIZE)
			__battery_rx_buffer_tail[idx] = 0;
	}
	restore_flags(flags);
}

static void __battery_rx_buffer_clear(int idx)                                  //RX缓冲区清空
{
	__battery_rx_buffer_head[idx] = __battery_rx_buffer_tail[idx] = 0;
}

static int __battery_rx_buffer_empty(int idx)                                   //判断缓冲区是否为空
{
	return (__battery_rx_buffer_tail[idx] == __battery_rx_buffer_head[idx]);
}

static void battery_usart_on_error(int uart_idx)
{

}

static void battery_usart_on_data(int idx, unsigned char data)                  //将数据放入RX缓冲区
{
	__battery_rx_buffer_put(idx, data);
}


#define TX_BUF_SIZE		128
static unsigned char __battery_tx_buffer[USART_N][TX_BUF_SIZE];                 //定义TX缓冲区
static unsigned short __battery_tx_buffer_head[USART_N], __battery_tx_buffer_tail[USART_N];

static unsigned char __battery_tx_buffer_get(int idx)                           //将TX缓冲区内容放入DATA
{
	unsigned char data;
	hal_flags flags;
	
	save_and_cli(flags);
	data = __battery_tx_buffer[idx][__battery_tx_buffer_tail[idx]++];
	__battery_tx_buffer_tail[idx] %= TX_BUF_SIZE;
	restore_flags(flags);
	return data;
}

static void __battery_tx_buffer_put(int idx, unsigned char data)                //将data放入TX的缓冲区中
{
	hal_flags flags;

	save_and_cli(flags);
	__battery_tx_buffer[idx][__battery_tx_buffer_head[idx]++] = data;
	__battery_tx_buffer_head[idx] %= TX_BUF_SIZE;
	restore_flags(flags);
}

static void __battery_wait_tx_buffer_space(int idx)
{
	unsigned short head = __battery_tx_buffer_head[idx];
	unsigned long timeout = 1000;
	
	while(((head+1)%TX_BUF_SIZE) == __battery_tx_buffer_tail[idx]) {
		if(--timeout == 0)
			break;
		__delay_x(2);
	}
}

static unsigned short __battery_tx_buffer_get_space(int idx)                    //获取TX剩余空间
{
	
	if(__battery_tx_buffer_tail[idx] > __battery_tx_buffer_head[idx])
		return __battery_tx_buffer_tail[idx] - __battery_tx_buffer_head[idx] - 1;
	else
		return TX_BUF_SIZE - (__battery_tx_buffer_head[idx] - __battery_tx_buffer_tail[idx]) - 1;
}

static unsigned short __battery_tx_buffer_get_used(int idx)
{
	return TX_BUF_SIZE - __battery_tx_buffer_get_space(idx) + 1;
}

static void __battery_tx_buffer_clear(int idx)                                  //清空tx
{
	__battery_tx_buffer_tail[idx] = __battery_tx_buffer_head[idx] = 0;
}

static int __battery_tx_buffer_empty(int idx)
{
	return __battery_tx_buffer_tail[idx] == __battery_tx_buffer_head[idx];
}

static void battery_usart1_on_sent(int idx)                                     //将数据存入到DR中
{
	if(!__battery_tx_buffer_empty(idx)) {
		char ch = __battery_tx_buffer_get(idx);
		USART_SendData(usarts[idx], ch);
	}
	else {
		battery_usart_TC_Int_Disable(idx);
	}
}


void battery_usart_irq_handler(int idx)
{
	USART_TypeDef *UARTn = usarts[idx];
	volatile unsigned short SR = UARTn->SR;
	
	if(SR & (USART_FLAG_RXNE | USART_FLAG_PE | USART_FLAG_FE | USART_FLAG_ORE | USART_FLAG_NE)) {
		unsigned short ch = USART_ReceiveData(UARTn);
		if(SR & (USART_FLAG_PE | USART_FLAG_FE | USART_FLAG_ORE | USART_FLAG_NE)) {
			battery_usart_on_error(idx);
			goto nodata;
		}
		if((SR & USART_FLAG_RXNE)) {
			battery_usart_on_data(idx, (unsigned char)ch);
		}
nodata:
		USART_ClearITPendingBit(UARTn, USART_IT_RXNE);
	}
	if(SR & USART_FLAG_TXE) {
		battery_usart1_on_sent(idx);
		USART_ClearITPendingBit(UARTn, USART_IT_TXE);
	}
}

void battery_usart_putc(int idx, int c)                                         //putc
{
	__battery_tx_buffer_put(idx, c);
	battery_usart_TC_Int_Enable(idx);
}

void battery_usart_puts(int idx, char *msg)                                     //puts
{
	while(*msg) {
		if ('\n' == *msg) {
			battery_usart_putc(idx, '\r');
		}
		battery_usart_putc(idx, *msg);
		msg ++;
	}
}

void USART1_IRQHandler(void)
{
	battery_usart_irq_handler(0);
}

void UART5_IRQHandler(void)
{
	battery_usart_irq_handler(1);
}

void USART6_IRQHandler(void)
{
	battery_usart_irq_handler(2);
}

void USART3_IRQHandler(void)
{
	battery_usart_irq_handler(3);
}

static void battery_uart_init(void)                                             //串口初始化
{
	USART_InitTypeDef usart_init;
	int i;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                  //开时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	for(i=0; i<4; i++) {                                                    //开串口
		USART_Cmd(usarts[i], DISABLE);
		USART_DeInit(usarts[i]);                                        //串口清零
	}
	
	__delay_ms(200);                                

	GPIO_PinAFConfig(BATT1_UART_TXD_PORT, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(BATT1_UART_RXD_PORT, GPIO_PinSource7, GPIO_AF_USART1);

	GPIO_PinAFConfig(BATT2_UART_TXD_PORT, GPIO_PinSource12, GPIO_AF_UART5);
	GPIO_PinAFConfig(BATT2_UART_RXD_PORT, GPIO_PinSource2, GPIO_AF_UART5);

	GPIO_PinAFConfig(BATT3_UART_TXD_PORT, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(BATT3_UART_RXD_PORT, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_PinAFConfig(BATT4_UART_TXD_PORT, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(BATT4_UART_RXD_PORT, GPIO_PinSource11, GPIO_AF_USART3);


	USART_StructInit(&usart_init);
	usart_init.USART_BaudRate = 115200;
	usart_init.USART_Mode = USART_Mode_Tx |USART_Mode_Rx;	

	for(i=0; i<4; i++) {			
		USART_Init(usarts[i], &usart_init);
		USART_ClearITPendingBit(usarts[i], USART_IT_TXE);
		USART_ClearITPendingBit(usarts[i], USART_IT_RXNE);
		USART_ITConfig(usarts[i], USART_IT_RXNE, ENABLE);
		USART_ITConfig(usarts[i], USART_IT_PE, ENABLE);
		USART_ITConfig(usarts[i], USART_IT_FE, ENABLE);
		USART_ITConfig(usarts[i], USART_IT_ORE, ENABLE);
		USART_ITConfig(usarts[i], USART_IT_NE, ENABLE);
		USART_Cmd(usarts[i], ENABLE);
	}
}

extern void usart_putc(int c);
extern unsigned long jiffies;

#define UART_TIMEOUT	1000

void cmd_bat(int argc, const char **argv)
{
	int i;
	srand(jiffies);
	if(argc >= 2) {
		if(strcmp(argv[1], "init") == 0) {
			battery_uart_init();
			for(i=0; i<USART_N; i++) {
				__battery_tx_buffer_clear(i);
				__battery_rx_buffer_clear(i);
			}
		}
		else {
			int uart_idx = a2dec((void*)argv[1]);
			if(uart_idx >= 0 && uart_idx <= 3) {
				unsigned long timeout;
				unsigned char buf[128+8];
				int buf_len;
				
				buf[0] = 0;
				for(i=2; i<argc; i++) {
					strcat((void*)buf, argv[i]);
					strcat((void*)buf, " ");
				}
				buf_len = strlen((void*)buf)-1;
				if(encode_cmd_buffer((void*)buf, buf_len) == 128) {
					buf[128] = '\r';
					buf[129] = 0;
					battery_usart_puts(uart_idx, (void*)buf);

					timeout = UART_TIMEOUT;
					buf_len = 0;
					while(timeout--)
					{
						if(!__battery_rx_buffer_empty(uart_idx)) {
							int ch = __battery_rx_buffer_get(uart_idx);
							buf[buf_len++] = ch;
							timeout = UART_TIMEOUT;
						}
						else {
							__delay_us(300);
						}
					}
					if(buf_len) {
						if(buf_len == 128) {
							int isstr = 1;
							buf_len = decode_cmd_buffer(buf, buf_len);
							for(i=0; i<buf_len; i++) {
								if((i%16) == 0)
									usart_putc('\n');
								hal_trace("%02x ", buf[i]);
								if((buf[i] < 0x20 || buf[i] > 127) && (buf[i] != '\r') && (buf[i] != '\n'))
									isstr = 0;
							}
							usart_putc('\n');
							if(isstr) {
								buf[buf_len] = 0;
								usart_puts(buf);
							}
						}
						else {
							buf[buf_len] = 0;
							usart_puts(buf);
						}
					}
				}
			}
		}
	}
	
}

