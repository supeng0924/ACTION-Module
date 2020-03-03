#include "stm32f4xx_usart.h"


void USART1_Init(uint32_t BaudRate);
void WirelessBluetoothUsartInit(uint32_t BaudRate);
 void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
char *itoa(int value, char *string, int radix);
void GyroscopeUsartInit(uint32_t BaudRate);
void Usart6Init(uint32_t BaudRate);
void uart_initPA2(uint32_t bound);
void uart_initpb10(uint32_t bound);

