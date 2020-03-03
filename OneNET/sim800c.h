#ifndef __SIM800C_H
#define __SIM800C_H

#include "stm32f4xx.h"
//进行是否有数据判断  由于收的数据中有0x00，所以选择0x01作为没数据进入的标志
#define NO_DATA             1
#define SEND_TYPE_NUM       2 //发送数的种类
#define SEND_CONNECT        0
#define WAIT_CONNECT        1
#define SEND_DATA           2
#define WAIT_SENDOK         3
#define CONNECT_CLEAR       4
#define SEND_TO_SIM800      USART2
#define RECEIVE_SIM800      USART3
void sim800c_main(void);

//++++++++++++++++++++++++函数++++++++++++++++++++++++++

//字符串比较函数
uint8_t compare_char(uint8_t *p1,uint8_t *contrast);//字符串比较函数
//提取数据函数
uint8_t deal_data(void);
//发送数据endsignal 决定是否加后缀
void SendData(USART_TypeDef* USARTx,uint8_t *p,uint8_t endsignal);
//队列下一个元素序号
uint16_t get_next_number(uint16_t x,uint16_t max_val);
//计算数据长度
uint16_t solve_data_length(uint16_t start,uint16_t end);
//复制后清空数据
void copy_clear_data(uint16_t start,uint16_t end);
//整型数据转换为字符串
void data_convert2send(int datasend);
//判断SIM800连接错误
uint8_t JudgeErrorClosed(void);
//++++++++++++++++++++++++函数++++++++++++++++++++++++++
#endif
