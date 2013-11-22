#ifndef _UART_H
#define _UART_H

#include <avr/io.h>

#define XTAL 16.000000
#define buff_MAX_SIZE 99
#define RT_EN PC3
#define ID_Addr 0x03
#define Baud_Rate 0x04
#define RD_E PC5    //Max485发送接收标志端口位
#define LED PC2		//LED
#define LED2 PD2		//LED

extern uint8_t ID;
extern uint32_t BAUD;
extern volatile uint8_t buf;
extern uint8_t data;
extern volatile uint8_t msg[20];
extern volatile uint8_t n;  //信息标识位
extern uint8_t head_flag;
extern volatile uint8_t status_flag;
extern volatile uint8_t sum;
extern uint8_t error;

//队列操作
extern volatile uint8_t buff[100]; //存储上位机信息
extern volatile uint16_t buff_front;
extern volatile uint16_t buff_rear;
extern volatile uint8_t flag_buff_FULL;
extern volatile uint8_t flag_buff_Error;

void Uart_Init(uint8_t DateBit,uint8_t Even_Odd_check,uint8_t StopBit);
void Send_Char(uint8_t data);
uint8_t IN_Queue(uint8_t e);
uint8_t OUT_Queue(uint8_t* e);
uint8_t Queue_Length(void);

#endif
