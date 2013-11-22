#include "uart.h"

//Uart
uint8_t ID = 0x01;
uint32_t BAUD = 38400;
volatile uint8_t buf = 0;
uint8_t data;
volatile uint8_t msg[20];
volatile uint8_t n = 0;  //��Ϣ��ʶλ
uint8_t head_flag;
volatile uint8_t status_flag = 0;
volatile uint8_t sum = 0;
uint8_t error = 0;

//���в���
volatile uint8_t buff[100]; //�洢��λ����Ϣ
volatile uint16_t buff_front = 0;
volatile uint16_t buff_rear = 0;
volatile uint8_t flag_buff_FULL = 0;
volatile uint8_t flag_buff_Error = 0;

void Uart_Init(uint8_t DateBit,uint8_t Even_Odd_check,uint8_t StopBit)
{
	DDRC |= _BV(RT_EN);

	unsigned long temp;
	temp = ((XTAL*62500)/BAUD-1); //����������ֵ����
	UBRRH = (temp/256); //�����ʸ߼Ĵ���
	UBRRL = (temp%256); //�����ʵͼĴ���
	UCSRA = (0<<RXC)|(0<<TXC)|(1<<UDRE)|(0<<FE)|(0<<DOR)|(0<<PE)|(0<<U2X)|(0<<MPCM);
	UCSRB = (0<<RXCIE)|(0<<TXCIE)|(0<<UDRIE)|(0<<RXEN)|(0<<TXEN)|(0<<UCSZ2)|(0<<RXB8)|(0<<TXB8);
	UCSRC = (1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0)|(0<<UCPOL);
	UCSRB = (1<<RXCIE)|(0<<TXCIE)|(0<<UDRIE)|(1<<RXEN)|(1<<TXEN)|(((DateBit&0X04)>>2)<<UCSZ2)|(0<<RXB8)|(0<<TXB8);
	UCSRC = (1<<URSEL)|(0<<UMSEL)|((Even_Odd_check&0x03)<<UPM0)|((StopBit&0X01)<<USBS)|((DateBit&0x03)<<UCSZ0)|(0<<UCPOL);
}

void Send_Char(uint8_t data)  
{      
	while(!(UCSRA & (1 << UDRE)));  
    UDR = data;
}

uint8_t IN_Queue(uint8_t e)
{
	if((flag_buff_FULL == 1)&&(buff_front == buff_rear))  //���ǰ�����Ƿ���
	{
		return 0;
	}
	else
	{
		buff[buff_rear] = e;
		buff_rear = (buff_rear + 1)%buff_MAX_SIZE;

		if(buff_rear == buff_front)  //��Ӻ�����Ƿ���
		{
			flag_buff_FULL = 1;
		}

		return 1;
	}
}

uint8_t OUT_Queue(uint8_t* e)
{
	if((flag_buff_FULL == 0)&&(buff_front == buff_rear))  //����ǰ�����Ƿ�Ϊ��
	{
		return 0;
	}
	else
	{
		*e = buff[buff_front];
		buff_front = (buff_front + 1)%buff_MAX_SIZE;

		if(buff_front == buff_rear)  //���Ӻ�����Ƿ�Ϊ��
		{
			flag_buff_FULL = 0;
		}

		return 1;
	}
}

uint8_t Queue_Length(void)
{
	if((flag_buff_FULL == 1)&&(buff_front == buff_rear))  //��������
	{
		return buff_MAX_SIZE;
	}
	else
	{
		return (buff_rear - buff_front + buff_MAX_SIZE)%buff_MAX_SIZE;  //buff_rear��һ��һֱ��buff_front��
	}
}
