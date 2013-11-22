/*
 * Hyd_Control.c
 *
 * Created: 2013/10/17 21:44:27
 *  Author: Will
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "uart.h"
#include "Variable.h"
#include "delay.h"

#define Debug 0
#define Analog_Servo 1
#define Digital_Servo 0
#define Loop 0

void Status_Packet(uint8_t status);

void IO_Init(void)
{
	DDRC &= ~_BV(PC0);
	DDRC &= ~_BV(PC1);
	PORTC&= ~_BV(PC0);
	PORTC&= ~_BV(PC1);  //置输入，AD转换
	
	DDRB |= _BV(PB1);

	DDRC |= _BV(LED);  //LED
	DDRD |= _BV(LED2);  //LED2

	DDRC |= _BV(RD_E);
	PORTC &= ~_BV(RD_E);  //485置接收
}

void T0_Init(void)
{
	TCNT0 = 223; //500Hz
	//TCNT2 = 10;  //T2初值:                           //计数器初始化值
	TCCR0 = (1<<CS02)|(0<<CS01)|(1<<CS00); //T0:1024：101; 256：100; 011:64 ; 010:8; 001:1;
	//TCCR2 |= (0<<WGM20) | (0<<WGM21) | (0<<COM21) | (0<<COM20) | (1<<CS22) | (1<<CS21) | (1<<CS20);//T2:1024分频
	TIFR = (1<<TOV0);                           // Clear any pending interrupt.
	//TIMSK |= (1<<TOIE0) | (1<<TOIE2);            //// Set the timer/counter0 interrupt masks.开T0，关T2；
	TIMSK = (1<<TOIE0);
}

void EEPROM_Init(void)
{
#if Debug

	eeprom_busy_wait();
	ID = eeprom_read_byte(0);
	
	while(ID != 0x01)
	{
		//直接使用EEPROM地址操作EEPROM
		//等待EEPROM读写就绪
		eeprom_busy_wait();
		//将0x41写入到EEPROM的0地址处
		eeprom_write_byte(0, 0x01);
		//等待EEPROM读写就绪
		eeprom_busy_wait();
		//从eeprom的0地址处读取一字节赋给ram变量temp
		ID = eeprom_read_byte(0);
	}
	
#endif
}

void ADC_Init(void)
{
	//TCNT0 = 250;
	ADCSRA = 0x00;
	ACSR = 0x00;  //关闭模拟比较器电源
	ADC_Channel = 0;
	ADMUX = (ADC_Channel&0x0f)|(0<<REFS1) | (1<<REFS0); //AVCC，右对齐，PC0电位计
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIF) | (1<<ADFR) | (0<<ADIE) | (1<<ADPS2) |(1<<ADPS1) | (1<<ADPS0);  //启动ADC功能，关中断，连续转换，128分频
	//转换精度要求为10位的话分频必须在50kHz-200kHz之间
}

void Pwm_Init(void)
{
	
#if Analog_Servo
	ICR1 = 40000;  //Make sure Max_pwm <= ICR1;
	//Max_pwm = ICR1;
	//OCR1A = 500;  //150
	//OCR1B = 500;  //150.
	//TCCR1A |= ((1 << COM1A1) | (1 << COM1B1) | (1 << WGM11));
	TCCR1A = (0<<COM1A1)|(1<<WGM11);   //关闭输出
	//OC1A OC1B 快速PWM，TOP值为ICR1，PWM启动,Prescaler: 001->1; 010:8; 011->64; 100->256; 101->1024
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
	//控制频率16000000/(256*312) = 50 Hz
#endif

#if Digital_Servo
	ICR1 = 156;  //Make sure Max_pwm <= ICR1;
	//Max_pwm = ICR1;
	//OCR1A = 500;  //150
	//OCR1B = 500;  //150.
	//TCCR1A |= ((1 << COM1A1) | (1 << COM1B1) | (1 << WGM11));
	TCCR1A = (1<<COM1A1)|(1<<WGM11);   //关闭输出
	//OC1A OC1B 快速PWM，TOP值为ICR1，PWM启动,Prescaler: 001->1; 010:8; 011->64; 100->256; 101->1024
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10);
	//控制频率16000000/(1024*156) = 100.16 Hz
#endif

}

void Servo_Control(uint16_t duty)
{
	
#if Analog_Servo
	if(duty == 0)
	{
		TCCR1A &= ~(1<<COM1A1);
	}
	else
	{
		OCR1A = duty;  //0.5ms-2.5ms
		TCCR1A = 0x82;
	}
#endif

#if Digital_Servo
	TCCR1A = 0x82;
	//OCR1A = duty;
	OCR1A = 75;
#endif

}

void Send_Uint16(uint16_t temp)
{
	PORTC |= _BV(RD_E);
	Send_Char(temp>>8);
	Send_Char(temp);
	delay_us(10);
	PORTC &= ~_BV(RD_E);
}

void Run(void)
{
	uint16_t temp = 0;
	switch(msg[2])
	{
		case 0x01:  //PING
		{
			Status_Packet(0x01);
		}
		break;
		case 0x02:  //Read Data
		switch(msg[3])
		{
			case 0x24:
			{
				if(msg[4] == 2)
				Status_Packet(0x24);
			}
			case 0x2B:
			{
				if(msg[4] == 1)
				Status_Packet(0x2B);
			}
			//Send_uint16(Temp_ADC);
			break;
			
		}
		break;
		case 0x03:  //Write Data
		{
			switch(msg[3])
			{
				case 0x03:  //ID
				if((msg[4] < 0xfd) && (msg[4] > 0))
				{
					//eeprom_busy_wait();
					//eeprom_write_byte(0x03,msg[4]);
				}
				break;
				case 0x04:  //Baud Rate
				if((msg[4] < 208) && (msg[4] > 0))
				{
					switch(msg[4])
					{
						case 0x16:
						BAUD = 115200;
						Uart_Init(3,0,0);
						break;
						case 0x22:
						BAUD = 57600;
						Uart_Init(3,0,0);
						break;
						case 0xCF:
						BAUD = 9600;
						Uart_Init(3,0,0);
						break;
					}
					
				}
				break;
				case 0x18:  //Torque Enable
				{
					flag_MASTER = (msg[4] == 1)?1:0;
				}
				break;
				case 0x19:  //LED
				{
					if(msg[4] == 1)
					PORTD |= _BV(LED);  //点亮LED
					else
					PORTD &= ~_BV(LED);  //关闭LED
				}
				break;
				case 0x1E:  //Goal Position
				{
					temp = msg[4];  //低8位
					temp |= (msg[5]<<8);  //高8位
					if(temp > Command_position)
					{
						direct_flag = 1;//前进
					}
					else
					{
						direct_flag = 0;//后退
					}
					Command_position = temp;
					flag_MASTER = 1;  //开始执行
					PORTC = (PORTC^(1<<LED));  //操作指示
				}
				
				break;
			}
		}
		break;
		case 0x04:  //Reg Write
		break;
		case 0x05:  //Action
		{
		}
		break;
		case 0x06:  //Reset
		{
			//eeprom_busy_wait();
			//eeprom_write_byte(ID_Addr,0x00);  //ID号置为0
			Command_position = 512;  //位置置中
			//eeprom_busy_wait();
			//eeprom_write_byte(PSL_Addr,0x00);
			//eeprom_busy_wait();
			//eeprom_write_byte(PSH_Addr,0x02);

		}
		break;
		case 0x07:  //Sync Write
		break;
	}
}

void Status_Packet(uint8_t status)  //返回状态
{
	//eeprom_busy_wait();
	//uint8_t ID = eeprom_read_byte(ID_Addr);
	//ID = eeprom_read_byte(ID_Addr);
	//ID = eeprom_read_byte(ID_Addr);
	uint8_t ID = 1;
	uint8_t length;
	uint8_t check_sum;

	PORTC |= _BV(RD_E);
	Send_Char(0xff);
	Send_Char(0xff);
	Send_Char(ID);
	switch(status)
	{
		case 0x01://Ping
		{
			length = 2;
			check_sum = ~(ID + length + error);
			Send_Char(length);
			Send_Char(error);
			Send_Char(check_sum);
			delay_us(100);
			PORTC &= ~_BV(RD_E);
		}
		break;
		case 0x24://当前位置
		{
			length = 4;
			uint8_t p1,p2;
			p1 = WR_ADC;p2 = WR_ADC>>8;
			check_sum = ~(ID + length + error + p1 + p2);
			Send_Char(length);
			Send_Char(error);
			Send_Char(p1);
			Send_Char(p2);
			Send_Char(check_sum);
			delay_us(100);
			PORTC &= ~_BV(RD_E);
		}
		break;
		case 0x2B://温度
		{
			uint8_t p1;
			length = 3;
			p1 = Temp_ADC;
			check_sum = ~(ID + length + error + p1);
			Send_Char(length);
			Send_Char(error);
			Send_Char(p1);
			Send_Char(check_sum);
			delay_us(100);
			PORTC &= ~_BV(RD_E);
		}
		break;
	}
	PORTC &= ~_BV(RD_E);//置接收
}

void PID_Run(void)
{
	//PORTD |= _BV(LED);  //点亮LED
	uint16_t temp = 0;
	uint16_t p=0;
	uint16_t d=0;
	Adjustment_amount = 0;
	if(flag_WADC == 1)
	{
		flag_WADC = 0;
		filtered_position = WR_ADC;
		//filtered_position = filter_update(WR_ADC);
		if(Command_position == filtered_position)
		{
			Servo_Control(Mid_position);
		}
		else
		{
			Difference = (int16_t)(Command_position - filtered_position);
			Difference_position = previous_position - filtered_position;
			previous_position = filtered_position;
			
			if(direct_flag == 1)
			{
				p = p1_Gain;
				d = d1_Gain;
			}
			else
			{
				p = p2_Gain;
				d = d2_Gain;
			}
			
			if((Difference > deadband) | ( Difference < (-deadband)))
			{
				if(Difference > 0)
				{
					temp = Min_position;
				}
				else
				{
					temp = Max_position;
				}
			}
			else
			{
				Adjustment_amount += (int32_t)Difference * (int32_t)p;
				Adjustment_amount += (int32_t)Difference_position * (int32_t)d;
				if(Adjustment_amount < 0 )
				{
					Adjustment_amount = -Adjustment_amount;
					Adjustment_amount = Adjustment_amount>>4;
					Adjustment_amount = -Adjustment_amount;
				}
				else
				{
					Adjustment_amount = Adjustment_amount>>4;
				}
				temp = Mid_position - Adjustment_amount;
				if(temp > Max_position) 
					temp = Max_position;
					
				if(temp < Min_position)
					temp = Min_position;
			}
			Servo_Control(temp);
		}
	}
	else
	{
		//Servo_Control(3000);//置中
	}
}

int main(void)
{
	
	IO_Init();
	PORTC &= ~_BV(LED);  //点亮LED
	Uart_Init(3,0,0);
	ADC_Init();
	T0_Init();
	Pwm_Init();
	delay_us(200);
	//EEPROM_Init();
	PORTC |= _BV(LED);  //关闭LED
	delay_ms(500);
	sei();

	while(1)
	{
		if(OUT_Queue(&data) == 1)  //出队操作是否成功，是否有数据需要处理
		{
			//Send_Char(data);
			if( n < 18)
			{
				if(data == 0xff)
				{
					if(status_flag == 1)
					{
						status_flag = 2;
						n = 0;
					}
					else
					{
						if(status_flag == 2)
						{
							sum += data;
							msg[n] = data;
							n++;
						}
						else
						{
							if(status_flag == 0)
							{
								if(n == 0)
								{
									status_flag = 1;
								}
							}
						}
					}
				}
				else
				{
					if(( n > 2 ) && ( msg[1] == (n-1) )) //正确
					{
						msg[n] = data;  //校验位，不计入和
						if((msg[0] == 0xFE)|(msg[0] == ID)) //ID是否符合或者是否是广播命令
						{
							sum = ~sum;
							if(msg[n] == sum) //数据校验是否正确
							{
								//Send_Char(0x55);
								Run();  //执行函数
							}
						}
						status_flag = 0;
						n = 0;
						sum = 0;
					}
					else
					{
						sum += data;
						msg[n] = data;
						n++;
					}
				}
			}
			else
			{
				n = 0;
			}
		}
		if(flag_MASTER == 1)
		{
			PID_Run();
		}
		else
		{
			Servo_Control(0);//取消力矩
		}
#if Loop
		if(nn > 50000)
		{
			nn = 0;
			if(Command_position == 300)
			{
				Command_position = 400;
			}
			else if(Command_position == 400)
			{
				Command_position = 300;
			}
		}
		nn++;
#endif
	}
}

ISR(USART_RXC_vect,ISR_BLOCK)  //接收中断 阻塞式
{
	buf = UDR;
	if(IN_Queue(buf) == 1)  //队列未满
	{
		flag_buff_Error = 0;  //队列操作正常
	}
	else  //队列已满
	{
		buff_rear = buff_front = 0;  //清队列
		//flag_buff_Error = 1;  //队列操作出错
	}
}

ISR(TIMER0_OVF_vect,ISR_NOBLOCK)//T0溢出中断 非阻塞式
{
	sei();
	while((ADCSRA & (1 << ADIF)) == 0);
	WR_ADC = ADCL;
	WR_ADC |= (uint16_t)(ADCH<<8);
	//OCR1A = (uint8_t)WR_ADC;
	flag_WADC = 1;
	PORTD = (PORTD^(1<<LED));  //操作指示
	TCNT0 = 223;
}