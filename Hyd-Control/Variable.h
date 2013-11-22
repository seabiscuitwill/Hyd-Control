#ifndef _VARIABLE_H
#define _VARIABLE_H

#include <avr/io.h>

#define FILTER_SHIFT 1

//PID
extern int16_t Difference;
extern int16_t Difference_position;
extern uint16_t Command_position;//850-100
extern int16_t previous_position;
extern int32_t Adjustment_amount;
extern int16_t deadband;       //PID������
extern uint16_t d1_Gain;  //200
extern uint16_t p1_Gain;  //100 ���� 60����
extern uint16_t d2_Gain;  //200
extern uint16_t p2_Gain;  //100 ѹ��4��P30 D60
extern uint16_t filtered_position;
extern volatile uint8_t direct_flag;

//PWM
extern uint16_t Max_position;//600
extern uint16_t Mid_position;//500
extern uint16_t Min_position;//400

//ADC
extern volatile uint8_t ADC_Channel;
extern volatile uint16_t  WR_ADC;//���λ��AD���
extern volatile uint16_t  Temp_ADC;//�¶�AD���
extern uint8_t flag_WADC;
extern uint8_t flag_TADC;
extern uint8_t flag_MASTER;

extern int Lock_time;  //��תʱ��
extern uint8_t flag_HOT;  //���ȱ�־
extern uint32_t filter_reg;

extern uint16_t filter_update(uint16_t input);

#endif


