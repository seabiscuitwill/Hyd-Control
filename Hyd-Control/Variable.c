#include "Variable.h"

//PID
int16_t Difference;
int16_t Difference_position;
uint16_t Command_position = 300;//850-100
int16_t previous_position = 0;
int32_t Adjustment_amount = 0;
int16_t deadband = 100;       //PID������

uint16_t d1_Gain = 50;  //ǰ������
uint16_t p1_Gain = 20;  //

uint16_t d2_Gain = 50;  //���˲���
uint16_t p2_Gain = 20;  //

uint16_t filtered_position = 0;
volatile uint8_t direct_flag = 1;

//PWM
uint16_t Max_position = 3250;//1660
uint16_t Mid_position = 3000;
uint16_t Min_position = 2750;//80

//ADת��
volatile uint8_t ADC_Channel = 0;
volatile uint16_t  WR_ADC = 0;//���λ��AD���
volatile uint16_t  Temp_ADC = 0;//�¶�AD���
uint8_t flag_WADC = 0;
uint8_t flag_MASTER = 1;

int Lock_time = 0;  //��תʱ��
uint8_t flag_HOT = 0;  //���ȱ�־

uint32_t filter_reg = 0;
uint16_t filter_update(uint16_t input)
{
    //Update the filter with the current input.
    filter_reg = filter_reg - (filter_reg >> FILTER_SHIFT) + input;

    //Scale output for unity gain.
    return (int16_t) (filter_reg >> FILTER_SHIFT);
}
