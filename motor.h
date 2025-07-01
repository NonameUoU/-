#include <stdio.h>
#include "pinctrl.h"
#include "common_def.h"

#define I2C_MASTER_BUS_ID 1

struct pwm_info
{
    uint16_t TIM1_PSCR;
    uint16_t TIM1_ARR;
    uint16_t TIM1_CCR1;
    uint16_t TIM1_CCR2;
    uint16_t TIM1_CCR3;
    uint16_t TIM1_CCR4;
    uint16_t TIM2_PSCR;
    uint16_t TIM2_ARR;
    uint16_t TIM2_CCR1;
    uint16_t TIM2_CCR2;
    uint16_t TIM2_CCR3;
};

void pwm_ReadData(struct pwm_info* PWM_info);
void pwm_write(uint8_t reg_data);
void pwm_writes(uint8_t* reg_data);
void left_wheel_set(uint16_t CRR,uint16_t limit,bool dir);
void right_wheel_set(uint16_t CRR,uint16_t limit,bool dir);
void motor_forward(uint16_t CRR,uint16_t limit,int FOEWARD_DELAY_MS);
void motor_back(uint16_t CRR,uint16_t limit,int BACK_DELAY_MS );
void motor_turn_left(uint16_t CRR,uint16_t limit,int TURN_DELAY_MS);
void motor_turn_right(uint16_t CRR,uint16_t limit,int TURN_DELAY_MS);
void motor_turn_stop(void);
void motor_forwardss(uint16_t CRR,uint16_t limit );