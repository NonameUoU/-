#include "pinctrl.h"
#include "soc_osal.h"
#include "app_init.h"
#include "motor.h"
#include "i2c.h"
#include "osal_timer.h"
//向pwm写入数据
void pwm_write(uint8_t reg_data){
    uint8_t buffer[] = {reg_data};
    i2c_data_t data = {0};
    data.send_buf = buffer;
    data.send_len = sizeof(buffer);
    errcode_t ret = uapi_i2c_master_write(I2C_MASTER_BUS_ID, 0x5A, &data);
    if (ret != 0) {
        printf("pwm_write: failed, %0X!\n", ret);
        return ;
    }
    printf("pwm_write: success! address:0x5A\r\n");
}
//向pwm写入数据
void pwm_writes(uint8_t* reg_data){
    i2c_data_t data = {0};
    data.send_buf = reg_data;
    data.send_len = sizeof(reg_data);
    errcode_t ret = uapi_i2c_master_write(I2C_MASTER_BUS_ID, 0x5A, &data);
    if (ret != 0) {
        printf("pwm_write: failed, %0X!\n", ret);
        return ;
    }
    printf("pwm_write: success! address:0x5A\r\n");
}
//设置右轮马达
void right_wheel_set(uint16_t CRR, uint16_t limit, bool dir) {
    if(CRR > limit) {
        CRR = limit;
    }
    uint8_t CRRH = (CRR >> 8) & 0xFF;
    uint8_t CRRL = CRR & 0xFF;
    if(dir) {
        uint8_t PWM1CH1[3] = {0x70, 0x00, 0x00};
        uint8_t PWM1CH2[3] = {0x80, CRRH, CRRL};
        pwm_writes(PWM1CH1);
        pwm_writes(PWM1CH2);
    } 
    else {
        uint8_t PWM1CH2[3] = {0x80, 0x00, 0x00};
        uint8_t PWM1CH1[3] = {0x70, CRRH, CRRL};
        pwm_writes(PWM1CH1);
        pwm_writes(PWM1CH2);
    }
}
//设置左轮马达
void left_wheel_set(uint16_t CRR, uint16_t limit, bool dir) {
    if(CRR > limit) {
        CRR = limit;
    }
    uint8_t CRRH = (CRR >> 8) & 0xFF;
    uint8_t CRRL = CRR & 0xFF;
    if(dir) {
        uint8_t PWM1CH2[3] = {0xA0, 0x00, 0x00};
        uint8_t PWM1CH1[3] = {0x90, CRRH, CRRL};
        pwm_writes(PWM1CH1);
        pwm_writes(PWM1CH2);
    } 
    else {
        uint8_t PWM1CH1[3] = {0x90, 0x00, 0x00};
        uint8_t PWM1CH2[3] = {0xA0, CRRH, CRRL};
        pwm_writes(PWM1CH1);
        pwm_writes(PWM1CH2);
    }
}
 //停止转动
void motor_turn_stop(void)
{   
right_wheel_set(0, 0,false ) ;
left_wheel_set(0, 0,true); 
}   

//持续前进
void motor_forwardss(uint16_t CRR,uint16_t limit )
{  
right_wheel_set(CRR,limit, true) ;
left_wheel_set(CRR,limit, true); 
}  
//前进一段时间
void motor_forward(uint16_t CRR,uint16_t limit,int FOEWARD_DELAY_MS )
{  
right_wheel_set(CRR,limit, true) ;
left_wheel_set(CRR,limit, true); 
osal_msleep(FOEWARD_DELAY_MS);
motor_turn_stop();
}
////后退一段时间
 void motor_back(uint16_t CRR,uint16_t limit,int BACK_DELAY_MS )
{  
right_wheel_set(CRR,limit,false ) ;
left_wheel_set(CRR,  limit, false); 
osal_msleep(BACK_DELAY_MS);
motor_turn_stop();
} 
////左转一段时间
void motor_turn_left(uint16_t CRR,uint16_t limit,int TURN_DELAY_MS    )
{  
right_wheel_set(CRR,limit,true ) ;
left_wheel_set(CRR,limit,false); 
osal_msleep(TURN_DELAY_MS );
motor_turn_stop();
} 
//右转一段时间
 void motor_turn_right(uint16_t CRR,uint16_t limit,int TURN_DELAY_MS)
{ 
right_wheel_set(CRR,limit,false ) ;
left_wheel_set(CRR,limit,true); 
osal_msleep(TURN_DELAY_MS );
motor_turn_stop();
}
 