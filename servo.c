#include "pinctrl.h"
#include "common_def.h"
#include "soc_osal.h"
#include "gpio.h"
#include "hal_gpio.h"
#include "systick.h"
#include "watchdog.h"
#include "app_init.h"
#include "tcxo.h"
#include "i2c.h"
#include "motor.h"
#define COUNT 10         // 通过计算舵机转到对应角度需要发送10个左右的波形
#define BSP_SG92R 2
#define FREQ_TIME 20000  // 20ms周期 (50Hz)
#define MIN_PULSE 500    // 对应0度的脉冲宽度(微秒)
#define MAX_PULSE 2500   // 对应180度的脉冲宽度(微秒)
#define TRIG_PIN    GPIO_00
#define ECHO_PIN    GPIO_01 

void SetAngle(unsigned int angle)
{
    unsigned int pulse = MIN_PULSE + (angle * (MAX_PULSE - MIN_PULSE)) / 180; //将角度改为对应脉冲宽度（微秒）
    unsigned int time = FREQ_TIME;
    uapi_gpio_set_val(BSP_SG92R, GPIO_LEVEL_HIGH);  //舵机转动至对应角度
    uapi_systick_delay_us(pulse);
    uapi_gpio_set_val(BSP_SG92R, GPIO_LEVEL_LOW);
    uapi_systick_delay_us(time - pulse);
}

//利用echo测距
float distance(void)
 {
   uapi_gpio_set_val(TRIG_PIN, GPIO_LEVEL_LOW);//先拉低电平 2μs（确保触发信号干净）
   osal_msleep(2);
    uapi_gpio_set_val(TRIG_PIN, GPIO_LEVEL_HIGH);//再拉高 10μs（传感器要求的最小触发时间）
   osal_msleep(10);
    uapi_gpio_set_val(TRIG_PIN, GPIO_LEVEL_LOW); // 最后恢复低电平
    while(uapi_gpio_get_val(ECHO_PIN) == GPIO_LEVEL_LOW); //等待传感器从 ECHO 引脚返回高电平脉冲
    uint64_t startTime = uapi_systick_get_ms();    //记录开始时间
    while(uapi_gpio_get_val(ECHO_PIN) == GPIO_LEVEL_HIGH);//等待传感器从 ECHO 引脚返回低电平脉冲
    uint64_t travelTime =  uapi_systick_get_ms()- startTime; //往返时间
  float distances = (travelTime*34.4)/2;  //换算距离
  return  distances;
 }

 //转动舵机并测距
 float scanDirection(unsigned int  angle) {
    for (int i = 0; i < COUNT; i++) {
      SetAngle(angle) ;
    }
  osal_msleep(500);  // 等待舵机到位
  float a= distance();
  return a;
} 
// 舵机居中(90度)
void RegressMiddle(void) 
{                                                                                                                                                                                                                                                                 
    for (int i = 0; i < COUNT; i++) {
        SetAngle(90);
    }
}
// 舵机右转(0度方向)
void servoTurnRight(void)
{
    for (int i = 0; i < COUNT; i++) {
        SetAngle(0);
    }
}
// 舵机左转(180度方向)
void servoTurnLeft(void)
{
    for (int i = 0; i < COUNT; i++) {
        SetAngle(180);
    }
}
// 舵机初始化
void S92RInit(void)
{
    uapi_pin_set_mode(BSP_SG92R, 0);
    uapi_gpio_set_dir(BSP_SG92R, GPIO_DIRECTION_OUTPUT);
        uapi_gpio_set_val(BSP_SG92R, GPIO_LEVEL_LOW);
}
 //初始化超声波传感器
void echoinit(void)
{
uapi_gpio_init();                 
uapi_pin_set_mode(TRIG_PIN, 0);
uapi_pin_set_mode(ECHO_PIN, 0);
uapi_gpio_set_dir(TRIG_PIN, GPIO_DIRECTION_OUTPUT);
uapi_gpio_set_dir(ECHO_PIN, GPIO_DIRECTION_INPUT);
}

//初始化RGB灯
void rgbinit(void)
{
 uapi_pin_set_mode(GPIO_07, 0);
uapi_gpio_set_dir(GPIO_07, GPIO_DIRECTION_OUTPUT);
uapi_gpio_set_val(GPIO_07, GPIO_LEVEL_LOW);
uapi_pin_set_mode(GPIO_09, 0);
uapi_gpio_set_dir(GPIO_09, GPIO_DIRECTION_OUTPUT);
uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_LOW);
uapi_pin_set_mode(GPIO_11, 0);
uapi_gpio_set_dir(GPIO_11, GPIO_DIRECTION_OUTPUT);
uapi_gpio_set_val(GPIO_11, GPIO_LEVEL_LOW);
}