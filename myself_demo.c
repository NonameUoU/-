#include "common_def.h"
#include "pinctrl.h"
#include "app_init.h"
#include "i2c.h"
#include "osal_debug.h"
#include "gpio.h"
#include "soc_osal.h"
#include "tcxo.h"
#include "motor.h"
#include "systick.h"
#include "servo.h"
#include "timer.h"
#include "chip_core_irq.h"
#include "hal_gpio.h"
#include "watchdog.h"
#include"ssd1306.h"
#include"ssd1306_fonts.h"
#define TRIG_PIN    GPIO_00
#define ECHO_PIN    GPIO_01 
#define BSP_SG92R 2
#define CONFIG_I2C_SCL_MASTER_PIN 15
#define CONFIG_I2C_SDA_MASTER_PIN 16
#define CONFIG_I2C_MASTER_PIN_MODE 2
#define I2C_MASTER_ADDR       0x00
#define I2C_SET_BANDRATE      8000
#define BACK_DELAY_MS   2000
#define TURN_DELAY_MS   800  
#define FOEWARD_DELAY_MS  1000
#define CRR 500 
#define limit  5000 
#define MYSELF_TASK_PRIO          24
#define MYSELF_TASK_STACK_SIZE    0x1000
#define SAFE_DISTANCE  30
#define COUNT 10  
static int myself_task(const char *arg)
{
  unused(arg);
  uapi_pin_set_mode(CONFIG_I2C_SCL_MASTER_PIN, CONFIG_I2C_MASTER_PIN_MODE);  //I2C初始化
  uapi_pin_set_mode(CONFIG_I2C_SDA_MASTER_PIN, CONFIG_I2C_MASTER_PIN_MODE);
  uint32_t baudrate = I2C_SET_BANDRATE;
  uint32_t hscode = I2C_MASTER_ADDR;
  errcode_t ret = uapi_i2c_master_init(1, baudrate, hscode);
  if (ret != 0) {
    printf("i2c init failed, ret = %0x\r\n", ret);
  }
 echoinit();
 S92RInit();
 rgbinit();
 pwm_write(0x16);                                          //马达初始化
 ssd1306_Init();                                         //初始化屏幕
 ssd1306_Fill(Black);                                   //设置显示的背景
 ssd1306_SetCursor(0, 0);                              //设置起始位置
 ssd1306_ClearOLED();                                //清除显示                 
 while(1){
   float dis =  distance();
   if (dis < SAFE_DISTANCE) {
     //小于安全距离亮红灯
     uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_HIGH); 
     uapi_gpio_set_val(GPIO_07, GPIO_LEVEL_LOW);
     uapi_gpio_set_val(GPIO_11, GPIO_LEVEL_LOW);
     motor_turn_stop(); 
     osal_msleep(200);
     float leftDist = scanDirection(150);   //左右测距，寻找出路
     osal_msleep(300);
     float rightDist = scanDirection(30);    
     osal_msleep(300);
     RegressMiddle(); 
     if (leftDist > rightDist && leftDist > SAFE_DISTANCE ) {
       // 左侧距离大于右侧，并大于安全距离：左转
       ssd1306_ClearOLED();  
       ssd1306_printf("go left");
       motor_turn_left(CRR, limit, TURN_DELAY_MS);
      } 
     else if (rightDist > SAFE_DISTANCE ) {
       //右侧距离大于左侧，并大于安全距离：右转
       ssd1306_ClearOLED();  
       ssd1306_printf("go right");
       motor_turn_right(CRR, limit, TURN_DELAY_MS);
      }
     else {
       //左右距离都小于安全距离：后退，亮黄灯
       uapi_gpio_set_val(GPIO_07, GPIO_LEVEL_HIGH);
       uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_LOW);
       uapi_gpio_set_val(GPIO_11, GPIO_LEVEL_LOW);
       ssd1306_ClearOLED();  
       ssd1306_printf("go back");
       motor_back( CRR,limit, BACK_DELAY_MS );
       motor_turn_stop();
      }
    }
    else {
     //大于安全距离，直行，亮绿灯
      uapi_gpio_set_val(GPIO_11, GPIO_LEVEL_HIGH);
      uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_LOW);    
      uapi_gpio_set_val(GPIO_07, GPIO_LEVEL_LOW);
      ssd1306_ClearOLED();
      ssd1306_printf("forward");
      motor_forwardss(CRR,limit);
    }
    osal_msleep(50);
  }
 return 0;
}

static void myself_entry(void)
{
  osal_task *task_handle = NULL;
  osal_kthread_lock();
  task_handle = osal_kthread_create((osal_kthread_handler)myself_task, 0, "MyselfTask", MYSELF_TASK_STACK_SIZE);
  if (task_handle != NULL) {
    osal_kthread_set_priority(task_handle, MYSELF_TASK_PRIO);
    osal_kfree(task_handle);
  }
  osal_kthread_unlock();
}
app_run(myself_entry);