#include "pinctrl.h"
#include "common_def.h"
#include "soc_osal.h"
#include "gpio.h"
#include "hal_gpio.h"
#include "systick.h"
#include "watchdog.h"
#include "app_init.h"

void SetAngle(unsigned int angle);
void RegressMiddle(void);
void servoTurnRight(void);
void servoTurnLeft(void);
void S92RInit(void);
float distance(void);
 float scanDirection(unsigned int  angle);
void echoinit(void);
void rgbinit(void);
void recordStep(int dx, int dy);
void printMazePath();
void recordStepBasedOnDirection() ;