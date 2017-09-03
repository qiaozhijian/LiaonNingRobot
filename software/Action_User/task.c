#include "stm32f4xx.h"
#include "usart.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "math.h"
#include "stm32f4xx_usart.h"
#include "arm_math.h"
#include "config.h"
#include "task.h"
extern Robot_t gRobot;

void robotInit()
{
}

/**
*设置机器人的状态
* 入口参数 status :
*   STATUS_SWEEP 
*   STATUS_CAMERA_WALK 
*   STATUS_CAMERA
*   STATUS_SHOOTER 
*   STATUS_FIX 
*   STATUS_AVOID
*   如果同时设置多个状态，那么各个状态之间按位或
*/
void setRobotStatus(uint8_t status)
{
    gRobot.status = status;
}
/**
*   入口参数    direction 车的方向 
*   GO_STRAIGHT
*   GO_LEFT
*   GO_RIGHT
*   GO_BACK
*/
void setDirection(uint8_t direction)
{
    gRobot.direction = direction;
}
//
void setPosition(Position_t *pos)
{
    gRobot.pos.x = pos->x;
    gRobot.pos.y = pos->y;
    gRobot.pos.angle = pos->angle;
}

void setLeftMove()
{
}

void setRightMove()
{
}
