#ifndef CONFIG_H
#define CONFIG_H

/* Private include ------------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_adc.h"
#include "arm_math.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "sweep.h"
#include "shoot.h"
#include "circle.h"
#include "fix.h"           
#include "adc.h"
#include "task.h"
#include "tools.h"
#include "pid.h"
#include "camera.h"
#include "avoid.h"
#include "motor.h"
#include "debug.h"
#include "dma.h"
#include "misc.h"
#include "MotionCard.h"

/* Private include ------------------------------------------------------------------------------------*/
#define PERIOD					      0.01f

/*      车的基本信息      */
//电机旋转一周的脉冲数
#define COUNT_PER_ROUND (4096.0f)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (120.0f)
//调试小车车长（单位：mm）
#define MOVEBASE_LENGTH (492.0f)
//调试小车车宽(单位：mm)
#define MOVEBASE_WIDTH (490.0f)
//轮子宽度（单位：mm）
#define WHEEL_WIDTH (40.0f)
//两个轮子中心距离（单位：mm）
#define WHEEL_TREAD (434.0f)
//宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)
//宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)
//宏定义发射机构航向电机ID
#define GUN_YAW_ID (7)
//宏定义送弹电机ID
#define PUSH_BALL_ID (6)
//宏定义送弹机构送弹时电机位置
#define PUSH_POSITION (4000)
//宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)
//宏定义收球电机ID
#define COLLECT_BALL_ID (8)
//宏定义左轮电机ID
#define LEFT_MOTOR_WHEEL_ID (2)
//宏定义右轮电机ID
#define RIGHT_MOTOR_WHEEL_ID (1)




//车的状态码
#define STATUS_SWEEP   1 //状态 基础扫场
#define STATUS_CAMERA_WALK   2 //状态 摄像头走形
#define STATUS_CAMERA  4 //状态 摄像头
#define STATUS_SHOOTER 8 //状态 射球
#define STATUS_FIX     16 //状态 矫正
#define STATUS_AVOID   32 //状态 避障



/*      车的方向        */
#define GO_STRAIGHT 0 
#define GO_LEFT     1
#define GO_RIGHT    2
#define GO_BACK     3


//和摄像头状态有关的定义
//摄像头发数状态的起始码
#define CAMERA_STATUS_1_START 0xDC
#define CAMERA_STATUS_2_START 0xDA
#define CAMERA_STATUS_3_START 0xD8
#define CAMERA_STATUS_4_START 0xC6
#define CAMERA_STATUS_5_END   0xC5


//蜂鸣器
#define BEEP_ON          		 GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define BEEP_OFF         		 GPIO_ResetBits(GPIOE, GPIO_Pin_7)


//x , y 的最大最小值
#define Y_MIN 0
#define X_MAX 2400
#define X_MIN -2400
#define Y_MAX 4800




#endif

