#ifndef CAMERA_H
#define CAMERA_H
#include "config.h"
typedef struct {
	
	int turnTime;
	int circleChangeSymbol;//记住circleChangeSymbol当摄像头找球搞得换圈的时候的时候进行清空turnTimeChange
	
}CameraBaseWalk3Par_t;

	//全场区域检查
	CameraBaseWalk3Par_t AreaCheck(float x, float y);
	//摄像头走形
	void CameraBaseWalk3(void);
	//单个区域的检查
	int CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY);
	//摄像头数据处理
	void Sub_Box(void);
	void SetTurnTimeChange(int temp);
	int GetTurnTimeChange(void);


#endif
