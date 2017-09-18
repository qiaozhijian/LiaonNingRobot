#ifndef CAMERA_H
#define CAMERA_H
#include "config.h"
	//全场区域检查
	void AreaCheck(float x, float y);
	//摄像头走形
	void CameraBaseWalk3(void);
	//单个区域的检查
	int CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY);
	//摄像头数据处理
	void Sub_Box(void);
	void SetTurnTimeChange(int temp);
	int GetTurnTimeChange(void);
void CheckOutline3(void);

#endif
