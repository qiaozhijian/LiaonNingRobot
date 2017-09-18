#include "config.h"
float bx_measure[4][];
float by_measure[4][];
float cx_measure[4][];
float cy_measure[4][];
/*************************白球测量表*********************/
//先测
/*
0:左激光,1:右激光,2:角度,3:距离
*/

bx_measure[0][]={686f, 976f ,1317f ,1629f ,1909f,2405f,2688f,2854f,3048f,3283f,
								3548f,3727f,4046f};
bx_measure[1][]={4151f ,3583f ,3523f ,3204f ,3008f,2408f,2120f,1959f,1767f,1536f,
								1721f,1090f,770f};
bx_measure[2][]={71.60f ,69.84f ,67.80f ,66.04f ,65.78f,65.23f,65.90f,66.93f,67.28f,69.69f,
								70.40f,71.18f,73.04f};
bx_measure[3][]={39.39f ,30.88f ,24.46f ,16.17f ,9.70f,-2.6f,-8.7f,-13.5f,-15.61f,-21.67f,
								-26.77f,-29.81f,-36.09f};

wx_measure[0][]={683f,982f,1315f,1625f,1910f,2794f,2958f,3158f,3339f,3530f,3801f,3992f,4090f,};
wx_measure[1][]={4147f,3865f,3514f,3195f,2919f,2030f,1868f,1665f,1487f,1304f,
								1028f,833f,723f,};
wx_measure[2][]={74.52,72.00f,69.65f,66.34f,66.81f,66.03f,66.63f,66.02f,66.55f,69.37f,
								69.37f,70.09f,70.90f};
wx_measure[3][]={41.53f,36.75f,29.95f,24.34f,17.28f,-4.19f,-7.55f,-11.47f,-15.84f,-21.04f,
								-26.90f,-30.63f,-32.46f};
	
by_measure[0][]={};
by_measure[1][]={};
by_measure[2][]={};
by_measure[3][]={};
	
wy_measure[0][]={};
wy_measure[1][]={};
wy_measure[2][]={};
wy_measure[3][]={};

Shoot_t shoot;
float lookup(float leftadc,float rightadc,Measure_t * measure)
{
	for (int i = 0; i <DATALENGTH; i++)
	{
		if (measure[i].leftadc / measure[i].rightadc<float(leftadc / rightadc) && float(leftadc / rightadc) < measure[i + 1].leftadc / measure[i + 1].rightadc);
		{
			shoot.angle = (measure[i + 1].angle - measure[i].angle) / ((measure[i + 1].leftadc / measure[i + 1].rightadc) - (measure[i].leftadc / measure[i].rightadc));
			shoot.speed= (measure[i + 1].speed - measure[i].speed) / ((measure[i + 1].leftadc / measure[i + 1].rightadc) - (measure[i].leftadc / measure[i].rightadc));
		}
	return shoot;
}
//二分法查表
float lookup(float leftadc,float rightadc,Measure_t * measure)
{
	for (int i = 0; i <DATALENGTH; i++)
	{
		if()
	}
	2400+(Laser_Disleft-Laser_Disright)/2;
}
/****************************************************************************
* 名    称：lookup(void)
* 功    能：主扫场控制程序
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int BinarySearch(float *array, float T)
{
	int low, high, mid;
	low = 0;
	high = length - 1;
	while (low <= high)
	{
		mid = (low + high) / 2;
		if (array[mid] < T)
		{
			low = mid + 1;
		}
		else if (array[mid]>T)
		{
			high = mid - 1;
		}
		else
		{
			return mid;
		}
	}
	return mid;
}


