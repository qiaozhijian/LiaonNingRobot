#include "Bspline.h"
#include "math.h"
#include "ringBuffer.h"
#include <stdlib.h>
#include "calculate.h"
#include "posSystem.h"

/*********************************************************************************
* @name 	CaculateBsplineLen
* @brief    用于计算两点确定的一条B样条曲线的长度
* @param	point1 起始点				单位 mm
* @param	point2 结束点				单位 mm
* @param	angle1 起始点方向    单位 度
* @param	angle2 结束点方向    单位 度
* @retval   返回该样条曲线长度  单位 mm
*********************************************************************************/
float CaculateBsplineLen(Point_t point1, Point_t point2, float angle1, float angle2)
{

		//常数项
		Point_t P[2];
		//插值点
		//Point_t *passPoint = NULL;
		//系数矩阵的逆矩阵
		float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
		//控制到达切矢方向的提前量
		float length = 0.0f;
		//零时控制点坐标
		Point_t dataPoint[2];
		//最终控制点坐标
		Point_t finalDataPoint[6];
		Point_t tempPoint;
		Point_t tempPointOld;
		int i, j;
		//该B样条曲线长度
		float u, b1, b2, b3, b0;



		length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

		//对常数项赋值    angle加90是为了处理陀螺仪角度与坐标系角度不符
		P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
		P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
		P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
		P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

		//清零
		for (i = 0; i < 2; i++)
		{
			dataPoint[i].x = 0;
			dataPoint[i].y = 0;
		}


		//d = inv(a).*P  求临时控制点坐标
		for (i = 0; i < 2; i++)
		{
			for (j = 0; j < 2; j++)
			{
				dataPoint[i].x += invA[i][j] * P[j].x;
				dataPoint[i].y += invA[i][j] * P[j].y;
			}
		}

		//复制最终控制点坐标
		finalDataPoint[0] = point1;
		finalDataPoint[1] = point1;
		finalDataPoint[2] = dataPoint[0];
		finalDataPoint[3] = dataPoint[1];
		finalDataPoint[4] = point2;
		finalDataPoint[5] = point2;


		length = 0.0f;
		//将B样条上第一点到首端点的值加上
		tempPointOld = point1;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 25; j++)
			{
				u = (float)(j)* 0.04f;
				b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
				b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
				b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
				b3 = 1.0f / 6 * u * u * u;

				tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
				tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

				length += CalculatePoint2PointDistance(tempPoint,tempPointOld);

				tempPointOld = tempPoint;
			}
		}
		//将B样条上最后一点到末端点的距离加上
		length += CalculatePoint2PointDistance(tempPoint, point2);
		
		return length;
	}


	
	
/*********************************************************************************
* @name 	SerchVirtualPoint
* @brief  根据距离起点的路径长度确定所对应的虚拟点
* @param	robotLen 距离起点的路径长度  单位 mm 
* @retval 返回 虚拟点坐标 和相应在所在样条曲线上的比例系数
*********************************************************************************/
PointU_t SerchVirtualPoint(float robotLen)
{
	int i = 0;
	PointU_t result;

	//确定目标长度在哪一段上
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//确定落在第i-1 到 i 上的距离
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	if((i-2) > 0)
	{
			DeleteData(i-2);
	}
	return result;

}



/*********************************************************************************
* @name 	SerchBsplineVirtualPoint
* @brief  根据距离所在样条曲线长度确定所对应的虚拟点
* @param	robotLen 距离所在样条曲线长度  单位 mm
* @retval 返回 虚拟点坐标 和相应在所在样条曲线上的比例系数
*********************************************************************************/
PointU_t SerchBsplineVirtualPoint(Point_t point1, Point_t point2, float angle1, float angle2,float robotlen)
{
	//常数项
	Point_t P[2];
	//系数矩阵的逆矩阵
	float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
	//控制到达切矢方向的提前量
	float length = 0.0f;
	//零时控制点坐标
	Point_t dataPoint[2];
	//最终控制点坐标
	Point_t finalDataPoint[6];
	Point_t tempPoint;
	Point_t tempPointOld;
	int i, j;
	//该B样条曲线长度
	float u, b1, b2, b3, b0;
	PointU_t result;



	length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

	//对常数项赋值    angle加90是为了处理陀螺仪角度与坐标系角度不符
	P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
	P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
	P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
	P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

	//清零
	for (i = 0; i < 2; i++)
	{
		dataPoint[i].x = 0;
		dataPoint[i].y = 0;
	}


	//d = inv(a).*P  求临时控制点坐标
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			dataPoint[i].x += invA[i][j] * P[j].x;
			dataPoint[i].y += invA[i][j] * P[j].y;
		}
	}


	//复制最终控制点坐标
	finalDataPoint[0] = point1;
	finalDataPoint[1] = point1;
	finalDataPoint[2] = dataPoint[0];
	finalDataPoint[3] = dataPoint[1];
	finalDataPoint[4] = point2;
	finalDataPoint[5] = point2;


	length = 0.0f;
	//将B样条上第一点到首端点的值加上
	tempPointOld = point1;
	int num = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 50; j++)
		{
			//用于计算U值
			num++;

			u = (float)(j)* 0.02f;

			b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
			b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
			b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
			b3 = 1.0f / 6 * u * u * u;

			tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
			tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

			length += CalculatePoint2PointDistance(tempPoint, tempPointOld);
			if (robotlen < length)
			{
				result.point = tempPoint;
				result.u = (float)num / 150.0f;
				return result;
			}

			tempPointOld = tempPoint;
		}
	}

	//将B样条上最后一点到末端点的距离加上
	result.point = point2;

	result.u = 1.0f;

	return result;
}

//确定虚拟目标点
PointU_t SerchVirtualPoint2(float robotLen)
{
	int i = 0;
	PointU_t result;

	//确定目标长度在哪一段上
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//确定落在第i-1 到 i 上的距离
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	return result;

}




int xxx;
int yyy;
/***************************************************************************
* @name		BspSegment
* @brief	把传入的B样条的关键点分段成15cm一段的若干有切线方向的点
* @num		传入的点个数
* @points	传入的B样条关键点
* @keyPoint	分段后存入的点
**************************************************************************/
int BspSegment(int num, Pose_t* points, KeyPointInf_t* keyPoint)
{
	//系数矩阵对角列
	float* a = NULL;
	a = (float *)malloc(sizeof(float)* num);
	//系数矩阵对角上列
	float* b = NULL;
	b = (float *)malloc(sizeof(float)* num);
	//系数矩阵对角下列
	float* c = NULL;
	c = (float *)malloc(sizeof(float)* num);
	//定义soluctionX、soluctionY为线性方程的解
	float* soluctionX = NULL;
	soluctionX = (float *)malloc(sizeof(float)* num);
	float* soluctionY = NULL;
	soluctionY = (float *)malloc(sizeof(float)* num);
	//定义dataX和dataY,用来存放inPoint里的X和Y坐标
	float* dataX = NULL;
	dataX = (float *)malloc(sizeof(float)* num);
	float* dataY = NULL;
	dataY = (float *)malloc(sizeof(float)* num);

	//定义controlPoint用来存放控制点
	Point_t* controlPoint = NULL;
	controlPoint = (Point_t*)malloc(sizeof(Point_t)*(num + 4));
	//存放画线的两个使用点
 	Point_t lines[2];

	//初始化 a,b,c
	a[0] = 18;
	a[num - 1] = 18;

	for (int i = 1; i < num - 1; i++)
	{
		a[i] = 4;
	}

	for (int i = 1; i < num - 1; i++)
	{
		b[i] = 1;
		c[i] = 1;
	}

	c[num - 1] = -9;
	b[0] = -9;



	for (int i = 0; i<num; i++)
	{
		dataX[i] = 6.0f * points[i].point.x;
		dataY[i] = 6.0f * points[i].point.y;
	}

	dataX[0] *= 1.5f;
	dataY[0] *= 1.5f;
	dataX[num - 1] *= 1.5f;
	dataY[num - 1] *= 1.5f;






	//计算outdataX,outdataY;

	//调用Matrix用追赶法求解线性方程
	Matrix(dataX, num, a, b, c, soluctionX);
	Matrix(dataY, num, a, b, c, soluctionY);



	controlPoint[num + 3].x = dataX[num - 1] / 9;
	controlPoint[num + 2].x = dataX[num - 1] / 9;
	controlPoint[0].x = dataX[0] / 9;
	controlPoint[1].x = dataX[0] / 9;


	for (int i = 0; i < num; i++)
	{
		controlPoint[i + 2].x = soluctionX[i];

	}

	controlPoint[num + 3].y = dataY[num - 1] / 9;
	controlPoint[num + 2].y = dataY[num - 1] / 9;
	controlPoint[0].y = dataY[0] / 9;
	controlPoint[1].y = dataY[0] / 9;


	for (int i = 0; i < num; i++)
	{
		controlPoint[i + 2].y = soluctionY[i];

	}



	//计算型值点，分成很近的点，取15cm一段
	//从初始点开始
	lines[0].x = (int)controlPoint[0].x;
	lines[0].y = (int)controlPoint[0].y;

	//累积的长度存入length
	float length = 0;
	float Len = 0;
	float startLength = 0;
	float endLength = 0;

	//计算分段之后的点的个数，然后分配内存空间
	int number = 0;


	for (int i = 0; i < num + 1; i++)
	{
		for (float u = 0.01f; u <= 1; u += 0.01f)
		{
			float b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
			float b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
			float b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
			float b3 = 1.0f / 6 * u * u * u;

			lines[1].x = (int)(b0 * controlPoint[i].x + b1 * controlPoint[i + 1].x + b2 * controlPoint[i + 2].x + b3 * controlPoint[i + 3].x);
			lines[1].y = (int)(b0 * controlPoint[i].y + b1 * controlPoint[i + 1].y + b2 * controlPoint[i + 2].y + b3 * controlPoint[i + 3].y);

			//gra.DrawLine(pen, lines[0], lines[1]);
			length += CalculatePoint2PointDistance(lines[0], lines[1]);
			if ( i <= 1)
				startLength += CalculatePoint2PointDistance(lines[0], lines[1]);
			if (i >= num -1)
				endLength += CalculatePoint2PointDistance(lines[0], lines[1]);
			if (length -Len > 149.25f)
			{
				Len = length;
				number++;
			}

			lines[0] = lines[1];
		}
	}
//	keyPoint = (Act3WheelPose *)malloc(sizeof(ActLine2)* number);
	int resultNum = number + 1;
	
	Len = 0;
	length = 0;
	
	lines[0].x = (int)controlPoint[0].x;
	lines[0].y = (int)controlPoint[0].y;
	
	

	number = 1;
	for (int i = 0; i < num + 1; i++)
	{

		for (float u = 0.01f; u <= 1; u += 0.01f)
		{
			float b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
			float b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
			float b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
			float b3 = 1.0f / 6 * u * u * u;

			lines[1].x = (int)(b0 * controlPoint[i].x + b1 * controlPoint[i + 1].x + b2 * controlPoint[i + 2].x + b3 * controlPoint[i + 3].x);
			lines[1].y = (int)(b0 * controlPoint[i].y + b1 * controlPoint[i + 1].y + b2 * controlPoint[i + 2].y + b3 * controlPoint[i + 3].y);

			//gra.DrawLine(pen, lines[0], lines[1]);
			length += CalculatePoint2PointDistance(lines[0], lines[1]);
			if (length - Len > 149)
			{
				Len = length;
				float Db0 = -1.0f / 2 * (u - 1) * (u - 1);
				float Db1 = 1.0f / 2 * (3 * u * u - 4 * u);
				float Db2 = 1.0f / 2 * (-3 * u * u + u * 2 + 1);
				float Db3 = 1.0f / 2 * u * u;
				
				//切线方向
				keyPoint[number].angle = CHANGE_TO_ANGLE * atan2f((float)(Db0 * controlPoint[i].y + Db1 * controlPoint[i + 1].y + Db2 * controlPoint[i + 2].y + Db3 * controlPoint[i + 3].y), \
					(float)(Db0 * controlPoint[i].x + Db1 * controlPoint[i + 1].x + Db2 * controlPoint[i + 2].x + Db3 * controlPoint[i + 3].x));


				xxx = keyPoint[number].point.x = lines[1].x;
				yyy = keyPoint[number].point.y = lines[1].y;

				
					
//				//姿态角度
//				if (num == 2)
//				{
//					keyPoint[number].poseAngle =CalculateAngleAdd( points[0].direction, CalculateAngleSub(points[1].direction,points[0].direction) / sumLen * length);
//				}
//				else 
//				{
//					if (i > 1 && i < num - 1)
//					{
//						keyPoint[number].poseAngle =CalculateAngleAdd( points[i - 1].direction , u * CalculateAngleSub(points[i].direction,points[i - 1].direction));
//					}
//					if (i <= 1)
//					{
//						keyPoint[number].poseAngle = CalculateAngleAdd(points[0].direction , CalculateAngleSub(points[1].direction,points[0].direction) / startLength * length);
//					}
//					if (i >= num - 1)
//					{
//						keyPoint[number].poseAngle = CalculateAngleAdd(points[num - 2].direction , CalculateAngleSub(points[num - 1].direction,points[num - 2].direction) * (length - beforeLength) / endLength);
//					}
//				}

				number++;
			}

			lines[0] = lines[1];
		}
	}
	
	
	keyPoint[0].point = GetPosPresent().point;
	keyPoint[0].angle = GetPosPresent().direction;
	keyPoint[0].poseAngle = keyPoint[1].poseAngle;
	keyPoint[resultNum].point = points[num-1].point;
	keyPoint[resultNum].angle = keyPoint[resultNum - 1].angle;
	keyPoint[resultNum].poseAngle = points[num - 1].direction;
	resultNum++;
	
	free(a);
	free(b);
	free(c);
	free(soluctionX);
	free(soluctionY);
	free(dataX);
	free(dataY);
	free(controlPoint);
	return resultNum;
}



