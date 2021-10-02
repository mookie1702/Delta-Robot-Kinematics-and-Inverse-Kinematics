#pragma once
#include <vector>
using namespace std;

struct PosStruct{
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度
};

class CHLMotionPlan{
private:
	double mJointAngleBegin[4];		// 起始点位的关节角度,单位度
	double mJointAngleEnd[4];		// 结束点位的关节角度，单位度
	double mStartMatrixData[16];	// 起始点位的转换矩阵数组
	double mEndMatrixData[16];		// 结束点位的转换矩阵数组
	double mSampleTime;				// 采样点位，单位S
	double mAngleVel;               // 关节速度，单位°/s
	double mAngleAcc;               // 关节加速度，单位°/s/s
	double mAngleDec;               // 关节减速度，单位°/s/s
	double mVel;					// 速度，单位m/s
	double mAcc;					// 加速度，单位m/s/s
	double mDec;					// 减速度，单位m / s / s
	bool mConfig[3];				// 机器人姿态

public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();
	void SetSampleTime(double sampleTime);							// 设置采样时间
	void SetPlanPoints(PosStruct startPos, PosStruct endPos);		// 输入起始点位和结束点位的笛卡尔坐标
	// 设置运动参数，角速度，角加速度，角减速度，速度，加速度，减速度
	void SetProfile(double AngleVel,double AngleAcc,double AngleDec,double vel, double acc, double dec);
	void GetPlanPoints();											// 关节空间梯形速度规划
	void GetPlanPoints_line();       								// 笛卡尔空间直线轨迹梯形速度规划

};