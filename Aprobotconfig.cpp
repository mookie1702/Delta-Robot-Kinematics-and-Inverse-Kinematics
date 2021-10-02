#include "AProbotconfig.h"
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <iostream>


using namespace std;
using namespace Eigen;

namespace APRobot{
	const double R = 200;	// 定义动平台到电机的距离
	const double r = 45;	// 定义静平台半径
	const double L = 350;	// 定义主动臂长
	const double l = 800;	// 定义从动臂长

	// 初始化TransMatrix
	double mTransMatrix[16] {0};

	// 初始化q
	double theta[6] {0};

	// delta机器人只使用一种姿态
	bool mConfig = 1;

	double norm(double* x) {
		double distance;
		distance = sqrt(pow(x[0],2) + pow(x[1],2) + pow(x[2],2));
		return distance;
	}

	void cross(double *x, double *y, double *result) {
		result[0] = x[1]*y[2] - x[2]*y[1];
		result[1] = x[2]*y[0] - x[0]*y[2];
		result[2] = x[0]*y[1] - x[1]*y[0];
	}

	double LOC(double l1, double l2, double d) {
		double theta;
		theta = acos( (l1*l1 + l2*l2 - d*d)/(2*l1*l2) );
		return theta;
	}


	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll){
		// 求旋转矩阵
		Matrix<double, 3, 3> yaw_m;
		Matrix<double, 3, 3> pitch_m;
		Matrix<double, 3, 3> roll_m;
		yaw = yaw * PI / 180;
		pitch = pitch * PI / 180;
		roll = roll * PI / 180;
		yaw_m << cos(yaw), -sin(yaw),0,
			     sin(yaw),  cos(yaw),0,
			            0,         0,1;
		pitch_m <<  cos(pitch), 0, sin(pitch),
			                 0, 1,          0,
			       -sin(pitch), 0, cos(pitch);
		roll_m << cos(roll), -sin(roll), 0,
			      sin(roll),  cos(roll), 0,
			              0,          0, 1;
		Matrix<double, 3, 3> result = yaw_m * pitch_m * roll_m;

		mTransMatrix[0] = result(0, 0);
		mTransMatrix[1] = result(0, 1);
		mTransMatrix[2] = result(0, 2);
		mTransMatrix[3] = x;
		mTransMatrix[4] = result(1, 0);
		mTransMatrix[5] = result(1, 1);
		mTransMatrix[6] = result(1, 2);
		mTransMatrix[7] = y;
		mTransMatrix[8] = result(2, 0);
		mTransMatrix[9] = result(2, 1);
		mTransMatrix[10] = result(2, 2);
		mTransMatrix[11] = z;
		mTransMatrix[12] = 0;
		mTransMatrix[13] = 0;
		mTransMatrix[14] = 0;
		mTransMatrix[15] = 1;
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4){
		robotBackward(mTransMatrix, mConfig, theta);
		angle1 = theta[0]*180/PI;
		angle2 = theta[1]*180/PI;
		angle3 = theta[2]*180/PI;
		angle4 = theta[3]*180/PI + 180;
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4){
		theta[0] = angle1 * PI / 180;
		theta[1] = angle2 * PI / 180;
		theta[2] = angle3 * PI / 180;
		theta[3] = angle4 * PI / 180;
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll){
		robotForward(theta, mTransMatrix, mConfig);
		x = mTransMatrix[3];
		y = mTransMatrix[7];
		z = mTransMatrix[11];
		double r11 = mTransMatrix[0];
		yaw = 0;
		pitch = 180;
		roll = acos(-1 * r11) * 180 / PI;
	}


	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），
				Scara机器人有2个解，Delta机器人有一个解，因此Delta可以不考虑config
				为了安全，实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, bool mconfig, double* theta){

		double x = TransVector[3];
		double y = TransVector[7];
		double z = TransVector[11];
		theta[0] = 0.5;
		theta[1] = 0.5;
		theta[2] = 0.5;

		// 求位置
		double alpha[3] = {0, 2*PI/3, 4*PI/3};
		double A1[3] = {R*cos(alpha[0]), R*sin(alpha[0]), 0};
		double A2[3] = {R*cos(alpha[1]), R*sin(alpha[1]), 0};
		double A3[3] = {R*cos(alpha[2]), R*sin(alpha[2]), 0};
		double B1[3] = {x + r*cos(alpha[0]), y + r*sin(alpha[0]), z};
		double B2[3] = {x + r*cos(alpha[1]), y + r*sin(alpha[1]), z};
		double B3[3] = {x + r*cos(alpha[2]), y + r*sin(alpha[2]), z};
		double C1[3] = {(R+L*cos(theta[0]))*cos(alpha[0]),(R+L*cos(theta[0]))* sin(alpha[0]),-L*sin(theta[0])};
		double C2[3] = {(R+L*cos(theta[1]))*cos(alpha[1]),(R+L*cos(theta[1]))* sin(alpha[1]),-L*sin(theta[1])};
		double C3[3] = {(R+L*cos(theta[2]))*cos(alpha[2]),(R+L*cos(theta[2]))* sin(alpha[2]),-L*sin(theta[2])};

		double temp[3];
		double A1O[3] = {-A1[0],-A1[1],-A1[2]};
		double AC1[3] = {C1[0]-A1[0], C1[1]-A1[1], C1[2]-A1[2]};
		cross(A1O,AC1,temp);
		double BD1[3] = { temp[0]/norm(temp), temp[1]/norm(temp), temp[2]/norm(temp)};
		double BA1[3] = {A1[0]-B1[0], A1[1]-B1[1], A1[2]-B1[2]};
		double D_BD1 = BD1[0]*BA1[0] + BD1[1]*BA1[1] + BD1[2]*BA1[2];
		double D1[3] = { D_BD1*BD1[0]+B1[0], D_BD1*BD1[1]+B1[1], D_BD1*BD1[2]+B1[2]};
		// cout << D1[0] << ", " << D1[1] << ", " << D1[2] << endl;
		double AD1[3] = {D1[0]-A1[0],D1[1]-A1[1],D1[2]-A1[2]};
		double D_AD1 = norm(AD1);
		double D_CD1 = sqrt(l*l-D_BD1*D_BD1);
		double DAO1 = LOC(norm(A1),D_AD1,norm(D1));
		double DAC1 = LOC(L, D_AD1, D_CD1);

		double A2O[3] = {-A2[0],-A2[1],-A2[2]};
		double AC2[3] = {C2[0]-A2[0], C2[1]-A2[1], C2[2]-A3[2]};
		cross(A2O,AC2,temp);
		double BD2[3] = { temp[0] / norm(temp), temp[1] / norm(temp), temp[2] / norm(temp) };
		double BA2[3] = { A2[0] - B2[0], A2[1] - B2[1], A2[2] - B2[2]};
		double D_BD2 = BD2[0] * BA2[0] + BD2[1] * BA2[1] + BD2[2] * BA2[2];
		double D2[3] = { D_BD2 * BD2[0] + B2[0], D_BD2 * BD2[1] + B2[1], D_BD2 * BD2[2] + B2[2] };
		// cout << D2[0] << ", " << D2[1] << ", " << D2[2] << endl;
		double AD2[3] = { D2[0] - A2[0],D2[1] - A2[1],D2[2] - A2[2]};
		double D_AD2 = norm(AD2);
		double D_CD2 = sqrt(l * l - D_BD2 * D_BD2);
		double DAO2 = LOC(norm(A2), D_AD2, norm(D2));
		double DAC2 = LOC(L, D_AD2, D_CD2);

		double A3O[3] = {-A3[0],-A3[1],-A3[2]};
		double AC3[3] = {C3[0]-A3[0], C3[1]-A3[1], C3[2]-A3[2]};
		cross(A3O,AC3,temp);
		double BD3[3] = { temp[0] / norm(temp), temp[1] / norm(temp), temp[2] / norm(temp) };
		double BA3[3] = { A3[0] - B3[0], A3[1] - B3[1], A3[2] - B3[2]};
		double D_BD3 = BD3[0] * BA3[0] + BD3[1] * BA3[1] + BD3[2] * BA3[2];
		double D3[3] = { D_BD3 * BD3[0] + B3[0], D_BD3 * BD3[1] + B3[1], D_BD3 * BD3[2] + B3[2] };
		// cout << D3[0] << ", " << D3[1] << ", " << D3[2] << endl;
		double AD3[3] = { D3[0] - A3[0],D3[1] - A3[1],D3[2] - A3[2] };
		double D_AD3 = norm(AD3);
		double D_CD3 = sqrt(l * l - D_BD3 * D_BD3);
		double DAO3 = LOC(norm(A3), D_AD3, norm(D3));
		double DAC3 = LOC(L, D_AD3, D_CD3);

		theta[0] = PI-DAO1-DAC1;
		theta[1] = PI-DAO2-DAC2;
		theta[2] = PI-DAO3-DAC3;

		// 求角度
		Matrix<double, 3, 3> result;
		result << TransVector[0], TransVector[1], TransVector[2],
			      TransVector[4], TransVector[5], TransVector[6],
			      TransVector[8], TransVector[9], TransVector[10];
		double roll1 = 0;
		double pitch1 = PI;
		Matrix<double, 3, 3> roll_1;
		Matrix<double, 3, 3> pitch_1;
		Matrix<double, 3, 3> origin;
		roll_1 << cos(roll1), -sin(roll1), 0,
				  sin(roll1), cos(roll1), 0,
			             0, 0, 1;
		pitch_1 << cos(pitch1), 0, sin(pitch1),
							 0, 1, 0,
			      -sin(pitch1), 0, cos(pitch1);
		origin = roll_1.inverse() * pitch_1.inverse() * result;
		theta[3] = acos(origin(0,0));
	}

	/********************************************************************
	ABSTRACT:	机器人正运动学

	INPUTS:		theta[6]: 6个关节角, 单位为弧度

	OUTPUTS:	config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米

	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* theta, double* TransVector, bool mconfig){
		// 求ZYZ坐标
		double alpha[3] = {0, 2*PI/3, 4*PI/3};
		double C1[3] = {(R-r+L*cos(theta[0]))*cos(alpha[0]),(R-r+L*cos(theta[0]))* sin(alpha[0]),-L*sin(theta[0])};
		double C2[3] = {(R-r+L*cos(theta[1]))*cos(alpha[1]),(R-r+L*cos(theta[1]))* sin(alpha[1]),-L*sin(theta[1])};
		double C3[3] = {(R-r+L*cos(theta[2]))*cos(alpha[2]),(R-r+L*cos(theta[2]))* sin(alpha[2]),-L*sin(theta[2])};
		double C12[3] = {C2[0]-C1[0],C2[1]-C1[1],C2[2]-C1[2]};
		double C23[3] = {C3[0]-C2[0],C3[1]-C2[1],C3[2]-C2[2]};
		double C13[3] = {C3[0]-C1[0],C3[1]-C1[1],C3[2]-C1[2]};
		double C31[3] = {-C13[0],-C13[1],-C13[2]};
		double R12 = norm(C12);
		double R23 = norm(C23);
		double R31 = norm(C31);
		double p = (R12 + R23 + R31) / 2;
		double D_EC = (R12*R23*R31)/(4*sqrt(p*(p-R12)*(p-R23)*(p-R31)));
		double F[3] = {(C1[0]+C2[0])/2, (C1[1]+C2[1])/2, (C1[2]+C2[2])/2};
		double temp[3] {0};
		double result[3]{ 0 };
		cross(C23, C31, temp);
		cross(temp, C12, result);
		double FE[3] = { result[0] / norm(result), result[1] / norm(result), result[2] / norm(result) };
		double D_FE = sqrt(pow(D_EC,2)-pow(R12,2)/4);
		double E[3] = {D_FE*FE[0]+F[0], D_FE*FE[1]+F[1], D_FE*FE[2]+F[2]};
		cross(C12, C23, result);
		double result1[3] = {-result[0], -result[1], -result[2]};
		double EP[3] = {result1[0]/norm(result), result1[1]/norm(result), result1[2]/norm(result)};
		double D_EP = sqrt(pow(l,2)-pow(D_EC,2));
		double P[3] = {E[0]+D_EP*EP[0], E[1]+D_EP*EP[1], E[2]+D_EP*EP[2]};

		// 求ZYZ角
		double origin_yaw = 0;
		double origin_pitch = PI;
		double origin_roll = PI;
		Matrix<double, 3, 3> yaw_m;
		Matrix<double, 3, 3> pitch_m;
		Matrix<double, 3, 3> roll_m;
		yaw_m << cos(origin_yaw), -sin(origin_yaw), 0,
			sin(origin_yaw), cos(origin_yaw), 0,
			0, 0, 1;
		pitch_m << cos(origin_pitch), 0, sin(origin_pitch),
			0, 1, 0,
			-sin(origin_pitch), 0, cos(origin_pitch);
		roll_m << cos(origin_roll), -sin(origin_roll), 0,
			sin(origin_roll), cos(origin_roll), 0,
			0, 0, 1;
		Matrix<double, 3, 3> origin = yaw_m * pitch_m * roll_m;
		double veried_roll = theta[3];
		roll_m << cos(veried_roll), -sin(veried_roll), 0,
			sin(veried_roll), cos(veried_roll), 0,
			0, 0, 1;
		Matrix<double, 3, 3> result2 = roll_m*origin;
		TransVector[0] = result2(0, 0);
		TransVector[1] = result2(0, 1);
		TransVector[2] = result2(0, 2);
		TransVector[3] = P[0];
		TransVector[4] = result2(1, 0);
		TransVector[5] = result2(1, 1);
		TransVector[6] = result2(1, 2);
		TransVector[7] = P[1];
		TransVector[8] = result2(2, 0);
		TransVector[9] = result2(2, 1);
		TransVector[10] = result2(2, 2);
		TransVector[11] = P[2];
		TransVector[12] = 0;
		TransVector[13] = 0;
		TransVector[14] = 0;
		TransVector[15] = 1;
	}
}
