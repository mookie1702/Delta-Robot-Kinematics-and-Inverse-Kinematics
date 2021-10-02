#include <iostream>
#include "MotionPlan.h"

using namespace std;

/* ʵ����: �켣�滮
 * Ҫ ��ʹ��C/C++ ��������ٶȹ滮������data.txt�ļ�
 * �滮���ͣ��ؽڿռ䡢�ѿ����ռ䣨ֱ�ߣ�
 * ��λ��������ʼ�����ֹ��
 * ����������Vel��Acc��Dec
 */

int main(){
    // ��ʼ��
    PosStruct Start;
    Start.x = 21.741; Start.y = 185.457; Start.z = -560.968;
    Start.yaw = 0; Start.pitch = 180; Start.roll = 22.190;

    // ��ֹ��
    PosStruct End;
    End.x = -175.239; End.y = 78.127; End.z = -600.793;
    End.yaw = 0; End.pitch = 180; End.roll = 18.325;

    // �����ٶȹ滮
    CHLMotionPlan trajectory1;
    trajectory1.SetPlanPoints(Start, End);
    trajectory1.SetProfile(10,10,10,10,10,10);  // vel ��/s�� acc ��/s.s, dec ��/s.s
    trajectory1.SetSampleTime(0.001);           // s
    trajectory1.GetPlanPoints();                // �ؽڿռ������ٶȹ滮
    trajectory1.GetPlanPoints_line();           // �ѿ����ռ�ֱ�߹켣�����ٶȹ滮
}
