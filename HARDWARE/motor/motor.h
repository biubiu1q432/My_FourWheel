#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"


typedef struct
{
    float dis;
    float odom;
    float vel;
} Singel_Motor_Stat;//�����������

typedef struct
{
    Singel_Motor_Stat LeftFrt;
    Singel_Motor_Stat leftBack;
    Singel_Motor_Stat RigFrt;
    Singel_Motor_Stat RigBack;
} All_Motor_Stat;//���е������


typedef struct
{	
	float Dis;//�ݴ�
	float Odom;//ȫ��
    float Vel;
    float Omeiga;

    float Sita;//��ԽǶ�
	float AbsSita;//���ԽǶ�
	float baseSita;//��׼�Ƕ� 
	float FrontLidarCaliDis;//�궨
	uint16_t SideLidarCaliDis;//�궨


	All_Motor_Stat motorStat;
} Car_Stat;//��������


typedef struct
{
	float Order_MAXvel;
    float Order_vel;
	float Order_omeiga;
	float Order_dis;
	float Order_sita;//Ŀ��Ƕ�

} CarOrderParam;//��λ������


typedef struct
{
	float MaxVel;
	float DertaVel;
	float LFvel;
	float LBvel;
    float RFvel;
	float RBvel;
	
	float Tar_LFvel;//�ֽ�
	float Tar_LBvel;
    float Tar_RFvel;
	float Tar_RBvel;

	float Tar_dis;
	float Tar_sita;

} SplitCarTargetParm;//�ֽ���Ŀ��ֵ

void __carStat_Update(SplitCarTargetParm* carTar);
void __carStatVel_Update(SplitCarTargetParm* carTar,float lf,float lb,float rf,float rb);
void __carStatDis_Update(SplitCarTargetParm* carTar,float CarDIS);
void __carStatOmeiga_Update(SplitCarTargetParm* carTar,float CarOmeiga);
void __carStatDertaVel_Update(SplitCarTargetParm* carTar,float Car_val);

//Ŀ��ֵ����
void Refresh_Car(SplitCarTargetParm* carTar,CarOrderParam* orderparam);
//��������
void Refresh_CarDis(Car_Stat* carstat);
//PWM����
void __PWM_MotorSet(float LeftFrt,float leftBack,float RigFrt,float RigtBack);
//���������ݻ�ȡ
int __MoterParamGet(TIM_HandleTypeDef* tim);
void Split_CarTarParam(SplitCarTargetParm* carTar,CarOrderParam* orderparam);

//Ӳ����ʼ��
void Motor_init(void);
//�˶�ѧ���
void CarInv_Kinematics(Car_Stat* carstat);


#endif


