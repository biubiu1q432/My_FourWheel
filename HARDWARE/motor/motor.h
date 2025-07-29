#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"


typedef struct
{
    float dis;
    float odom;
    float vel;
} Singel_Motor_Stat;//单个电机参数

typedef struct
{
    Singel_Motor_Stat LeftFrt;
    Singel_Motor_Stat leftBack;
    Singel_Motor_Stat RigFrt;
    Singel_Motor_Stat RigBack;
} All_Motor_Stat;//所有电机参数


typedef struct
{	
	float Dis;//暂存
	float Odom;//全程
    float Vel;
    float Omeiga;

    float Sita;//相对角度
	float AbsSita;//绝对角度
	float baseSita;//基准角度 
	float FrontLidarCaliDis;//标定
	uint16_t SideLidarCaliDis;//标定


	All_Motor_Stat motorStat;
} Car_Stat;//整车参数


typedef struct
{
	float Order_MAXvel;
    float Order_vel;
	float Order_omeiga;
	float Order_dis;
	float Order_sita;//目标角度

} CarOrderParam;//上位机命令


typedef struct
{
	float MaxVel;
	float DertaVel;
	float LFvel;
	float LBvel;
    float RFvel;
	float RBvel;
	
	float Tar_LFvel;//分解
	float Tar_LBvel;
    float Tar_RFvel;
	float Tar_RBvel;

	float Tar_dis;
	float Tar_sita;

} SplitCarTargetParm;//分解后的目标值

void __carStat_Update(SplitCarTargetParm* carTar);
void __carStatVel_Update(SplitCarTargetParm* carTar,float lf,float lb,float rf,float rb);
void __carStatDis_Update(SplitCarTargetParm* carTar,float CarDIS);
void __carStatOmeiga_Update(SplitCarTargetParm* carTar,float CarOmeiga);
void __carStatDertaVel_Update(SplitCarTargetParm* carTar,float Car_val);

//目标值重置
void Refresh_Car(SplitCarTargetParm* carTar,CarOrderParam* orderparam);
//距离重置
void Refresh_CarDis(Car_Stat* carstat);
//PWM设置
void __PWM_MotorSet(float LeftFrt,float leftBack,float RigFrt,float RigtBack);
//编码器数据获取
int __MoterParamGet(TIM_HandleTypeDef* tim);
void Split_CarTarParam(SplitCarTargetParm* carTar,CarOrderParam* orderparam);

//硬件初始化
void Motor_init(void);
//运动学逆解
void CarInv_Kinematics(Car_Stat* carstat);


#endif


