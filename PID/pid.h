#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "motor.h"

//pid¼ÆËãÆ÷
typedef struct 
{
	float actual;
	float target;
	float err;
	float err_last;
	float err_pre;
	float err_sum;
	float output;
	float Kp,Ki,Kd;	
} PidWheel;


typedef struct 
{
	PidWheel lf;
	PidWheel lb;
	PidWheel rf;
	PidWheel rb;
} PidCar;



	
float __Realize_PID(PidWheel * pid,float err);
float __Incremental_PID(PidWheel * pid,float target,float actual);
void PidParam_Init(PidWheel* pid,float kp,float ki,float kd);

void PidMlpi_Param_Init(PidCar* pidvel,float kp,float ki,float kd);

void WheelVelSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* pidvel);

int CarDisSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid);
int CarSitaSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidWheel* sitapid);
int CarDisCalibration(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid);
int CarSitaCalibration(SplitCarTargetParm* carTar,Car_Stat* carstat,PidWheel* sitapid);

#endif


