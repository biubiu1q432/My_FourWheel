#include "pid.h"
#include "tim.h"
#include "math.h"


#define CLAMP_MAX(x, max) ((x) > (max) ? (max) : (x))
float fastest_fabsf(float x) {
    // 联合类型实现位级操作
    union { float f; uint32_t i; } u = { x };
    u.i &= 0x7FFFFFFF;  // 清除符号位(第31位)
    return u.f;
}



/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarDisCalibration
  * @brief   车体位置标定
  * @param   carTar: [输入/出] 
**			 carstat: [输入/出] 
**			 dispid: [输入/出] 
/* -------------------------------- end -------------------------------- */
int CarDisCalibration(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid){
	
	//err
	float err = carTar->Tar_dis-(carstat->CalibrationDis);

	//判定
	if((fastest_fabsf(err) <= 0.1f)) return 1;	

	
	__carStatVel_Update(carTar,
		CLAMP_MAX(__Realize_PID(&dispid->lf,err),(carTar->MaxVel)),
		CLAMP_MAX(__Realize_PID(&dispid->lb,err),(carTar->MaxVel)),
		CLAMP_MAX(__Realize_PID(&dispid->rb,err),(carTar->MaxVel)),
		CLAMP_MAX(__Realize_PID(&dispid->rf,err),(carTar->MaxVel))		
		);

	return 0;
}



/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarSitaCalibration
  * @brief   车体角度标定
  * @param   carTar: [输入/出] 
**			 carstat: [输入/出] 
**			 sitapid: [输入/出] 
/* -------------------------------- end -------------------------------- */
int CarSitaCalibration(SplitCarTargetParm* carTar,Car_Stat* carstat,PidWheel* sitapid){
	
	float err = carTar->Tar_dis-(carstat->CalibrationDis);	
	//目标速度转换
	float dertavel = __Realize_PID(sitapid,err);
	
	//判定
	if(fastest_fabsf(err) < 1.0f){
		dertavel=0;
		return 1;
	} 
	__carStatDertaVel_Update(carTar,dertavel);
	return 0;
}



/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarSitaSet
  * @brief   角度环
  * @param   carTar: [输入/出] 
**			 carstat: [输入/出] 
**			 dispid: [输入/出] 
**/
/* -------------------------------- end -------------------------------- */
//extern Car_Stat carStat;
//extern CarOrderParam OrderParam;
//extern SplitCarTargetParm Cartar;
int CarSitaSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidWheel* sitapid){
	float err = (carTar->Tar_sita)-(carstat->Sita);
	
	
	//目标速度转换
	float dertavel = __Realize_PID(sitapid,err);
		
	//判定
	if(fastest_fabsf(err) < 0.5f){
		dertavel=0;
		return 1;
	} 
	
	if(carTar->MaxVel!=0)__carStatDertaVel_Update(carTar,CLAMP_MAX(dertavel,carTar->MaxVel));
	if(carTar->MaxVel ==0)__carStatDertaVel_Update(carTar,dertavel);
	
	//printf("err:%f	v:%f\r\n",err,CLAMP_MAX(dertavel,carTar->MaxVel));

	return 0;
}

/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarDisSet
  * @brief   更新carTar四个速度值，后续传入底层速度环
  * @param   carTar: [输入/出] 
**			 carstat: [输入/出] 
 **/
/* -------------------------------- end -------------------------------- */
int CarDisSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid){
	
	//err
	float err = carTar->Tar_dis-(carstat->Dis);

	
	//判定
	if((fastest_fabsf(err) <= 0.1f)) return 1;	
	
	
	__carStatVel_Update(carTar,
		CLAMP_MAX(__Realize_PID(&dispid->lf,err),(carTar->MaxVel)),
		CLAMP_MAX(__Realize_PID(&dispid->lb,err),(carTar->MaxVel)),
		CLAMP_MAX(__Realize_PID(&dispid->rb,err),(carTar->MaxVel)),
		CLAMP_MAX(__Realize_PID(&dispid->rf,err),(carTar->MaxVel))		
		);

	return 0;
}



/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    WheelVelSet
  * @brief   底层速度环，执行来自carTar的目标速度
  * @param   carTar: [输入/出] 
**			 carstat: [输入/出] 
**			 pid: [输入/出] 
  * @retval
  * @author  楠瘦
  * @Data    2025-05-25
 **/
/* -------------------------------- end -------------------------------- */



void WheelVelSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* pidvel){
	
	__carStat_Update(carTar);
	
	//printf("%f,%f\r\n",(carTar->Tar_LFvel),(carstat->motorStat.LeftFrt.vel));
	
	
	float lf_output = __Incremental_PID(&pidvel->lf,(carTar->Tar_LFvel),(carstat->motorStat.LeftFrt.vel));
	float lb_output = __Incremental_PID(&pidvel->lb,(carTar->Tar_LBvel),(carstat->motorStat.leftBack.vel));
	float rf_output = __Incremental_PID(&pidvel->rf,(carTar->Tar_RFvel),(carstat->motorStat.RigFrt.vel));
	float rb_output = __Incremental_PID(&pidvel->rb,(carTar->Tar_RBvel),(carstat->motorStat.RigBack.vel));
		
	__PWM_MotorSet(lf_output,lb_output,rf_output,rb_output);
}


void PidMlpi_Param_Init(PidCar* pidcar,float kp,float ki,float kd){
	pidcar->lf.Kp = kp;
	pidcar->lf.Ki= ki;
	pidcar->lf.Kd = kd;
	
	pidcar->lb.Kp = kp;
	pidcar->lb.Ki= ki;
	pidcar->lb.Kd = kd;
	
	pidcar->rf.Kp = kp;
	pidcar->rf.Ki= ki;
	pidcar->rf.Kd = kd;
	
	pidcar->rb.Kp = kp;
	pidcar->rb.Ki= ki;
	pidcar->rb.Kd = kd;
}


void PidParam_Init(PidWheel* pid,float kp,float ki,float kd){
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

/**************************************************************************
@bref: 位置式PID控制器
@para	：当前位置	pid->actual_dis
@return: 指定速度	pid->output_val
**************************************************************************/

float __Realize_PID(PidWheel * pid,float err)
{

	pid->err = err;		
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->output = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);	
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;
		
	return pid->output;
}


/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值，目标值
返回  值：电机PWM
根据增量式离散PID公式 
pwm=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]		对应关系（增量式-》位置式）kp->kd ki->kp kd->ki
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
**************************************************************************/
float __Incremental_PID(PidWheel * pid,float target,float actual)
{ 		
	//计算
	pid->actual = actual;//传递真实值 
	pid->target = target;
	pid->err = pid->target - pid->actual;//当前误差=目标值-真实值   
	
	pid->output  += (pid->Kd*(pid->err - pid->err_last))               /* 比例环节 */
									 + (pid->Kp * pid->err)                           /* 积分环节 */
									 + (pid->Ki*(pid->err - 2*pid->err_last + pid->err_pre));  /* 微分环节 */ 
    
	pid->err_pre=pid->err_last;                                   /* 保存上上次偏差 */
	pid->err_last=pid->err;	                                    /* 保存上一次偏差 */
	
	//	float P = (pid->Kp * pid->err);
	//	float d = pid->Kd*(pid->err - pid->err_last);
	//	float i = pid->Ki*(pid->err - 2*pid->err_last + pid->err_pre);
	//printf("%f,%f,%f,%f,%f\r\n",actual,pid->output,P,d,i);

	return pid->output;                                            /* 输出结果 */
}

	
