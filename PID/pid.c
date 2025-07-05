#include "pid.h"
#include "tim.h"
#include "math.h"


#define ABS_CLAMP_MAX(val, abs_max) \
    (fabsf(val) > fabsf(abs_max) ? \
        (copysignf(fabsf(abs_max), val)) : \
        (val))
		
		
float fastest_fabsf(float x) {
    // ��������ʵ��λ������
    union { float f; uint32_t i; } u = { x };
    u.i &= 0x7FFFFFFF;  // �������λ(��31λ)
    return u.f;
}



/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarDisCalibration
  * @brief   ����λ�ñ궨
  * @param   carTar: [����/��] 
**			 carstat: [����/��] 
**			 dispid: [����/��] 
/* -------------------------------- end -------------------------------- */
int CarDisCalibration(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid){
	
	//err
	float err = carTar->Tar_dis-(carstat->CalibrationDis);

	//�ж�
	if((fastest_fabsf(err) <= 0.1f)) return 1;	
	
	__carStatVel_Update(carTar,
		ABS_CLAMP_MAX(__Realize_PID(&dispid->lf,err),(carTar->MaxVel)),
		ABS_CLAMP_MAX(__Realize_PID(&dispid->lb,err),(carTar->MaxVel)),
		ABS_CLAMP_MAX(__Realize_PID(&dispid->rb,err),(carTar->MaxVel)),
		ABS_CLAMP_MAX(__Realize_PID(&dispid->rf,err),(carTar->MaxVel))		
		);

	return 0;
}



/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarSitaCalibration
  * @brief   ����Ƕȱ궨
  * @param   carTar: [����/��] 
**			 carstat: [����/��] 
**			 sitapid: [����/��] 
/* -------------------------------- end -------------------------------- */
int CarSitaCalibration(SplitCarTargetParm* carTar,Car_Stat* carstat,PidWheel* sitapid){
	
	float err = carTar->Tar_dis-(carstat->CalibrationDis);	
	//Ŀ���ٶ�ת��
	float dertavel = __Realize_PID(sitapid,err);
	
	//�ж�
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
  * @brief   �ǶȻ�
  * @param   carTar: [����/��] 
**			 carstat: [����/��] 
**			 dispid: [����/��] 
**/
/* -------------------------------- end -------------------------------- */
int CarSitaSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidWheel* sitapid){
	float err = (carTar->Tar_sita)-(carstat->Sita);
	
	//�ж�
	if(fastest_fabsf(err) < 0.5f){
		carTar->DertaVel=0;//�޲���
		return 1;
	} 

	//Ŀ���ٶ�ת��
	float dertavel = __Realize_PID(sitapid,err);
		
	//printf("sita:%.2f err:%.2f v:%.2f\r\n",carstat->Sita,err,dertavel);
	
	__carStatDertaVel_Update(carTar,ABS_CLAMP_MAX(dertavel,carTar->MaxVel));
	

	
	return 0;
}

/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarDisSet
  * @brief   ����carTar�ĸ��ٶ�ֵ����������ײ��ٶȻ�
  * @param   carTar: [����/��] 
**			 carstat: [����/��] 
 **/
/* -------------------------------- end -------------------------------- */
int CarDisSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid){
	
	//err
	float err = carTar->Tar_dis-(carstat->Dis);

	
	//�ж�
	if((fastest_fabsf(err) <= 0.1f)) return 1;	
	
	__carStatVel_Update(carTar,
		ABS_CLAMP_MAX(__Realize_PID(&dispid->lf,err),(carTar->MaxVel)),
		ABS_CLAMP_MAX(__Realize_PID(&dispid->lb,err),(carTar->MaxVel)),
		ABS_CLAMP_MAX(__Realize_PID(&dispid->rb,err),(carTar->MaxVel)),
		ABS_CLAMP_MAX(__Realize_PID(&dispid->rf,err),(carTar->MaxVel))		
		);

	return 0;
}



/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    WheelVelSet
  * @brief   �ײ��ٶȻ���ִ������carTar��Ŀ���ٶ�
  * @param   carTar: [����/��] 
**			 carstat: [����/��] 
**			 pid: [����/��] 
  * @retval
  * @author  ���
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
@bref: λ��ʽPID������
@para	����ǰλ��	pid->actual_dis
@return: ָ���ٶ�	pid->output_val
**************************************************************************/
int cnt = 0;

float __Realize_PID(PidWheel * pid,float err)
{

	cnt+=1;
	
	pid->err = err;		
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
	
	float P = pid->Kp*pid->err;
	float I = pid->Ki*pid->err_sum;
	float D = pid->Kd*(pid->err - pid->err_last);
	
    I = ABS_CLAMP_MAX(I, Integral_Limit);
	
//	if(cnt%4 ==0 ){
//		
//		//printf("%.2f	%.2f\r\n",pid->err,pid->err_last);
//		printf("%.1f  %.1f  %.1f %.1f  %.1f\r\n",err,P,I,D,(P + I + D));
//		cnt = 0;
//	}
	
	//ʹ��PID���� ��� = Kp*��ǰ���  +  Ki*����ۼ�ֵ + Kd*(��ǰ���-�ϴ����)
	pid->output = P + I + D;	
	//�����ϴ����: �����ֵ���ϴ����
	pid->err_last = pid->err;
		
	return pid->output;
}


/**************************************************************************
�������ܣ�����PID������
��ڲ�����ʵ��ֵ��Ŀ��ֵ
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]		��Ӧ��ϵ������ʽ-��λ��ʽ��kp->kd ki->kp kd->ki
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
**************************************************************************/
float __Incremental_PID(PidWheel * pid,float target,float actual)
{ 		
	//����
	pid->actual = actual;//������ʵֵ 
	pid->target = target;
	pid->err = pid->target - pid->actual;//��ǰ���=Ŀ��ֵ-��ʵֵ   
	
	
	float P = (pid->Kp * pid->err);
	float D = pid->Kd*(pid->err - pid->err_last);
	float I = pid->Ki*(pid->err - 2*pid->err_last + pid->err_pre);
	
	I= ABS_CLAMP_MAX(fastest_fabsf(I),Integral_Limit);
	    	
	pid->output += P+I+D;

	//	pid->output  += (pid->Kd*(pid->err - pid->err_last))               /* �������� */
//									 + (pid->Kp * pid->err)                           /* ���ֻ��� */
//									 + (pid->Ki*(pid->err - 2*pid->err_last + pid->err_pre));  /* ΢�ֻ��� */
	
	
	pid->err_pre=pid->err_last;                                   /* �������ϴ�ƫ�� */
	pid->err_last=pid->err;	                                    /* ������һ��ƫ�� */
	
	
	//printf("%f,%f,%f,%f,%f\r\n",actual,pid->output,P,d,i);

	return pid->output;                                            /* ������ */
}

	
