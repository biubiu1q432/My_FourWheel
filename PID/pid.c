#include "pid.h"
#include "tim.h"
#include "math.h"
		
		
#define ABS_CLAMP(val, max_abs) \
    ((val) > (max_abs) ? (max_abs) : \
    ((val) < -(max_abs) ? -(max_abs) : (val)))
		
		
float fastest_fabsf(float x) {
    // ��������ʵ��λ������
    union { float f; uint32_t i; } u = { x };
    u.i &= 0x7FFFFFFF;  // �������λ(��31λ)
    return u.f;
}


float roundToTwoDecimals(float num) {
    return roundf(num * 100) / 100;  // �ȳ�100���������룬�ٳ�100
}


/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    CarDisCalibration
  * @brief   ����λ�ñ궨
  * @param   carTar: [����/��] 
**			 carstat: [����/��] 
**			 dispid: [����/��] 
/* -------------------------------- end -------------------------------- */
int CarDisCalibrationStable(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid){
	
	float lf,lb,rb,rf,err;
	
	//err
	err =  (carstat->SideLidarCaliDis) - carTar->Tar_dis;
		
	//�ж�
	if((fastest_fabsf(err) <= 1)) return 1;	
	
	else{
	
		
		if (fastest_fabsf(err) >= 300.0f) {
			float speed = err > 0 ? 20.0f : -20.0f;
			// �����������ȫ��ǰ��
			lf = lb = rb = rf = speed;
		} 
		
		else if (fastest_fabsf(err) >= 200.0f) {
			// �е���������е��ٶ�
			float speed = err > 0 ? 15.0f : -15.0f;
			lf = lb = rb = rf = speed;
		}
		
		else if (fastest_fabsf(err) >= 150.0f) {
			// �е���������е��ٶ�
			float speed = err > 0 ? 10.0f : -10.0f;
			lf = lb = rb = rf = speed;
		}
		
		else if (fastest_fabsf(err) >= 90.0f) {
			// �е���������е��ٶ�
			float speed = err > 0 ? 8.0f : -8.0f;
			lf = lb = rb = rf = speed;
		}
		
		else if (fastest_fabsf(err) >= 60.0f) {
			// С�����������
			float speed = err > 0 ? 5.0f : -5.0f;
			lf = lb = rb = rf = speed;
		}
	
		//pid����
		else{
			lf  = __Realize_PID(&dispid->lf,err);
			lb	=__Realize_PID(&dispid->lb,err);
			rb	=__Realize_PID(&dispid->rb,err);
			rf	=__Realize_PID(&dispid->rf,err);
		}
	
		
		//printf("%d,%.1f,%.1f\r\n",carstat->SideLidarCaliDis,err,lf);
		
		
		__carStatVel_Update(carTar,
		ABS_CLAMP(lf,(carTar->MaxVel)),
		ABS_CLAMP(lb,(carTar->MaxVel)),
		ABS_CLAMP(rb,(carTar->MaxVel)),
		ABS_CLAMP(rf,(carTar->MaxVel))		
		);

	}

	return 0;
}


int CarDisCalibration(SplitCarTargetParm* carTar,Car_Stat* carstat,PidCar* dispid){
	
	float lf,lb,rb,rf,err;
	
	//err
	err = (carstat->FrontLidarCaliDis)-carTar->Tar_dis;
		
	//�ж�
	if((fastest_fabsf(err) <= 0.02f)) return 1;	
	
	else{
	
		if (fastest_fabsf(err) >= 10.0f) {
			float speed = err > 0 ? 15.0f : -15.0f;
			// �����������ȫ��ǰ��
			lf = lb = rb = rf = speed;
		} 
		
		else if (fastest_fabsf(err) > 5.0f) {
			// С�����������
			float speed = err > 0 ? 13.0f : -13.0f;
			lf = lb = rb = rf = speed;
		}
		
		else if (fastest_fabsf(err) > 1.0f) {
			// С�����������
			float speed = err > 0 ? 10.0f : -10.0f;
			lf = lb = rb = rf = speed;
		}
		
		
		else{
			lf  = __Realize_PID(&dispid->lf,err);
			lb	=__Realize_PID(&dispid->lb,err);
			rb	=__Realize_PID(&dispid->rb,err);
			rf	=__Realize_PID(&dispid->rf,err);
		}
		
		
		__carStatVel_Update(carTar,

		ABS_CLAMP(lf,(carTar->MaxVel)),
		ABS_CLAMP(lb,(carTar->MaxVel)),
		ABS_CLAMP(rb,(carTar->MaxVel)),
		ABS_CLAMP(rf,(carTar->MaxVel))		
		);

	}

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
	
	
	float err = carTar->Tar_dis-(carstat->SideLidarCaliDis);	
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
int cnt = 0;

float calculateAngleError(float target, float current) {
    // 1. ����ԭʼ���
    float error = target - current;
    
    // 2. �����淶��[-180, 180]����
    error = fmodf(error, 360.0f);
    if (error < -180.0f) {
        error += 360.0f;
    } else if (error > 180.0f) {
        error -= 360.0f;
    }
    
    // 3. ����߽��������179.9�㵽-180������䣩
    if (fabsf(error) > 179.9f) {
        // ѡ�����·��
        error = (error > 0) ? (error - 360.0f) : (error + 360.0f);
    }
    
    return error;
}

int CarSitaSet(SplitCarTargetParm* carTar,Car_Stat* carstat,PidWheel* sitapid){
	float err = calculateAngleError(carTar->Tar_sita,carstat->Sita);
	float derta_val = 0;
	
	//�ж�
	if(fastest_fabsf(err) < 0.5f){
		derta_val=0;//�޲���
		__carStatDertaVel_Update(carTar,derta_val);
		
		return 1;
	}
	
	//�ֶ�
	else{
		
		if(err<=-20){
			derta_val = -60;
		}
		
		else if(err>=20){
			derta_val = 60;
		}
		
		//pid
		else{
			derta_val = __Realize_PID(sitapid,err);
		}
		
		//printf("%.2f,%.2f,%.2f\r\n",carstat->Sita,carTar->Tar_sita,derta_val);

				
		__carStatDertaVel_Update(carTar,ABS_CLAMP(derta_val,carTar->MaxVel));
		return 0;
	
	}
		
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
		ABS_CLAMP(__Realize_PID(&dispid->lf,err),(carTar->MaxVel)),
		ABS_CLAMP(__Realize_PID(&dispid->lb,err),(carTar->MaxVel)),
		ABS_CLAMP(__Realize_PID(&dispid->rb,err),(carTar->MaxVel)),
		ABS_CLAMP(__Realize_PID(&dispid->rf,err),(carTar->MaxVel))		
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
	

	float lf_output = __Incremental_PID(&pidvel->lf,(carTar->Tar_LFvel),(carstat->motorStat.LeftFrt.vel));
	float lb_output = __Incremental_PID(&pidvel->lb,(carTar->Tar_LBvel),(carstat->motorStat.leftBack.vel));
	float rf_output = __Incremental_PID(&pidvel->rf,(carTar->Tar_RFvel),(carstat->motorStat.RigFrt.vel));
	float rb_output = __Incremental_PID(&pidvel->rb,(carTar->Tar_RBvel),(carstat->motorStat.RigBack.vel));
	
	
	//printf("%.2f,%.2f,%.2f\r\n",(carTar->Tar_RBvel),(carstat->motorStat.RigBack.vel),rb_output);


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
extern PidCar pidvel;
extern PidCar piddis;

float __Realize_PID(PidWheel * pid,float err)
{

	
	pid->err =err;

	
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
	
	float P = pid->Kp*pid->err;
	float I = pid->Ki*pid->err_sum;
	float D = pid->Kd*(pid->err - pid->err_last);    
	
    I = ABS_CLAMP(I, Integral_Limit);
	
	//ʹ��PID���� ��� = Kp*��ǰ���  +  Ki*����ۼ�ֵ + Kd*(��ǰ���-�ϴ����)
	pid->output = P + I + D;	
	//�����ϴ����: �����ֵ���ϴ����
	pid->err_last = pid->err;
	
	//printf("%.2f,%.2f,%.2f\r\n",P,I,D,);

		
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
	
	
	
	float P = pid->Kp * (pid->err - pid->err_last);
	float I = pid->Ki * pid->err; 
	float D = pid->Kd * (pid->err - 2*pid->err_last + pid->err_pre);
	
		    	
	pid->output += P+I+D;
	I = ABS_CLAMP(I, Integral_Limit);

	
	pid->err_pre=pid->err_last;                                   /* �������ϴ�ƫ�� */
	pid->err_last=pid->err;	                                    /* ������һ��ƫ�� */
	
	
//	if(pid==&pidvel.rb){
//		printf("%.2f,%.2f\r\n",target,actual);
//	}
//	
	
	return pid->output;                                            /* ������ */
}

	
