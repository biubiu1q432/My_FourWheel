#include "motor.h"
#include "lowpass_filter.h"

#define LIMIT(val) ((val) > Arr ? Arr : ((val) < -Arr ? -Arr : (val)))

#define SPEED_LIMIT  10.0f  // 下限幅绝对值阈值

//inline void clamp_speed_bid(float* speed) {
//    
//	if(*speed >){
//		if (*speed < SPEED_LIMIT) {
//        *speed = SPEED_LIMIT;  // 正向超限 → 赋上限值
//		} 

//	}
//	
//	else{
//		 if (*speed > -SPEED_LIMIT) {
//        *speed = -SPEED_LIMIT; // 负向超限 → 赋下限值（保留符号）
//		}
//	}
//	

//}

void __carStat_Update(SplitCarTargetParm* carTar){
	
	carTar->Tar_LFvel =	carTar->LFvel - carTar->DertaVel;
	carTar->Tar_LBvel = carTar->LBvel - carTar->DertaVel;
	carTar->Tar_RFvel = carTar->RFvel + carTar->DertaVel;
	carTar->Tar_RBvel = carTar->RBvel + carTar->DertaVel;
	
		
}

/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    __carStatDertaVel_Update
  * @brief  TARvel = vel + dertaVel
  * @param   carTar: [输入/出] 
**			 Car_val: [输入/出] 
--------------------------- end -------------------------------- */
void __carStatDertaVel_Update(SplitCarTargetParm* carTar,float Car_val){
	carTar->DertaVel = Car_val;
//	clamp_speed_bid(&carTar->DertaVel);
//	printf("%f\r\n",carTar->DertaVel);
}


//速度环更新
void __carStatVel_Update(SplitCarTargetParm* carTar,float lf,float lb,float rf,float rb){
	
	carTar->LFvel = lf;
	carTar->LBvel = lb;
	carTar->RFvel = rf;
	carTar->RBvel = rb;
}

//位置环更新
void __carStatDis_Update(SplitCarTargetParm* carTar,float CarDIS){
	carTar->Tar_dis = CarDIS;
}


void Motor_init(){
   
   //编码器初始化
   HAL_TIM_Encoder_Start(&TIM_MOTOR_A_EC,TIM_CHANNEL_ALL);
   HAL_TIM_Encoder_Start(&TIM_MOTOR_B_EC,TIM_CHANNEL_ALL);
   HAL_TIM_Encoder_Start(&TIM_MOTOR_C_EC,TIM_CHANNEL_ALL);
   HAL_TIM_Encoder_Start(&TIM_MOTOR_D_EC,TIM_CHANNEL_ALL);
   //电机初始化
	HAL_TIM_PWM_Start(&TIM_MOTOR_A_1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM_MOTOR_A_2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM_MOTOR_B,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM_MOTOR_B,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM_MOTOR_C,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM_MOTOR_C,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM_MOTOR_D,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM_MOTOR_D,TIM_CHANNEL_4);
}


//目标值重置
void Refresh_Car(SplitCarTargetParm* carTar,CarOrderParam* orderparam){

	carTar->LFvel = 0;
	carTar->LBvel = 0;
	carTar->RFvel = 0;
	carTar->RBvel = 0;

	carTar->Tar_sita = 0;
	carTar->Tar_dis =0;
	carTar->MaxVel = Def_Max_Vel;
	orderparam->Order_dis=0;
	orderparam->Order_omeiga=0;
	orderparam->Order_vel=0;
	orderparam->Order_sita=0;
	
}



//距离重置
void Refresh_CarDis(Car_Stat* carstat){
	
	carstat->motorStat.LeftFrt.dis = 0;
	carstat->motorStat.leftBack.dis = 0;
	carstat->motorStat.RigFrt.dis = 0;
	carstat->motorStat.RigBack.dis = 0;
	carstat->Dis = 0;
	carstat->FrontLidarCaliDis =0;
	carstat->SideLidarCaliDis=0;
	
	
}


/* -------------------------------- begin  -------------------------------- */
/**
  * @Name    Split_CarTarParam
  * @brief   运动学正解，把orderparam分解到carTar中
  * @param   更新Tar_LBvel
  * @param   更新Tar_LFvel
  * @param   更新Tar_RFvel
  * @param   更新Tar_RBvel，Tar_dis，MaxVel
 **/
/* -------------------------------- end -------------------------------- */

void Split_CarTarParam(SplitCarTargetParm* carTar,CarOrderParam* orderparam){
	
	float deritaVel = (orderparam->Order_omeiga)*WheelToCenter;
	
	carTar->LBvel = (orderparam->Order_vel) + deritaVel;
	carTar->LFvel =(orderparam->Order_vel) + deritaVel;
	carTar->RFvel = (orderparam->Order_vel)-deritaVel;
	carTar->RBvel =(orderparam->Order_vel)-deritaVel;
	
	carTar->Tar_dis = (orderparam->Order_dis);
	carTar->Tar_sita = orderparam->Order_sita;
	
	if(orderparam->Order_MAXvel)carTar->MaxVel = orderparam->Order_MAXvel;
	else carTar->MaxVel  = Def_Max_Vel;


}

/**
 * @description 逆运动学，车体运动状态存在catstat
 */
extern LOWPASS_FILTER_T velFilter;
void CarInv_Kinematics(Car_Stat* carstat){
	
	//编码器
	int lf = __MoterParamGet(&TIM_MOTOR_A_EC);
	int lb = __MoterParamGet(&TIM_MOTOR_B_EC);
	int rf = __MoterParamGet(&TIM_MOTOR_C_EC);
	int rb = __MoterParamGet(&TIM_MOTOR_D_EC);
	
	


	//距离
	float lf_dis = (lf/OnceEcNum)*wheelCircumference;
	float lb_dis = (lb/OnceEcNum)*wheelCircumference;
	float rf_dis = (rf/OnceEcNum)*wheelCircumference;
	float rb_dis = (rb/OnceEcNum)*wheelCircumference;

	
	//速度
	float lf_vel = lf_dis/Ts; 
	float lb_vel = lb_dis/Ts;
	float rf_vel = rf_dis/Ts;
	float rb_vel = rb_dis/Ts;
	float LeftVel = (lf_vel+lb_vel)/2.0f;
	float RigVel = (rf_vel+rb_vel)/2.0f;
	
	//printf("%d,%f\r\n",rb,rb_vel);


	//电机
	carstat->motorStat.LeftFrt.vel = lf_vel;
	carstat->motorStat.leftBack.vel = lb_vel;
	carstat->motorStat.RigFrt.vel = rf_vel;
	carstat->motorStat.RigBack.vel = rb_vel;
	
	carstat->motorStat.LeftFrt.odom += lf_dis;
	carstat->motorStat.leftBack.odom += lb_dis;
	carstat->motorStat.RigFrt.odom += rf_dis;
	carstat->motorStat.RigBack.odom += rb_dis;
	
	carstat->motorStat.LeftFrt.dis += lf_dis;
	carstat->motorStat.leftBack.dis += lb_dis;
	carstat->motorStat.RigFrt.dis += rf_dis;
	carstat->motorStat.RigBack.dis += rb_dis;
	

	//车体
	carstat->Vel =  (LeftVel+RigVel)/2.0f;//车速度
	carstat->Omeiga  = (RigVel-LeftVel)/WheelToCenter;//车角速度
	carstat ->Odom += (lf_dis+lb_dis+rf_dis+rb_dis)/4.0f;//车里程计
	
	carstat ->Dis += (lf_dis+lb_dis+rf_dis+rb_dis)/4.0f;
	
		
	
}
#include <stdbool.h>  
int __MoterParamGet(TIM_HandleTypeDef* tim){
	int tmp = __HAL_TIM_GET_COUNTER(tim);

	__HAL_TIM_SET_COUNTER(tim,0);
	if(tmp > 5000) tmp -= 10000; 
	return tmp;


//    int cnt = __HAL_TIM_GET_COUNTER(tim);
//    bool is_downcounting = __HAL_TIM_IS_TIM_COUNTING_DOWN(tim); // 使用HAL标准宏替代寄存器操作[5,8](@ref)
//    __HAL_TIM_SET_COUNTER(tim, 0); // 复位计数器
//	
//	if(cnt == 0) return cnt;
//	
//	else{
//		if(is_downcounting) 
//			return cnt - TIM_EC_ARR;
//		
//		else{
//			return cnt;
//		
//		}
//	
//	}
//	

	
}

void __PWM_MotorSet(float LeftFrt,float leftBack,float RigFrt,float RigtBack){
	
	
	
	LeftFrt = LIMIT(LeftFrt);
    leftBack = LIMIT(leftBack);
    RigFrt = LIMIT(RigFrt);
    RigtBack = LIMIT(RigtBack);
	
	
	//printf("%f	%f	%f	%f\r\n",LeftFrt,leftBack,RigFrt,RigtBack);
	
	
	/*A*/
	if(LeftFrt <= 0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_A_2,TIM_CHANNEL_1,Arr);
		__HAL_TIM_SetCompare(&TIM_MOTOR_A_1,TIM_CHANNEL_1,Arr+LeftFrt);		
	} 
	else if(LeftFrt >0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_A_1,TIM_CHANNEL_1,Arr);
		__HAL_TIM_SetCompare(&TIM_MOTOR_A_2,TIM_CHANNEL_1,Arr-LeftFrt);
	} 

	/*B*/
	if(leftBack <= 0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_B,TIM_CHANNEL_1,Arr);
		__HAL_TIM_SetCompare(&TIM_MOTOR_B,TIM_CHANNEL_2,leftBack+Arr);		
	} 
	else if(leftBack >0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_B,TIM_CHANNEL_2,Arr);
		__HAL_TIM_SetCompare(&TIM_MOTOR_B,TIM_CHANNEL_1,Arr-leftBack);
	} 
	
	/*C*/
	if(RigFrt <= 0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_C,TIM_CHANNEL_2,Arr);

		__HAL_TIM_SetCompare(&TIM_MOTOR_C,TIM_CHANNEL_1,RigFrt+Arr);		
	} 
	else if(RigFrt >0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_C,TIM_CHANNEL_1,Arr);

		__HAL_TIM_SetCompare(&TIM_MOTOR_C,TIM_CHANNEL_2,Arr-RigFrt);
	} 
	
	/*D*/
	if(RigtBack <= 0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_D,TIM_CHANNEL_3,Arr);

		__HAL_TIM_SetCompare(&TIM_MOTOR_D,TIM_CHANNEL_4,RigtBack+Arr);		
	} 
	else if(RigtBack >0){
		//方向
		__HAL_TIM_SetCompare(&TIM_MOTOR_D,TIM_CHANNEL_4,Arr);

		__HAL_TIM_SetCompare(&TIM_MOTOR_D,TIM_CHANNEL_3,Arr-RigtBack);
	} 
}



