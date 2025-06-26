/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "usart.h"
#include "mpu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/*mpu*/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
char mpu_tmp;
float yaw_pre;
volatile int mpuFlag=1;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static float MpuYawInit(int cnt);
static float normalizeYaw(float yaw);
static void BaseSitaUpdate(Car_Stat* carStat,SplitCarTargetParm* Cartar);


/*lidar*/
u16 receive_cnt;
u8 confidence;
u16 distance,noise,reftof;
u32 peak,intg;
u8 dis;

LidarPointTypedef Pack_Data[12];
LidarPointTypedef Pack_sum;
uint8_t Lidar_Receive_buf[1];          
LidarPointTypedef Pack_Data[12];
LidarPointTypedef Pack_sum;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


LOWPASS_FILTER_T velFilter;
Car_Stat carStat;
CarOrderParam OrderParam;
SplitCarTargetParm Cartar;
enum  MoveTaskNum TASKNUM;

PidCar pidvel;
PidCar piddis;
PidWheel pidsita;

extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

uint8_t ORDER_DATA[20];
volatile int TaskOverFlag=1;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MoveControlTask */
osThreadId_t MoveControlTaskHandle;
const osThreadAttr_t MoveControlTask_attributes = {
  .name = "MoveControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ReadMpuTask */
osThreadId_t ReadMpuTaskHandle;
const osThreadAttr_t ReadMpuTask_attributes = {
  .name = "ReadMpuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for OdarGetTask */
osThreadId_t OdarGetTaskHandle;
const osThreadAttr_t OdarGetTask_attributes = {
  .name = "OdarGetTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for DataSendTask */
osThreadId_t DataSendTaskHandle;
const osThreadAttr_t DataSendTask_attributes = {
  .name = "DataSendTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for LidarCaliTask */
osThreadId_t LidarCaliTaskHandle;
const osThreadAttr_t LidarCaliTask_attributes = {
  .name = "LidarCaliTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MoveControl(void *argument);
void ReadMpu(void *argument);
void OdarGet(void *argument);
void DataSend(void *argument);
void ReadLidar(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
    return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MoveControlTask */
  MoveControlTaskHandle = osThreadNew(MoveControl, NULL, &MoveControlTask_attributes);

  /* creation of ReadMpuTask */
  ReadMpuTaskHandle = osThreadNew(ReadMpu, NULL, &ReadMpuTask_attributes);

  /* creation of OdarGetTask */
  OdarGetTaskHandle = osThreadNew(OdarGet, NULL, &OdarGetTask_attributes);

  /* creation of DataSendTask */
  DataSendTaskHandle = osThreadNew(DataSend, NULL, &DataSendTask_attributes);

  /* creation of LidarCaliTask */
  LidarCaliTaskHandle = osThreadNew(ReadLidar, NULL, &LidarCaliTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

    /* Infinite loop */
    for(;;)
    {

			//printf("abs:%f		base:%f	rev:%f	  	tar:%f\r\n",carStat.AbsSita,carStat.baseSita,carStat.Sita,Cartar.Tar_sita);
	
				
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        osDelay(50);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MoveControl */
/**
* @brief Function implementing the MoveControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MoveControl */
void MoveControl(void *argument)
{
  /* USER CODE BEGIN MoveControl */
	TASKNUM =Free;
	
	int SitaOverFlag  =0;
	static  int MoveReadyCnt=0;
	static int CntFlag=1;
	
    const TickType_t xFrequency = pdMS_TO_TICKS(Ts*1000); // 10ms周期
	const TickType_t xDelay = pdMS_TO_TICKS(1000); // 延迟1000毫秒
	
    PidMlpi_Param_Init(&pidvel,1.2,0.1,1.2);
    PidMlpi_Param_Init(&piddis,1.5,0.0001,10);
    PidParam_Init(&pidsita,0.515,0.0001,1.5);

    LOWPASS_FILTER_Init(&velFilter, 0.01);
	
    while(mpuFlag){
		vTaskDelay(xDelay);
    }
		
	TickType_t xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
    {		
						
		MoveReadyCnt+=1;
		if(MoveReadyCnt>=100 && (CntFlag != 0)){
				
//				TASKNUM = AheadSpDis;
//				Cartar.MaxVel = 20;
//				Cartar.Tar_dis = 100;
			
//				Cartar.LBvel=-10;
//				Cartar.LFvel=-10;
//				Cartar.RBvel=-10;
//				Cartar.RFvel=-10;

				printf("READY!\r\n");		
				MoveReadyCnt =0;
				CntFlag =0;
		}
		
		/*逆运动学解算*/
		CarInv_Kinematics(&carStat);
		

		/*角度环*/
        SitaOverFlag = CarSitaSet(&Cartar,&carStat,&pidsita);
		
		/*指定距离，前进*/
        if(TASKNUM == AheadSpDis) {
            TaskOverFlag = CarDisSet(&Cartar,&carStat,&piddis);
			if(TaskOverFlag) {
                printf("AheadSpDis TASK OVER !\r\n");
				TASKNUM = Free;
            }
        }

        /*旋转指定角度*/
        else if(TASKNUM == RotateSpAngle) {
            if(SitaOverFlag) {
                TaskOverFlag = 1;
				printf("RotateSpAngle TASK OVER !\r\n");
				BaseSitaUpdate(&carStat,&Cartar);//坐标系重置
				TASKNUM = Free;
            }
		}
        
		/*纵向标定*/
		else if(TASKNUM == CalibraDis){
			carStat.CalibrationDis = distance;
			TaskOverFlag = CarDisCalibration(&Cartar,&carStat,&piddis);
			if(TaskOverFlag) {
				LidarUart_ISREN(0);
				
				printf("CalibraDis TASK OVER !\r\n");
				BaseSitaUpdate(&carStat,&Cartar);//坐标系重置
				TASKNUM = Free;
				
				Refresh_CarDis(&carStat);
				Refresh_Car(&Cartar,&OrderParam);
				
			}
		
		}
		
		/*角度横向标定*/
		else if(TASKNUM == CalibraAngle){
			carStat.CalibrationDis = distance;
			float err = Cartar.Tar_dis-carStat.CalibrationDis;


			TaskOverFlag = CarSitaCalibration(&Cartar,&carStat,&pidsita);
			if(TaskOverFlag) {
				LidarUart_ISREN(0);
				printf("CalibraAngle TASK OVER !\r\n");
				BaseSitaUpdate(&carStat,&Cartar);//坐标系重置
				TASKNUM = Free;
			}
		
		}
		
		/*空闲*/
        if(TASKNUM==Free) {
            Refresh_CarDis(&carStat);
			Refresh_Car(&Cartar,&OrderParam);
        }



//		switch(TASKNUM) {
//			/* 指定距离，前进 */
//			case AheadSpDis:
//				TaskOverFlag = CarDisSet(&Cartar, &carStat, &piddis);
//				if(TaskOverFlag) {
//					printf("AheadSpDis TASK OVER !\r\n");
//					TASKNUM = Free;
//				}
//				break;
//			
//			/* 旋转指定角度 */
//			case RotateSpAngle:
//				if(SitaOverFlag) {
//					TaskOverFlag = 1;
//					printf("RotateSpAngle TASK OVER !\r\n");
//					BaseSitaUpdate(&carStat, &Cartar);  // 坐标系重置
//					TASKNUM = Free;
//				}
//				break;
//			
//			/* 纵向标定 */
//			case CalibraDis:
//				carStat.CalibrationDis = distance;
//				TaskOverFlag = CarDisCalibration(&Cartar, &carStat, &piddis);
//				if(TaskOverFlag) {
//					LidarUart_ISREN(0);
//					printf("CalibraDis TASK OVER !\r\n");
//					BaseSitaUpdate(&carStat, &Cartar);  // 坐标系重置
//					TASKNUM = Free;
//				}
//				break;
//			
//			/* 角度横向标定 */
//			case CalibraAngle:
//				carStat.CalibrationDis = distance;
//				float err = Cartar.Tar_dis - carStat.CalibrationDis;
//				// 粗调逻辑（原注释保留）
//				TaskOverFlag = CarSitaCalibration(&Cartar, &carStat, &pidsita);
//				if(TaskOverFlag) {
//					LidarUart_ISREN(0);
//					printf("CalibraAngle TASK OVER !\r\n");
//					BaseSitaUpdate(&carStat, &Cartar);  // 坐标系重置
//					TASKNUM = Free;
//				}
//				break;
//			
//			/* 空闲状态 */
//			case Free:
//				Refresh_CarDis(&carStat);
//				Refresh_Car(&Cartar, &OrderParam);
//				break;
//				
//			default:
//				// 可选：处理未定义的任务编号
//				break;
//		}

		
		WheelVelSet(&Cartar,&carStat,&pidvel);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  /* USER CODE END MoveControl */
}

/* USER CODE BEGIN Header_ReadMpu */
/**
* @brief Function implementing the ReadMpuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadMpu */
void ReadMpu(void *argument)
{
  /* USER CODE BEGIN ReadMpu */
    float yaw;
    float fAcc[3], fGyro[3], fAngle[3];
    int i;
	
	int ReadMpu=0;
	static int CntFlag=1;

    MPU_Usart_Init(9600);

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);//初始化JY61
    WitSerialWriteRegister(SensorUartSend); //注册写回调函数
    WitRegisterCallBack(SensorDataUpdata);//注册获取传感器数据回调函数
    WitDelayMsRegister(Delayms);
    AutoScanSensor();  //自动搜索传感器

    carStat.baseSita = MpuYawInit(10);
    /* Infinite loop */
    for(;;)
    {
        CmdProcess();
        if(s_cDataUpdate) {
            fAngle[2] = sReg[Roll+2] / 32768.0f * 180.0f;
            if(s_cDataUpdate & ANGLE_UPDATE) {
                s_cDataUpdate &= ~ANGLE_UPDATE;
				
				carStat.AbsSita = fAngle[2];//绝对
				carStat.Sita = normalizeYaw(fAngle[2] - carStat.baseSita);//相对
            }
        }
		
		ReadMpu+=1;
		if(ReadMpu>=10 &&(CntFlag!=0)){
			mpuFlag = 0;
			CntFlag =0;
		} 
        
		osDelay(25);
    }
  /* USER CODE END ReadMpu */
}

/* USER CODE BEGIN Header_OdarGet */
/**
* @brief Function implementing the OdarGetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OdarGet */
void OdarGet(void *argument)
{
  /* USER CODE BEGIN OdarGet */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	#define HEADER_CHAR '@'
	#define FOOTER_CHAR '#'
   
	/* Infinite loop */
    for(;;)
    {

        // 阻塞等待通知
        ulTaskNotifyTake(            // 比xTaskNotifyWait更简洁
            pdTRUE,                  // 退出时清零通知值
            portMAX_DELAY            // 无限等待
        );

        //命令表格：
        //指令1： 指定v,w	v,w		
        //指令2： 指定距离	max_v,dis
        //指令3:  指定角度	max_v:sita
		//指令4： 距离标定	dis
		//指令5： 角度标定  dis
		
		// 检查最小有效长度：帧头 + | + 命令号 + 2个参数 + 帧尾
		size_t data_len = strlen((char*)ORDER_DATA);
        if(data_len < 6) {  // 最小如：@|1|0|0|#
            printf("ERR: Data too short! Len=%d\r\n", data_len);
            continue;
        }
        
        // 检查帧头
        if(ORDER_DATA[0] != HEADER_CHAR) {
            printf("ERR: Missing header! Got %c\r\n", ORDER_DATA[0]);
            continue;
        }
        
        // 检查帧尾
        if(ORDER_DATA[data_len-1] != FOOTER_CHAR) {
            printf("ERR: Missing footer! Got %c\r\n", ORDER_DATA[data_len-1]);
            continue;
        }
        
        //确认有三个竖线分隔符
        int separator_count = 0;
        for(int i=0; i<data_len; i++) {
            if(ORDER_DATA[i] == '|') separator_count++;
        }
        if(separator_count < 3) {
            printf("ERR: Missing separators! Count=%d\r\n", separator_count);
            continue;
        }
        
		int Task_type;
        float value1 = 0, value2 = 0;
        int parsed = sscanf((char*)ORDER_DATA, "@|%d|%f|%f|#", &Task_type, &value1, &value2);
        
        // 检查解析结果
        if(parsed < 3) {
            printf("ERR: Parse failed! Got %d fields, expected 3\r\n", parsed);
            printf("Raw data: %s\r\n", ORDER_DATA);
            continue;
        }
 
        // 任务可用性检查
        if(!TaskOverFlag) {
            printf("ERR: Task busy, wait for completion!\r\n");
            continue;
        }

		
        TaskOverFlag = 0;
        switch(Task_type) {
            case 1: // 指定v,w: v,w
                printf("[TASK1]: v=%.2f, w=%.2f\r\n", value1, value2);
                OrderParam.Order_vel = value1;
                OrderParam.Order_omeiga = value2;
                TASKNUM = Nav2Interfaces;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);
                break;
                
            case 2: // 指定距离: max_v, dis
                printf("[TASK2]: max_v=%.2f, dis=%.2f\r\n", value1, value2);
                OrderParam.Order_MAXvel = value1;
                OrderParam.Order_dis = value2;
                TASKNUM = AheadSpDis;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);
                break;
                
            case 3: // 指定角度: max_v, sita
                printf("[TASK3]: max_v=%.2f, sita=%.2f\r\n", value1, value2);
                if(fabs(value2) <= 180.0f) {
                    OrderParam.Order_MAXvel = value1;
                    OrderParam.Order_sita = value2;
                    TASKNUM = RotateSpAngle;
                    Refresh_CarDis(&carStat);
                    Split_CarTarParam(&Cartar, &OrderParam);
                } else {
                    printf("TASK3]ERR: Angle out of range! %.1f°\n", value2);
                }
                break;
                
            case 4: // 距离标定: dis
                printf("[TASK4]: CalibraDis=%.2f\r\n", value1);
                OrderParam.Order_dis = value1;
                TASKNUM = CalibraDis;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);
                // LidarUart_ISREN(1); 取消注释启用激光
                break;
                
            case 5: // 角度标定: dis
                printf("[TASK5]: CalibraAngle=%.2f\r\n", value1);
                OrderParam.Order_dis = value1;
                TASKNUM = CalibraAngle;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);
                LidarUart_ISREN(1); // 启用激光
                break;
                
            default:
                printf("ERR: NO THIS TASK! %d\r\n", Task_type);
                break;
        }		
		
/*====================无帧检验====================*/		
//        char task_seg[3];
//        float value1, value2;
//        sscanf(ORDER_DATA, "%[^|]|%f|%f", task_seg, &value1, &value2);

//        if(!TaskOverFlag) {
//            printf("undone!,wait for it\r\n");
//        }

//        else if((strcmp(task_seg, "1") == 0)) {
//            TaskOverFlag =0;
//			
//			OrderParam.Order_vel  = value1;
//            OrderParam.Order_omeiga = value2;
//            TASKNUM = Nav2Interfaces;
//            
//			Refresh_CarDis(&carStat);//距离重置
//            Split_CarTarParam(&Cartar,&OrderParam);//任务分解
//        }

//        else if((strcmp(task_seg, "2") == 0)) {
//            TaskOverFlag =0;

//            printf("[TASK2] val:%f	dis:%f\r\n",value1,value2);

//            OrderParam.Order_MAXvel  = value1;
//            OrderParam.Order_dis =  value2;
//            TASKNUM = AheadSpDis;
//            
//			Refresh_CarDis(&carStat);//距离重置
//            Split_CarTarParam(&Cartar,&OrderParam);//任务分解
//        }

//        else if((strcmp(task_seg, "3") == 0)) {

//			printf("[TASK3] tarSita: %f		MAXvel:	%f\r\n",value2,value1);
//			if(fabs(value2)<=180){
//				TaskOverFlag =0;
//				OrderParam.Order_MAXvel =value1;
//				OrderParam.Order_sita = value2;
//				TASKNUM = RotateSpAngle;
//				
//				Refresh_CarDis(&carStat);//距离重置
//				Split_CarTarParam(&Cartar,&OrderParam);
//			
//			}
//			
//			else{
//				printf("angle error!\r\n");	
//			}
//		
//			
//        }
//		
//        else if((strcmp(task_seg, "4") == 0)) {
//			TaskOverFlag =0;

//			
//			printf("[TASK4] CalibrationDis: %f\r\n",value1);
//            OrderParam.Order_dis = value1;
//            TASKNUM = CalibraDis;
//      
//			Refresh_CarDis(&carStat);//距离重置
//            Split_CarTarParam(&Cartar,&OrderParam);
//			
//			//LidarUart_ISREN(1);//使能激光串口				
////			xTaskNotifyFromISR(
////				LidarCaliTaskHandle,    
////				0,
////				eNoAction, 			
////				&xHigherPriorityTaskWoken
////			);
//        
//		}
//		
//		
//        else if((strcmp(task_seg, "5") == 0)) {
//			TaskOverFlag =0;

//		
//			printf("[TASK3] CalibrationDis: %f\r\n",value1);
//            OrderParam.Order_dis = value1;
//            TASKNUM = CalibraAngle;
//			
//			Refresh_CarDis(&carStat);//距离重置
//            Split_CarTarParam(&Cartar,&OrderParam);
//			
//			LidarUart_ISREN(1);//使能激光串口				
////			xTaskNotifyFromISR(
////				LidarCaliTaskHandle,    
////				0,
////				eNoAction, 			
////				&xHigherPriorityTaskWoken
////			);
//        
//		}
//		
//        else {
//            printf("COMMOND ERROR\r\n");
//        }
/*====================无帧检验====================*/		

    }
  /* USER CODE END OdarGet */
}

/* USER CODE BEGIN Header_DataSend */
/**
* @brief Function implementing the DataSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataSend */
void DataSend(void *argument)
{
  /* USER CODE BEGIN DataSend */
    uint8_t buffer[4];
    uint8_t Done[] = "Done";
    /* Infinite loop */
    for(;;)
    {

//        // 阻塞等待通知
//        ulTaskNotifyTake(            // 比xTaskNotifyWait更简洁
//            pdTRUE,                  // 退出时清零通知值
//            portMAX_DELAY            // 无限等待
//        );

//        //DONE
//        if(TaskOverFlag) {
//            HAL_UART_Transmit_DMA(&huart3,Done,sizeof(Done));
//            TaskOverFlag = 0;
//        }
		
		osDelay(200);

    }
  /* USER CODE END DataSend */
}

/* USER CODE BEGIN Header_ReadLidar */
/**
* @brief Function implementing the LidarCaliTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadLidar */
void ReadLidar(void *argument)
{
  /* USER CODE BEGIN ReadLidar */
  /* Infinite loop */
  for(;;)
  {
	  
	//阻塞等待通知
	ulTaskNotifyTake(            // 比xTaskNotifyWait更简洁
            pdTRUE,                  // 退出时清零通知值
            portMAX_DELAY            // 无限等待
        );
  
  }
  /* USER CODE END ReadLidar */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/*ISR*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    //MPU
    if(huart==(&huart2) ) {
        WitSerialDataIn(mpu_tmp);
        HAL_UART_Receive_IT(&huart2,&mpu_tmp,1);
    }
	
	//Lidar
	else if(huart == (&huart4)){
		static uint8_t state = 0;			//	
		static uint8_t crc = 0;				//
		static uint8_t cnt = 0;				//
		static uint8_t PACK_FLAG = 0;  //
		static uint8_t data_len  = 0;  //
		static uint32_t timestamp = 0; //
		static uint8_t state_flag = 1; //
		uint8_t temp_data;
		temp_data=Lidar_Receive_buf[0];	
		if(state< 4) 																					
		{                                          
				if(temp_data == HEADER) state ++;
				else state = 0;
		}
		else if(state<10&&state>3)
				{
						switch(state)
						{
								case 4:   
									if(temp_data == device_address)              /* �豸��ַ��֤ */
									{							
													state ++;
													crc = crc + temp_data;									
													break;        
									} 
									else state = 0,crc = 0;
								case 5:   
									if(temp_data == PACK_GET_DISTANCE)					 /* ��ȡ������������ */
									{  
													PACK_FLAG = PACK_GET_DISTANCE;
													state ++;	
													crc = crc + temp_data;	
													break;									
									}		 

									else if(temp_data == PACK_RESET_SYSTEM) 		 /* ��λ���� */
									{
													PACK_FLAG = PACK_RESET_SYSTEM;
													state ++; 
													crc = crc + temp_data;	
													break;	
									}
									else if(temp_data == PACK_STOP)							 /* ֹͣ�������ݴ������� */
									{ 
													PACK_FLAG = PACK_STOP;
													state ++; 
													crc = crc + temp_data;	
													break;
									}
									else if(temp_data == PACK_ACK)							 /* Ӧ�������� */
									{  
													PACK_FLAG = PACK_ACK;
													state ++;
													crc = crc + temp_data;	
													break;
									}			 				 
									else if(temp_data == PACK_VERSION)					 /* ��ȡ��������Ϣ���� */
									{ 
													PACK_FLAG = PACK_VERSION,
													state ++,
													crc = crc + temp_data;	   	     
													break;
									}
									else state = 0,crc = 0;
								case 6: if(temp_data == chunk_offset)          /* ƫ�Ƶ�ַ */
												{  
													state ++;
													crc = crc + temp_data;
													break; 	  
												}	
												else state = 0,crc = 0;
								case 7: if(temp_data == chunk_offset)
												{  
													state ++;
													crc = crc + temp_data;
													break;
												}
												else state = 0,crc = 0;
								case 8: 
										data_len = (u16)temp_data;								 /* ���ݳ��ȵͰ�λ */
										state ++; 
										crc = crc + temp_data;
										break;																			 
								case 9: 
										data_len = data_len + ((u16)temp_data<<8); 			 /* ���ݳ��ȸ߰�λ */
										state ++;
										crc = crc + temp_data;
										break; 
								default: break;
						}
				}
				else if(state == 10 ) state_flag = 0;                    /*��switch������ʱstateΪ10����temp_data��Ϊ���볤�ȸ߰�λ���ݣ�������һ���ж�*/
				if(PACK_FLAG == PACK_GET_DISTANCE&&state_flag == 0)      /* ��ȡһ֡���ݲ�У�� */
				{
						if(state>9)
						{
								if(state<190)
								{
										static uint8_t state_num;
										state_num = (state-10)%15;
										switch(state_num)
										{
												case 0: 
													Pack_Data[cnt].distance = (uint16_t)temp_data ;				 /* �������ݵͰ�λ */
													crc = crc + temp_data;
													state++;
													break;        
												case 1: 
													Pack_Data[cnt].distance = ((u16)temp_data<<8) + Pack_Data[cnt].distance;	 /* �������� */
													crc = crc + temp_data;
													state++;
													break; 
												case 2:
													Pack_Data[cnt].noise = (u16)temp_data;				 /* ���������Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 3:
													Pack_Data[cnt].noise = ((u16)temp_data<<8) + Pack_Data[cnt].noise;				 /* �������� */
													crc = crc + temp_data;
													state++;
													break; 
												case 4:
													Pack_Data[cnt].peak = (u32)temp_data;				 										 /* ����ǿ����Ϣ�Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 5:
													Pack_Data[cnt].peak = ((u32)temp_data<<8) + Pack_Data[cnt].peak;
													crc = crc + temp_data;
													state++;
													break; 
												case 6:
													Pack_Data[cnt].peak = ((u32)temp_data<<16) + Pack_Data[cnt].peak;	
													crc = crc + temp_data;
													state++;
													break; 
												case 7:
													Pack_Data[cnt].peak = ((u32)temp_data<<24) + Pack_Data[cnt].peak;				    /* ����ǿ����Ϣ */
													crc = crc + temp_data;
													state++;
													break; 
												case 8:
													Pack_Data[cnt].confidence = temp_data;				 /* ���Ŷ� */
													crc = crc + temp_data;
													state++;
													break; 
												case 9:
													Pack_Data[cnt].intg = (u32)temp_data;															/* ���ִ����Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 10:
													Pack_Data[cnt].intg = ((u32)temp_data<<8) + Pack_Data[cnt].intg;
													crc = crc + temp_data;
													state++;
													break; 
												case 11:
													Pack_Data[cnt].intg = ((u32)temp_data<<16) + Pack_Data[cnt].intg;
													crc = crc + temp_data;
													state++;
													break; 
												case 12:
													Pack_Data[cnt].intg = ((u32)temp_data<<24) + Pack_Data[cnt].intg;				  	 /* ���ִ��� */
													crc = crc + temp_data;
													state++;
													break; 
												case 13:
													Pack_Data[cnt].reftof = (int16_t)temp_data;				 								 /* �¶ȱ���ֵ�Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 14:
													Pack_Data[cnt].reftof = ((int16_t)temp_data<<8) +Pack_Data[cnt].reftof;			/* �¶ȱ���ֵ */
													crc = crc + temp_data;
													state++;
													cnt++;							 /* ������һ�������� */
													break; 
												default: break;
										}
							}
										if(state == 191) timestamp = temp_data,state++,crc = crc + temp_data;
										else if(state == 192) timestamp = ((u32)temp_data<<8) + timestamp,state++,crc = crc + temp_data; 
										else if(state == 193) timestamp = ((u32)temp_data<<16) + timestamp,state++,crc = crc + temp_data;
										else if(state == 194) timestamp = ((u32)temp_data<<24) + timestamp,state++,crc = crc + temp_data; 
										else if(state==195)
										{
													if(temp_data == crc)  
													{
															data_process();  	 
															receive_cnt++;	 	 /* ������յ���ȷ���ݵĴ��� */
													}
													distance = Pack_Data[0].distance;
													crc = 0;
													state = 0;
													state_flag = 1;
													cnt = 0; 							 /* ��λ*/
										}
										 if(state == 190) state++;
							
						}
				}
			HAL_UART_Receive_IT(&huart4,Lidar_Receive_buf,sizeof(Lidar_Receive_buf));//����5�ص�����ִ�����֮����Ҫ�ٴο��������жϵȴ���һ�ν����жϵķ���
	
	
	}	
	
	
	
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //命令接收中断DMA
    if(huart == &huart3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3,ORDER_DATA,sizeof(ORDER_DATA));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
        //唤醒 OdarGet
        xTaskNotifyFromISR(
            OdarGetTaskHandle,     // 目标任务句柄
            0,
            eNoAction, 			// 仅唤醒
            &xHigherPriorityTaskWoken
        );
        //触发任务切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/*ISR*/


static void BaseSitaUpdate(Car_Stat* carStat,SplitCarTargetParm* Cartar){
	carStat->baseSita += Cartar->Tar_sita;
	carStat->baseSita = normalizeYaw(carStat->baseSita);
}

/**
 * @brief 将Yaw角规范化到[-180°, 180°]范围
 * @param yaw 原始Yaw角度值(单位：度)
 * @return 规范化后的Yaw角度，范围[-180, 180]
 */
static float normalizeYaw(float yaw) {
    // 第一步：将角度限制在0°-360°范围内
    yaw = fmodf(yaw, 360.0f);

    // 处理负数情况：确保在[0, 360)范围内
    if (yaw < 0) {
        yaw += 360.0f;
    }

    // 第二步：将>180°的角度转换为负角度（周期性映射）
    if (yaw > 180.0f) {
        yaw -= 360.0f;
    }

    return yaw;
}

static float MpuYawInit(int cnt) {
    float init_yaw = 0;
    float fAngle[3];
    int c_=0;
    while(c_<=cnt) {
        CmdProcess();
        if(s_cDataUpdate) {
            fAngle[2] = sReg[Roll+2] / 32768.0f * 180.0f;
            if(s_cDataUpdate & ANGLE_UPDATE) {
                c_+=1;
                s_cDataUpdate &= ~ANGLE_UPDATE;
                init_yaw += fAngle[2];
            }
        }
    }
    init_yaw = init_yaw/(float)c_;
    return init_yaw;

}


void CopeCmdData(unsigned char ucData)
{
    static unsigned char s_ucData[50], s_ucRxCnt = 0;

    s_ucData[s_ucRxCnt++] = ucData;
    if(s_ucRxCnt<3)return;										//Less than three data returned
    if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
    if(s_ucRxCnt >= 3)
    {
        if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
        {
            s_cCmd = s_ucData[0];
            memset(s_ucData,0,50);
            s_ucRxCnt = 0;
        }
        else
        {
            s_ucData[0] = s_ucData[1];
            s_ucData[1] = s_ucData[2];
            s_ucRxCnt = 2;

        }
    }

}


static void CmdProcess(void)
{
    switch(s_cCmd)
    {
    case 'a':
        if(WitStartAccCali() != WIT_HAL_OK)
            printf("\r\nSet AccCali Error\r\n");
        break;
    case 'm':
        if(WitStartMagCali() != WIT_HAL_OK)
            printf("\r\nSet MagCali Error\r\n");
        break;
    case 'e':
        if(WitStopMagCali() != WIT_HAL_OK)
            printf("\r\nSet MagCali Error\r\n");
        break;
    case 'u':
        if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK)
            printf("\r\nSet Bandwidth Error\r\n");
        break;
    case 'U':
        if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK)
            printf("\r\nSet Bandwidth Error\r\n");
        break;
    case 'B':
        if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK)
            printf("\r\nSet Baud Error\r\n");
        else
            MPU_Usart_Init(c_uiBaud[WIT_BAUD_115200]);
        break;
    case 'b':
        if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
            printf("\r\nSet Baud Error\r\n");
        else
            MPU_Usart_Init(c_uiBaud[WIT_BAUD_9600]);
        break;
    case 'R':
        if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK)
            printf("\r\nSet Rate Error\r\n");
        break;
    case 'r':
        if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)
            printf("\r\nSet Rate Error\r\n");
        break;
    case 'C':
        if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK)
            printf("\r\nSet RSW Error\r\n");
        break;
    case 'c':
        if(WitSetContent(RSW_ACC) != WIT_HAL_OK)
            printf("\r\nSet RSW Error\r\n");
        break;
    case 'h':
        //ShowHelp();
        break;
    }
    s_cCmd = 0xff;
}

// 有static（文件内私有）
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    HAL_UART_Transmit(&huart2, p_data, uiSize, HAL_MAX_DELAY);
}

static void Delayms(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
        case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

static void AutoScanSensor(void)
{
    int i, iRetry;

    for(i = 1; i < 10; i++)
    {
        MPU_Usart_Init(c_uiBaud[i]);
        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            HAL_Delay(100);
            if(s_cDataUpdate != 0)
            {
                //printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
                //ShowHelp();
                return ;
            }
            iRetry--;
        } while(iRetry);
    }
    printf("can not find sensor\r\n");
    printf("please check your connection\r\n");
}


static void SendData(float value) {

    uint8_t buffer[4];
    buffer[0] = *((uint8_t*)&(value));
    buffer[1] = *((uint8_t*)&(value) + 1);
    buffer[2] = *((uint8_t*)&(value) + 2);
    buffer[3] = *((uint8_t*)&(value) + 3);
    HAL_UART_Transmit_DMA(&huart3, buffer, sizeof(buffer));

}


static void Debug_Acutal_Dis() {
    //目标距离
    printf("tar:%f		actual:%f  \r\n",Cartar.Tar_dis,carStat.Dis);
}
static void Debug_Tar_Dis() {
    //实时距离
    printf("%f		%f		%f		%f\r\n",carStat.motorStat.LeftFrt.dis,
           carStat.motorStat.leftBack.dis,
           carStat.motorStat.RigFrt.dis,
           carStat.motorStat.RigBack.dis
          );

}
static void Debug_Acutal_Vel() {
    //实时速度
    printf("%f		%f		%f		%f\r\n",(carStat.motorStat.LeftFrt.vel),
           (carStat.motorStat.leftBack.vel),
           (carStat.motorStat.RigFrt.vel),
           (carStat.motorStat.RigBack.vel));
}
static void Debug_Tar_Vel() {
    //目标速度
    printf("%f		%f		%f		%f\r\n",(Cartar.Tar_LBvel),
           (Cartar.Tar_LFvel),
           (Cartar.Tar_RBvel),
           (Cartar.Tar_RFvel));
}

static void Debug_mpu() {

}

static void Debug_Odom() {
    printf("%f\r\n",carStat.Odom);

}

/*
	task name		task 当前状态 	 													task 优先级   				最小剩余 task 栈空间

					X: running     正在运行 			 								越大优先级越高	    剩余！
					B: blocked    等待（自动，osdelay,二值等待）
					R: ready        就绪
					D: deleted     删除
					S: suspended挂起（手动）
*/
static void Debug_taskStat() {
    char InfoBuffer[200];
    vTaskList(InfoBuffer);
    printf("taskName ttaskState ttaskPrio ttaskStack ttaskNum\r\n");
    printf("%s",InfoBuffer);
    printf("\r\n");
    printf("\r\n");
}

/* USER CODE END Application */

