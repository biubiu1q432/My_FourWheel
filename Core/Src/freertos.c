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
#include "semphr.h"
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

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static float MpuYawInit(int cnt);
static float normalizeYaw(float yaw);
static void BaseSitaUpdate(Car_Stat* carStat,SplitCarTargetParm* Cartar);

/*lidar*/
uint8_t Lidar_Receive_buf[1];
u16 receive_cnt;
u8 confidence;
u16 distance,noise,reftof;
u32 peak,intg;

static uint8_t lidar_state = 0;
static uint16_t lidar_distance = 0;

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
PidCar pidCalidis;
PidCar pidCalidisDown;

PidWheel pidsita;


extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

uint8_t ORDER_DATA[20];

volatile int TaskOverFlag=0;
volatile int ResponseLidarGet=0;
volatile int EnUpLidarDisRead=0;
volatile int EnUpLidarDisTask=0;

volatile int mpuFlag=1;



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
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataSendTask */
osThreadId_t DataSendTaskHandle;
const osThreadAttr_t DataSendTask_attributes = {
    .name = "DataSendTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AnlayOderDataFlag */
osSemaphoreId_t AnlayOderDataFlagHandle;
const osSemaphoreAttr_t AnlayOderDataFlag_attributes = {
    .name = "AnlayOderDataFlag"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MoveControl(void *argument);
void ReadMpu(void *argument);
void OdarGet(void *argument);
void DataSend(void *argument);

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

    /* Create the semaphores(s) */
    /* creation of AnlayOderDataFlag */
    AnlayOderDataFlagHandle = osSemaphoreNew(1, 0, &AnlayOderDataFlag_attributes);

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
        //printf("%d,%f\r\n",carStat.SideLidarCaliDis,Cartar.Tar_LBvel);
		//printf("%d\r\n",lidar_distance);
        //printf("%.1f,%.1f,%.1f\r\n",carStat.Dis,Cartar.Tar_dis,Cartar.Tar_LBvel);
        //printf("%.1f\r\n",carStat.Sita);
        HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        osDelay(80);
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
    Cartar.MaxVel = Def_Max_Vel;

    int SitaOverFlag =0;
    static  int MoveReadyCnt=0;
    static int CntFlag=1;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    const TickType_t xFrequency = pdMS_TO_TICKS(Ts*1000); // 10ms周期
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // 延迟1000毫秒

    PidMlpi_Param_Init(&pidvel,5.5,1.0,0);
    PidMlpi_Param_Init(&piddis,2.0,0,1.2);
    PidParam_Init(&pidsita,2.85,0.008,4);
	
	
	PidMlpi_Param_Init(&pidCalidis,10,0.05,4);
	PidMlpi_Param_Init(&pidCalidisDown,0.12,0.0002,1);


    while(mpuFlag) {
        vTaskDelay(xDelay);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();


    /* Infinite loop */
    for(;;)
    {

        MoveReadyCnt+=1;
        if(MoveReadyCnt>=50 && (CntFlag != 0)) {
            printf("READY\r\n");
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
                TASKNUM = Free;

                xTaskNotifyGive(DataSendTaskHandle);

            }
        }

        /*旋转指定角度*/
        else if(TASKNUM == RotateSpAngle) {
            if(SitaOverFlag) {
                TaskOverFlag = 1;
                BaseSitaUpdate(&carStat,&Cartar);//坐标系重置
                TASKNUM = Free;

                xTaskNotifyGive(DataSendTaskHandle);

            }
        }

        /*上位机雷达标定*/
        else if(TASKNUM == CalibraUpLidarDis && (EnUpLidarDisTask==1) ) {
            TaskOverFlag += CarDisCalibration(&Cartar,&carStat,&pidCalidis);
            if(TaskOverFlag>=3) {
                EnUpLidarDisRead = 0;
                EnUpLidarDisTask = 0;
                TASKNUM = Free;

                xTaskNotifyGive(DataSendTaskHandle);
            }
        }

        /*下位机激光标定*/
        else if(TASKNUM == CalibraDownLidarDis) {
            TaskOverFlag += CarDisCalibrationStable(&Cartar,&carStat,&pidCalidisDown);

            if(TaskOverFlag>=5) {
                LidarUart_ISREN(0);
                TASKNUM = Free;

                xTaskNotifyGive(DataSendTaskHandle);
            }
        }
 
        /*空闲*/
        if(TASKNUM==Free) {
            Refresh_CarDis(&carStat);
            Refresh_Car(&Cartar,&OrderParam);
        }

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
        if(ReadMpu>=10 &&(CntFlag!=0)) {
            mpuFlag = 0;
            CntFlag =0;
        }

        osDelay(20);
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

    /* Infinite loop */
    for(;;)
    {
        //命令表格：
        //指令1： 指定v,w	v,w
        //指令2： 指定距离	max_v,dis
        //指令3:  指定角度	max_v:sita
        //指令4： 距离标定	dis
        //指令5： 角度标定  dis
        if(xSemaphoreTake(AnlayOderDataFlagHandle, portMAX_DELAY) == pdTRUE) {

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
            if(TASKNUM!=Free) {
                printf("ERR: Task busy, wait for completion!\r\n");
                continue;
            }

            TaskOverFlag = 0;
            ResponseLidarGet = 0;
            EnUpLidarDisRead = 0;
            EnUpLidarDisTask = 0;

            switch(Task_type) {
            case 1: // 指定v,w: v,w
                printf("[TASK1]: v=%.2f, w=%.2f\r\n", value1, value2);
                OrderParam.Order_vel = value1;
                OrderParam.Order_omeiga = value2;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);
			
				TASKNUM = Nav2Interfaces;
                break;

            case 2: // 指定距离: max_v, dis
                printf("[TASK2]: max_v=%.2f, dis=%.2f\r\n", value1, value2);
                OrderParam.Order_MAXvel = value1;
                OrderParam.Order_dis = value2;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);
			
				TASKNUM = AheadSpDis;
                break;

            case 3: // 指定角度: max_v, sita
                printf("[TASK3]: max_v=%.2f, sita=%.2f\r\n", value1, value2);
                if(fabs(value2) <= 180.0f) {
                    OrderParam.Order_MAXvel = value1;
                    OrderParam.Order_sita = value2;
                    Refresh_CarDis(&carStat);
                    Split_CarTarParam(&Cartar, &OrderParam);
					
					TASKNUM = RotateSpAngle;
                } 
				else printf("TASK3]ERR: Angle out of range! %.1f°\n", value2);
                break;

            case 4: // 上位机距离标定: dis
                printf("[TASK4]: CalibraUpLidarDis=%.2f val:%.2f\r\n", value1,value2);
                OrderParam.Order_dis = value1;
                OrderParam.Order_MAXvel = value2;
                TASKNUM = CalibraUpLidarDis;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);

                //唤醒DataSend响应lidar
                ResponseLidarGet= 1;
                xTaskNotify(
                    DataSendTaskHandle,
                    0,
                    eNoAction
                );
				TASKNUM = CalibraUpLidarDis;
                break;

            case 5: // 下位机距离标定: dis
                printf("[TASK5]: CalibraDownLidarDis=%.2f\r\n", value1,value2);
                OrderParam.Order_dis = value1;
				OrderParam.Order_MAXvel = value2;
                Refresh_CarDis(&carStat);
                Split_CarTarParam(&Cartar, &OrderParam);
                LidarUart_ISREN(1); // 启用激光
						
				TASKNUM = CalibraDownLidarDis;
                break;

            case 6: //调试
                printf("[TASK6]: valdebug=%.2f\r\n", value1);
                Cartar.RBvel = value1;
                Cartar.RFvel = value1;
                Cartar.LBvel = value1;
                Cartar.LFvel = value1;
                Cartar.MaxVel = 1000;
			
                break;

            default:
                printf("ERR: NO THIS TASK! %d\r\n", Task_type);
                break;
            }
        }
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

    uint8_t Done[] = "Done\r\n";
    uint8_t UpLidarAllowStart[] = "UpLidarAllowStart\r\n";

    /* Infinite loop */
    for(;;)
    {
        //阻塞等待通知
        ulTaskNotifyTake(            // 比xTaskNotifyWait更简洁
            pdTRUE,                  // 退出时清零通知值
            portMAX_DELAY            // 无限等待
        );

        //DONE
        if(TaskOverFlag) {
            HAL_UART_Transmit_DMA(&huart3,Done,sizeof(Done));
            TaskOverFlag = 0;
        }

        //LidarGet
        else if(ResponseLidarGet) {
            HAL_UART_Transmit_DMA(&huart3,UpLidarAllowStart,sizeof(UpLidarAllowStart)-1);
            ResponseLidarGet = 0;
            EnUpLidarDisRead = 1;
        }
    }
    /* USER CODE END DataSend */
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
    else if(huart == (&huart4)) {
        uint8_t temp_data = Lidar_Receive_buf[0];

        // 状态机处理
        switch(lidar_state) {
        // 帧头检测 (0-3状态)
        case 0:
        case 1:
        case 2:
        case 3:
            if(temp_data == HEADER) {
                lidar_state++;
            } else {
                lidar_state = 0;
            }
            break;

        // 设备地址检测 (状态4)
        case 4:
            if(temp_data == device_address) {
                lidar_state++;
            } else {
                lidar_state = 0;
            }
            break;

        // 包类型检测 (状态5)
        case 5:
            if(temp_data == PACK_GET_DISTANCE) {
                lidar_state++;
            } else {
                lidar_state = 0; // 非距离包，重置状态机
            }
            break;

        // 跳过不必要数据 (状态6-9)
        case 6:
        case 7:
        case 8:
        case 9:
            lidar_state++;
            break;

        // 距离低字节 (状态10)
        case 10:
            lidar_distance = temp_data; // 存储低字节
            lidar_state++;
            break;

        // 距离高字节 (状态11)
        case 11:
            lidar_distance |= (temp_data << 8); // 组合高字节
			carStat.SideLidarCaliDis = lidar_distance;
            lidar_state = 0; // 重置状态机

            break;

        default:
            lidar_state = 0;
            break;
        }
        // 重启接收
        HAL_UART_Receive_IT(&huart4, Lidar_Receive_buf, sizeof(Lidar_Receive_buf));
    }


}

int cnt_ = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(huart == &huart3) {

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3,ORDER_DATA,sizeof(ORDER_DATA));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);


        //lidar
        if((ORDER_DATA[0] == '|')) {

            if(EnUpLidarDisRead) {
                cnt_+=1;
                float dis;
                int parsed = sscanf((char*)ORDER_DATA, "|%f|", &dis);
                carStat.FrontLidarCaliDis = dis;
                if(cnt_>=10) {
                    EnUpLidarDisTask = 1;
                    cnt_=0;
                }
            }

            else {
                printf("unget uplidartask\r\n");
            }

        }

        else if(ORDER_DATA[0] == '@') {
            xSemaphoreGiveFromISR(AnlayOderDataFlagHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }
}



static void BaseSitaUpdate(Car_Stat* carStat,SplitCarTargetParm* Cartar) {
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


/* USER CODE END Application */

