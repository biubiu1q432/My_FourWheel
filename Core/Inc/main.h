/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_B_1_Pin GPIO_PIN_5
#define MOTOR_B_1_GPIO_Port GPIOE
#define MOTOR_B_2_Pin GPIO_PIN_6
#define MOTOR_B_2_GPIO_Port GPIOE
#define EC_D_1_Pin GPIO_PIN_0
#define EC_D_1_GPIO_Port GPIOA
#define EC_D_2_Pin GPIO_PIN_1
#define EC_D_2_GPIO_Port GPIOA
#define MPU_TX_Pin GPIO_PIN_2
#define MPU_TX_GPIO_Port GPIOA
#define MPU_RX_Pin GPIO_PIN_3
#define MPU_RX_GPIO_Port GPIOA
#define MOTOR_C_1_Pin GPIO_PIN_9
#define MOTOR_C_1_GPIO_Port GPIOE
#define MOTOR_C_2_Pin GPIO_PIN_11
#define MOTOR_C_2_GPIO_Port GPIOE
#define MOTOR_D_1_Pin GPIO_PIN_13
#define MOTOR_D_1_GPIO_Port GPIOE
#define MOTOR_D_2_Pin GPIO_PIN_14
#define MOTOR_D_2_GPIO_Port GPIOE
#define MPU_SCL_Pin GPIO_PIN_10
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_11
#define MPU_SDA_GPIO_Port GPIOB
#define Serial_TX_Pin GPIO_PIN_8
#define Serial_TX_GPIO_Port GPIOD
#define Serial_RX_Pin GPIO_PIN_9
#define Serial_RX_GPIO_Port GPIOD
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define Debug_TX_Pin GPIO_PIN_9
#define Debug_TX_GPIO_Port GPIOA
#define Debug_RX_Pin GPIO_PIN_10
#define Debug_RX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define EC_A_1_Pin GPIO_PIN_15
#define EC_A_1_GPIO_Port GPIOA
#define Lidar_TX_Pin GPIO_PIN_10
#define Lidar_TX_GPIO_Port GPIOC
#define Lidar_RX_Pin GPIO_PIN_11
#define Lidar_RX_GPIO_Port GPIOC
#define EC_A_2_Pin GPIO_PIN_3
#define EC_A_2_GPIO_Port GPIOB
#define EC_B_1_Pin GPIO_PIN_4
#define EC_B_1_GPIO_Port GPIOB
#define EC_B_2_Pin GPIO_PIN_5
#define EC_B_2_GPIO_Port GPIOB
#define EC_C_1_Pin GPIO_PIN_6
#define EC_C_1_GPIO_Port GPIOB
#define EC_C_2_Pin GPIO_PIN_7
#define EC_C_2_GPIO_Port GPIOB
#define MOTOR_A_1_Pin GPIO_PIN_8
#define MOTOR_A_1_GPIO_Port GPIOB
#define MOTOR_A_2_Pin GPIO_PIN_9
#define MOTOR_A_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//单位:cm
#define WheelToCenter 30.0f
#define wheelDiameter 6.5f
#define pi 3.1415925
#define wheelCircumference pi*wheelDiameter
#define Arr 1000.0f
#define Ts 0.025f
#define ModeratingRatio 34.014
#define PPR 500
#define OnceEcNum (ModeratingRatio*PPR*4)


//通信
#define HEADER_CHAR '@'
#define FOOTER_CHAR '#'


//限幅
#define Integral_Limit 100
#define Def_Max_Vel 40.f


//电机位置
//A	C
//B	D

#define	 TIM_MOTOR_A_1 htim10
#define	 TIM_MOTOR_A_2 htim11
#define	 TIM_MOTOR_B htim9
#define	 TIM_MOTOR_C htim1
#define	 TIM_MOTOR_D htim1

#define	 TIM_MOTOR_A_EC htim2
#define	 TIM_MOTOR_B_EC htim3
#define	 TIM_MOTOR_C_EC htim4
#define	 TIM_MOTOR_D_EC htim5


enum MoveTaskNum {
    Nav2Interfaces,    // 默认值0
    AheadSpDis,    // 自动赋值为1
	RotateSpAngle,  // 自动赋值为2
	CalibraDis,
	CalibraAngle,
	Free
};





/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
