/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
	  	
extern char  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA;         		//����״̬���?

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define false 0
#define true 1

#define HEADER 0xAA							/* ��ʼ�� */
#define device_address 0x00     /* �豸��ַ */
#define chunk_offset 0x00       /* ƫ�Ƶ�ַ���� */
#define PACK_GET_DISTANCE 0x02 	/* ��ȡ������������ */
#define PACK_RESET_SYSTEM 0x0D 	/* ��λ���� */
#define PACK_STOP 0x0F 				  /* ֹͣ�������ݴ������� */
#define PACK_ACK 0x10           /* Ӧ�������� */
#define PACK_VERSION 0x14       /* ��ȡ��������Ϣ���� */

typedef struct {
	int16_t distance;  						/* �������ݣ�����Ŀ����뵥�?mm */
	uint16_t noise;		 						/* ������������ǰ���������µ��ⲿ����������Խ��˵������Խ�� */
	uint32_t peak;								/* ����ǿ����Ϣ������Ŀ�귴��صĹ�ǿ��?*/
	uint8_t confidence;						/* ���Ŷȣ��ɻ��������ͽ���ǿ����Ϣ�ںϺ�Ĳ�����Ŀ��Ŷ� */
	uint32_t intg;     						/* ���ִ�������ǰ�����������Ļ��ִ��� */
	int16_t reftof;   						/* �¶ȱ���ֵ������оƬ�ڲ��¶ȱ仯����ֵ��ֻ��һ���¶ȱ仯���޷�����ʵ�¶ȶ�Ӧ */
}LidarPointTypedef;

struct AckResultData{
	uint8_t ack_cmd_id;						/* �𸴵����� id */
	uint8_t result; 							/* 1��ʾ�ɹ�,0��ʾʧ�� */
};

struct LiManuConfig
{
	uint32_t version; 						/* ����汾��?*/
	uint32_t hardware_version; 		/* Ӳ���汾�� */
	uint32_t manufacture_date; 		/* �������� */
	uint32_t manufacture_time; 		/* ����ʱ�� */
	uint32_t id1; 								/* �豸 id1 */
	uint32_t id2; 								/* �豸 id2 */
	uint32_t id3; 								/* �豸 id3 */
	uint8_t sn[8]; 								/* sn */
	uint16_t pitch_angle[4]; 			/* �Ƕ���Ϣ */
	uint16_t blind_area[2]; 			/* ä����Ϣ */
	uint32_t frequence; 					/* ���ݵ�Ƶ */
};
/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef MPU_Usart_Init(int BaudRate);
void data_process(void);
HAL_StatusTypeDef LidarUart_ISREN(int En);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

