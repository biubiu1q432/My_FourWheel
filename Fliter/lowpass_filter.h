/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 22:57:43
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 12:58:46
 * @Description: 低通滤波器模块的头文件
 *              实现了一个简单的一阶低通滤波器，用于平滑输入信号
 */
#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__

/**
 * @brief 低通滤波器结构体
 */
typedef struct {
  float tf;                    // 时间常数
  float prev_y;               // 上一次的输出值
  unsigned long prev_timestamp; // 上一次采样的时间戳
} LOWPASS_FILTER_T;

/**
 * @brief 初始化低通滤波器
 * @param f 滤波器结构体指针
 * @param time_const 时间常数，值越大滤波效果越强
 */
void LOWPASS_FILTER_Init(LOWPASS_FILTER_T *f, float time_const);

/**
 * @brief 计算低通滤波器的输出
 * @param f 滤波器结构体指针
 * @param x 输入值
 * @return float 滤波后的输出值
 */
float LOWPASS_FILTER_Calc(LOWPASS_FILTER_T *f, float x);

#endif

