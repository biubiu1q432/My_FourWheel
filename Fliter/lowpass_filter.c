/*
 * @Author: Rick rick@guaik.io
 * @Date: 2023-06-28 22:57:48
 * @LastEditors: Rick
 * @LastEditTime: 2023-06-29 14:39:32
 * @Description: 低通滤波器模块的实现文件
 *              实现了一个基于时间常数的一阶低通滤波器
 */
#include "lowpass_filter.h"
#include "main.h"

/**
 * @brief 初始化低通滤波器
 * @param f 滤波器结构体指针
 * @param time_const 时间常数，值越大滤波效果越强
 */
void LOWPASS_FILTER_Init(LOWPASS_FILTER_T *f, float time_const) {
  f->tf = time_const;                    // 设置时间常数
  f->prev_y = 0.0f;                      // 初始化上一次输出为0
  f->prev_timestamp = HAL_GetTick();     // 记录初始化时间戳
}


/**
 * @brief 计算低通滤波器的输出
 * @param f 滤波器结构体指针
 * @param x 输入值
 * @return float 滤波后的输出值
 */
float LOWPASS_FILTER_Calc(LOWPASS_FILTER_T *f, float x) {
  unsigned long timestamp = HAL_GetTick();  // 获取当前时间戳
  float delta = (timestamp - f->prev_timestamp) * 1e-3f;  // 计算时间差（转换为秒）
  
  // 处理时间差异常情况
  if (delta < 0.0f)
    delta = 1e-3f;  // 如果时间差为负，使用最小采样时间
  else if (delta > 0.3f) {  // 如果时间差过大（超过300ms）
    f->prev_y = x;  // 直接使用输入值作为输出
    f->prev_timestamp = timestamp;
    return x;
  }

  // 计算滤波系数
  float alpha = f->tf / (f->tf + delta);
  // 计算滤波输出：y = α * y_prev + (1-α) * x
  float y = alpha * f->prev_y + (1.0f - alpha) * x;
  
  // 更新状态
  f->prev_y = y;
  f->prev_timestamp = timestamp;
  return y;
}
