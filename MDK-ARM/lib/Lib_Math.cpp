/*
 * @Author: Elaina
 * @Date: 2024-10-17 18:30:06
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 22:03:03
 * @FilePath: \MDK-ARM\Lib\Lib_Math.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "Lib_Math.h"
float Sqrt(float x)
{
    if (x <= 0.0f)
        return 0.0f; // 处理 x <= 0 的情况

    float xhalf = 0.5f * x;

    // 进行位级操作以近似求倒数平方根
    int i = *(int *)&x;
    i = 0x5f3759df - (i >> 1); // 魔数和位移操作
    x = *(float *)&i;

    // 一次牛顿迭代提升精度
    x = x * (1.5f - xhalf * x * x);

    // 可选的第二次牛顿迭代（提升精度，但略微增加计算成本）
    x = x * (1.5f - xhalf * x * x);

    // 直接返回倒数平方根
    return 1 / x;
}
float pow(float x, uint8_t n)
{
    float result = 1.0f;
    for (int i = 0; i < n; i++)
    {
        result *= x;
    }
    return result;
}
float LinearInterpolation(float x, float x1, float x2, float y1, float y2)
{
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

float CubicSplineInterpolation(float x, float x1, float x2, float y1, float y2, float y1_, float y2_)
{
    float h = x2 - x1;
    float a = (y2 - y1) / h - h * (y2_ + 2 * y1_) / 6;
    float b = y1_ / 2;
    float c = (y2_ - y1_) / (6 * h);
    return a * pow(x - x1, 3) + b * pow(x - x1, 2) + c * pow(x - x1, 3);
}
