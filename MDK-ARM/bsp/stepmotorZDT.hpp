/**
 * @file stepmotorZDT.hpp
 * @author Elaina (1463967532@qq.com)
 * @brief 张大头步进电机控制器
 * @version 0.1
 * @date 2025-04-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef _STEPMOTORZDT_HPP
#define _STEPMOTORZDT_HPP
#include "main.h"
#include "stdint.h"
uint8_t Step_Pos_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
/**
 * @brief 张大头步进电机控制器类
 *
 */
class StepMotorZDT_t 
{
public:
    /**
     * @brief Construct a new StepMotorZDT_t object
     *
     * @param id 电机id
     * @param USART  串口指针
     * @param have_pub_permission 是否有发布同步命令权限
     * @param dir 正转方向 0 为CW，其余值为CCW
     */
    StepMotorZDT_t(uint8_t id, UART_HandleTypeDef *USART, bool have_pub_permission = false, int8_t dir = 0) : _USART(USART)
    {
        if(dir!= 0 && dir != 1)
        {
            Error_Handler(); // 方向不合法
        }
        _have_pub_permission = have_pub_permission;
        _posDirection = dir;
        _id = id;
    }
    void SetAccSpeedPos(uint8_t acc_target, float vel_target, float distance);
    void SetLinearSpeed(float target);
    float GetLinearSpeed();
    float GetThetaError() { return _target_pose_error; }
    void Update();
    void UARTCallback(uint8_t *data);
private:
    UART_HandleTypeDef *_USART;
    int _id = 0; // 电机ID
    int8_t _posDirection = 0; // 正转方向
    double _step_angle=1.8;  //步距角,单位为度
    int8_t _div_num=16; // 步进电机细分数，默认16细分
    int16_t _target_rpm = 0;
    double _wheel_diameter = 0.080;     // 轮子直径
    bool _have_pub_permission = false; // 是否有发布权限
    uint8_t _cmd_buffer[20] = {0};     // 命令缓冲区

    float _target_pose_error = 9999999.0f; // 目标误差,开始是一个很大的数,单位为度

};

#endif