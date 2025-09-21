/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 01:02:15
 * @FilePath: \MDK-ARM\Hardware\Kinematic.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include "stdint.h"
#include "math.h"
#define PI 3.1415926535

// 左上为0，右上为1，左下为2，右下为3
struct cmd_vel_t
{
    float linear_x;
    float linear_y;
    float angular_z;
};
struct odom_t
{
    float x;
    float y;
    float yaw;
    odom_t operator+(const odom_t &other) const;
    odom_t operator-(const odom_t &other) const;
};
enum class_t
{
    O_shape,
    X_shape,
    Rub_shape,
};
class KinematicOdom_t
{
public:
    KinematicOdom_t(float aAddb=0.22, class_t diclass = X_shape)
    {
        this->_aAddb = aAddb; // a b =轮距总和的一半
    
        this->diclass = diclass;
        CurrentOdom = {0, 0, 0};
        CurrentVel = {0, 0, 0};
        TargetOdom = {0, 0, 0};
        TargetVel = {0, 0, 0};
        OdomError = {0.0, 0.0, 0.0}; // 默认误差
    }
    void Forward(cmd_vel_t &cmd_vel_in, float *current_speed);

    void Inv(cmd_vel_t const &cmd_vel_in, float *speed_control);

    void InvFromGroundSpeed(cmd_vel_t const &cmd_vel_in, float *speed_control); // 传入当前的odom_t,表示基于大地坐标系推算电机速度
    void
    TransFormOdom(const odom_t &odom_target, float *rotation_dis,
                  float *translation_dis,
                  float *trans_speed); // 将目标里程计转换成电机的旋转与位移
    void CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in); // 从底盘速度更新里程计
    void CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in, float yaw);
    void UpdateOdom(const odom_t &odom_in); // 更新里程计
    odom_t TargetOdom;
    odom_t CurrentOdom;
    cmd_vel_t CurrentVel;
    cmd_vel_t TargetVel;
    odom_t OdomError;

private:
    float _yaw_zero = 0; // 偏航角零点仅在imu模式使用
    float _aAddb= 0.2;  //a b =轮距总和的一半
    class_t diclass = X_shape;
    // 三个轴的位置环pid控制,采用增量式pid
};
// 可以沿着x,y前进与绕着z轴旋转，现在定义小车前进方向为x，左边为y的右手系
inline KinematicOdom_t KinematicOdom;
#endif
