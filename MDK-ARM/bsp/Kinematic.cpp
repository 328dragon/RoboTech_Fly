/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 00:56:07
 * @FilePath: \MDK-ARM\Hardware\Kinematic.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "Kinematic.h"
#define PI 3.1415926
static float normalRad(float rad)
{
  if (rad > PI)
  {
    rad -= 2 * PI;
  }
  else if (rad < -PI)
  {
    rad += 2 * PI;
  }
  return rad;
}
odom_t odom_t::operator+(const odom_t &other) const
{
  return {x + other.x, y + other.y, normalRad(yaw + other.yaw)};
}
odom_t odom_t::operator-(const odom_t &other) const
{
  return {x - other.x, y - other.y, normalRad(yaw - other.yaw)};
}
/**
 * @brief 根据自身坐标系解算运动学逆解(只算一次)
 * @param {cmd_vel_t} *cmd_vel_in
 * @param {float} *speed_control
 * @return {*}
 */
void KinematicOdom_t::Inv(cmd_vel_t const &cmd_vel_in, float *speed_control)
{
  // o型
  switch (diclass)
  {
  case class_t::O_shape:
    speed_control[0] = cmd_vel_in.linear_y - cmd_vel_in.linear_x + cmd_vel_in.angular_z * _aAddb;
    speed_control[1] = cmd_vel_in.linear_y + cmd_vel_in.linear_x - cmd_vel_in.angular_z * _aAddb;
    speed_control[2] = cmd_vel_in.linear_y - cmd_vel_in.linear_x - cmd_vel_in.angular_z * _aAddb;
    speed_control[3] = cmd_vel_in.linear_y + cmd_vel_in.linear_x + cmd_vel_in.angular_z * _aAddb;
    break;

    // x型
  case class_t::X_shape:
    // 左上 (0) 轮：受 linear_x、linear_y、angular_z 的影响
    speed_control[0] = cmd_vel_in.linear_x - cmd_vel_in.linear_y - cmd_vel_in.angular_z * _aAddb;

    // 右上 (1) 轮：受 linear_x、linear_y、angular_z 的影响
    speed_control[1] = cmd_vel_in.linear_x + cmd_vel_in.linear_y + cmd_vel_in.angular_z * _aAddb;

    // 左下 (2) 轮：受 linear_x、linear_y、angular_z 的影响
    speed_control[2] = cmd_vel_in.linear_x + cmd_vel_in.linear_y - cmd_vel_in.angular_z * _aAddb;

    // 右下 (3) 轮：受 linear_x、linear_y、angular_z 的影响
    speed_control[3] = cmd_vel_in.linear_x - cmd_vel_in.linear_y + cmd_vel_in.angular_z * _aAddb;
    break;
  case class_t::Rub_shape:
    // 左上 (0) 轮：受 linear_x,angular_z 的影响
    speed_control[0] = cmd_vel_in.linear_x - cmd_vel_in.angular_z * _aAddb;
    // 右上 (1) 轮：受 linear_y,angular_z 的影响
    speed_control[1] = cmd_vel_in.linear_y + cmd_vel_in.angular_z * _aAddb;
    // 左下 (2) 轮：与左上轮相同
    speed_control[2] = speed_control[0];
    // 右下 (3) 轮：与右上轮相同
    speed_control[3] = speed_control[1];
  default:
    break;
  }
}

/**
 * @brief 根据大地坐标系解算运动学逆解(需要根据当前姿态实时计算)
 * @param {cmd_vel_t} *cmd_vel_in
 * @param {float} *speed_control
 * @param {odom_t} *odom_in
 * @return {*}
 */
void KinematicOdom_t::InvFromGroundSpeed(cmd_vel_t const &cmd_vel_in, float *speed_control)
{
  float yaw = CurrentOdom.yaw; // 当前的偏航角
  cmd_vel_t cmd_vel_body;
  cmd_vel_body.angular_z = cmd_vel_in.angular_z;
  // 将大地坐标系下的目标速度转换为车身坐标系下的目标速度
  float target_vx = cmd_vel_in.linear_x;
  float target_vy = cmd_vel_in.linear_y;
  // float target_omega = cmd_vel_in.angular_z;

  cmd_vel_body.linear_x = target_vx * cos(yaw) + target_vy * sin(yaw);
  cmd_vel_body.linear_y = -target_vx * sin(yaw) + target_vy * cos(yaw);
  Inv(cmd_vel_body, speed_control);
}
/**
 * @brief 从当前车身的速度推算底盘的速度
 * @param {cmd_vel_t} *cmd_vel_in
 * @param {float} *current_speed
 * @return {*}
 */
void KinematicOdom_t::Forward(cmd_vel_t &cmd_vel_in, float *current_speed)
{
  float v0 = current_speed[0]; // 左上轮子速度
  float v1 = current_speed[1]; // 右上轮子速度
  float v2 = current_speed[2]; // 左下轮子速度
  float v3 = current_speed[3]; // 右下轮子速度

  // 修正后的正解算公式
  cmd_vel_in.linear_x = (v0 + v1 + v2 + v3) / 4.0;                // X轴方向速度
  cmd_vel_in.linear_y = (-v0 + v1 + v2 - v3) / 4.0;               // Y轴方向速度
  cmd_vel_in.angular_z = (-v0 + v1 - v2 + v3) / (4.0 * (_aAddb)); // 角速度
}

/**
 * @brief 里程计更新函数，需要传递进dt(单位为ms)
 * @param {uint16_t} dt 传入的时间间隔
 * @param {cmd_vel_t} *cmd_vel_in 传入当前的速度
 * @param {odom_t} *odom_in 传入被跟新里程计
 * @return {*}
 */

void KinematicOdom_t::CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in)
{
  float delta_t = (float)dt / 1000;
  float dyaw = cmd_vel_in.angular_z * delta_t;
  float dx = cmd_vel_in.linear_x * delta_t;
  float dy = cmd_vel_in.linear_y * delta_t;
  odom_in.yaw += dyaw;
  odom_in.y += dx * sinf(odom_in.yaw) + dy * cosf(odom_in.yaw);
  odom_in.x += dx * cosf(odom_in.yaw) - dy * sinf(odom_in.yaw);
}

void KinematicOdom_t::CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in, float yaw)
{
  float delta_t = (float)dt / 1000;
  float dx = cmd_vel_in.linear_x * delta_t;
  float dy = cmd_vel_in.linear_y * delta_t;
  odom_in.yaw = normalRad(yaw - _yaw_zero); // 确保偏航角在 -PI 到 PI 之间
  odom_in.y += dx * sinf(odom_in.yaw) + dy * cosf(odom_in.yaw);
  odom_in.x += dx * cosf(odom_in.yaw) - dy * sinf(odom_in.yaw);
}

/**
 * @brief 更新里程计
 *
 * @param odom_in
 */
void KinematicOdom_t::UpdateOdom(const odom_t &odom_in)
{
  // 归一化偏航角
  _yaw_zero -= odom_in.yaw - CurrentOdom.yaw; // 确保偏航角在 -PI 到 PI 之间
  _yaw_zero = normalRad(_yaw_zero);
  CurrentOdom = odom_in;
}
/**
 * @brief 将目标里程计转换为基于当前姿态的里程计,变换方向为先旋转再平移
 * @param odom_target 目标里程计
 * @param target_linear 基于当前姿态目标线距离
 * @return {*}
 */
void KinematicOdom_t::TransFormOdom(const odom_t &odom_target,
                                    float *rotation_dis,
                                    float *translation_dis, float *trans_speed)
{
  float dyaw = odom_target.yaw - CurrentOdom.yaw;
  // 将当前坐标系旋转到目标坐标系
  float cos_yaw = cosf(dyaw);
  float sin_yaw = sinf(dyaw);
  float current_ratation_x = cos_yaw * CurrentOdom.x - sin_yaw * CurrentOdom.y;
  float current_ratation_y = sin_yaw * CurrentOdom.x + cos_yaw * CurrentOdom.y;
  // 计算目标坐标系下的平移x y
  float dx_global = odom_target.x - current_ratation_x;
  float dy_global = odom_target.y - current_ratation_y;
  // 先计算麦轮旋转四个轮的移动距离
  dyaw = normalRad(dyaw); // 确保角度在 -PI 到 PI 之间
  float rotation_distance_temp = (_aAddb)*dyaw;
  rotation_dis[0] = -rotation_distance_temp; // 左上
  rotation_dis[1] = rotation_distance_temp;  // 右上
  rotation_dis[2] = -rotation_distance_temp; // 左下
  rotation_dis[3] = rotation_distance_temp;  // 右下
  // 再计算麦轮平移四个轮的移动距离
  auto dis1 = dx_global - dy_global; // 左上 右下
  auto dis2 = dx_global + dy_global; // 右上 左下
  translation_dis[0] = dis1;         // 左上
  translation_dis[1] = dis2;         // 右上
  translation_dis[2] = dis2;         // 左下
  translation_dis[3] = dis1;         // 右下
  // 计算平移速度缩放
  auto dis_all = sqrt(dis1 * dis1 + dis2 * dis2);
  trans_speed[0] = abs(dis1) / (dis_all);
  trans_speed[1] = abs(dis2) / (dis_all);
  trans_speed[2] = abs(dis2) / (dis_all);
  trans_speed[3] = abs(dis1) / (dis_all);
}