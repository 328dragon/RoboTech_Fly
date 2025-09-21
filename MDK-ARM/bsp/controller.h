/**
 * @file controller.h
 * @author Elaina (1463967532@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-10-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "Lib_Common.h"
#include "Kinematic.h"
#include "stepmotorZDT.hpp"
#include "pid_template.h"
enum ControlMode_t
{

    location_contorl,    // 位置闭环
    speed_control_self,  // 自身坐标系速度开环
    speed_control_groud, // 大地坐标系速度开环
    step_ground,          //步进电机步距控制
};
class StepController_t
{
public:
    StepController_t() = default;
    StepController_t(StepMotorZDT_t **MotorList)
    {
        this->MotorList = MotorList;
        for (int i = 0; i < 4; i++)
        {
            target_speed[i] = 0;
            current_speed[i] = 0;
        }
    }
    void KinematicAndControlUpdate(uint16_t dt);
    void KinematicAndControlUpdate(uint16_t dt, float yaw);
    void SetMotorTargetSpeed(float *target_speed);
    void Clear();
    void SetVelTarget(cmd_vel_t cmd_vel_in); // 设置目标速度
    void SetGroundVelTarget(cmd_vel_t cmd_vel_in); // 设置大地坐标系下的目标速度
    SimpleStatus_t &SetStepGroudPosition(
        const odom_t &target_odom, int8_t acc,float linear_x,
        bool clearodom = false); // 设置步进电机大地坐标系下的目标速度

    SimpleStatus_t &SetClosePosition(const odom_t &target_odom, const odom_t &target_error = {0.1, 0.1, 0.2}, bool clearodom = false);
    SimpleStatus_t _status = SimpleStatus_t();

private:
    void ControlUpdate(uint16_t dt);
    void MotorUpdate(uint16_t dt);
    void StatusUpdate(); // 更新状态
    // LibList_t<IMotorSpeed_t *> *MotorList;
    StepMotorZDT_t **MotorList = {nullptr};
    float target_speed[4];
    float current_speed[4];
    ControlMode_t _ControlMode = ControlMode_t::speed_control_self;
    float _step_motor_rotation_dis[4] = {0}; // 步进电机旋转位移
    float _step_motor_translation_dis[4] = {0}; // 步进电机平移位移
    float _step_trans_speed[4] = {0}; // 步进电机平移速度平移时候四个电机速度是不一样的
    float _target_linear_speed = 0.0f; // 目标线速度
    float _target_acc=0.0f; // 目标加速度
    bool _is_angle_publish = false; // 是否发布了目标角度
    bool _is_angle_finish = false; // 是否到达目标角度
    bool _is_distance_finish = false; // 是否到达目标距离
    bool _is_distance_publish = false; // 是否发布了目标距离
    bool _is_odom_update = false; // 是否更新了里程计
    pid_Increment_template_t<float, float> pid_x = pid_Increment_template_t<float, float>({0.3, 1, 0.2, -0.4, 0.4});
    pid_Increment_template_t<float, float> pid_y = pid_Increment_template_t<float, float>({0.3, 1, 0.2, -0.4, 0.4});
    pid_Increment_template_t<float, float> pid_yaw = pid_Increment_template_t<float, float>({0.6, 2, 0.2, -0.6, 0.6});
};
inline StepController_t Controller; // 全局控制器实例
#endif