/*
 * @Author: Elaina
 * @Date: 2024-10-17 23:26:43
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-19 01:00:30
 * @FilePath: \MDK-ARM\Hardware\controller.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "controller.h"
static float normalRad(float rad) {
  if (rad > PI) {
    rad -= 2 * PI;
  } else if (rad < -PI) {
    rad += 2 * PI;
  }
  return rad;
}
/**
 * @brief 设置步进电机速度,内部使用
 * 
 * @param target_speed 
 */
void StepController_t::SetMotorTargetSpeed(float *target_speed)
{
    // MotorList->Foreach([this, target_speed](IMotorSpeed_t *motor)
    //                    { motor->set_speed_target(target_speed[motor->_id]); });
    for (int i = 0; i < 4; i++)
    {
        if (MotorList[i] != nullptr)
        {
            MotorList[i]->SetLinearSpeed(target_speed[i]);
        }
    }
}
/**
 * @brief 获得当前电机的速度
 *
 * @param dt
 */
void StepController_t::MotorUpdate(uint16_t dt)
{
    // MotorList->Foreach([this, dt](IMotorSpeed_t *motor)
    //                    { motor->update((void *)&dt);
    // 											current_speed[motor->_id] = motor->get_linear_speed(); });
    for (int i = 0; i < 4; i++)
    {
        if (MotorList[i] != nullptr)
        {
            current_speed[i] = MotorList[i]->GetLinearSpeed();
            MotorList[i]->Update();
        }
    }
}
/**
 * @brief 状态更新
 * 
 */
void StepController_t::StatusUpdate()
{
    if (_ControlMode == ControlMode_t::location_contorl)
    {
        if (_status.isResolved() == false)
        {
            odom_t &current_odom = KinematicOdom.CurrentOdom;
            odom_t &target_odom = KinematicOdom.TargetOdom;
            odom_t &_odom_error = KinematicOdom.OdomError;
            float y_diff= normalRad(target_odom.yaw - current_odom.yaw);
            if (fabs(current_odom.x - target_odom.x) < _odom_error.x && fabs(current_odom.y - target_odom.y) < _odom_error.y && fabs(y_diff) < _odom_error.yaw)
            {
                // _ControlStatus = finish;
                _status.resolve();
            }
        }
        else {
            // _ControlMode = ControlMode_t::speed_control_self; //不会自动切换回速度控制模式
        }
    }
    //避免在速度控制时候误触发位置控制的状态机
    else if (_ControlMode == ControlMode_t::speed_control_self)
    {
        _status.resolve();
    }
    else if(_ControlMode == ControlMode_t::speed_control_groud)
    {
        _status.resolve(); // 大地坐标系下的速度控制不需要状态机
    }
    else if (_ControlMode == ControlMode_t::step_ground)
    {
        float angle_error_min=0.0f; //轴旋转的角度误差,单位为度
        for(int i = 0; i < 4; i++)
        {
            if(fabs(MotorList[i]->GetThetaError())>angle_error_min)
            {
                angle_error_min=fabs(MotorList[i]->GetThetaError()); // 取最大的角度误差
            }
        }
        //如果在角度旋转过程中
        if(!_is_angle_finish)
        {
            if (angle_error_min < 0.1f && _is_angle_publish) // 角度误差小于0.1度
            {
                _is_angle_finish = true; // 角度到达目标
            }
        }
        else {
            if(angle_error_min < 0.1f && _is_distance_publish) // 轴误差小于0.01度
            {
                _is_distance_finish = true; // 平移到达目标
                if(_is_odom_update)
                {
                    _status.resolve();
                    _ControlMode = ControlMode_t::speed_control_self; // 切换回速度控制模式
                    _is_angle_finish = false; // 重置角度完成标志
                    _is_distance_finish = false; // 重置平移完成标志
                    _is_distance_publish = false; // 重置平移发布标志
                    _is_angle_publish = false; // 重置角度发布标志
                    _is_odom_update = false; // 重置里程计更新标志
                }
            }
        }
    }
}
/**
 * @brief 设置速度目标(不传入bool默认为自身坐标系)
 * @param {float} linear_x
 * @param {float} linear_y
 * @param {float} angular_z
 * @param {bool} use_ground_control
 * @return {*}
 */
void StepController_t::SetVelTarget(cmd_vel_t cmd_vel_in)
{
    KinematicOdom.TargetVel = cmd_vel_in;
    _ControlMode = ControlMode_t::speed_control_self;
    KinematicOdom.Inv(KinematicOdom.TargetVel, target_speed);
}
/**
 * @brief 设置大地坐标系下的速度目标
 * 
 * @param cmd_vel_in 
 */
void StepController_t::SetGroundVelTarget(cmd_vel_t cmd_vel_in)
{
    if (_ControlMode != speed_control_self) {
        return ; // 如果当前不是速度控制模式,则不允许设置大地坐标系下的速度目标
    }
    KinematicOdom.TargetVel = cmd_vel_in;
    _ControlMode = ControlMode_t::speed_control_groud;
    KinematicOdom.Inv(KinematicOdom.TargetVel, target_speed);
}
/**
 * @brief 控制速度更新,根据当前的里程计与运动状态更新控制量
 * @param {odom_t} *odom_in
 * @return {*}
 * @note:
 */
void StepController_t::ControlUpdate(uint16_t dt) {
  switch (_ControlMode) {
  case ControlMode_t::location_contorl: {
    // if (_status.isResolved())
    // {
    //     _ControlMode = ControlMode_t::speed_control_self;
    //     kinematic->Inv({0, 0, 0}, target_speed);
    //     return;
    // }
    const odom_t &target_odom = KinematicOdom.TargetOdom;
    const float yaw_diff = normalRad(target_odom.yaw) - KinematicOdom.CurrentOdom.yaw;
    float current_yaw = KinematicOdom.CurrentOdom.yaw;
    //确保dyaw是最小弧线,扩大current_yaw到-PI到PI之外
    if(yaw_diff>PI)
    {
        current_yaw += 2*PI;
    }
    else if(yaw_diff<-PI)
    {
        current_yaw -= 2*PI;
    }
    // 将kinematic的目标速度当成前馈速度,pid修正位置误差
    float vx = pid_x.cal(target_odom.x, KinematicOdom.CurrentOdom.x) +
               KinematicOdom.TargetVel.linear_x;
    float vy = pid_y.cal(target_odom.y, KinematicOdom.CurrentOdom.y) +
               KinematicOdom.TargetVel.linear_y;
    float v_yaw = pid_yaw.cal(normalRad(target_odom.yaw), current_yaw) +
                  KinematicOdom.TargetVel.angular_z;

    //从当前全局坐标系速度推算电机速度
    KinematicOdom.InvFromGroundSpeed({vx, vy, v_yaw}, target_speed);
    // 应用控制量
    SetMotorTargetSpeed(target_speed);
    break;
  }
  // 自身坐标下的速度控制什么都不用弄
  case ControlMode_t::speed_control_self: {
    SetMotorTargetSpeed(target_speed);
    break;
  }
  // 大地坐标下的速度控制
  case ControlMode_t::speed_control_groud: {
    cmd_vel_t &target_val = KinematicOdom.TargetVel;
    KinematicOdom.InvFromGroundSpeed(target_val, target_speed);
    // 应用控制量
    SetMotorTargetSpeed(target_speed);
    break;
  }
  case ControlMode_t::step_ground: {
    
    // 旋转距离在设置里面发布过了
    if (_is_angle_finish) {
      if (!_is_distance_publish) {
        for (int i = 0; i < 4; i++) {
          MotorList[i]->SetAccSpeedPos(_target_acc*_step_trans_speed[i], _target_linear_speed*_step_trans_speed[i],
                                              _step_motor_translation_dis[i]);
        }
        _is_distance_publish = true; // 发布了目标距离
      }
      //已经发布过目标距离了,则更新电机误差
      else {
        MotorUpdate(0); // 更新电机误差
      }
      if (_is_distance_finish) {
        KinematicOdom.UpdateOdom(KinematicOdom.TargetOdom); // 更新里程计
        _is_odom_update = true;                       // 更新了里程计
      }

    }
    else {
        if(!_is_angle_publish)
        {
          for (int i = 0; i < 4; i++) {
            MotorList[i]->SetAccSpeedPos(_target_acc, _target_linear_speed, _step_motor_rotation_dis[i]);
          }
            _is_angle_publish = true; // 发布了目标角度
        }
        else {
            MotorUpdate(0); // 更新电机误差
        }
    }
    break;
  }
  }
}
/**
 * @brief 设置位置闭环目标
 * @param {odom_t} target_odom 目标位置
 * @param {odom_t} target_error 误差
 * @param {bool} privilege 是否清除里程计
 * @return {KinematicState_t} 返回当前的状态
 * @note
 */
SimpleStatus_t &StepController_t::SetClosePosition(const odom_t &target_odom, const odom_t &target_error, bool clearodom)
{
    KinematicOdom.TargetOdom = target_odom;
    KinematicOdom.OdomError = target_error;

    if (clearodom)
    {
        KinematicOdom.UpdateOdom({0.0, 0.0, 0.0}); // 清除里程计
    }
    _ControlMode = ControlMode_t::location_contorl;
    _status.start();
    return _status;
}
/**
 * @brief 设置步进电机大地坐标系下的目标速度
 * 
 * @param target_odom 目标位置
 * @param acc 步进电机加速度
 * @param linear_x 步进电机线速度
 * @param clearodom 是否清除里程计
 * @return SimpleStatus_t& 当前是否完成的状态
 * @note 不准,严格吃装配,放弃维护
 */
SimpleStatus_t &StepController_t::SetStepGroudPosition(
    const odom_t &target_odom, int8_t acc, float linear_x, bool clearodom) {
    // 如果当前不是速度控制模式,则不允许设置位置闭环
    if (_ControlMode != speed_control_self) {
        return _status;
    }
    KinematicOdom.TargetOdom = target_odom;

    if (clearodom) {
        KinematicOdom.UpdateOdom({0.0, 0.0, 0.0});
    }
    _ControlMode = ControlMode_t::step_ground;
    _status.start();
    KinematicOdom.TransFormOdom(target_odom, _step_motor_rotation_dis, _step_motor_translation_dis,_step_trans_speed);
    _target_acc = acc;
    _target_linear_speed = linear_x; // 设置目标线速度
    return _status;
}
/**
 * @brief 更新运动学和控制,更新里程计和控制量,通过当前电机速度反算位置
 * 
 * @param dt 
 */
void StepController_t::KinematicAndControlUpdate(uint16_t dt)
{
    // 先更新电机速度
    // MotorUpdate(dt);
    for (int i = 0; i < 4; i++) {
      current_speed[i] = MotorList[i]->GetLinearSpeed(); // 获取当前电机速度
    }
    KinematicOdom.Forward(KinematicOdom.CurrentVel, current_speed);
    // 先更新里程计
    KinematicOdom.CalculationUpdate(dt, KinematicOdom.CurrentVel,
                                     KinematicOdom.CurrentOdom);
    // 再更新控制量
    ControlUpdate(dt);
    // 更新状态
    StatusUpdate();
}
/**
 * @brief 通过当前电机速度反算位置和陀螺仪来算位置
 * 
 * @param dt 
 * @param yaw 
 */
void StepController_t::KinematicAndControlUpdate(uint16_t dt, float yaw)
{
    for (int i = 0; i < 4; i++) {
        current_speed[i] = MotorList[i]->GetLinearSpeed(); // 获取当前电机速度
    }
    KinematicOdom.Forward(KinematicOdom.CurrentVel, current_speed);
    // 先更新里程计
    KinematicOdom.CalculationUpdate(dt, KinematicOdom.CurrentVel,
                                     KinematicOdom.CurrentOdom, yaw);
    // 再更新控制量
    ControlUpdate(dt);
    // 更新状态
    StatusUpdate();
}
/**
 * @brief 状态重置
 * 
 */
void StepController_t::Clear()
{
    for (int i = 0; i < 4; i++)
    {
        target_speed[i] = 0;
        current_speed[i] = 0;
    }
    KinematicOdom.UpdateOdom({0.0, 0.0, 0.0});
    _status.isResolved();
    _ControlMode = ControlMode_t::speed_control_self;
}