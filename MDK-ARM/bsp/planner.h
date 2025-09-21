#ifndef __PLANNER_H
#define __PLANNER_H
#include "Lib_Math.h"
#include "Lib_Common.h"
#include "Kinematic.h"

enum PlannerMode_t
{
    OpenControl,
    CloseControl,
};
class Planner_t
{
public:
    Planner_t() 
    {
        _target_odom = {0, 0, 0};
        _spline[0] = CubicSpline();
        _spline[1] = CubicSpline();
        _spline[2] = CubicSpline();
    }
    SimpleStatus_t &LoactaionOpenControl(const odom_t &target_odom, float max_v, const cmd_vel_t &target_vel = {0, 0, 0}, bool clearodom = false);
    SimpleStatus_t &LoactaionCloseControl(const odom_t &target_odom, float max_linear, float max_angular, const odom_t &target_error = {0.1, 0.1, 0.2}, bool clearodom = false);
    void update(uint16_t dt);
    void Clear();
private:
    float _target_t;
    uint16_t _current_t;
    odom_t _start_odom;
    odom_t _target_odom;
    CubicSpline _spline[3];
    float _max_linear_acc = 1.0f; // 最大线加速度
    float _max_angular_acc = 1.0f; // 最大角加速度
    TrapezoidalSpline _trapezoidal_spline[3]; // 三个方向的梯形加减速插值
    SimpleStatus_t _promise = SimpleStatus_t();
    PlannerMode_t _controlmode = PlannerMode_t::OpenControl;
};
inline Planner_t Planner;
#endif
