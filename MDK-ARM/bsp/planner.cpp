#include "planner.h"
#include "controller.h"
static float normalRad(float rad) {
  if (rad > PI) {
    rad -= 2 * PI;
  } else if (rad < -PI) {
    rad += 2 * PI;
  }
  return rad;
}
void Planner_t::update(uint16_t dt)
{
    _current_t += dt;
    float t = (float)_current_t / 1000.0;
    if (_promise.isResolved() == false)
    {
        switch (_controlmode)
        {
        case PlannerMode_t::OpenControl:
        { 
            break;
            // 不使用开环模式有问题
        }
        case PlannerMode_t::CloseControl:
        {
            if(t>_target_t)
            {
                t=_target_t; // 确保时间不超过目标时间
            }
            cmd_vel_t target_vel;
            //通过一次微分计算目标速度
            target_vel.linear_x = _trapezoidal_spline[0].velocity(t);
            target_vel.linear_y = _trapezoidal_spline[1].velocity(t);
            target_vel.angular_z = _trapezoidal_spline[2].velocity(t);
            KinematicOdom.TargetVel = target_vel;
            // 计算目标里程计
            odom_t target_odom;
            target_odom.x = _trapezoidal_spline[0].position(t) ;
            target_odom.y = _trapezoidal_spline[1].position(t) ;
            target_odom.yaw = _trapezoidal_spline[2].position(t) ;
            Controller.SetClosePosition(target_odom, KinematicOdom.OdomError, false);
            odom_t &error = KinematicOdom.OdomError;
            odom_t &current = KinematicOdom.CurrentOdom;
            float yaw_diff = normalRad(_target_odom.yaw - current.yaw);
            // 规划器的目标与control的不是同一个, control是基于插样,可能提前到达目标
            if (abs(_target_odom.x - current.x) < error.x && abs(_target_odom.y - current.y) < error.y && abs(yaw_diff) < error.yaw && t>=_target_t)
            {
                _promise.resolve();
                _controlmode = PlannerMode_t::OpenControl; // 切换回开环模式
            }
            // _promise=_controller->_status;
            break;
        }
        default:
            break;
        }
    }
}
/**
 * @brief 设置位置闭环目标
 * 
 * @param target_odom  目标位置,角度应该在-PI到PI,有正负
 * @param max_linear 平移最大速度,绝对值
 * @param max_angular 旋转最大速度,绝对值
 * @param target_error 误差范围,默认0.1,0.1,0.2
 * @param clearodom 是否清除里程计
 * @return SimpleStatus_t& 当前是否完成的状态
 */
SimpleStatus_t &Planner_t::LoactaionCloseControl(const odom_t &target_odom, float max_linear, float max_angular, const odom_t &target_error, bool clearodom)
{
    _promise.start();
    // 确保目标yaw在-PI到PI之间
    const odom_t current_odom = KinematicOdom.CurrentOdom;
    odom_t odom_diff = target_odom - current_odom;
    if (clearodom)
    {
        KinematicOdom.UpdateOdom({0.0, 0.0, 0.0}); // 清除里程计
    }
    max_linear=fabs(max_linear);
    const float dis = Sqrt(odom_diff.x * odom_diff.x + odom_diff.y * odom_diff.y);
    const float x_scale=odom_diff.x / dis ;
    const float y_scale=odom_diff.y / dis ;
    _trapezoidal_spline[0] = TrapezoidalSpline(current_odom.x, target_odom.x,max_linear*x_scale, _max_linear_acc*x_scale);
    _trapezoidal_spline[1] = TrapezoidalSpline(current_odom.y, target_odom.y,max_linear*y_scale, _max_linear_acc*y_scale);
    //单独对yaw 做处理,扩展到-PI到PI之外
    const float yaw_diff = target_odom.yaw - current_odom.yaw;
    float yaw_target_optimize;
    if (yaw_diff > PI) {
        yaw_target_optimize= target_odom.yaw - 2 * PI;
    }
    else if (yaw_diff < -PI) {
        yaw_target_optimize= target_odom.yaw + 2 * PI;
    }
    else {
        yaw_target_optimize = target_odom.yaw;
    }
    _trapezoidal_spline[2] = TrapezoidalSpline(current_odom.yaw, yaw_target_optimize, max_angular, _max_angular_acc);
    // 重置时间
    _current_t = 0;
    _target_t=fmax(_trapezoidal_spline[0].max_t(), _trapezoidal_spline[1].max_t());
    _target_t = fmax(_target_t, _trapezoidal_spline[2].max_t());
    // 设置控制模式
    KinematicOdom.OdomError = target_error;
    _controlmode = PlannerMode_t::CloseControl;
    _target_odom = target_odom;
    _start_odom = KinematicOdom.CurrentOdom;
    return _promise;
}
/**
 * @brief 已经弃用
 * 
 * @param target_odom 
 * @param max_v 
 * @param target_vel 
 * @param clearodom 
 * @return SimpleStatus_t& 
 */
SimpleStatus_t &Planner_t::LoactaionOpenControl(const odom_t &target_odom, float max_v, const cmd_vel_t &target_vel, bool clearodom)
{
    _promise.start();
    const odom_t current_odom = KinematicOdom.CurrentOdom;
    odom_t odom_diff=target_odom - current_odom;
    if (clearodom)
    {
        KinematicOdom.UpdateOdom({0.0, 0.0, 0.0}); // 清除里程计
    }

    float target_t = Sqrt(odom_diff.x * odom_diff.x + odom_diff.y * odom_diff.y) / (max_v * 0.5) + fabs(odom_diff.yaw) / (max_v * 0.5);
    _spline[0] = CubicSpline({0, current_odom.x}, {target_t, target_odom.x}, {0, target_vel.linear_x});
    _spline[1] = CubicSpline({0, current_odom.y}, {target_t, target_odom.y}, {0, target_vel.linear_y});
    //对yaw单独做处理,寻找最小的角度差
    const float yaw_diff = target_odom.yaw - current_odom.yaw;
    float yaw_target_optimize;
    if (yaw_diff > PI) {
        yaw_target_optimize= target_odom.yaw - 2 * PI;
    }
    else if (yaw_diff < -PI) {
        yaw_target_optimize= target_odom.yaw + 2 * PI;
    }
    else {
        yaw_target_optimize = target_odom.yaw;
    }
    //将目标yaw 拓展到-PI到PI之外
    // _spline[2] = CubicSpline({0, 0}, {target_t, current_yaw + yaw_diff}, {0, target_vel.angular_z});
    _spline[2] = CubicSpline({0, current_odom.yaw}, {target_t, yaw_target_optimize}, {0, target_vel.angular_z});
    _target_t = target_t;
    _current_t = 0;

    _controlmode = PlannerMode_t::OpenControl;
    return _promise;
}
void Planner_t::Clear()
{
    _target_odom = {0, 0, 0};
    _spline[0] = CubicSpline();
    _spline[1] = CubicSpline();
    _spline[2] = CubicSpline();
    _current_t = 0;
    _target_t = 0;
    _controlmode = PlannerMode_t::OpenControl;
    _promise.resolve();
}