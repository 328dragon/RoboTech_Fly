#ifndef __LIB_MATH_H
#define __LIB_MATH_H
#include "stdint.h"
#include "math.h"

float Sqrt(float x);
float pow(float x, uint8_t n);
struct Point
{
    double x;
    double y;
};
class Polynomial3_t
{
public:
    Polynomial3_t() = default;
    Polynomial3_t(float a, float b, float c, float d)
    {
        data.a = a;
        data.b = b;
        data.c = c;
        data.d = d;
    }
    Polynomial3_t(float *data)
    {
        this->data.a = data[0];
        this->data.b = data[1];
        this->data.c = data[2];
        this->data.d = data[3];
    }
    float operator()(float x)
    {
        return data.a * x * x * x + data.b * x * x + data.c * x + data.d;
    }
    float d1_x(float x)
    {
        return 3 * data.a * x * x + 2 * data.b * x + data.c;
    }
    float d2_x(float x)
    {
        return 6 * data.a * x + 2 * data.b;
    }
    float d3_x(float x)
    {
        return 6 * data.a;
    }

    union Lib_Math_Polynomial3_t_Data_t
    {
        float data[4];
        struct
        {
            float a;
            float b;
            float c;
            float d;
        };
    };

private:
    Lib_Math_Polynomial3_t_Data_t data;
};

class CubicSpline
{
public:
    CubicSpline() = default;
    CubicSpline(const Point &p0, const Point &p1, const Point &slope)
    {
        if(p1.x== p0.x && p1.y == p0.y)
        {
            return; //避免出现nan
        }
        // 两个点
        x0 = p0.x;
        y0 = p0.y;
        x1 = p1.x;
        y1 = p1.y;

        // 斜率
        m0 = slope.x;
        m1 = slope.y;

        // 计算参数
        float h = x1 - x0;
        a = y0;
        b = m0;
        c = (3 * (y1 - y0) / (h * h)) - (m0 + 2 * m1) / h;
        d = (2 * (y0 - y1) / (h * h * h)) + (m0 + m1) / (h * h);
    }

    float operator()(float x)
    {
        if (x < x0 || x > x1)
        {
            return 0;
        }
        float h = x - x0;
        return a + b * h + c * h * h + d * h * h * h;
    }
    float dx(float x)
    {
        if (x < x0 || x > x1)
        {
            return 0;
        }
        float h = x - x0;
        return b + 2 * c * h + 3 * d * h * h;
    }

private:
    float x0, y0, x1, y1;
    float m0, m1;
    float a, b, c, d;
};
class TrapezoidalSpline 
{
    public:
    TrapezoidalSpline() = default;

    /**
    * @brief 构造函数，用于初始化梯形加减速插值。
    * @param x_start 运动起始位置。
    * @param x_end 运动结束位置。
    * @param max_v 最大速度（绝对值）。
    * @param max_a 最大加速度（绝对值）。
    */
    TrapezoidalSpline(float x_start, float x_end, float max_v, float max_a) 
    {
        // 确保输入的约束值是正数，直接取绝对值
        this->v_max =abs(max_v);
        this->a_max =abs(max_a);

        // 如果运动距离为零，或者约束无效，则运动时间为零
        if (x_end == x_start || this->v_max <= 0.0f || this->a_max <= 0.0f) 
        {
            x0 = x_start;
            this->x_end = x_end;
            t_end = 0.0f;
            is_trapezoidal = false;
            t_accel = 0.0f;
            t_coast = 0.0f;
            t_decel = 0.0f;
            return;
        }

        // 运动参数
        t0 = 0.0f; // 默认从t=0开始
        x0 = x_start;
        this->x_end = x_end;

        float dx = this->x_end - x0;
        // 根据位移方向调整加速度方向
        float direction = (dx >= 0) ? 1.0f : -1.0f;
        this->a_max *= direction;

        // 计算加速/减速阶段的位移
        float d_a = 0.5f * (this->v_max * this->v_max) /abs(this->a_max);

        // 判断是三角形轨迹还是梯形轨迹
        if (abs(dx) > 2.0f * d_a) {
            // 梯形轨迹：包含匀速段
            is_trapezoidal = true;
            t_accel = this->v_max /abs(this->a_max);
            float d_u =abs(dx) - 2.0f * d_a;
            t_coast = d_u / this->v_max;
            t_decel = t_accel;
        } 
        else 
        {
            // 三角形轨迹：没有匀速段
            is_trapezoidal = false;
            t_accel = std::sqrt(abs(dx) /abs(this->a_max));
            t_coast = 0.0f;
            t_decel = t_accel;
            this->v_max =
               abs(this->a_max) * t_accel * direction; // 更新实际达到的最大速度
        }

        // 计算运动总时间
        t_end = t_accel + t_coast + t_decel;
    }
    /**
     * @brief 通过最大时间与最大加速度来确定运动参数。
     * @param x_start 运动起始位置。
     * @param x_end 运动结束位置。
     * @param max_t 允许的最大运动时间。
     * @param max_a 最大加速度（绝对值）。
     */
    void byTimeAndAcc(float x_start, float x_end, float max_t, float max_a)
    {
        // 确保输入的约束值是正数，直接取绝对值
        this->a_max =abs(max_a);

        // 如果运动距离为零，或者约束无效，则运动时间为零
        if (x_end == x_start || this->a_max <= 0.0f || max_t <= 0.0f)
        {
            x0 = x_start;
            this->x_end = x_end;
            t_end = 0.0f;
            is_trapezoidal = false;
            t_accel = 0.0f;
            t_coast = 0.0f;
            t_decel = 0.0f;
            this->v_max = 0.0f;
            return;
        }

        // 运动参数
        t0 = 0.0f;
        x0 = x_start;
        this->x_end = x_end;
        float dx = this->x_end - x0;

        // 根据位移方向调整加速度方向
        float direction = (dx >= 0) ? 1.0f : -1.0f;
        this->a_max *= direction;

        // 加速/减速阶段所需的最短时间
        float t_accel_min = std::sqrt(abs(dx) /abs(this->a_max));

        // 判断是三角形轨迹还是梯形轨迹
        if (max_t <= 2.0f * t_accel_min)
        {
            // 三角形轨迹：没有匀速段，时间不够
            is_trapezoidal = false;
            t_accel = t_accel_min;
            t_coast = 0.0f;
            t_decel = t_accel;
            // 更新实际达到的最大速度
            this->v_max =abs(this->a_max) * t_accel * direction;
            t_end = 2.0f * t_accel; // 运动总时间
        }
        else
        {
            // 梯形轨迹：包含匀速段，时间充足
            is_trapezoidal = true;
            // 通过总时间计算加速时间
            float temp_t_accel = (max_t - std::sqrt(max_t * max_t - 4.0f *abs(dx) /abs(this->a_max))) / 2.0f;
            t_accel = temp_t_accel;
            t_decel = t_accel;

            // 计算匀速段时间
            t_coast = max_t - 2.0f * t_accel;

            // 计算最大速度
            this->v_max =abs(this->a_max) * t_accel * direction;
            t_end = max_t; // 运动总时间
        }
    }
    // 内部的成员函数都将基于 t0 = 0 进行计算
    float position(float t) const 
    {
        if (t < 0.0f)
        {
            return x0; // 如果时间小于0，返回起始位置
        }
        if (t > t_end)
        {
            return x_end;
        }

        if (t <= t_accel) 
        {
            return x0 + 0.5f * a_max * std::pow(t, 2);
        } 
        else if (t <= t_accel + t_coast) 
        {
            float x_accel_end = x0 + 0.5f * a_max * std::pow(t_accel, 2);
            return x_accel_end + a_max * (t_accel) * (t - t_accel);
        } 
        else 
        {
            float x_coast_end =
                x0 + 0.5f * a_max * std::pow(t_accel, 2) + a_max * (t_accel)*t_coast;
            float v_coast = a_max * (t_accel);
            return x_coast_end + v_coast * (t - (t_accel + t_coast)) -
                    0.5f * a_max * std::pow(t - (t_accel + t_coast), 2);
        }
    }

    float velocity(float t) const 
    {
        if (t < 0.0f || t > t_end)
        {
            return 0.0f;
        }
        if (t <= t_accel) 
        {
            return a_max * t;
        } 
        else if (t <= t_accel + t_coast) 
        {
            return a_max * t_accel;
        } 
        else 
        {
            float v_coast = a_max * t_accel;
            return v_coast - a_max * (t - (t_accel + t_coast));
        }
    }

    float acceleration(float t) const 
    {
        if (t < 0.0f || t > t_end)
        {
            return 0.0f;
        }
        if (t >= 0.0f && t < t_accel) 
        {
            return a_max;
        } 
        else if (t > t_accel + t_coast && t <= t_end) 
        {
            return -a_max;
        } 
        else 
        {
            return 0.0f;
        }
    }
    float max_t() const 
    {
        return t_end;
    }
    private:
    float t0 = 0.0f; // 默认从0开始
    float x0, x_end;
    float v_max, a_max;

    bool is_trapezoidal = false;
    float t_accel, t_coast, t_decel;
    float t_end;
};

float LinearInterpolation(float x, float x1, float x2, float y1, float y2);
float CubicSplineInterpolation(float x, float x1, float x2, float y1, float y2, float y1_, float y2_);

#endif
