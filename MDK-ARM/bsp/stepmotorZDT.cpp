#include "stepmotorZDT.hpp"
#include "Lib_Common.h"
#include "math.h"
#define abs(x) ((x) > 0 ? (x) : -(x))
/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
uint8_t Step_Pos_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 返回命令长度
    return (uint8_t)13;
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                // 地址
    cmd[1] = 0xF6;                // 功能码
    cmd[2] = dir;                 // 方向
    cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
    cmd[5] = acc;                 // 加速度，注意：0是直接启动
    cmd[6] = snF;                 // 多机同步运动标志
    cmd[7] = 0x6B;                // 校验字节

    // 发送命令
    return (uint8_t)8;
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
uint8_t Step_Synchronous_motion(uint8_t *cmd, uint8_t addr)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0xFF; // 功能码
    cmd[2] = 0x66; // 辅助码
    cmd[3] = 0x6B; // 校验字节
    return (4);
}

void StepMotorZDT_t::SetLinearSpeed(float target)
{
    _target_rpm = (int16_t)(target * 60 / (3.14 * _wheel_diameter)); // 转化为转速(RPM)
    // 发送数据到电机
    uint8_t len;
    if (_target_rpm > 0)
    {
        len = Step_Vel_Control(_cmd_buffer, _id, _posDirection, (uint16_t)_target_rpm, 0, true);
    }
    else
    { // 反转
        auto dir_trans = _posDirection == 0 ? 1 : 0;
        len = Step_Vel_Control(_cmd_buffer, _id, dir_trans, (uint16_t)(-_target_rpm), 0, true);
    }
    HAL_UART_Transmit(_USART, _cmd_buffer, len, 1000); // 发送数据到电机
    delay(1);                                          // 傻逼电机需要延迟避免重包
    if (_have_pub_permission)
    {
        // 发布同步信号
        len = Step_Synchronous_motion(_cmd_buffer, 0);     // 发送数据到电机
        HAL_UART_Transmit(_USART, _cmd_buffer, len, 1000); // 发送数据到电机
        delay(1);
    }
}
void StepMotorZDT_t::Update()
{
    //获得电机数据
    uint8_t cmd[3] = {0};
    cmd[0] = _id; // 电机地址
    cmd[1] = 0x36;
    cmd[2] = 0x6B;                           // 校验字节
    // HAL_UART_Transmit(_USART, cmd, 3, 1000); //获得绝对位置
    // delay(1);
    cmd[1]=0x37; //获得电机位置误差
    HAL_UART_Transmit(_USART, cmd, 3, 1000); //获得电机位置误差
    delay(1);
}
float StepMotorZDT_t::GetLinearSpeed()
{
    // 返回电机的线速度
    //读取电机的速度
   
    return _target_rpm * 3.1415926 * _wheel_diameter / 60; // 转化为米每秒
}
void StepMotorZDT_t::UARTCallback(uint8_t *data)
{
    // 处理接收到的数据
    if (data[0] == _id ) // 检查地址和功能码
    {
        // 解析速度数据
        // if (data[1] == 0x36 && data[7] == 0x6B) // 确保数据长度足够
        // {
        //     uint64_t pose_abs= (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 解析位置数据
        //     _current_pose_raw = (data[2] == 0x0) ? pose_abs : -pose_abs; // 更新当前电机位置(电机维护方向)
        //     if(_dir!=0)
        //     {
        //         _current_pose_raw = -_current_pose_raw; // 如果是反转方向，则取反(控制维护的方向)
        //     }
        // }
        if(data[1] == 0x37 && data[7] == 0x6B) 
        {
            // 解析位置误差数据
            int64_t pose_error = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 解析位置误差数据
            pose_error= (data[2] == 0x0) ? pose_error : -pose_error; // 更新目标误差(电机维护方向)
            _target_pose_error=pose_error*360/65536.0;
            if(_posDirection!=0)
            {
                _target_pose_error = -_target_pose_error; // 如果是反转方向，则取反(控制维护的方向)
            }
        }
    }
}
/**
 * @brief 设置电机的加速度、速度和位置
 * 
 * @param acc_target 加速度取值范围1-255
 * @param vel_target 转速,单位为RPM,只接受正值
 * @param distance 线距离,包含正负
 */
void StepMotorZDT_t::SetAccSpeedPos(uint8_t acc_target, float vel_target, float distance)
{
    // 设置加速度、速度和位置
    //将角度转换为脉冲数
    if(acc_target>255 || acc_target<1)
    {
        return; // 加速度不合法
    }
    _target_pose_error=9999999.0f; // 重置目标误差
    _target_rpm =(uint16_t)(abs(vel_target) * 60 / (3.14 * _wheel_diameter)); // 转化为转速(RPM)
    double angle =abs(distance) * 180*2 / (_wheel_diameter * 3.1415926535); // 计算目标角度 正值
    uint16_t clk = (uint16_t)(angle *_div_num/ _step_angle); // 计算脉冲数
    // 发送数据到电机
    uint8_t dir = distance >= 0 ? _posDirection : (_posDirection == 0 ? 1 : 0); // 根据前进方向设置方向
    uint8_t len = Step_Pos_Control(_cmd_buffer, _id, dir, (uint16_t)_target_rpm, acc_target, clk, false, true);
    HAL_UART_Transmit(_USART, _cmd_buffer, len, 1000); // 发送数据到电机
    delay(1); // 傻逼电机需要延迟避免重包
    if (_have_pub_permission)
    {
        // 发布同步信号
        len = Step_Synchronous_motion(_cmd_buffer, 0);     // 发送数据到电机
        HAL_UART_Transmit(_USART, _cmd_buffer, len, 1000); // 发送数据到电机
        delay(1);
    }

}