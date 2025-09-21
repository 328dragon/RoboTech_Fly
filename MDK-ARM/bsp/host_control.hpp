/*
 * @Author: Elaina
 * @Date: 2024-07-11 16:06:19
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-19 01:11:19
 * @FilePath: \MDK-ARM\Hardware\connect.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __CONNECT_H
#define __CONNECT_H
#include "main.h"
#include <stdarg.h>
#include "string.h"
#include "stdio.h"
#include "controller.h"
#include "planner.h"
#include "bsp_usart.h"
// 上位机控制类
class HostControl_t
{
public:
    HostControl_t()
    {
    }
    HostControl_t(UART_HandleTypeDef *huart) 
    {

        // USARTServiceInit(_huart);

        USART_Init_Config_s init_config;
        init_config.recv_buff_size = 50;
        init_config.usart_handle = huart;
        init_config.param = this;
        init_config.module_callback = USARTCallback; // 这里传入的是静态函数,需要注意参数类型
        // { reciveCallback(_instance.recv_buff); };
        USARTRegister(&_instance, &init_config);
    }
    // 兼容c的函数指针回调
    static void USARTCallback(void *param)
    {
        // 把 void* 转回 HostControl 指针
        HostControl_t *self = static_cast<HostControl_t *>(param);

        // 调用类内的成员函数
        self->reciveCallback(self->_instance.recv_buff);
    }
    /// @brief
    /// @param data
    void reciveCallback(uint8_t *data)
    {
        static float speed[3];
        if (data[0] == 0xFF && data[13] == 0xFE)
        {
            // 四字节转浮点数
            for (int i = 0; i < 12; i += 4)
            {
                speed[i / 4] = Byte2Float(data + 1 + i);
                // 限幅
                if (speed[i / 4] > 0.8)

                {
                    speed[i / 4] = 0.8;
                }
                else if (speed[i / 4] < -0.8)
                {
                    speed[i / 4] = -0.8;
                }
            }
            Controller.SetVelTarget({speed[0], speed[1], speed[2]});
        }
        // 大地速度
        else if (data[0] == 0xFD && data[13] == 0xFC)
        {
            for (int i = 0; i < 12; i += 4)
            {
                speed[i / 4] = Byte2Float(data + 1 + i);
                // 限幅
                if (speed[i / 4] > 0.8)

                {
                    speed[i / 4] = 0.8;
                }
                else if (speed[i / 4] < -0.8)
                {
                    speed[i / 4] = -0.8;
                }
            }
            Controller.SetGroundVelTarget({speed[0], speed[1], speed[2]});
        }
        else if (data[0] == 0xFB && data[13] == 0xFA)
        {
            for (int i = 0; i < 12; i += 4)
            {
                speed[i / 4] = Byte2Float(data + 1 + i);
                // _controller->SetClosePosition({speed[0], speed[1], speed[2]}, {0.01, 0.01, 0.02});
            }
            Planner.LoactaionCloseControl({speed[0], speed[1], speed[2]}, 1.3,0.5, {0.01, 0.01, 0.01}, false);
            // _controller->SetClosePosition({speed[0], speed[1], speed[2]}, {0.01, 0.01, 0.02});
        }
        //清零
        else if(data[0]==0x0d && data[1]==0x00 && data[2]==0x07 && data[3]==0x21)
        {
            Controller.Clear(); // 清除控制器状态
            Planner.Clear(); // 清除规划器状态
        }
        else if (data[0] == 0xEC && data[2] == 0XCE)
        {
            task_id = (int)data[1];
        }
    }
    static float Byte2Float(uint8_t *byte)
    {
        float f;
        uint8_t *p = (uint8_t *)&f;
        p[0] = byte[0];
        p[1] = byte[1];
        p[2] = byte[2];
        p[3] = byte[3];
        return f;
    }
    int8_t task_id = -1;

private:
    USARTInstance _instance;
};
inline HostControl_t * HostPtr = nullptr; // 全局指针,用于访问上位机控制类
#endif