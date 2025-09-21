/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:06
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-08-17 22:23:49
 * @FilePath: \MDK-ARMg:\project\stm32\f405rgb6\08_guosai\Core\Inc\maincpp.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#ifndef __MAINCPP_H
#define __MAINCPP_H
#ifdef __cplusplus
#include "cmsis_os.h"
#include "main.h"

extern "C" {
#endif

void main_cpp(void);
// void MyHAL_UARTECallback();
#ifdef __cplusplus
}
#endif
#endif