/*
 * @Author: Elaina
 * @Date: 2024-08-17 21:13:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 23:09:14
 * @FilePath: \MDK-ARMg:\project\stm32\f405rgb6\08_guosai\Core\Src\maincpp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
//2（0）   1（1）
//
//3（2）   4（3）

#include "maincpp.h"
#define PI 3.1415926535
#include "FreeRTOS.h"
#include "Kinematic.h"
#include "bsp_usart.h"
#include "controller.h"
#include "host_control.hpp"
#include "planner.h"
#include "stepmotorZDT.hpp"
#include "task.h"
#include "ch040.h"

float DEBUG1 = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
// 实例化Map并将初始点设置成startInfo
StepMotorZDT_t *stepmotor_list_ptr[4];
TaskHandle_t Chassic_control_handle; // 底盘更新
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划
USARTInstance StepMotorUart;         // 步进电机串口实例
USARTInstance ch040Uart;             // ch040串口实例
TaskHandle_t Ontest_handle;
void ontest(void *pvParameters);
void OnChassicControl(void *pvParameters);
void OnKinematicUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void StepCallBack(void *param);
void ch040CallBack(void *param);
// 现在有三种控制方法
/*一是基于自身坐标系下的速度闭环*/
/*二是基于大地坐标系下的速度闭环*/
/*三是基于自身坐标系下的位置闭环*/
/*一只要一开始给一个控制量*/
/*二与三需要实时更新*/
void main_cpp(void)
{
  // stepmotor_ptr = new StepMotorZDT_t(1, &huart1, true, 1);
  // stepmotor_list_ptr = new LibList_t<StepMotorZDT_t *>();
  USART_Init_Config_s init_config;
//底盘控制串口
  init_config.recv_buff_size = 50;
  init_config.usart_handle = &huart3;
  init_config.param = nullptr;
  init_config.module_callback = StepCallBack; // 这里传入的是静态函数,需要注意参数类型
  USARTRegister(&StepMotorUart, &init_config);
//ch040陀螺仪串口
  init_config.usart_handle = &huart6;
  init_config.recv_buff_size = 100;
  init_config.module_callback = ch040CallBack; // 这里传入的是静态函数,需要注意参数类型
  USARTRegister(&ch040Uart, &init_config);
//电机实例化
  stepmotor_list_ptr[0] = new StepMotorZDT_t(2, &huart3, false, 0);
  stepmotor_list_ptr[1] = new StepMotorZDT_t(1, &huart3, false, 1);
  stepmotor_list_ptr[2] = new StepMotorZDT_t(3, &huart3, false, 0);
  stepmotor_list_ptr[3] = new StepMotorZDT_t(4, &huart3, true, 1);
  KinematicOdom = KinematicOdom_t(0.17,class_t::Rub_shape); // 初始化运动学模型
  // 需要用reinterpret_cast转换到父类指针类型
  Controller = StepController_t(stepmotor_list_ptr);
  //上位机控制
  HostPtr =
      new HostControl_t(&huart4);
  //任务开启
  BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 600, NULL,
                               3, &Chassic_control_handle);
  BaseType_t ok3 =
      xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
  BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 1000, NULL, 4,
                               &Planner_update_handle);
		 BaseType_t ok5 = xTaskCreate(ontest, "ontest_work", 200, NULL, 2,
                               &Ontest_handle);													 
  //   if (ok != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS)
  if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS)
  {
		
    // 任务创建失败，进入死循环
    while (1)
    {
      // uart_printf("create task failed\n");
    }
  }
}

void Onmaincpp(void *pvParameters)
{
  UNUSED(pvParameters);
	vTaskDelay(1000);
	ch040.setYawZero();

 auto& result=Controller.SetClosePosition({0.2, 0, 0});//4.45
   while(!result.isResolved())
 {

    vTaskDelay(10);
  }
// //原地转弯
result=Controller.SetClosePosition({0.2, 0,1.7});//4.45
   while(!result.isResolved())
{
vTaskDelay(10);
}
vTaskDelay(100);


////清空里程计
ch040.setYawZero();
Controller.Clear();

//走到左边道
 result=Controller.SetClosePosition({0.5, 0, 0});
   while(!result.isResolved())
{
vTaskDelay(10);
}
//Controller.Clear();
ch040.setYawZero();
vTaskDelay(100);
////旋转一个
 result=Controller.SetClosePosition({0.5, 0, 1.7});
   while(!result.isResolved())
{
vTaskDelay(10);
}
vTaskDelay(100);
//////清空里程计
ch040.setYawZero();
Controller.Clear();
vTaskDelay(500);
////回家
  result=Controller.SetClosePosition({0.2, 0,  0});
    while(!result.isResolved())
    {
      vTaskDelay(10);
    }
	
	



  while (1)
  {

    vTaskDelay(1000);
  }
}

void ontest(void *pvParameters)
{
  UNUSED(pvParameters);
  while (1)
  {
//    Controller.SetVelTarget({DEBUG1, DEBUG2, DEBUG3});
    vTaskDelay(1000);
  }
}

void OnPlannerUpdate(void *pvParameters)
{
  UNUSED(pvParameters);
  uint16_t last_tick = xTaskGetTickCount();
  // Kinematic.init(0.6, 2, 0.2); // 初始化运动学模型
  while (1)
  {
    uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
    last_tick = xTaskGetTickCount();
    Planner.update(dt);
    vTaskDelay(40);
  }
}
void OnChassicControl(void *pvParameters)
{
  UNUSED(pvParameters);
  uint16_t last_tick = xTaskGetTickCount();

  while (1)
  {
    uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
    last_tick = xTaskGetTickCount();
    // ChassisControl_ptr->KinematicAndControlUpdate(dt, imu.getyaw());
    Controller.KinematicAndControlUpdate(dt, ch040.getYaw());
    // 步进不需要速度环，此处仅为了读取电机速度
    // ChassisControl_ptr->MotorUpdate(dt);
    vTaskDelay(2);
  }
}
void StepCallBack(void *param)
{
  UNUSED(param);
  for (auto i = 0; i < 4; i++)
  {
    if (stepmotor_list_ptr[i] != nullptr)
    {
      stepmotor_list_ptr[i]->UARTCallback(StepMotorUart.recv_buff);
    }
  }
}
void ch040CallBack(void *param)
{
  ch040.analyze_data(ch040Uart.recv_buff);
}


// .............................................'RW#####EEEEEEEEEEEEEEEEEEEEEEEEWW%%%%%%N%%%%%%NW"...........
// ............................................/W%E$$$$EEEE######EEEEEEEEEEEEEEEE%%@NN@@$@@N%%%%N%]~`........
// ........................................i}}I&XIIYYXF&R#E$$$$$EEE##EEEEEEEEEEEE$N$#$K1:!YW@N%%%%@N$KY]+";..
// .....................................!>>li!"~~~'~~~~~!"i/1lIFK#E$$$EEEEEEE$$EEE%I::.....,]E@@@NNN@M$E$R>..
// ....................................+1"""i>"""""!~''''~~~!!~~!>/]Y&#$$$EEEEWWEEE$F,.......:>IRE$#&I/>'....
// ...................................;*lX&NM@@NW$#RFIl1i"!~~"">>!~~~!i}Y&#$W$EW%$EEMi...........::...'l1....
// ]}/+>~,............................,*YRNNNN@@MMMMMMMM@WRF*1>!~"!~!!~~!>+1IK$W%%W%1.................!*+....
// FFF&K&FYYYI]/"'`....................!K%W$$$$$$$EEEEEEE$W%%%WE&I]+!~~~!">"~~i*#%@#...................';....
// }}}}}}]l*XR#$WWERXl/!,:........,>>i/YK&&&&KKKKRR##EE$$$$$EEEE$$$EKYl/>!'~!"!+]IRNI..................'':...
// lllll]]]]}IYYXFK#W%N%$RFl+~`..`X/>>>!~~~~~!!"""">>ii+/}*YXK#EE$$WWWW$#Fl+"'~+**]*FI"................>i....
// ]]]]]]]]]]YXXXXYYXFRE$WW%%W#FlXl;!">+//i">"~'''''~!""!~~~!""i/1]*YFR#$%%WE&l/1]**lI&!.............>]]ll~..
// ]]]]]]]]]*XXXXXXXXYYX&R$$EE$$WWRR#WWWWW$E##KXI*1>!~!!""""!!!~~~'~~!!"+}I&R$NNWKYll*E"............"}/,~I&'.
// ]]]]]]]]lYXXXXXXXXXXXYYXKE$$E#E$$$$$$$$$$$$$$WWW$#X}1+>>""!''''~!!">""""""/]Y#W%$FRY............./+,.~lF>.
// ]]]]]]]]YXXXXXXXXXXXXXXXYYFKEW$E#EEEEEEEEEEEEEEEE$$WWW$$E#RFYl/+i!''!>"!!~!i]]]*XR#1'............!I/!]XI`.
// ]]]]]]]IXXXXXXXXXXXXXXXXXXYYYFE%WEEEEEEEEEEEEEEEEEEEEEEE$$$%%NN%$EKY]+i"!!"ilII*l]lXK/.:..........;+1/>:..
// ]]]]]]IXYXXXXXXXYYYYXXXXXXXYYR$RK$%$EEEEEEEEEEEEEEEEEEEEEEEE##EE$%NNNWE#R&&XI**llll]Y*.......::`,,`:::....
// ]]]]]*XXXXXXXXXXYYYYYYYXXXY&$#I/>/YE%$EEEEEEEEEEEEEEEEEEEEEEEEEEEE#EE$$WW$$W$ER&Y**]&}~+]IFRE$WW%%%%W$$#KX
// ]]]]lYXXXXXXXYYYYYYYYYYXXYK#I/ii+i>lYKWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$$WW$$#@%NMMM@@NNNN%%NNNNNN@@
// ]]]]YXXXXXXYYYYYYYYYYYXYYKX1iiiii+l1>i}KWWE#EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$W%@N%%%%%%%%%%%%%%%%%%%
// ]]]*XXXXXXXXXXXXYYYYYXYY&*++iiii+]+>++>11X$%$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N@@NNNN%%%%%%NNN%%
// ]]]YXXXXXXXYYYYYYXXYXYX&}i+iiiii1+iiii+*>>+*RWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N%W$$$$$$$$$WW%%
// ]]*XXYYYYYXX&K#$&YXXXXK}>+iiiii++iiii+FI>+i>>}F$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// }}IYYXFK#E%N%%NEYXXXYK}>iiiiiiiiii+>1I}]>iiii>"+*R$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// &XK#EWN@@%#YWN$YYXXY&l>iiiiii++iii>}I"il>iiiiiii"+1IR$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// $NN@N$&}!`.*NWXYXXYFI1/ii++i"!i>+>}l"!i]"iiiiiiii/i>/1IEW$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// "Y&l>,.:~1F@%KYXXXXF}Yi+i"',.';:,1]"+!'/;i">iiii>/i/1"1]IIX#$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// ......*@M@%RFIYYXYF1F}!'`::::!`."]""~'!]~"":~"iii/i}/+F***>i*YF#E$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// ......`+FWNWERFXYXl+Y`::::::,".!}~,:.:.~',*:::,'!+/}!/]!>l/"}X>i1lXRE$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// .........;/XEN@N%Wi&].`:::`:!',]/1i~,::`>`}>.`::.~1""1;::~1"+]li">>/llXRE$$$$$EE##EEEEEEEEEEEEEEE####EEEE$
// ............:'+lFK}N+:`:::`:!!&%NW$W$&]~,'!/~.``.~!.!''>+/IY>]lX/""11>ii+1lY&#$WWWW$$$$$E$$$$$$$WWW%%%W$#&
// .................>i~~,`:::`,}#M#}"'F%$W$}`;'!;.:`,','1XFK@@@RF@@@&~~~,!>ii>>+i/}lIXF&KR#$R#RRRKK&XIl1i!'`.
// ................`+'.~'`::`:`}$X`.::"&KFK&,:`'~!:`,'`~!;.;X&FKK$&l##l'::`;!>+/+i>>>>>>ii>I!.:..............
// ...:;..`!/:.....;i;."!`::`:;I+~.:.!E&FW#K':'',!"''';...;lYR#K&#K;"1#]~`::,:>i~i++++iii/}++................
// ...;Y.;/Y`......'>;.>+,::`.~E]:::.'K/"l}i`:`:::`;;'!;`.iK/}%&lRI.`1*'``:,,:+"~,,~"i++i1#+},...............
// ...:,>/.......~"'.;l;:``:'Y]'```./+~'';::``::::::`,`:~/~~i"!/'.:i;:`:,,:/+;''.::,'!"**>li...............
// ....:,.;;.......~"':.>i:`:',+l>;'';'";;,``:`:::::::::`,`'~~~~+":`~~```;;'>li!;`::::,.`Xi.1l`..............
// .......'];......`i;;.:I'::'I>1>'~~'';;,,```::::::::```,;;''~~''~~'`,'>>>>'/"~`,::::,`,X".`l+..............
// ........~~......."'~`/>+:::lll";'';;;,,,``:``;~::::```,;;'''~!>>"!!>ii!;:,~,'+!::::`;,Y~..,Y'.....::......
// .......:,.........!~/!.~+`.>]*>:,,,,,,````:`;;;`::````,,,,;;;~!'1/"!;`:::!~+]+"`::`:;;*~...~l.....`>,.....
// ........>..........>i!!++!'`/1Ii`::```````:::::`:::::`````,,,,`'i``:::::;]}/iii,:`::`!*~....>".....`/`....
// ........,`........;>.'>"~.i++]/ll+'`::::::``````::::::````::..,+```:`:`:"+ii+++':`:`:;Fi.....i`.....;+....
// ........`,.......`+:~!.;':i++/+i/}}1+>~;`::.....::::::...:,~i]X~`,:`:`:;/+++++1!:`::`.]I.....`"......+;...
// ........:~......:i,~'..";,ii+/+++++/1}]]}1/i>"!!;,,,,,>}lII**Y>`,:`:;,,/++++/+1+`:::`:,F,.....~,.....'>...
// ................"~'`..:1,,+i1+++//////}111}}}IY$K">>>!*NFl&X]>,,:`.'~`*]++++//+}':`:``.>1.....:".....'i...
// ...............~i':...!1.'+//+++//+/+]l+1]lIF]/Kl"">>+11>"1&i``::`!~;]I]++++//i1i:`::~;./;.....",....!;...
// ..............,1;.....]~.~+}++++/+///*]Y&F$Kl+}1!i++}1+i"11'`:::;!~i*]l]+//+/1++/,:`:~i``/.....'".........
// .............:]~....."}.:"}/+++////i1XRRYF*]lFKY/lI/`;,:"+~!:::,!"l&XYFY++/+/]++/~:`:~+".~i....;/.........
// .............+>.....`*;::/}++++//+/*#El}FIl*F&X]I*IX+`;1}'i::,~>+FYIX&#%#/++1]i++>:`:~++;.+'...~l.........
// ............;}......+!.:'l+++++1+/#N@/'i#F1!1]*"l*I*]+"+l}1+i+i>iRRE$$$ENIi+1]++//;::!++":`i.:.1].........
// ............/~....."i.`.+1i+++/1i*WW&~!1Wi`,+i/]Il>'`:.'I*Y>!>}FE$$EEEE#%*i+}}++1}!:`>+i+,.!~."Ii.........
// ............]'....'/.`;`1+++++//i&EW*~~YF'>+}]1//i"`.:''.Y+iY#$$EEEEEEEEW]i+}/++/*i`,++i/]::>+*l,.........
// ...........`l"...`/`:"~'/}/+++//YWEW*!+*+>iiI]]/">>i;,!`.lX$$EEEEEEEEEEEWN*+1++++Y1;'+i+i*+.~l}~..........
// ...........`l/,..+;.,+'~]*+//+1EW$E$X+1"""iY&i1l>"""ii+`,EWEEEEEEEEEEEEEE$W1++++i*]'>+i++//~.+;...........
// ............]}/`"".:!+~!I}+//iY%#$$EX1~""iFI*~i1]"">!/]`"%EEEEEEEEEEEEEEE$$1i++++/1"+++++};".!;...........
// ............'Y/1+.`:>+>/]1+//iXW#E$$**X>+F*/";">]l!"+&%I+WE$$$$$$$E$$WW%N%}i++++/>i+i++++]:;;,>...........
// .............~l}`::;i+i]i}+/+l@%$$E$&XYYY}!1,.!i>XIYX#N$REWEEEEEEEEEEEEE$N1+/+++/'iiiiii/1.`>:+...........
// ..............,i::;}i+i],*+++$@$EE$E%&']!~"+;~;>!"*}>$$$$EEE$EEEEEEEEEEE#%Fi//+1~'/i+++il>..>`i`..........
// ..............';:./]i++}.}li]NEEEE$W$}:i]+i!;~;i>i>,>%F/*$EEWWEEEEEEEEEEEE%Xi+//./+i++++*`..";i`..........