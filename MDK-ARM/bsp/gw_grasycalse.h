#ifndef __GW_GRASYCALSE_H
#define  __GW_GRASYCALSE_H

#include "main.h"
#include "pid_template.h"


/* 默认地址 */
#define GW_GRAY_ADDR_DEF 0x4C
//PING命令
#define GW_GRAY_PING 0xAA
#define GW_GRAY_PING_OK 0x66
#define GW_GRAY_PING_RSP GW_GRAY_PING_OK

/* 开启开关数据模式 */
#define GW_GRAY_DIGITAL_MODE 0xDD

/* 开启连续读取模拟数据模式 */
#define GW_GRAY_ANALOG_BASE_ 0xB0
#define GW_GRAY_ANALOG_MODE  (GW_GRAY_ANALOG_BASE_ + 0)

/* 传感器归一化寄存器(v3.6及之后的固件) */
#define GW_GRAY_ANALOG_NORMALIZE 0xCF

/* 循环读取单个探头模拟数据 n从1开始到8 */
#define GW_GRAY_ANALOG(n) (GW_GRAY_ANALOG_BASE_ + (n))

 namespace GW_grasycalse
{
//****中心：3、4*****//
//0||1||2||3||4||5||6||7
enum Gw_GrayscaleMode_t
{
    GrasyCross,
    GrasyOnLine,
    OutLine,
};
class Gw_Grayscale_t
{
public:

    Gw_Grayscale_t() = default;
    Gw_Grayscale_t(I2C_HandleTypeDef * _I2c, unsigned char _Salve_Adress)
    {
        I2C_handle=_I2c;
        Salve_Adress=_Salve_Adress;
        pid.target_update(0);
    }
    unsigned char gw_ping(void );
    void read_data(void);//灰度读值
    float ReturnXControl(void)
    {
        float error = 0;

			      error -= data[0] * 9;
        error -= data[1] * 5;
        error -= data[2]*3;
        error -= data[3]*1;
        error += data[4] * 1;
        error += data[5] * 3;
			 error += data[6] * 5;
			 error += data[7] * 9;
        control = pidx.update(error);
        return control;
    }
    float ReturnCotorl(void)
    {
        float error = 0;
        // error -= data[0] * 9;
        error -= data[1] * 3;
        error -= data[2]*1;
        error += data[3]*1;
        error += data[4] * 3;
        // error += data[5] * 9;
        control = -pid.update(error); 
        return control;
    }
    bool IsCurrentMode(Gw_GrayscaleMode_t mode)
    {
        read_data();
        switch (mode)
        {
        case GrasyCross:
            return data[2] && data[3] && data[4] && data[5];
            break;
        case GrasyOnLine:
            return data[3] || data[4];
            break;
        case OutLine://全白线返回0，其他全返回1
            return data[1] || data[2]|| data[3]|| data[4] || (data[5]||data[0])||data[6]||data[7];
            break;
        default:
            return false;
            break;
        }
    }

protected:
    float control;
    uint8_t data[8];//有八个光电管，左0右8
	I2C_HandleTypeDef * I2C_handle;
    pid_base_template_t<float,float> pid = pid_base_template_t<float,float>({0.1, 0, 0.1, -3, 3});
    pid_base_template_t<float,float> pidx = pid_base_template_t<float,float>({0.025, 0, 0.01, -0.3, 0.3});
private:
unsigned char Salve_Adress;
unsigned char Digital_read_data;
unsigned char Anolog_read_data[8];
};

};

#endif

