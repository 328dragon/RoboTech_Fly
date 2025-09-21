#ifndef CH040_H
#define CH040_H
#include  "main.h"
#include "string.h"
class ch040_t
{
    public:
      
        void analyze_data(uint8_t *data)
        {
            uint8_t data_length = data[2];
            if (data_length == 76) // HI91浮点型数据输入
            {
                memcpy(acc, &data[18], 12);
                memcpy(gyr, &data[30], 12);
                memcpy(mag, &data[42], 12);
                memcpy(imu, &data[54], 12);
                memcpy(quat, &data[66], 16);
            }
            // 处理yaw线转化成弧度
            yaw_raw= imu[2] * 3.1415926 / 180.0f;
            // 进行归一化
            if(yaw_raw<-3.1415926f)
            {
                yaw_raw += 2 * 3.1415926f;
            }
            else if(yaw_raw>3.1415926f)
            {
                yaw_raw -= 2 * 3.1415926f;
            }
            getYaw();
        }
        float getYaw()
        {
            yaw= yaw_raw - yaw_zero; // 减去零点偏移
            if (yaw < -3.1415926f)
            {
                yaw += 2 * 3.1415926f;
            }
            else if (yaw > 3.1415926f)
            {
                yaw -= 2 * 3.1415926f;
            }
            return yaw;
        }
        void setYawZero()
        {
            yaw_zero = yaw_raw; // 设置当前yaw为零点
        }
    private:
        float yaw_zero = 0;
        float yaw;
        // uint8_t origin_data[76];
        float acc[3];
        float gyr[3];
        float mag[3];
        float imu[3];
        float quat[4];
        float yaw_raw = 0;
};
inline ch040_t ch040; // 声明一个全局变量
#endif
