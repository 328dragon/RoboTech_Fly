#include "gw_grasycalse.h"
using namespace GW_grasycalse;

// 底层
unsigned char IIC_ReadByte(I2C_HandleTypeDef *_i2c, unsigned char Salve_Adress)
{
    unsigned char dat;
    HAL_I2C_Master_Receive(_i2c, Salve_Adress << 1, &dat, 1, 1000);
    return dat;
}

unsigned char IIC_ReadBytes(I2C_HandleTypeDef *_i2c, unsigned char Salve_Adress, unsigned char Reg_Address, unsigned char *Result, unsigned char len)
{
    return HAL_I2C_Mem_Read(_i2c, Salve_Adress, Reg_Address, I2C_MEMADD_SIZE_8BIT, Result, len, 1000) == HAL_OK;
}

unsigned char IIC_WriteByte(I2C_HandleTypeDef *_i2c, unsigned char Salve_Adress, unsigned char Reg_Address, unsigned char data)
{
    unsigned char dat[2] = {Reg_Address, data};
    return HAL_I2C_Master_Transmit(_i2c, Salve_Adress, dat, 2, 1000) == HAL_OK;
}
unsigned char IIC_WriteBytes(I2C_HandleTypeDef *_i2c, unsigned char Salve_Adress, unsigned char Reg_Address, unsigned char *data, unsigned char len)
{

    return HAL_I2C_Mem_Write(_i2c, Salve_Adress, Reg_Address, I2C_MEMADD_SIZE_8BIT, data, len, 1000) == HAL_OK;
}

unsigned char IIC_Get_Digtal(I2C_HandleTypeDef *_i2c, unsigned char Salve_Adress)
{
    unsigned char dat;
    IIC_ReadBytes(_i2c, Salve_Adress << 1, GW_GRAY_DIGITAL_MODE, &dat, 1);
    return dat;
}
unsigned char IIC_Get_Anolog(I2C_HandleTypeDef *_i2c, unsigned char Salve_Adress, unsigned char *Result, unsigned char len)
{

    if (IIC_ReadBytes(_i2c, Salve_Adress << 1, GW_GRAY_ANALOG_BASE_, Result, len))
        return 1;
    else
        return 0;
}
unsigned char IIC_Anolog_Normalize(I2C_HandleTypeDef *_i2c, unsigned char Salve_Adress, uint8_t Normalize_channel)
{
    return IIC_WriteByte(_i2c,Salve_Adress << 1, GW_GRAY_ANALOG_NORMALIZE, Normalize_channel);
}


// 封装后
unsigned char Gw_Grayscale_t::gw_ping(void)
{
    unsigned char dat;
    IIC_ReadBytes(I2C_handle, Salve_Adress << 1, GW_GRAY_PING, &dat, 1);

    if ((dat == GW_GRAY_PING_OK))
    {
        return 0;
    }
    else
        return 1;
}

void Gw_Grayscale_t::read_data(void)
{
    Digital_read_data = IIC_Get_Digtal(I2C_handle, Salve_Adress);
    for (int i = 0; i < 8; i++)
    {
        data[i] = 1 - ((Digital_read_data >> i) & 0x01); // 读取侧边数字灰度传感器数据
    }

    // 获取传感器模拟量结果
    if (IIC_Get_Anolog(I2C_handle, Salve_Adress, Anolog_read_data, 8))
    {
    }

    // 获取传感器归一化结果
    IIC_Anolog_Normalize(I2C_handle,Salve_Adress, 0xff); // 所有通道归一化都打开
  
}
