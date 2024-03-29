
/*
 * L3GD20.c
 *
 *  Created on: Feb 25, 2022
 *      Author: ajanx
 */
#include "L3GD20.h"
#include "stm32h743xx.h"

HAL_StatusTypeDef L3GD20_CheckDevice(L3GD20 *dev)
{
	uint8_t id=0x0;
	if(HAL_I2C_IsDeviceReady(dev->i2cHandle, L3GD20_SA0H_R, 5, 100) == HAL_OK)
		L3GD20_ReadRegister(dev, L3GD20_WHO_AM_I, &id);
	return (id==L3GD20_Device_ID) ? (HAL_OK) : (HAL_ERROR);
}
#ifdef DEBUG_EN
L3GD20_ErrorTypeDef L3GD20_Init(L3GD20 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t* errHandling)
#else
L3GD20_ErrorTypeDef L3GD20_Init(L3GD20 *dev, I2C_HandleTypeDef *i2cHandle)
#endif
{
	dev->i2cHandle = i2cHandle;
#ifdef DEBUG_EN
	*errHandling = 0x0;
#endif
	uint8_t dataWr;
	if(L3GD20_CheckDevice(dev) == HAL_ERROR) {
#ifdef DEBUG_EN
		*errHandling = 0xF;
#endif
		return L3GD20_SENSOR_CONNECTION_ERROR;
	}
	
	dataWr = ODR380_CO100 | ENABLE_ALL_AXES | POWER_NORMAL_MODE;
	if(L3GD20_WriteRegister(dev, L3GD20_CTRL_REG1, &dataWr)!= HAL_OK) {
		#ifdef DEBUG_EN
		(*errHandling |= 0x1<<1);
		#endif
	}

	dataWr = HPF_NORMAL_MODE | FS250 | HPF_CUTOFF_FREQ5;
	L3GD20GyroSens = L3GD20_GYRO_SENS_250DPS;
		if(L3GD20_WriteRegister(dev, L3GD20_CTRL_REG2, &dataWr)!= HAL_OK) {
			#ifdef DEBUG_EN
			(*errHandling |= 0x1<<2);
			#endif
		}

	#ifdef DEBUG_EN
		return (*errHandling==0x0) ? L3GD20_REGISTER_WRITE_OK : L3GD20_REGISTER_WRITE_ERROR;
	#endif

	#ifndef DEBUG_EN
	return L3GD20_SENSOR_CONNECTION_OK;
	#endif
}

HAL_StatusTypeDef L3GD20_ReadGyro(L3GD20 *dev)
{
	uint8_t inComeData[6]; HAL_StatusTypeDef state; int16_t tempData[3];
	state  = L3GD20_ReadRegister(dev, L3GD20_OUT_X_L, &inComeData[0]);
	state |= L3GD20_ReadRegister(dev, L3GD20_OUT_X_H, &inComeData[1]);
	state |= L3GD20_ReadRegister(dev, L3GD20_OUT_Y_L, &inComeData[2]);
	state |= L3GD20_ReadRegister(dev, L3GD20_OUT_Y_H, &inComeData[3]);
	state |= L3GD20_ReadRegister(dev, L3GD20_OUT_Z_L, &inComeData[4]);
	state |= L3GD20_ReadRegister(dev, L3GD20_OUT_Z_H, &inComeData[5]);
	if (state==HAL_OK)
	{
		tempData[0] = (int16_t) (inComeData[1]<<8 | inComeData[0]);
		tempData[1] = (int16_t) (inComeData[3]<<8 | inComeData[2]);
		tempData[2] = (int16_t) (inComeData[5]<<8 | inComeData[4]);
		dev->gyroData[0]=(float)(tempData[0]) * L3GD20GyroSens * DPS_TO_RADS;
		dev->gyroData[1]=(float)(tempData[1]) * L3GD20GyroSens * DPS_TO_RADS;
		dev->gyroData[2]=(float)(tempData[2]) * L3GD20GyroSens * DPS_TO_RADS;
	}
	return state;
}

void CalGyro(L3GD20 *dev, int16_t ReadingDuration)
{
	uint32_t sum = 0;
    uint8_t ReadingInterval = 5;    /*ms*/
    uint32_t beginTime = HAL_GetTick();
    while (HAL_GetTick() - beginTime < ReadingDuration)
    {
		L3GD20_ReadGyro(dev);
		dev->gyroBias[0] += dev->gyroData[0];
		dev->gyroBias[1] += dev->gyroData[1];
		dev->gyroBias[2] += dev->gyroData[2];
        sum++;
		HAL_Delay(ReadingInterval);
    }
    dev->gyroBias[0] /= sum;
    dev->gyroBias[1] /= sum;
    dev->gyroBias[2] /= sum;
}
void ReadCalGyro(L3GD20 *dev)
{
	L3GD20_ReadGyro(dev);
	dev->gyroData[0] -= dev->gyroBias[0];
	dev->gyroData[1] -= dev->gyroBias[1];
	dev->gyroData[2] -= dev->gyroBias[2];
}
HAL_StatusTypeDef L3GD20_ReadRegister(L3GD20 *dev, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Read(dev->i2cHandle, L3GD20_SA0H_R, reg, 1, data,
			I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
}

HAL_StatusTypeDef L3GD20_WriteRegister(L3GD20 *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, L3GD20_SA0H_W, reg,
			I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
