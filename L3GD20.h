/*
 * L3GD20.h
 *
 *  Created on: Feb 25, 2022
 *      Author: ajanx
 */

#define C_SIMULATION
#ifndef C_SIMULATION
#include "stm32h7xx_hal.h"
#endif
#include <stdlib.h>

#define L3GD20_WHO_AM_I        ((uint8_t)0X0F) /*DEFAULT   11010100*/
#define L3GD20_CTRL_REG1       ((uint8_t)0X20) /*DEFAULT   00000111*/
#define L3GD20_CTRL_REG2       ((uint8_t)0X21) /*DEFAULT   00000000*/
#define L3GD20_CTRL_REG3       ((uint8_t)0X22) /*DEFAULT   00000000*/
#define L3GD20_CTRL_REG4       ((uint8_t)0X23) /*DEFAULT   00000000*/
#define L3GD20_CTRL_REG5       ((uint8_t)0X24) /*DEFAULT   00000000*/
#define L3GD20_OUT_TEMP        ((uint8_t)0X26) /*OUTPUT*/
#define L3GD20_STATUS_REG      ((uint8_t)0X27) /*OUTPUT*/
#define L3GD20_OUT_X_L         ((uint8_t)0X28) /*OUTPUT*/
#define L3GD20_OUT_X_H         ((uint8_t)0X29) /*OUTPUT*/
#define L3GD20_OUT_Y_L         ((uint8_t)0X2A) /*OUTPUT*/
#define L3GD20_OUT_Y_H         ((uint8_t)0X2B) /*OUTPUT*/
#define L3GD20_OUT_Z_L         ((uint8_t)0X2C) /*OUTPUT*/
#define L3GD20_OUT_Z_H         ((uint8_t)0X2D) /*OUTPUT*/
#define L3GD20_FIFO_CTRL_REG   ((uint8_t)0X2E) /*DEFAULT   00000000*/
#define L3GD20_FIFO_SRC_REG    ((uint8_t)0X2F) /*OUTPUT*/

#define L3GD20_SA0H_R			((uint8_t)(0xD7))
#define L3GD20_SA0H_W			((uint8_t)(0xD6))

#define L3GD20_Device_ID		((uint8_t)(0xD4))	/*WHO_AM_I register content*/

#define L3GD20_GYRO_SENS_250DPS (0.00875F) /*Sensitivity at 250 dps*/
#define L3GD20_GYRO_SENS_500DPS (0.0175F)  /*Sensitivity at 500 dps*/
#define L3GD20_GYRO_SENS_2000DPS (0.070F)  /*Sensitivity at 2000 dps*/

typedef struct {
    #ifndef C_SIMULATION
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
    #endif
	/* Gyroscope data (X, Y, Z)*/
	int16_t gyro[3];
	/* Temperature data in deg */
	int16_t temp_C;
} L3GD20;

typedef enum{
    //Axis Enable Bits
    Yen = 0X01, Xen = 0X02, Zen = 0X04,
    //Power down mode enable
    PD = 0X08,
    //Bandwidth selection
    BW0 = 0X10, BW1 = 0X20,
    //Output data rate selection
    DR0 = 0X40, DR1 = 0X80
}L3GD20_CTRL1TypeDef;
//************ REGISTER CTRL1 ************//
/*Data output rate & bandwidth configuration settings*/
#define ODR95_CO12_5    ((uint8_t)(0X00))               /*OUTPUT DATA RATE 95  HZ - CUT-OFF FREQ 12.5 HZ*/
#define ODR95_CO25      ((uint8_t)(BW0))              /*OUTPUT DATA RATE 95  HZ - CUT-OFF FREQ 25   HZ*/
#define ODR190_CO12_5   ((uint8_t)(DR0))              /*OUTPUT DATA RATE 190 HZ - CUT-OFF FREQ 12.5 HZ*/
#define ODR190_CO25     ((uint8_t)(DR0|BW0))          /*OUTPUT DATA RATE 190 HZ - CUT-OFF FREQ 25   HZ*/
#define ODR190_CO50     ((uint8_t)(DR0|BW1))          /*OUTPUT DATA RATE 190 HZ - CUT-OFF FREQ 50   HZ*/
#define ODR190_CO70     ((uint8_t)(DR0|BW0|BW1))      /*OUTPUT DATA RATE 190 HZ - CUT-OFF FREQ 70   HZ*/
#define ODR380_CO20     ((uint8_t)(DR1))              /*OUTPUT DATA RATE 380 HZ - CUT-OFF FREQ 20   HZ*/
#define ODR380_CO25     ((uint8_t)(DR1|BW0))          /*OUTPUT DATA RATE 380 HZ - CUT-OFF FREQ 25   HZ*/
#define ODR380_CO50     ((uint8_t)(DR1|BW1))          /*OUTPUT DATA RATE 380 HZ - CUT-OFF FREQ 50   HZ*/
#define ODR380_CO100    ((uint8_t)(DR1|BW0|BW1))      /*OUTPUT DATA RATE 380 HZ - CUT-OFF FREQ 100  HZ*/
#define ODR760_CO30     ((uint8_t)(DR0|DR1))          /*OUTPUT DATA RATE 760 HZ - CUT-OFF FREQ 30   HZ*/
#define ODR760_CO35     ((uint8_t)(DR0|DR1|BW0))      /*OUTPUT DATA RATE 760 HZ - CUT-OFF FREQ 35   HZ*/
#define ODR760_CO50     ((uint8_t)(DR0|DR1|BW1))      /*OUTPUT DATA RATE 760 HZ - CUT-OFF FREQ 50   HZ*/
#define ODR760_CO100    ((uint8_t)(DR0|DR1|BW0|BW1))  /*OUTPUT DATA RATE 760 HZ - CUT-OFF FREQ 100  HZ*/
/*Enable gyro mesurement on x y z axes*/
#define ENABLE_ALL_AXES     ((uint8_t)(Xen|Yen|Zen)
/*Power mode selection configuration*/
#define POWER_DOWN_MODE     ((uint8_t)(0X00))
#define POWER_NORMAL_MODE   ((uint8_t)(PD | ENABLE_ALL_AXES))
#define SLEEP_MODE          ((uint8_t)(PD))

typedef enum{
    /*High-pass filter cutoff frequency selection*/
    HPCF3 = 0x08, HPCF2 = 0x04, HPCF1 = 0x02, HPCF0 = 0x01,
    /*High-pass filter mode selection*/
    HPM1 = 0x10, HPM0 = 0x20
}L3GD20_CTRL2TypeDef;

//************ REGISTER CTRL2 ************//
/*High-pass filter mode configuration*/
#define HPF_NORMAL_MODE_RST ((uint8_t)(0x00))       /*Normal mode (reset reading HP_RESET_FILTER)*/
#define HPF_REF_SIG_FIL     ((uint8_t)(HPM0))       /*Reference signal for filtering */
#define HPF_NORMAL_MODE     ((uint8_t)(HPM1))       /*Normal mode*/
#define HPF_INT_RST         ((uint8_t)(HPM0|HPM1))  /*Autoreset on interrupt even*/

/*High-pass filter cut off frequency configuration [Hz]*/
#define HPF_CUTOFF_FREQ0    ((uint8_t)0x00)              /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ1    ((uint8_t)HPCF0)             /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ2    ((uint8_t)HPCF1)             /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ3    ((uint8_t)HPCF0|HPCF1)       /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ4    ((uint8_t)HPCF2)             /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ5    ((uint8_t)HPCF2|HPCF0)       /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ6    ((uint8_t)HPCF2|HPCF1)       /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ7    ((uint8_t)HPCF2|HPCF1|HPCF0) /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ8    ((uint8_t)HPCF3)             /*Refer to datasheet p.33*/
#define HPF_CUTOFF_FREQ9    ((uint8_t)HPCF3|HPCF0)       /*Refer to datasheet p.33*/

//************ REGISTER CTRL4 ************//
#define FS250   ((uint8_t)(0X00))
#define FS500   ((uint8_t)(0X10))
#define FS2000  ((uint8_t)(0X20))

//************ REGISTER CTRL5 ************//
#define HPF_EN  ((uint8_t)(0x10))

typedef enum {
	REGISTER_WRITE_ERROR=0x00,
	REGISTER_WRITE_OK=0x01,
	SENSOR_CONNECTION_ERROR=0x02,
	SENSOR_CONNECTION_OK=0x03
} L3GD20_ErrorTypeDef;

float L3GD20_SENS;

#ifndef C_SIMULATION

HAL_StatusTypeDef L3GD20_CheckDevice(L3GD20 *dev);
LSM303_ErrorTypeDef L3GD20_Init(L3GD20 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t* errHandling);
HAL_StatusTypeDef L3GD20_ReadGyro(L3GD20 *dev);
HAL_StatusTypeDef L3GD20_ReadRegister(L3GD20 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef L3GD20_WriteRegister(L3GD20 *dev, uint8_t reg, uint8_t *data);

#endif