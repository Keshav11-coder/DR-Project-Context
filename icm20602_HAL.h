#ifndef ICM20602_H
#define ICM20602_H

#include "main.h"
#include <stdbool.h>
#include <stdio.h>

// WHO_AM_I
#define ICM20602_REG_WHO_AM_I       0x75
#define ICM20602_VAL_WHO_AM_I       0x12

// Power management
#define ICM20602_REG_PWR_MGMT_1     0x6B
#define ICM20602_REG_PWR_MGMT_2     0x6C
#define ICM20602_REG_USER_CTRL      0x6A

// Configuration
#define ICM20602_REG_SMPLRT_DIV     0x19
#define ICM20602_REG_CONFIG         0x1A
#define ICM20602_REG_GYRO_CONFIG    0x1B
#define ICM20602_REG_ACCEL_CONFIG   0x1C
#define ICM20602_REG_ACCEL_CONFIG_2 0x1D

// Data
#define ICM20602_REG_ACCEL_XOUT_H   0x3B

// SPI read flag
#define ICM20602_READ               0x80

typedef enum
{
    ICM20602_STATUS_OK = 0,

    ICM20602_ERR_HAL,             // HAL_ERROR / HAL_BUSY / HAL_TIMEOUT
    ICM20602_ERR_INVALID_ARG,     // NULL pointer / invalid argument
    ICM20602_ERR_NOT_DETECTED,    // WHO_AM_I does not match
    ICM20602_ERR_NOT_INITIALIZED, // Driver/device has not been initialized
    ICM20602_ERR_NOT_VERIFIED,    // Initialization/device state not verified
    ICM20602_ERR_INVALID_DATA,    // Received data is unusable/invalid
} ICM20602_StatusCode_t;

ICM20602_StatusCode_t ICM20602_Detect(void);
ICM20602_StatusCode_t ICM20602_Init(GPIO_TypeDef *cs_port, uint16_t cs_pin);
ICM20602_StatusCode_t ICM20602_Verify(void);

ICM20602_StatusCode_t ICM20602_WriteRegister(uint8_t reg, uint8_t data);
ICM20602_StatusCode_t ICM20602_ReadRegister(uint8_t reg, uint8_t *data);

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} ICM20602_Data_t;

ICM20602_StatusCode_t ICM20602_ReadRawData(ICM20602_Data_t *data);

void __ICM20602_Handle_ErrCode(ICM20602_StatusCode_t status);

#endif
