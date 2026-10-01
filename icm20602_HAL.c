/*
 * icm20602.c
 *
 * Created on: May 29, 2026
 * Author: anonymous
 */

#include <icm20602_HAL.h>
#include <stdbool.h>
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

/* --------------------------------------------------------------------------
 * CS configuration
 * -------------------------------------------------------------------------- */

static GPIO_TypeDef *s_cs_port = NULL;
static uint16_t s_cs_pin = 0;

static void ICM20602_CS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * Set output latch HIGH before configuring the pin as an output.
     */
    HAL_GPIO_WritePin(
        s_cs_port,
        s_cs_pin,
        GPIO_PIN_SET
    );

    GPIO_InitStruct.Pin   = s_cs_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
        s_cs_port,
        &GPIO_InitStruct
    );
}

static void CS_Low(void)
{
    HAL_GPIO_WritePin(
        s_cs_port,
        s_cs_pin,
        GPIO_PIN_RESET
    );
}

static void CS_High(void)
{
    HAL_GPIO_WritePin(
        s_cs_port,
        s_cs_pin,
        GPIO_PIN_SET
    );
}

/* --------------------------------------------------------------------------
 * Low-level SPI
 * -------------------------------------------------------------------------- */

ICM20602_StatusCode_t ICM20602_WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {
        reg & 0x7F,
        data
    };

    CS_Low();

    HAL_StatusTypeDef hal_status =
        HAL_SPI_Transmit(
            &hspi1,
            tx,
            2,
            HAL_MAX_DELAY
        );

    CS_High();

    if (hal_status != HAL_OK)
    {
        return ICM20602_ERR_HAL;
    }

    return ICM20602_STATUS_OK;
}

ICM20602_StatusCode_t ICM20602_ReadRegister(
    uint8_t reg,
    uint8_t *data
)
{
    if (data == NULL)
    {
        return ICM20602_ERR_INVALID_ARG;
    }

    uint8_t tx[2] = {
        reg | ICM20602_READ,
        0xFF
    };

    uint8_t rx[2] = {
        0,
        0
    };

    CS_Low();

    HAL_StatusTypeDef hal_status =
        HAL_SPI_TransmitReceive(
            &hspi1,
            tx,
            rx,
            2,
            HAL_MAX_DELAY
        );

    CS_High();

    if (hal_status != HAL_OK)
    {
        *data = 0;
        return ICM20602_ERR_HAL;
    }

    *data = rx[1];

    return ICM20602_STATUS_OK;
}

/* --------------------------------------------------------------------------
 * Public functions
 * -------------------------------------------------------------------------- */

ICM20602_StatusCode_t ICM20602_Detect(void)
{
    uint8_t id = 0;

    ICM20602_StatusCode_t status =
        ICM20602_ReadRegister(
            ICM20602_REG_WHO_AM_I,
            &id
        );

    if (status != ICM20602_STATUS_OK)
    {
        return status;
    }

    if (id != ICM20602_VAL_WHO_AM_I)
    {
        return ICM20602_ERR_NOT_DETECTED;
    }

    return ICM20602_STATUS_OK;
}

ICM20602_StatusCode_t ICM20602_Init(
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin
)
{
	if (cs_port == NULL || cs_pin == 0)
	{
		return ICM20602_ERR_INVALID_ARG;
	}

	s_cs_port = cs_port;
	s_cs_pin  = cs_pin;

    /*
     * Establish a known-safe CS state before touching SPI.
     */
    ICM20602_CS_Init();

    ICM20602_StatusCode_t status;

    /* ----------------------------------------------------------------------
     * Reset chip
     * ---------------------------------------------------------------------- */

    status = ICM20602_WriteRegister(
        ICM20602_REG_PWR_MGMT_1,
        0x80
    );

    if (status != ICM20602_STATUS_OK)
    {
        return status;
    }

    HAL_Delay(100);

    /* ----------------------------------------------------------------------
     * Disable I2C interface
     * ---------------------------------------------------------------------- */

    status = ICM20602_WriteRegister(
        ICM20602_REG_USER_CTRL,
        0x10
    );

    if (status != ICM20602_STATUS_OK)
    {
        return status;
    }

    /* ----------------------------------------------------------------------
     * Wake up and select PLL clock
     * ---------------------------------------------------------------------- */

    status = ICM20602_WriteRegister(
        ICM20602_REG_PWR_MGMT_1,
        0x01
    );

    if (status != ICM20602_STATUS_OK)
    {
        return status;
    }

    HAL_Delay(15);

    /* ----------------------------------------------------------------------
     * Enable accelerometer and gyroscope axes
     * ---------------------------------------------------------------------- */

    status = ICM20602_WriteRegister(
        ICM20602_REG_PWR_MGMT_2,
        0x00
    );

    if (status != ICM20602_STATUS_OK)
    {
        return status;
    }

    /* ----------------------------------------------------------------------
     * Sample rate divider
     * ---------------------------------------------------------------------- */

    status = ICM20602_WriteRegister(
        ICM20602_REG_SMPLRT_DIV,
        0x00
    );

    if (status != ICM20602_STATUS_OK)
    {
        return status;
    }

    return ICM20602_STATUS_OK;
}

ICM20602_StatusCode_t ICM20602_Verify(void)
{
    uint8_t id = 0;

    ICM20602_StatusCode_t status =
        ICM20602_ReadRegister(
            ICM20602_REG_WHO_AM_I,
            &id
        );

    if (status != ICM20602_STATUS_OK)
    {
        return status;
    }

    if (id != ICM20602_VAL_WHO_AM_I)
    {
        return ICM20602_ERR_NOT_VERIFIED;
    }

    return ICM20602_STATUS_OK;
}

ICM20602_StatusCode_t ICM20602_ReadRawData(
    ICM20602_Data_t *data
)
{
    if (data == NULL)
    {
        return ICM20602_ERR_INVALID_ARG;
    }

    /*
     * Always establish a known output state before attempting the
     * transaction. This means callers never receive stale data if
     * this function fails.
     */
    data->accel_x = 0;
    data->accel_y = 0;
    data->accel_z = 0;
    data->temp    = 0;
    data->gyro_x  = 0;
    data->gyro_y  = 0;
    data->gyro_z  = 0;

    /*
     * Burst read:
     *
     * TX:
     *   [READ | ACCEL_XOUT_H] + 14 dummy bytes
     *
     * RX:
     *   [dummy] + 14 sensor bytes
     */
    uint8_t tx[15] = {
        ICM20602_REG_ACCEL_XOUT_H | ICM20602_READ
    };

    uint8_t rx[15] = {0};

    CS_Low();

    HAL_StatusTypeDef hal_status =
        HAL_SPI_TransmitReceive(
            &hspi1,
            tx,
            rx,
            15,
            HAL_MAX_DELAY
        );

    CS_High();

    if (hal_status != HAL_OK)
    {
        /*
         * Data was already cleared above.
         * Propagate the actual transport failure.
         */
        return ICM20602_ERR_HAL;
    }

    /* ----------------------------------------------------------------------
     * Convert raw sensor bytes
     * ---------------------------------------------------------------------- */

    data->accel_x =
        (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);

    data->accel_y =
        (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);

    data->accel_z =
        (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

    data->temp =
        (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);

    data->gyro_x =
        (int16_t)(((uint16_t)rx[9] << 8) | rx[10]);

    data->gyro_y =
        (int16_t)(((uint16_t)rx[11] << 8) | rx[12]);

    data->gyro_z =
        (int16_t)(((uint16_t)rx[13] << 8) | rx[14]);

    return ICM20602_STATUS_OK;
}

void __ICM20602_Handle_ErrCode(ICM20602_StatusCode_t status)
{
    switch (status)
    {
        case ICM20602_STATUS_OK:
            printf("STATUS_OK\r\n");
            break;

        case ICM20602_ERR_HAL:
            printf("ERR_HAL\r\n");
            break;

        case ICM20602_ERR_INVALID_ARG:
            printf("ERR_INVALID_ARG\r\n");
            break;

        case ICM20602_ERR_NOT_DETECTED:
            printf("ERR_NOT_DETECTED\r\n");
            break;

        case ICM20602_ERR_NOT_INITIALIZED:
            printf("ERR_NOT_INITIALIZED\r\n");
            break;

        case ICM20602_ERR_NOT_VERIFIED:
            printf("ERR_NOT_VERIFIED\r\n");
            break;

        case ICM20602_ERR_INVALID_DATA:
            printf("ERR_INVALID_DATA\r\n");
            break;

        default:
            printf("UNKNOWN_STATUS\r\n");
            break;
    }
}
