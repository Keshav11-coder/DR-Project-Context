#ifndef MTF02P_HAL_H
#define MTF02P_HAL_H

#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"

#include <stdint.h>


#define MTF02P_RING_SIZE       512U
#define MTF02P_MSP_MAX_PAYLOAD 32U


/* Functions represented by decoded MTF02P frames. */
typedef enum
{
    MTF02P_RANGEFINDER,
    MTF02P_OPTICAL_FLOW,
    MTF02P_NO_FUNCTION
} MTF02P_Function_t;


/* Decoded sensor data returned to the application. */
typedef struct
{
    MTF02P_Function_t function;

    uint8_t quality_distance;
    int32_t distance;

    uint8_t quality_motion;
    int32_t motion_x;
    int32_t motion_y;

} MTF02P_Data_t;

typedef struct
{
	uint32_t frames_ok;
	uint32_t frames_bad_crc;
	uint32_t frames_bad_size;
} MTF02P_Statistics_t;


/* Public driver status codes. */
typedef enum
{
    MTF02P_STATUS_OK = 0,
    MTF02P_STATUS_FRAME_READY,

    MTF02P_ERR_NOT_INITIALIZED,

    MTF02P_ERR_UART_INIT,
    MTF02P_ERR_DMA_INIT,
    MTF02P_ERR_DMA_START,

    MTF02P_ERR_NO_DATA,
    MTF02P_ERR_BAD_CRC,
    MTF02P_ERR_BAD_SIZE,
    MTF02P_ERR_RX_OVERRUN,
    MTF02P_ERR_INVALID_ARGUMENT,
    MTF02P_ERR_UNKNOWN_FUNCTION,

	MTF02P_ERR_INVALID_RANGE,
	MTF02P_ERR_INVALID_FLOW

} MTF02P_StatusCode_t;


/*
 * Initializes USART1, DMA reception, the circular receive buffer and the
 * MSP parser state.
 */
MTF02P_StatusCode_t MTF02P_Init(const uint32_t gpioPin);


/*
 * Receives available MSP data, validates the frame and decodes supported
 * rangefinder and optical-flow functions into the supplied data structure.
 */
MTF02P_StatusCode_t MTF02P_ReadRawData(MTF02P_Data_t *data);


MTF02P_Statistics_t MTF02P_Statistics();

/*
 * Attempts to receive and decode a valid MTF02P frame during the call.
 * STATUS_OK indicates that a recognized valid frame was decoded.
 */
MTF02P_StatusCode_t MTF02P_Detect(void);

/* % TEMPORARY % */
void __MTF02P_Handle_ErrCode(const MTF02P_StatusCode_t status_ref);


#endif /* MTF02P_HAL_H */
