#include "mtf02p_HAL.h"

#include <stdio.h>
#include <string.h>


/* STM32 HAL peripheral state and DMA receive storage. */
static UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_rx;

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

static uint8_t mtf02p_rx_ring[MTF02P_RING_SIZE];


/* MSP parser state. */
typedef enum
{
    MSP_STATE_IDLE,
    MSP_STATE_HEADER_X,
    MSP_STATE_HEADER_ARROW,
    MSP_STATE_FLAG,
    MSP_STATE_FUNC_LOW,
    MSP_STATE_FUNC_HIGH,
    MSP_STATE_SIZE_LOW,
    MSP_STATE_SIZE_HIGH,
    MSP_STATE_PAYLOAD,
    MSP_STATE_CHECKSUM
} MTF02P_MSP_State_t;


/* MSP parser state and diagnostic counters. */
typedef struct
{
    MTF02P_MSP_State_t state;

    uint8_t  flag;
    uint16_t function;
    uint16_t size;

    uint8_t  payload[MTF02P_MSP_MAX_PAYLOAD];
    uint16_t payload_index;

    uint8_t crc;

    uint32_t frames_ok;
    uint32_t frames_bad_crc;
    uint32_t frames_bad_size;

} MTF02P_MSP_Parser_t;


/* DMA producer and software consumer positions. */
static volatile uint32_t mtf02p_dma_total = 0U;
static uint32_t mtf02p_ring_read_total = 0U;


/* Parser and driver initialization state. */
static MTF02P_MSP_Parser_t mtf02p_parser;
static uint8_t mtf02p_initialized = 0U;

typedef struct
{
    uint8_t  valid;
    uint16_t function;
    uint16_t size;
    uint8_t  payload[MTF02P_MSP_MAX_PAYLOAD];
} MTF02P_DecodedFrame_t;

static MTF02P_DecodedFrame_t mtf02p_last_frame;

static MTF02P_Statistics_t mtf02p_statistics;


/* Original HAL DMA completion callback and driver wrapper callback state. */
static void (*mtf02p_dma_original_cplt_callback)(DMA_HandleTypeDef *) = NULL;


/*
 * Calculates one CRC-8/DVB-S2 update for a single byte.
 */
static uint8_t mtf02p_crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (crc & 0x80U)
        {
            crc = (uint8_t)((crc << 1) ^ 0xD5U);
        }
        else
        {
            crc = (uint8_t)(crc << 1);
        }
    }

    return crc;
}


/*
 * Tracks DMA circular-buffer completion events and preserves the HAL
 * completion callback installed by the UART receive operation.
 */
static void MTF02P_DMA_RxCpltCallback(DMA_HandleTypeDef *hdma)
{
    mtf02p_dma_total += MTF02P_RING_SIZE;

    if (mtf02p_dma_original_cplt_callback != NULL)
    {
        mtf02p_dma_original_cplt_callback(hdma);
    }
}


/*
 * Processes one received byte through the MSP parser.
 */
static MTF02P_StatusCode_t MTF02P_MSP_Feed(
    MTF02P_MSP_Parser_t *parser,
    uint8_t byte)
{
    MTF02P_StatusCode_t status = MTF02P_STATUS_OK;

    switch (parser->state)
    {
        case MSP_STATE_IDLE:

            if (byte == '$')
            {
                parser->state = MSP_STATE_HEADER_X;
            }

            break;


        case MSP_STATE_HEADER_X:

            if (byte == 'X')
            {
                parser->state = MSP_STATE_HEADER_ARROW;
            }
            else if (byte == '$')
            {
                parser->state = MSP_STATE_HEADER_X;
            }
            else
            {
                parser->state = MSP_STATE_IDLE;
            }

            break;


        case MSP_STATE_HEADER_ARROW:

            if (byte == '<')
            {
                parser->crc = 0U;
                parser->state = MSP_STATE_FLAG;
            }
            else if (byte == '$')
            {
                parser->state = MSP_STATE_HEADER_X;
            }
            else
            {
                parser->state = MSP_STATE_IDLE;
            }

            break;


        case MSP_STATE_FLAG:

            parser->flag = byte;
            parser->crc = mtf02p_crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_FUNC_LOW;

            break;


        case MSP_STATE_FUNC_LOW:

            parser->function = byte;
            parser->crc = mtf02p_crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_FUNC_HIGH;

            break;


        case MSP_STATE_FUNC_HIGH:

            parser->function |= (uint16_t)byte << 8;
            parser->crc = mtf02p_crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_SIZE_LOW;

            break;


        case MSP_STATE_SIZE_LOW:

            parser->size = byte;
            parser->crc = mtf02p_crc8_dvb_s2(parser->crc, byte);
            parser->state = MSP_STATE_SIZE_HIGH;

            break;


        case MSP_STATE_SIZE_HIGH:

            parser->size |= (uint16_t)byte << 8;
            parser->crc = mtf02p_crc8_dvb_s2(parser->crc, byte);

            if (parser->size > MTF02P_MSP_MAX_PAYLOAD)
            {
                parser->frames_bad_size++;
                parser->state = MSP_STATE_IDLE;
                status = MTF02P_ERR_BAD_SIZE;
            }
            else if (parser->size == 0U)
            {
                parser->state = MSP_STATE_CHECKSUM;
            }
            else
            {
                parser->payload_index = 0U;
                parser->state = MSP_STATE_PAYLOAD;
            }

            break;


        case MSP_STATE_PAYLOAD:

            parser->payload[parser->payload_index++] = byte;
            parser->crc = mtf02p_crc8_dvb_s2(parser->crc, byte);

            if (parser->payload_index >= parser->size)
            {
                parser->state = MSP_STATE_CHECKSUM;
            }

            break;


        case MSP_STATE_CHECKSUM:

            parser->state = MSP_STATE_IDLE;

            if (byte == parser->crc)
            {
                parser->frames_ok++;

                /*
                 * Atomic, frame-local snapshot.
                 *
                 * This copy occurs at the exact instant this frame's
                 * checksum validates, before any byte belonging to a
                 * subsequent frame can modify parser->function,
                 * parser->size, or parser->payload.
                 */
                mtf02p_last_frame.function = parser->function;
                mtf02p_last_frame.size     = parser->size;

                memcpy(
                    mtf02p_last_frame.payload,
                    parser->payload,
                    parser->size
                );

                mtf02p_last_frame.valid = 1U;

                status = MTF02P_STATUS_FRAME_READY;
            }
            else
            {
                parser->frames_bad_crc++;
                status = MTF02P_ERR_BAD_CRC;
            }

            break;


        default:

            parser->state = MSP_STATE_IDLE;

            break;
    }

    return status;
}

static uint32_t MTF02P_DMA_ProducerPosition(void)
{
    uint32_t total_before;
    uint32_t total_after;
    uint16_t remaining;

    do
    {
        total_before = mtf02p_dma_total;

        remaining =
            (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        total_after = mtf02p_dma_total;
    }
    while (total_before != total_after);

    return total_after +
           (MTF02P_RING_SIZE - remaining);
}


/*
 * Resynchronizes the receive consumer and parser after DMA buffer overrun.
 */
static void MTF02P_ResetAfterOverrun(uint32_t producer)
{
    mtf02p_ring_read_total = producer;

    /*
     * Reset parser state only.
     * Do NOT clear diagnostic counters.
     */
    mtf02p_parser.state = MSP_STATE_IDLE;
    mtf02p_parser.flag = 0U;
    mtf02p_parser.function = 0U;
    mtf02p_parser.size = 0U;
    mtf02p_parser.payload_index = 0U;
    mtf02p_parser.crc = 0U;

    mtf02p_last_frame.valid = 0U;
}


/*
 * Drains newly received bytes from the DMA circular buffer and reports
 * parser events observed during the operation.
 */
static MTF02P_StatusCode_t MTF02P_UART1_Drain(
    MTF02P_MSP_Parser_t *parser)
{
    uint32_t producer;
    uint32_t unread;

    uint8_t frame_ready = 0U;
    uint8_t bad_crc = 0U;
    uint8_t bad_size = 0U;

    producer = MTF02P_DMA_ProducerPosition();

    unread = producer - mtf02p_ring_read_total;

//    printf(
//		"RX OVERRUN DEBUG: producer=%lu read=%lu unread=%lu "
//		"dma_total=%lu ndtr=%u\r\n",
//		(unsigned long)producer,
//		(unsigned long)mtf02p_ring_read_total,
//		(unsigned long)unread,
//		(unsigned long)mtf02p_dma_total,
//		(unsigned)__HAL_DMA_GET_COUNTER(&hdma_usart1_rx)
//	);

    if (unread > MTF02P_RING_SIZE)
    {
        MTF02P_ResetAfterOverrun(producer);
        return MTF02P_ERR_RX_OVERRUN;
    }

    while (mtf02p_ring_read_total != producer)
    {
        uint16_t index =
            (uint16_t)(mtf02p_ring_read_total % MTF02P_RING_SIZE);

        uint8_t byte = mtf02p_rx_ring[index];

        mtf02p_ring_read_total++;

        MTF02P_StatusCode_t byte_status =
            MTF02P_MSP_Feed(parser, byte);

        switch (byte_status)
        {
            case MTF02P_STATUS_FRAME_READY:
                frame_ready = 1U;
                break;

            case MTF02P_ERR_BAD_CRC:
                bad_crc = 1U;
                break;

            case MTF02P_ERR_BAD_SIZE:
                bad_size = 1U;
                break;

            case MTF02P_STATUS_OK:
            default:
                break;
        }
    }

    if (frame_ready != 0U)
    {
        return MTF02P_STATUS_FRAME_READY;
    }

    if (bad_crc != 0U)
    {
        return MTF02P_ERR_BAD_CRC;
    }

    if (bad_size != 0U)
    {
        return MTF02P_ERR_BAD_SIZE;
    }

    return MTF02P_STATUS_OK;
}


/*
 * Initializes USART1, DMA reception, the circular receive buffer and the
 * MSP parser state.
 */
MTF02P_StatusCode_t MTF02P_Init(const uint32_t gpioPin)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin       = gpioPin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;

    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        mtf02p_initialized = 0U;

        return MTF02P_ERR_UART_INIT;
    }

    hdma_usart1_rx.Instance                 = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        mtf02p_initialized = 0U;
        return MTF02P_ERR_DMA_INIT;
    }

    __HAL_LINKDMA(
        &huart1,
        hdmarx,
        hdma_usart1_rx);

    mtf02p_ring_read_total = 0U;
    mtf02p_dma_total = 0U;
    mtf02p_parser = (MTF02P_MSP_Parser_t){0};
    mtf02p_parser.state = MSP_STATE_IDLE;

    mtf02p_last_frame = (MTF02P_DecodedFrame_t){0};

    mtf02p_dma_original_cplt_callback = NULL;

    __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    if (HAL_UART_Receive_DMA(
            &huart1,
            mtf02p_rx_ring,
            MTF02P_RING_SIZE) != HAL_OK)
    {
        mtf02p_initialized = 0U;
        return MTF02P_ERR_DMA_START;
    }

    mtf02p_dma_original_cplt_callback =
        hdma_usart1_rx.XferCpltCallback;

    hdma_usart1_rx.XferCpltCallback =
        MTF02P_DMA_RxCpltCallback;

    mtf02p_initialized = 1U;
    return MTF02P_STATUS_OK;
}

MTF02P_Statistics_t MTF02P_Statistics()
{
	mtf02p_statistics.frames_ok = mtf02p_parser.frames_ok;
	mtf02p_statistics.frames_bad_crc = mtf02p_parser.frames_bad_crc;
	mtf02p_statistics.frames_bad_size = mtf02p_parser.frames_bad_size;

	return mtf02p_statistics;
}


/*
 * Receives available MSP data, validates the frame and decodes supported
 * rangefinder and optical-flow functions into the supplied data structure.
 */
MTF02P_StatusCode_t MTF02P_ReadRawData(MTF02P_Data_t *data)
{
    MTF02P_StatusCode_t status;

    if (mtf02p_initialized == 0U)
    {
        return MTF02P_ERR_NOT_INITIALIZED;
    }

    if (data == NULL)
    {
        return MTF02P_ERR_INVALID_ARGUMENT;
    }

    status =
        MTF02P_UART1_Drain(&mtf02p_parser);

    if (status == MTF02P_ERR_RX_OVERRUN)
    {
        data->function = MTF02P_NO_FUNCTION;

        return MTF02P_ERR_RX_OVERRUN;
    }

    if (status == MTF02P_ERR_BAD_CRC)
    {
        data->function = MTF02P_NO_FUNCTION;

        return MTF02P_ERR_BAD_CRC;
    }

    if (status == MTF02P_ERR_BAD_SIZE)
    {
        data->function = MTF02P_NO_FUNCTION;

        return MTF02P_ERR_BAD_SIZE;
    }

    if (status != MTF02P_STATUS_FRAME_READY)
    {
        data->function = MTF02P_NO_FUNCTION;

        return MTF02P_ERR_NO_DATA;
    }

    if (mtf02p_last_frame.valid == 0U)
    {
        /*
         * Defensive consistency check. MTF02P_UART1_Drain() reported
         * FRAME_READY, therefore this should not normally occur.
         */
        data->function = MTF02P_NO_FUNCTION;
        return MTF02P_ERR_NO_DATA;
    }

    switch (mtf02p_last_frame.function)
    {
        case 0x1F01:
        {
            if (mtf02p_last_frame.size != 5U)
            {
                data->function = MTF02P_NO_FUNCTION;
                mtf02p_last_frame.valid = 0U;
                return MTF02P_ERR_BAD_SIZE;
            }

            int32_t distance_mm =
                (int32_t)(
                    (uint32_t)mtf02p_last_frame.payload[1] |
                    ((uint32_t)mtf02p_last_frame.payload[2] << 8) |
                    ((uint32_t)mtf02p_last_frame.payload[3] << 16) |
                    ((uint32_t)mtf02p_last_frame.payload[4] << 24)
                );

            data->quality_distance =
                mtf02p_last_frame.payload[0];

            data->distance = distance_mm;
            data->function = MTF02P_RANGEFINDER;

            mtf02p_last_frame.valid = 0U;

            return MTF02P_STATUS_OK;
        }

        case 0x1F02:
        {
            if (mtf02p_last_frame.size != 9U)
            {
                data->function = MTF02P_NO_FUNCTION;
                mtf02p_last_frame.valid = 0U;
                return MTF02P_ERR_BAD_SIZE;
            }

            uint8_t quality =
                mtf02p_last_frame.payload[0];

            int32_t motion_x =
                (int32_t)(
                    (uint32_t)mtf02p_last_frame.payload[1] |
                    ((uint32_t)mtf02p_last_frame.payload[2] << 8) |
                    ((uint32_t)mtf02p_last_frame.payload[3] << 16) |
                    ((uint32_t)mtf02p_last_frame.payload[4] << 24)
                );

            int32_t motion_y =
                (int32_t)(
                    (uint32_t)mtf02p_last_frame.payload[5] |
                    ((uint32_t)mtf02p_last_frame.payload[6] << 8) |
                    ((uint32_t)mtf02p_last_frame.payload[7] << 16) |
                    ((uint32_t)mtf02p_last_frame.payload[8] << 24)
                );

            data->quality_motion = quality;
            data->motion_x = motion_x;
            data->motion_y = motion_y;
            data->function = MTF02P_OPTICAL_FLOW;

            mtf02p_last_frame.valid = 0U;

            return MTF02P_STATUS_OK;
        }

        default:
        {
            data->function = MTF02P_NO_FUNCTION;

            mtf02p_last_frame.valid = 0U;

            return MTF02P_ERR_UNKNOWN_FUNCTION;
        }
    }
}


/*
 * Attempts to receive and decode a valid MTF02P frame during the call.
 * STATUS_OK indicates that a recognized valid frame was decoded.
 */
MTF02P_StatusCode_t MTF02P_Detect(void)
{
    MTF02P_Data_t data;

    if (mtf02p_initialized == 0U)
    {
        return MTF02P_ERR_NOT_INITIALIZED;
    }

    MTF02P_StatusCode_t status = MTF02P_ReadRawData(&data);

    return status;
}


/*
 * Prints a diagnostic message corresponding to a public MTF02P status code.
 */
void __MTF02P_Handle_ErrCode(
    const MTF02P_StatusCode_t status_ref)
{
    switch (status_ref)
    {
        case MTF02P_ERR_NOT_INITIALIZED:
            printf("[MTF02P] ERR_NOT_INITIALIZED\r\n");
            break;

        case MTF02P_ERR_UART_INIT:
            printf("[MTF02P] ERR_UART_INIT\r\n");
            break;

        case MTF02P_ERR_DMA_INIT:
            printf("[MTF02P] ERR_DMA_INIT\r\n");
            break;

        case MTF02P_ERR_DMA_START:
            printf("[MTF02P] ERR_DMA_START\r\n");
            break;

        case MTF02P_ERR_NO_DATA:
            printf("[MTF02P] ERR_NO_DATA\r\n");
            break;

        case MTF02P_ERR_BAD_CRC:
            printf("[MTF02P] ERR_BAD_CRC\r\n");
            break;

        case MTF02P_ERR_BAD_SIZE:
            printf("[MTF02P] ERR_BAD_SIZE\r\n");
            break;

        case MTF02P_ERR_RX_OVERRUN:
            printf("[MTF02P] ERR_RX_OVERRUN\r\n");
            break;

        case MTF02P_ERR_INVALID_ARGUMENT:
            printf("[MTF02P] ERR_INVALID_ARGUMENT\r\n");
            break;

        case MTF02P_ERR_UNKNOWN_FUNCTION:
            printf("[MTF02P] ERR_UNKNOWN_FUNCTION\r\n");
            break;

        default:
            break;
    }
}
