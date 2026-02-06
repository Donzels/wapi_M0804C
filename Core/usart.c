#include "usart.h"

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

extern m0804c_handler_t g_wapi_handler_inst;

typedef struct
{
    UART_HandleTypeDef *uart_handle;
    DMA_HandleTypeDef *dma_handle;
} uart_recv_handler_t;

/*************************************** wapi *******************************************/
#define WAPI_RECV_BUF_SIZE 256

static uint8_t wapi_recv_buffer[WAPI_RECV_BUF_SIZE];

uart_recv_handler_t wapi_recv_handler =
    {
        .uart_handle = &huart1,
        .dma_handle = &hdma_usart1_rx,
};

static void usart_init(void)
{
    MX_USART1_UART_Init();
    __HAL_UART_ENABLE_IT(wapi_recv_handler.uart_handle, UART_IT_IDLE);
    HAL_UART_Receive_DMA(wapi_recv_handler.uart_handle, wapi_recv_buffer, sizeof(wapi_recv_buffer));
    return;
}

static void usart_write(uint8_t *const p_data, uint16_t len)
{
    HAL_UART_Transmit_DMA(wapi_recv_handler.uart_handle, p_data, len);
    // HAL_UART_Transmit(wapi_recv_handler.uart_handle, p_data, len, 1000);
    MAIN_DEBUG_GREEN_STRING(p_data, len);
    MAIN_DEBUG_OUT("\r\n");
}

static uint16_t get_counter(void)
{
    return ((uint16_t)__HAL_DMA_GET_COUNTER(wapi_recv_handler.dma_handle));
}

static void set_counter(uint16_t counter)
{
    (wapi_recv_handler.dma_handle)->Instance->CNDTR = counter;
}

static void uart_deinit(void)
{
    return;
}

uart_ops_t g_wapi_uart_ops =
{
        .pf_uart_init = usart_init,
        .pf_uart_write = usart_write,
        .pf_get_counter = get_counter,
        .pf_set_counter = set_counter,
        .pf_uart_deinit = uart_deinit
};

recv_buf_att_t g_wapi_uart_rx_buf =
{
        .buffer_size = sizeof(wapi_recv_buffer),
        .recv_buf = wapi_recv_buffer
};

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (wapi_recv_handler.uart_handle == huart)
    {
        m0804c_at_send_complete_isr_cb(&g_wapi_handler_inst);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (wapi_recv_handler.uart_handle == huart)
    {
        HAL_UART_Receive_DMA(wapi_recv_handler.uart_handle, wapi_recv_buffer, sizeof(wapi_recv_buffer));
        m0804c_at_error_recv_isr_cb(&g_wapi_handler_inst);
    }
}
/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
   if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        m0804c_at_notify_recv_isr_cb(&g_wapi_handler_inst);
    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}