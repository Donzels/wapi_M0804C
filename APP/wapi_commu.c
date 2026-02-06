#include "wapi_commu.h"

/* -------------------------------------------------------------------------- */
/*                        Forward declarations (OSAL)                         */
/* -------------------------------------------------------------------------- */
int32_t osal_sema_countings_create(void **p_sema_handle, uint32_t max_count, uint32_t init_count);
int32_t osal_sema_binary_create(void **p_sema_handle);
void osal_sema_delete(void *sema_handle);
int32_t osal_sema_give(void *sema_handle);
int32_t osal_sema_take(void *sema_handle, uint32_t timeout);

int32_t osal_timer_create(void **timer_handle, const char *timer_name, uint32_t timer_period,
                          uint8_t auto_reload, void (*timer_cb)(void *timer_handle, void *arg), void *arg);
int32_t osal_timer_start(void *timer_handle, uint32_t ticks_to_wait);
int32_t osal_timer_stop(void *timer_handle, uint32_t ticks_to_wait);
int32_t osal_timer_delete(void *timer_handle, uint32_t ticks_to_wait);
int32_t osal_timer_reset(void *timer_handle, uint32_t ticks_to_wait);
int32_t osal_delay(uint32_t ticks);

static int32_t osal_thread_create_adapter(const char *name,
                                          void (*task)(void *),
                                          size_t stack_size,
                                          uint32_t priority,
                                          void **handle,
                                          void *arg);
static void osal_thread_delete_adapter(void *const handle);
static int32_t osal_queue_create_adapter(size_t num, size_t size, void **handle);
static int32_t osal_queue_put_adapter(void *queue, const void *item, uint32_t timeout);
static int32_t osal_queue_get_adapter(void *queue, const void *item, uint32_t timeout);
static uint32_t osal_enter_critical_adapter(void);
static void osal_exit_critical_adapter(uint32_t primask);
static void wapi_os_delay_adapter(int32_t ticks);

static void m0804c_open(struct m0804c_handler *const self);
static void m0804c_close(struct m0804c_handler *const self);

static void wapi_process_success_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);
static void wapi_process_err_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);

static wapi_info_t *m0804c_get_wapi_info(struct m0804c_handler *const self);
static cert_file_t *m0804c_get_cert_file(struct m0804c_handler *const self);

static void wapi_process_success_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);
static void wapi_process_err_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);

m0804c_handler_t g_wapi_handler_inst = {0};

uart_rx_os_interface_t g_uart_os_interface = 
{
    .pf_os_thread_create  = osal_thread_create_adapter,
    .pf_os_thread_delete  = osal_thread_delete_adapter,
    .pf_os_queue_create   = osal_queue_create_adapter,
    .pf_os_queue_put      = osal_queue_put_adapter,
    .pf_os_queue_get      = osal_queue_get_adapter,
    .pf_os_enter_critical = osal_enter_critical_adapter,
    .pf_os_exit_critical  = osal_exit_critical_adapter,
};

/* AT handler OSAL table */
at_os_interface_t g_at_os_interface = 
{
    .pf_sema_binary_create    = osal_sema_binary_create,
    .pf_sema_delete           = osal_sema_delete,
    .pf_sema_give             = osal_sema_give,
    .pf_sema_take             = osal_sema_take,
    .pf_timer_create          = osal_timer_create,
    .pf_timer_start           = osal_timer_start,
    .pf_timer_stop            = osal_timer_stop,
    .pf_timer_delete          = osal_timer_delete,
};

static m0804c_os_interface_t g_wapi_os_interface = {
    .pf_os_delay = wapi_os_delay_adapter,
};

static frame_parse_att_t wapi_frame_parse_att = 
{
    .recv_buf_att = &g_wapi_uart_rx_buf, /**< Use built-in receive buffer configuration */
    .parse_algo = NULL,    /**< Use built-in parse algorithm */
}; /* Use built-in frame parsing configuration */

static uart_proto_input_arg_t wapi_uart_proto_input_arg = 
{
    .frame_parse_att = &wapi_frame_parse_att,  /* Use built-in frame parsing configuration */
    .uart_ops = &g_wapi_uart_ops,        /* To be assigned */
    .os_interface = &g_uart_os_interface,    /* To be assigned */
};

static at_input_arg_t  wapi_at_input_arg = 
{
    .uart_proto_input_arg = &wapi_uart_proto_input_arg, /* Use built-in UART protocol layer input arguments */
    .at_cmd_set_table = NULL,     /* Use built-in AT command table */
    .at_os_interface = &g_at_os_interface       /* To be assigned */
};

static m0804c_pwr_ops_t wapi_pwr_ops = 
{
    .pf_m0804c_open = m0804c_open,
    .pf_m0804c_close = m0804c_close,
};

static wapi_data_provider_t wapi_data_provider = 
{
    .pf_get_cert_file = m0804c_get_cert_file,
    .pf_get_wapi_info = m0804c_get_wapi_info,
}; 

static wapi_callback_t wapi_callbacks = 
{
    .pf_process_success_cb = wapi_process_success_cb,
    .pf_process_err_cb = wapi_process_err_cb,
};

static wapi_m0804c_input_arg_t wapi_input_arg = 
{
    .at_input_arg = &wapi_at_input_arg, /* Use built-in AT handler input arguments */
    .os_interface = &g_wapi_os_interface, /* To be assigned */
    .pwr_ops = &wapi_pwr_ops,     /* To be assigned */
    .data_provider = &wapi_data_provider,/* To be assigned */
    .callbacks = &wapi_callbacks     /* To be assigned */
};

static at_status_t wapi_at_recv_parse(uint8_t *buf, uint16_t len, void *arg, void *holder)
{
    WAPI_COMMU_DEBUG_OUT("WAPI AT RECV PARSE CALLBACK\r\n");
    WAPI_COMMU_DEBUG_STRING(buf, len);      
    return AT_OK;
}

static void wapi_commu_task(void *argument)
{
    uint8_t buf[64] = {0xDE, 0xAD, 0xBE, 0xEF};
    for(uint8_t i=4; i<64; i++)
    {
        buf[i] = i;
    }
    while (1)
    {
        m0804c_send(&g_wapi_handler_inst, buf, sizeof(buf), wapi_at_recv_parse);
        osDelay(1000);
    }
}

void wapi_commu_init(void)
{
    wapi_status_t ret = WAPI_OK;
    ret = m0804c_inst(&g_wapi_handler_inst, &wapi_input_arg); 
    if (WAPI_OK != ret)
    {
        WAPI_COMMU_DEBUG_ERR("WAPI handler instance failed: %d\r\n", ret);
        return;
    } 
    WAPI_COMMU_DEBUG_OUT("WAPI handler instance success\r\n");
    ret = m0804c_init(&g_wapi_handler_inst);
    if (WAPI_OK != ret)
    {
        WAPI_COMMU_DEBUG_ERR("WAPI handler init failed: %d\r\n", ret);
        return;
    }
    ret = m0804c_use_cert_conn(&g_wapi_handler_inst);
    if (WAPI_OK != ret)
    {
        WAPI_COMMU_DEBUG_ERR("WAPI handler use cert connect failed: %d\r\n", ret);
        return;
    }
    WAPI_COMMU_DEBUG_OUT("WAPI handler use cert connect success\r\n");
    const osThreadAttr_t wapi_thread_attr = {
    .name = "wapi_commu",
    .stack_size = 256 * 2,
    .priority = (osPriority_t)osPriorityRealtime7,
    };
    osThreadNew(wapi_commu_task, NULL, &wapi_thread_attr);
    
}





#if 1
static void m0804c_open(struct m0804c_handler *const self)
{
    HAL_GPIO_WritePin(GPIOB, WAPI_WAKE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, WAPI_PWR_Pin, GPIO_PIN_RESET);    
    osDelay(2000);   
}
static void m0804c_close(struct m0804c_handler *const self)
{
    HAL_GPIO_WritePin(GPIOB, WAPI_WAKE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, WAPI_PWR_Pin, GPIO_PIN_SET);  
    osDelay(2000);
}

static wapi_info_t *m0804c_get_wapi_info(struct m0804c_handler *const self)
{
    return get_wapi_info();
}

static cert_file_t *m0804c_get_cert_file(struct m0804c_handler *const self)
{
    return get_cert_file();
}

static void wapi_process_success_cb(struct m0804c_handler *const self, wapi_process_type_t process_type)
{
    switch (process_type)
    {
        case PROCESS_INIT:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS INIT SUCCESS\r\n");
            break;
        case PROCESS_CERT_AUTH:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS CERT AUTH SUCCESS\r\n");
            break;
        case PROCESS_PWD_AUTH:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS PWD AUTH SUCCESS\r\n");
            break;
        case PROCESS_CONNECT:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS CONNECT SUCCESS\r\n");
            break;        
        default:
            break;
    }
}

static void wapi_process_err_cb(struct m0804c_handler *const self, wapi_process_type_t process_type)
{
    switch (process_type)
    {
        case PROCESS_INIT:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS INIT ERROR\r\n");
            break;
        case PROCESS_CERT_AUTH:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS CERT AUTH ERROR\r\n");
            break;
        case PROCESS_PWD_AUTH:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS PWD AUTH ERROR\r\n");
            break;
        case PROCESS_CONNECT:
            WAPI_COMMU_DEBUG_OUT("WAPI PROCESS CONNECT ERROR\r\n");
            break;        
        default:
            break;
    }
}
#endif

#if 1
/* -------------------------------------------------------------------------- */
/*                   OSAL implementations using CMSIS-RTOS2                   */
/* -------------------------------------------------------------------------- */
/* Semaphore wrappers */
int32_t osal_sema_countings_create(void **p_sema_handle, uint32_t max_count, uint32_t init_count)
{
    if (!p_sema_handle)
        return -1;
    osSemaphoreId_t sem = osSemaphoreNew(max_count, init_count, NULL);
    if (!sem)
        return -1;
    *p_sema_handle = sem;
    return 0;
}

int32_t osal_sema_binary_create(void **p_sema_handle)
{
    return osal_sema_countings_create(p_sema_handle, 1, 0);
}

void osal_sema_delete(void *sema_handle)
{
    if (sema_handle)
        osSemaphoreDelete((osSemaphoreId_t)sema_handle);
}

int32_t osal_sema_give(void *sema_handle)
{
    return osSemaphoreRelease((osSemaphoreId_t)sema_handle);
}

int32_t osal_sema_take(void *sema_handle, uint32_t timeout)
{
    return osSemaphoreAcquire((osSemaphoreId_t)sema_handle, timeout);
}

/* Timer wrappers */
typedef struct
{
    void (*cb)(void *timer_handle, void *arg);
    void *arg;
    osTimerId_t timer;
} osal_timer_ctx_t;

static void osal_timer_shim(void *argument)
{
    osal_timer_ctx_t *ctx = (osal_timer_ctx_t *)argument;
    if (ctx && ctx->cb)
        ctx->cb((void *)ctx, ctx->arg);
}

int32_t osal_timer_create(void **timer_handle, const char *timer_name, uint32_t timer_period,
                          uint8_t auto_reload, void (*timer_cb)(void *timer_handle, void *arg), void *arg)
{
    if (!timer_handle || !timer_cb)
        return -1;
    osal_timer_ctx_t *ctx = (osal_timer_ctx_t *)malloc(sizeof(osal_timer_ctx_t));
    if (!ctx)
        return -1;
    ctx->cb = timer_cb;
    ctx->arg = arg;
    osTimerAttr_t attr = {
        .name = timer_name,
    };
    osTimerType_t type = auto_reload ? osTimerPeriodic : osTimerOnce;
    ctx->timer = osTimerNew(osal_timer_shim, type, ctx, &attr);
    if (!ctx->timer)
    {
        free(ctx);
        return -1;
    }
    *timer_handle = (void *)ctx;
    return 0;
}

int32_t osal_timer_start(void *timer_handle, uint32_t ticks_to_wait)
{
    if (!timer_handle)
        return -1;
    osal_timer_ctx_t *ctx = (osal_timer_ctx_t *)timer_handle;
    return osTimerStart(ctx->timer, ticks_to_wait);
}

int32_t osal_timer_stop(void *timer_handle, uint32_t ticks_to_wait)
{
    (void)ticks_to_wait;
    if (!timer_handle)
        return -1;
    osal_timer_ctx_t *ctx = (osal_timer_ctx_t *)timer_handle;
    return osTimerStop(ctx->timer);
}

int32_t osal_timer_delete(void *timer_handle, uint32_t ticks_to_wait)
{
    (void)ticks_to_wait;
    if (!timer_handle)
        return -1;
    osal_timer_ctx_t *ctx = (osal_timer_ctx_t *)timer_handle;
    osTimerDelete(ctx->timer);
    free(ctx);
    return 0;
}

int32_t osal_timer_reset(void *timer_handle, uint32_t ticks_to_wait)
{
    if (!timer_handle)
        return -1;
    osal_timer_ctx_t *ctx = (osal_timer_ctx_t *)timer_handle;
    return osTimerStart(ctx->timer, ticks_to_wait);
}

/* Delay wrapper */
int32_t osal_delay(uint32_t ticks)
{
    return osDelay(ticks);
}

/* UART proto OS interface wiring (thread/queue/critical) */
static int32_t osal_thread_create_adapter(const char *name,
                                          void (*task)(void *),
                                          size_t stack_size,
                                          uint32_t priority,
                                          void **handle,
                                          void *arg)
{
    const osThreadAttr_t attr = {
        .name = name,
        .stack_size = stack_size,
        .priority = (osPriority_t)priority,
    };
    osThreadId_t tid = osThreadNew(task, arg, &attr);
    if (!tid)
        return -1;
    if (handle)
        *handle = tid;
    return 0;
}

static void osal_thread_delete_adapter(void *const handle)
{
    if (handle)
        osThreadTerminate((osThreadId_t)handle);
}

static int32_t osal_queue_create_adapter(size_t num, size_t size, void **handle)
{
    osMessageQueueId_t q = osMessageQueueNew(num, size, NULL);
    if (!q)
        return -1;
    if (handle)
        *handle = q;
    return 0;
}

static int32_t osal_queue_put_adapter(void *queue, const void *item, uint32_t timeout)
{
    return osMessageQueuePut((osMessageQueueId_t)queue, item, 0, timeout);
}

static int32_t osal_queue_get_adapter(void *queue, const void *item, uint32_t timeout)
{
    return osMessageQueueGet((osMessageQueueId_t)queue, (void *)item, NULL, timeout);
}

static uint32_t osal_enter_critical_adapter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void osal_exit_critical_adapter(uint32_t primask)
{
    if (primask == 0)
        __enable_irq();
}

static void wapi_os_delay_adapter(int32_t ticks)
{
    osDelay((uint32_t)ticks);
}
#endif





