#include "wapi_commu.h"
#include "main.h"

/* -------------------------------------------------------------------------- */
/*                        Forward declarations (OSAL)                         */
/* -------------------------------------------------------------------------- */

static void m0804c_open(struct m0804c_handler *const self);
static void m0804c_close(struct m0804c_handler *const self);

static void wapi_process_success_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);
static void wapi_process_err_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);

static wapi_info_t *m0804c_get_wapi_info(struct m0804c_handler *const self);
static cert_file_t *m0804c_get_cert_file(struct m0804c_handler *const self);

static void wapi_process_success_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);
static void wapi_process_err_cb(struct m0804c_handler *const self, wapi_process_type_t process_type);

extern recv_buf_att_t g_wapi_uart_rx_buf;
extern uart_ops_t g_wapi_uart_ops;

m0804c_handler_t g_wapi_handler_inst = {0};

uart_rx_os_interface_t g_uart_os_interface = 
{
    .pf_os_thread_create  = osal_task_create,
    .pf_os_thread_delete  = osal_task_delete,
    .pf_os_queue_create   = osal_queue_create,
    .pf_os_queue_put      = osal_queue_send,
    .pf_os_queue_get      = osal_queue_receive,
    .pf_os_enter_critical = osal_enter_critical,
    .pf_os_exit_critical  = osal_exit_critical,
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
    .pf_os_delay_ms = osal_task_delay_ms,
};

static frame_parse_att_t wapi_frame_parse_att = 
{
    .recv_buf_att = &g_wapi_uart_rx_buf, /**< Use built-in receive buffer configuration */
    .parse_algo = NULL,    /**< Use built-in parse algorithm */
}; /* Use built-in frame parsing configuration */

static rx_thread_att_t wapi_rx_thread_att = 
{
    .parse_thread_att = 
    {
        .stack_depth = WAPI_COMMU_PARSE_THREAD_STACK_DEPTH,
        .thread_priority = WAPI_COMMU_PARSE_THREAD_PRIORITY - 1
    }
};

static uart_proto_input_arg_t wapi_uart_proto_input_arg = 
{
    .frame_parse_att = &wapi_frame_parse_att,  /* Use built-in frame parsing configuration */
    .uart_ops = &g_wapi_uart_ops,        /* To be assigned */
    .os_interface = &g_uart_os_interface,    /* To be assigned */
    .thread_att = &wapi_rx_thread_att
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
    uint8_t buf[32] = {0xDE, 0xAD, 0xBE, 0xEF};
    for(uint8_t i=4; i<sizeof(buf); i++)
    {
        buf[i] = i;
    }
    while (1)
    {
        m0804c_send(&g_wapi_handler_inst, buf, sizeof(buf), wapi_at_recv_parse);
        osal_task_delay_ms(5000);
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

    osal_task_create("wapi_commu", wapi_commu_task, 1024, 20, NULL, NULL);
}



#if 1
static void m0804c_open(struct m0804c_handler *const self)
{
    HAL_GPIO_WritePin(GPIOB, WAPI_WAKE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, WAPI_PWR_Pin, GPIO_PIN_RESET);    
    osal_task_delay_ms(2000);   
    WAPI_COMMU_DEBUG_OUT("WAPI M0804C opened\r\n");
}
static void m0804c_close(struct m0804c_handler *const self)
{
    HAL_GPIO_WritePin(GPIOB, WAPI_WAKE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, WAPI_PWR_Pin, GPIO_PIN_SET);  
    osal_task_delay_ms(2000);
    WAPI_COMMU_DEBUG_OUT("WAPI M0804C closed\r\n");
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






