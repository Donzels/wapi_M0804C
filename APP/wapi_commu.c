#include "wapi_commu.h"
#include "main.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*                        Forward declarations (OSAL)                         */
/* -------------------------------------------------------------------------- */

static void m0804c_open(struct m0804c_handler *const self);
static void m0804c_close(struct m0804c_handler *const self);

static wapi_info_t *m0804c_get_wapi_info(struct m0804c_handler *const self);
static cert_file_t *m0804c_get_cert_file(struct m0804c_handler *const self);

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
    .recv_buf_att = &g_wapi_uart_rx_buf, 
    .parse_algo = NULL,    /**< Use built-in parse algorithm */
}; 

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
    .frame_parse_att = &wapi_frame_parse_att,  
    .uart_ops = &g_wapi_uart_ops,        
    .os_interface = &g_uart_os_interface,    
    .thread_att = &wapi_rx_thread_att
};

static at_input_arg_t  wapi_at_input_arg = 
{
    .uart_proto_input_arg = &wapi_uart_proto_input_arg, 
    .at_cmd_set_table = NULL,     /* Use built-in AT command table */
    .at_os_interface = &g_at_os_interface       
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

static wapi_m0804c_input_arg_t wapi_input_arg = 
{
    .at_input_arg = &wapi_at_input_arg,
    .os_interface = &g_wapi_os_interface, 
    .pwr_ops = &wapi_pwr_ops,     
    .data_provider = &wapi_data_provider
};

/* Forward declaration */
static void wapi_event_observer_notify(wapi_observer_t *observer, 
                                       m0804c_handler_t *handler,
                                       wapi_event_t event);

/* Event handlers */
static void wapi_event_handle_init_success(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_init_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_cert_auth_success(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_cert_auth_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_pwd_auth_success(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_pwd_auth_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_connected(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_connect_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_disconnected(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_error(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);
static void wapi_event_handle_unknown(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);

typedef struct
{
    uint16_t event_counts[WAPI_EVENT_COUNT];
    wapi_event_t last_event;
} wapi_event_ctx_t;

static wapi_event_ctx_t g_wapi_event_ctx = {0};

static void wapi_event_record(wapi_observer_t *observer, wapi_event_t event);

/* WAPI event observer instance */
static wapi_observer_t wapi_event_observer = 
{
    .on_notify = wapi_event_observer_notify,
    .observer_context = &g_wapi_event_ctx
};

static void wapi_at_recv_parse(uint8_t *buf, uint16_t len, void *arg)
{
    /* It's meaningless, as M0804C will response debug info after send */
    WAPI_COMMU_DEBUG_OUT("WAPI AT RECV PARSE CALLBACK\r\n");
    WAPI_COMMU_DEBUG_STRING(buf, len);
}

typedef void (*wapi_event_handler_t)(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event);

static const wapi_event_handler_t wapi_event_handlers[WAPI_EVENT_COUNT] =
{
    [WAPI_EVENT_INIT_SUCCESS]      = wapi_event_handle_init_success,
    [WAPI_EVENT_INIT_FAILED]       = wapi_event_handle_init_failed,
    [WAPI_EVENT_CERT_AUTH_SUCCESS] = wapi_event_handle_cert_auth_success,
    [WAPI_EVENT_CERT_AUTH_FAILED]  = wapi_event_handle_cert_auth_failed,
    [WAPI_EVENT_PWD_AUTH_SUCCESS]  = wapi_event_handle_pwd_auth_success,
    [WAPI_EVENT_PWD_AUTH_FAILED]   = wapi_event_handle_pwd_auth_failed,
    [WAPI_EVENT_CONNECTED]         = wapi_event_handle_connected,
    [WAPI_EVENT_CONNECT_FAILED]    = wapi_event_handle_connect_failed,
    [WAPI_EVENT_DISCONNECTED]      = wapi_event_handle_disconnected,
    [WAPI_EVENT_ERROR]             = wapi_event_handle_error
};

/**
 * @brief Observer callback for WAPI events
 * 
 * This replaces the old process success/error callbacks with a more flexible
 * observer pattern that can handle multiple events.
 */
static void wapi_event_observer_notify(wapi_observer_t *observer, 
                                       m0804c_handler_t *handler,
                                       wapi_event_t event)
{
    if (event < WAPI_EVENT_COUNT && wapi_event_handlers[event])
    {
        wapi_event_handlers[event](observer, handler, event);
    }
    else
    {
        wapi_event_handle_unknown(observer, handler, event);
    }
}

static void wapi_event_record(wapi_observer_t *observer, wapi_event_t event)
{
    if (!observer || !observer->observer_context)
        return;

    wapi_event_ctx_t *ctx = (wapi_event_ctx_t *)observer->observer_context;
    if (event < WAPI_EVENT_COUNT)
    {
        ctx->event_counts[event]++;
    }
    ctx->last_event = event;
    WAPI_COMMU_DEBUG_OUT("WAPI EVENT REC: %s cnt=%u\r\n", wapi_get_event_name(event),
                         (event < WAPI_EVENT_COUNT) ? ctx->event_counts[event] : 0);
}

static void wapi_event_handle_init_success(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI INIT SUCCESS\r\n");
}

static void wapi_event_handle_init_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI INIT FAILED\r\n");
}

static void wapi_event_handle_cert_auth_success(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI CERT AUTH SUCCESS\r\n");
}

static void wapi_event_handle_cert_auth_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI CERT AUTH FAILED\r\n");
}

static void wapi_event_handle_pwd_auth_success(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI PWD AUTH SUCCESS\r\n");
}

static void wapi_event_handle_pwd_auth_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI PWD AUTH FAILED\r\n");
}

static void wapi_event_handle_connected(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI CONNECTED\r\n");
}

static void wapi_event_handle_connect_failed(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI CONNECT FAILED\r\n");
}

static void wapi_event_handle_disconnected(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI DISCONNECTED\r\n");
}

static void wapi_event_handle_error(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI ERROR\r\n");
}

static void wapi_event_handle_unknown(wapi_observer_t *observer, m0804c_handler_t *handler, wapi_event_t event)
{
    (void)handler;
    wapi_event_record(observer, event);
    WAPI_COMMU_DEBUG_OUT("APP WAPI UNKNOWN EVENT: %d\r\n", event);
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
    
    /* Attach observer for event notifications */
    ret = wapi_subject_attach(&g_wapi_handler_inst, &wapi_event_observer);
    if (WAPI_OK != ret)
    {
        WAPI_COMMU_DEBUG_ERR("Failed to attach WAPI event observer: %d\r\n", ret);
        return;
    }
    WAPI_COMMU_DEBUG_OUT("WAPI event observer attached\r\n");
    
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
#endif






