/**
 ******************************************************************************
 * File Name          : WAPI_M0804C.c
 * Description        : WAPI M0804C Module Handler Implementation
 *                      Implements AT command-based WAPI device initialization,
 *                      configuration, connection management, certificate upload,
 *                      and data transmission with FreeRTOS task scheduling.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "WAPI_M0804C.h"
#include <stdio.h>


/* AT handler OSAL adapter provided below */

#define AT_TIMEOUT_TICK_STANDARD            500
#define AT_TIMEOUT_TICK_LONG                30000
#define AT_INTERVAL_TICK                    1000

/**
 * @brief Single AT command retry count within a process step
 * 
 * When a single AT command fails or times out (within table_process), the command
 * will be retried up to AT_ERR_REPEAT_CNT times before considering that step failed
 * and moving to the next level of retry (table_process exit ? generic_process retry).
 * 
 * This is the innermost retry level, handling transient communication issues at the
 * AT command level (e.g., UART receive buffer full, temporary device unresponsiveness).
 * 
 * Used in: table_process() - inner retry loop for individual AT commands
 * Value 4 means: Execute once, if fails retry 3 more times = 4 total attempts per step
 * 
 * Retry hierarchy (from innermost to outermost):
 * 1. AT_ERR_REPEAT_CNT (4) - Single AT command level
 * 2. WAPI_PROCESS_RETRY_MAX (2) - Process table level
 * 3. WAPI_PROCESS_FAIL_MAX (3) - Thread level
 */
#define AT_ERR_REPEAT_CNT                   4

/**
 * @brief Table-level retry count for WAPI process sequences
 * 
 * When a complete process table fails (e.g., wapi_process_init), the entire table
 * will be re-executed up to WAPI_PROCESS_RETRY_MAX times before reporting failure
 * to the thread handler. This handles systematic failures that might recover with
 * fresh attempt (e.g., device temporarily unresponsive).
 * 
 * Used in: generic_process()
 * Value 2 means: Execute once, if fails retry once more = 2 total attempts
 */
#define WAPI_PROCESS_RETRY_MAX              2

/**
 * @brief Maximum consecutive process failures before entering permanent error state
 * 
 * After a complete process table fails (exhausted WAPI_PROCESS_RETRY_MAX attempts),
 * the thread increments fail_cnt. When fail_cnt reaches WAPI_PROCESS_FAIL_MAX,
 * the thread stops retrying and invokes pf_process_err_cb() callback to signal
 * permanent failure. This prevents infinite retry loops on systemic failures.
 * 
 * Used in: generic_wapi_thread()
 * Value 3 means: Allow 3 consecutive table-level failures before declaring error
 * Flow: success?fail?fail?fail?error_callback
 */
#define WAPI_PROCESS_FAIL_MAX               3

#define SEND_BUF_SIZE                       128       

#define CUR_SOCKET                          1

#define PRIV_DATA(self)     (self)->priv_data                /* Private internal data */
#define AT_OS(self)         (self)->input_arg->at_input_arg->at_os_interface
#define UP_OS(self)         (self)->input_arg->at_input_arg->uart_proto_input_arg->os_interface

#include <stdlib.h>
#define MALLOC(size)        malloc(size)  
#define FREE(ptr)           free(ptr)

typedef enum
{
    TEST = 0,
    GET_VERSION,
    SET_ECHO,
    SET_BAND,
    AT_REBOOT,
    SET_TX_PWR,
    SET_LOW_PWR,
    DISCONN_TRANS,
    SET_IP,
    CONN_WAPI_BY_CERT,
    CONN_WAPI_BY_PWD,
    CHECK_LINK_LAYER,
    TCP_UDP_CONN,
    RECV_DATA,
    SEND_DATA,
    UPLOAD_CERT_START,
    CHECK_CERT,
    DISCONN_SOCKET,
}at_func_t;

typedef enum
{
    PROCESS_OK = 0,
    PROCESS_ERR
}process_status_t;

typedef enum
{
    CONN_BY_NOTHING = 0,
    CONN_BY_CERT,
    CONN_BY_PWD
}wapi_conn_mode_t;

/* Callbacks for different process stages */
typedef struct
{
    wapi_process_type_t process_type;                     /* Type of the WAPI process */
    const char *process_name;                             /* Name of the process for logging/debugging */
    void (*pf_process_start)(m0804c_handler_t *self);      /* Called before process starts */
    void (*pf_process_success)(m0804c_handler_t *self);    /* Called when process succeeds */
    void (*pf_process_retry)(m0804c_handler_t *self);      /* Called when process fails and will retry */
}wapi_process_callbacks_t;

typedef void (*pf_wapi_process_fun_t)(m0804c_handler_t *const self);
typedef void (*pf_process_complete_t)(m0804c_handler_t *const self, uint8_t index, process_status_t process_status);

typedef struct 
{
    pf_wapi_process_fun_t pf_wapi_process_fun; 
    pf_process_complete_t pf_process_cpl;
    uint32_t recv_timeout_tick;
    uint32_t process_success_interval_tick;
    uint32_t process_fail_interval_tick;
}wapi_process_t;

typedef struct
{
    bool is_success;
}wapi_recv_state_t;

typedef struct m0804c_priv_data
{
    bool is_inited;
    wapi_handler_state_t current_state;  /* Current state machine state */
    wapi_conn_mode_t wapi_conn_mode;
    void *process_syn_sema_handle;
    void *multi_send_syn_sema_handle;
    void *recv_state_queue_handle;
    void *init_start_sema_handle;
    void *init_success_sema_handle;
#if IS_USE_CONN_BY_CERT
    void *use_cert_sema_handle;
#endif
#if IS_USE_CONN_BY_PWD
    void *use_pwd_sema_handle;
#endif
    void *connect_cfg_success_sema_handle;
    at_handler_t *at_handler;
    at_cmd_set_t *at_cmd_table_copy; /* Mutable copy of AT command table with arg set to self */
    uint8_t wapi_send_buf[SEND_BUF_SIZE];
    wapi_subject_t *observer_subject;   /* Observer pattern subject for event notification */
}m0804c_priv_data_t;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

/**
 * @brief State transition validation and execution for WAPI handler
 *
 * Validates if the state transition is allowed and updates the state.
 * Logs transitions for debugging purposes.
 *
 * @param self Pointer to WAPI handler instance
 * @param new_state Target state to transition to
 * @return wapi_status_t WAPI_OK if transition is valid, error code otherwise
 */
static wapi_status_t wapi_state_transition(m0804c_handler_t *const self, wapi_handler_state_t new_state);

/* AT parsing functions */
static void at_recv_parse_ok(uint8_t *buf, uint16_t len, void *arg);
static void at_recv_parse_tcp_connect(uint8_t *buf, uint16_t len, void *arg);
static void at_recv_parse_reboot(uint8_t *buf, uint16_t len, void *arg);
static void at_recv_parse_upload_cert_start(uint8_t *buf, uint16_t len, void *arg);
static void at_recv_parse_link_layer_check(uint8_t *buf, uint16_t len, void *arg);
static void recv_force_correct(uint8_t *buf, uint16_t len, void *arg);
static void check_connect(uint8_t *buf, uint16_t len, void *arg);

/* WAPI operation functions */
static void wapi_test(m0804c_handler_t *const self);
static void wapi_no_echo(m0804c_handler_t *const self);
static void wapi_get_version(m0804c_handler_t *const self);
static void wapi_check_cert(m0804c_handler_t *const self);
static void wapi_both_2p4_5g(m0804c_handler_t *const self);
static void wapi_reboot(m0804c_handler_t *const self);
static void wapi_set_tx_pwr(m0804c_handler_t *const self);
static void wapi_disable_low_pwr(m0804c_handler_t *const self);
static void wapi_disconn_transect(m0804c_handler_t *const self);
static void wapi_set_net_config(m0804c_handler_t *const self);
static void wapi_connect_by_cert(m0804c_handler_t *const self);
static void wapi_connect_by_pwd(m0804c_handler_t *const self);
static void wapi_check_link_layer_connect(m0804c_handler_t *const self);
static void wapi_tcp_connect(m0804c_handler_t *const self);
static void wapi_tcp_disconnect(m0804c_handler_t *const self);
static void wapi_recv_data(m0804c_handler_t *const self);
static void wapi_upload_as_cert(m0804c_handler_t *const self);
static void wapi_upload_as_cert_file(m0804c_handler_t *const self);
static void wapi_upload_asue_cert(m0804c_handler_t *const self);
static void wapi_upload_asue_cert_file(m0804c_handler_t *const self);

/* Process functions */
static wapi_status_t wapi_init_process(m0804c_handler_t *const self);
static wapi_status_t connect_by_cert_process(m0804c_handler_t *const self);
static wapi_status_t connect_by_pwd_process(m0804c_handler_t *const self);
static wapi_status_t connect_net_process(m0804c_handler_t *const self);
static wapi_status_t cert_upload_process(m0804c_handler_t *const self);
static wapi_status_t disconn_process(m0804c_handler_t *const self);

/* Process callbacks functions */
static void wapi_process_complete_cb(m0804c_handler_t *const self, uint8_t index, process_status_t process_status);
static void init_process_start(m0804c_handler_t *self);
static void init_process_retry(m0804c_handler_t *self);
static void init_process_success(m0804c_handler_t *self);
#if IS_USE_CONN_BY_CERT
static void conn_cfg_by_cert_process_start(m0804c_handler_t *self);
static void conn_cfg_by_cert_process_retry(m0804c_handler_t *self);
static void conn_cfg_by_cert_process_success(m0804c_handler_t *self);
#endif
#if IS_USE_CONN_BY_PWD
static void conn_cfg_by_pwd_process_start(m0804c_handler_t *self);
static void conn_cfg_by_pwd_process_retry(m0804c_handler_t *self);
static void conn_cfg_by_pwd_process_success(m0804c_handler_t *self);
#endif
static void conn_process_start(m0804c_handler_t *self);
static void conn_process_retry(m0804c_handler_t *self);
static void conn_process_success(m0804c_handler_t *self);

/* Utility functions */
static void reset_wapi_state(m0804c_handler_t *self);
static wapi_status_t wapi_send_data(m0804c_handler_t *self, uint8_t *buf, uint16_t length,
                                    pf_at_recv_parse_t recv_parse_cb);
static wapi_status_t m0804c_start_recv(m0804c_handler_t *const self);

static wapi_status_t wapi_subject_init(m0804c_handler_t *const self);
/* ============================================================================
 * Global Data
 * ============================================================================ */
static wapi_info_t g_default_wapi_info = 
{
    .server_ip = {192,168,0,195},
    .server_port = 666,
    .local_port = 777,
    .local_ip = {192,168,0,66},
    .local_ip_mask = {255,255,255,0},
    .local_gateway = {192,168,0,4},
    .ssid = "WAPI-24G-8825",
    .pwd = "123456abc",
    .is_exist_certicate = false
};

static at_handler_t *wapi_get_at_handler(m0804c_handler_t *const self)
{
    if (self && PRIV_DATA(self) && PRIV_DATA(self)->is_inited)
        return PRIV_DATA(self)->at_handler;
    return NULL;
}

extern uart_ops_t g_wapi_uart_ops;
extern recv_buf_att_t g_wapi_uart_rx_buf;

static const at_cmd_set_t m0804c_at_table[] = 
{
    {TEST, "AT\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {GET_VERSION, "ATI\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {SET_ECHO, "AT+ECHO=%d\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {SET_BAND, "AT+BAND=%d\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {AT_REBOOT, "AT+REBOOT\r\n", {.items = {{at_recv_parse_reboot, NULL}}, .count = 1}},
    {SET_TX_PWR, "AT+TXPWR=0,22\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {SET_LOW_PWR, "AT+SETDP=%d\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {DISCONN_TRANS, "AT+WSDISCNCT\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {SET_IP, "AT+WFIXIP=%d,%d.%d.%d.%d,%d.%d.%d.%d,%d.%d.%d.%d\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {CONN_WAPI_BY_CERT, "AT+WAPICT,%d,%s\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {CONN_WAPI_BY_PWD, "AT+WAPICT,%d,%s,%s\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {CHECK_LINK_LAYER, "AT+WAPICT=?\r\n", {.items = {{at_recv_parse_link_layer_check, NULL}}, .count = 1}},
    {TCP_UDP_CONN, "AT+NCRECLNT=%s,%d.%d.%d.%d,%d,%d,%d,%d,%d,%d,%d\r\n", {.items = {{at_recv_parse_tcp_connect, NULL}}, .count = 1}},
    {RECV_DATA, "AT+NRECV,%d,%d,%d\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {SEND_DATA, "AT+NSEND,%d,%d,", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {UPLOAD_CERT_START, "AT+UPCERT=%s\r\n", {.items = {{at_recv_parse_upload_cert_start, NULL}}, .count = 1}},
    {CHECK_CERT, "AT+UPCERT=?\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
    {DISCONN_SOCKET, "AT+NSTOP,%d\r\n", {.items = {{at_recv_parse_ok, NULL}}, .count = 1}},
};

static at_cmd_set_table_t g_m0804c_at_cmd_set_table = 
{
    .table = m0804c_at_table,
    .table_len = sizeof(m0804c_at_table)/sizeof(m0804c_at_table[0])
};

/* ============================================================================
 * Process Tables Definition
 * ============================================================================ */
static wapi_process_t wapi_process_init[] = 
{
    {wapi_no_echo,          wapi_process_complete_cb, 2*AT_TIMEOUT_TICK_STANDARD, 0, AT_INTERVAL_TICK},
    // {wapi_check_cert,       wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   0, AT_INTERVAL_TICK},
    {wapi_both_2p4_5g,      wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   0, AT_INTERVAL_TICK},
    {wapi_set_tx_pwr,       wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   0, AT_INTERVAL_TICK},
    {wapi_disable_low_pwr,  wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   0, AT_INTERVAL_TICK},
    {wapi_disconn_transect, wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   0, AT_INTERVAL_TICK},
    {wapi_set_net_config,   wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   0, AT_INTERVAL_TICK},  
};

#if IS_USE_CONN_BY_CERT
static wapi_process_t wapi_process_use_cert[] = 
{
    {wapi_check_cert,      wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD, 0, AT_INTERVAL_TICK},
    {wapi_connect_by_cert, wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD, 5*AT_INTERVAL_TICK, 5*AT_INTERVAL_TICK},    
};
#endif

#if IS_USE_CONN_BY_PWD
static wapi_process_t wapi_process_use_pwd[] = 
{
    {wapi_connect_by_pwd, wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD, 5*AT_INTERVAL_TICK, 5*AT_INTERVAL_TICK},    
};
#endif

static wapi_process_t wapi_process_conn_net[] = 
{    
    {wapi_check_link_layer_connect, wapi_process_complete_cb, 5*AT_TIMEOUT_TICK_STANDARD, 3*AT_INTERVAL_TICK, 3*AT_INTERVAL_TICK},
    {wapi_tcp_connect,              wapi_process_complete_cb, AT_TIMEOUT_TICK_LONG,       0, AT_INTERVAL_TICK},
    {wapi_recv_data,                wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   0, AT_INTERVAL_TICK},
};

static wapi_process_t wapi_process_disconn[] = 
{    
    {wapi_tcp_disconnect, wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD, 0, AT_INTERVAL_TICK},
};

static wapi_process_t wapi_process_upload_certiface[] = 
{
    // {wapi_test,                  wapi_process_complete_cb, 2*AT_TIMEOUT_TICK_STANDARD, AT_INTERVAL_TICK, AT_INTERVAL_TICK},
    {wapi_no_echo,               wapi_process_complete_cb, 2*AT_TIMEOUT_TICK_STANDARD, AT_INTERVAL_TICK, AT_INTERVAL_TICK},
    {wapi_upload_as_cert,        wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   AT_INTERVAL_TICK, AT_INTERVAL_TICK},
    {wapi_upload_as_cert_file,   wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   AT_INTERVAL_TICK, AT_INTERVAL_TICK},
    {wapi_upload_asue_cert,      wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   AT_INTERVAL_TICK, AT_INTERVAL_TICK},
    {wapi_upload_asue_cert_file, wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   AT_INTERVAL_TICK, AT_INTERVAL_TICK},
    {wapi_check_cert,            wapi_process_complete_cb, AT_TIMEOUT_TICK_STANDARD,   AT_INTERVAL_TICK, AT_INTERVAL_TICK}
};

/* ============================================================================
 * Process Callbacks Structures
 * ============================================================================ */
static wapi_process_callbacks_t init_callbacks = {
    .process_type = PROCESS_INIT,
    .process_name = "WAPI Init",
    .pf_process_start = init_process_start,
    .pf_process_success = init_process_success,
    .pf_process_retry = init_process_retry,
};

#if IS_USE_CONN_BY_CERT
static wapi_process_callbacks_t conn_cfg_by_cert_callbacks = {
    .process_type = PROCESS_CERT_AUTH,
    .process_name = "WAPI Conn by Cert",
    .pf_process_start = conn_cfg_by_cert_process_start,
    .pf_process_success = conn_cfg_by_cert_process_success,
    .pf_process_retry = conn_cfg_by_cert_process_retry,
};
#endif

#if IS_USE_CONN_BY_PWD
static wapi_process_callbacks_t conn_cfg_by_pwd_callbacks = {
    .process_type = PROCESS_PWD_AUTH,
    .process_name = "WAPI Conn by Pwd",
    .pf_process_start = conn_cfg_by_pwd_process_start,
    .pf_process_success = conn_cfg_by_pwd_process_success,
    .pf_process_retry = conn_cfg_by_pwd_process_retry,
};
#endif

static wapi_process_callbacks_t conn_callbacks = {
    .process_type = PROCESS_CONNECT,
    .process_name = "WAPI Conn Net",
    .pf_process_start = conn_process_start,
    .pf_process_success = conn_process_success,
    .pf_process_retry = conn_process_retry,
};

/* ============================================================================
 * Process Callbacks Implementation
 * ============================================================================ */
static void wapi_process_complete_cb(m0804c_handler_t *const self,\
                                     uint8_t index, process_status_t process_status)
{      
    if(PROCESS_OK != process_status)  
    {
        WAPI_DEBUG_ERR("Process %u failed", index);
    }         
}

/* Init process callbacks */
static void init_process_start(m0804c_handler_t *self)
{
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->init_start_sema_handle, OS_DELAY_MAX);
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->init_success_sema_handle, 0);
    self->input_arg->pwr_ops->pf_m0804c_open(self);
    PRIV_DATA(self)->wapi_conn_mode = CONN_BY_NOTHING;
    reset_wapi_state(self);
    /* Transition to INITING state */
    wapi_state_transition(self, WAPI_STATE_INITING);
}

static void init_process_retry(m0804c_handler_t *self)
{
    self->input_arg->pwr_ops->pf_m0804c_close(self);
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->init_start_sema_handle);
}

static void init_process_success(m0804c_handler_t *self)
{
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->init_success_sema_handle);
    /* Transition to INITED state after successful initialization */
    wapi_state_transition(self, WAPI_STATE_INITED);
}

#if IS_USE_CONN_BY_CERT
static void conn_cfg_by_cert_process_start(m0804c_handler_t *self)
{
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->use_cert_sema_handle, OS_DELAY_MAX);
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->connect_cfg_success_sema_handle, 0);
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->init_success_sema_handle, OS_DELAY_MAX);
    /* Transition to CONFIGURING_CONN state */
    wapi_state_transition(self, WAPI_STATE_CONFIGURING_CONN);
}

static void conn_cfg_by_cert_process_retry(m0804c_handler_t *self)
{    
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->init_success_sema_handle);
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->use_cert_sema_handle);
}

static void conn_cfg_by_cert_process_success(m0804c_handler_t *self)
{
    PRIV_DATA(self)->wapi_conn_mode = CONN_BY_CERT;
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->connect_cfg_success_sema_handle);
    /* Transition to CONFIGURED state */
    wapi_state_transition(self, WAPI_STATE_CONFIGURED);
}
#endif

#if IS_USE_CONN_BY_PWD
static void conn_cfg_by_pwd_process_start(m0804c_handler_t *self)
{
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->use_pwd_sema_handle, OS_DELAY_MAX);
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->connect_cfg_success_sema_handle, 0);
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->init_success_sema_handle, OS_DELAY_MAX);
    /* Transition to CONFIGURING_CONN state */
    wapi_state_transition(self, WAPI_STATE_CONFIGURING_CONN);
}

static void conn_cfg_by_pwd_process_retry(m0804c_handler_t *self)
{    
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->init_success_sema_handle);
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->use_pwd_sema_handle);
}

static void conn_cfg_by_pwd_process_success(m0804c_handler_t *self)
{
    PRIV_DATA(self)->wapi_conn_mode = CONN_BY_PWD;
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->connect_cfg_success_sema_handle);
    /* Transition to CONFIGURED state */
    wapi_state_transition(self, WAPI_STATE_CONFIGURED);
}
#endif
/* Conn process success callback */
static void conn_process_start(m0804c_handler_t *self)
{
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->connect_cfg_success_sema_handle, OS_DELAY_MAX);
    /* Transition to CONNECTING state */
    wapi_state_transition(self, WAPI_STATE_CONNECTING);
}

static void conn_process_retry(m0804c_handler_t *self)
{
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->connect_cfg_success_sema_handle);
}

static void conn_process_success(m0804c_handler_t *self)
{
    /* No need for trans_send_flag - the CONNECTED state itself indicates readiness for transmission */
    at_recv_hook_register(PRIV_DATA(self)->at_handler, check_connect, self);
    /* Transition to CONNECTED state */
    wapi_state_transition(self, WAPI_STATE_CONNECTED);
    // m0804c_start_recv(self);
}

/* ============================================================================
 * State Machine Implementation
 * ============================================================================ */
static wapi_status_t wapi_state_transition(m0804c_handler_t *const self, wapi_handler_state_t new_state)
{
    if (!self || !PRIV_DATA(self))
        return WAPI_ERR_PARAM_INVALID;

    wapi_handler_state_t old_state = PRIV_DATA(self)->current_state;

    static const bool transition_table[WAPI_STATE_COUNT][WAPI_STATE_COUNT] =
    {
        /*                            to 
                                      UNINIT INITING INITED  CFG_CONN CONFIGURED CONNECTING CONNECTED DISCONN  ERROR */
        /* from UNINIT */           { false, true,   false,  false,   false,     false,     false,    false,   true  },
        /* from INITING */          { false, false,  true,   false,   false,     false,     false,    false,   true  },
        /* from INITED */           { false, false,  false,  true,    false,     false,     false,    false,   true  },
        /* from CONFIGURING_CONN */ { false, false,  false,  false,   true,      false,     false,    false,   true  },
        /* from CONFIGURED */       { false, false,  false,  false,   false,     true,      false,    false,   true  },
        /* from CONNECTING */       { false, false,  false,  false,   false,     false,     true,     false,   true  },
        /* from CONNECTED */        { false, false,  false,  true,    false,     false,     false,    true,    true  },
        /* from DISCONNECTING */    { false, false,  true,   false,   false,     false,     false,    false,   true  },
        /* from ERROR */            { true,  true,   false,  false,   false,     false,     false,    false,   false }
    };

    bool is_valid = false;
    if ((old_state < WAPI_STATE_COUNT) && (new_state < WAPI_STATE_COUNT))
    {
        is_valid = transition_table[old_state][new_state];
    }
    
    if (!is_valid)
    {
        WAPI_DEBUG_ERR("WAPI Invalid state transition: %s -> %s",
                       wapi_get_state_name(old_state), wapi_get_state_name(new_state));
        return WAPI_ERR_OTHERS;
    }
    
    PRIV_DATA(self)->current_state = new_state;
    WAPI_DEBUG_OUT("WAPI State transition: %s -> %s", wapi_get_state_name(old_state), wapi_get_state_name(new_state));
    return WAPI_OK;
}

/* ============================================================================
 * Utility Functions Implementation
 * ============================================================================ */
/* Generic substring search utility function - returns position or -1 if not found */
static int16_t find_substring_in_buffer(uint8_t *buf, uint16_t len, const char* string)
{
    if (!buf || !string)
        return -1;
    
    uint16_t str_len = strlen(string);
    if (0 == str_len || len < str_len)
        return -1;

    /* Use sliding window to traverse buf and search for substring */
    for (uint16_t i = 0; i <= len - str_len; i++)
    {
        /* Starting from buf[i], compare str_len bytes with string */
        if (0 == memcmp(&buf[i], string, str_len))
            return (int16_t)i;
    }
    
    return -1;
}

/* ============================================================================
 * AT Parsing Functions
 * ============================================================================ */

static void at_recv_parse_base(uint8_t *buf, uint16_t len, const char* string, void *arg)
{
    m0804c_handler_t *self = (m0804c_handler_t *)arg;
    if (!self)
        return;
    
    // WAPI_DEBUG_STRING(buf, len);
    
    wapi_recv_state_t wapi_recv_state = {.is_success = false};
    int32_t stat;
    uint16_t string_len;
    int16_t pos;
    
    /* Basic parameter validation (null pointer/invalid length) */
    if(!buf || !string)
    {
        WAPI_DEBUG_ERR("Invalid parameter: buf or string is NULL");
        goto exit;
    }
    
    /* Boundary check: substring length is 0 or longer than buf ¡ú impossible to contain */
    string_len = strlen(string);
    if (0 == string_len || len < string_len)
    {
        WAPI_DEBUG_ERR("Length mismatch: buf_len=%u, pattern_len=%u", len, string_len);
        goto exit;
    }

    pos = find_substring_in_buffer(buf, len, string);
    if (pos >= 0)
    {
        WAPI_DEBUG_OUT("Pattern matched at pos=%d, buffer_len=%u", pos, len);
        wapi_recv_state.is_success = true;
        stat = UP_OS(self)->pf_os_queue_put(PRIV_DATA(self)->recv_state_queue_handle, &wapi_recv_state, 0);
        if(0 != stat)
        {
            WAPI_DEBUG_ERR("Queue put failed (stat=%d)", stat);
        }           
        return; /* Found substring, match successful */
    }

    /* Traversal complete, substring not found */
    WAPI_DEBUG_ERR("Pattern not found in buffer");
    
    exit:        
        stat = UP_OS(self)->pf_os_queue_put(PRIV_DATA(self)->recv_state_queue_handle, &wapi_recv_state, 0);   
        if(0 != stat)
        {
            WAPI_DEBUG_ERR("Failed to put state in queue (stat=%d)", stat);
        }   
        return; 

}

static void at_recv_parse_ok(uint8_t *buf, uint16_t len, void *arg)
{
    at_recv_parse_base(buf, len, "+OK", arg);
}

static void at_recv_parse_tcp_connect(uint8_t *buf, uint16_t len, void *arg)
{
    at_recv_parse_base(buf, len, "tcp alive", arg);
}

static void at_recv_parse_reboot(uint8_t *buf, uint16_t len, void *arg)
{
    at_recv_parse_base(buf, len, "Chip re", arg);
}

static void at_recv_parse_upload_cert_start(uint8_t *buf, uint16_t len, void *arg)
{
    at_recv_parse_base(buf, len, "Start recv", arg);/* for upload */
}

static void at_recv_parse_link_layer_check(uint8_t *buf, uint16_t len, void *arg)
{
    at_recv_parse_base(buf, len, "WAPI STATUS IS 1", arg);
}

static void recv_force_correct(uint8_t *buf, uint16_t len, void *arg)
{
    m0804c_handler_t *self = (m0804c_handler_t *)arg;
    if (!self)
        return;
    /* receive dummy data, so directly return OK  */
    wapi_recv_state_t wapi_recv_state = {.is_success = true};
    UP_OS(self)->pf_os_queue_put(PRIV_DATA(self)->recv_state_queue_handle, &wapi_recv_state, 0);
}

static void sync_multi_send(uint8_t *buf, uint16_t len, void *arg)
{    
    m0804c_handler_t *self = (m0804c_handler_t *)arg;
    if (!self)
        return;
    at_reset_send_state(PRIV_DATA(self)->at_handler);
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->multi_send_syn_sema_handle);
}

static void multi_send_complete_cb(uint8_t *buf, uint16_t len, void *arg)
{    
    sync_multi_send(buf, len, arg);
    recv_force_correct(buf, len, arg);
    // at_recv_parse_ok(buf, len, arg);
}

static at_status_t multi_send_recv_timeout(void *arg)
{    
    m0804c_handler_t *self = (m0804c_handler_t *)arg;
    if (!self)
        return AT_ERR_PARAM_INVALID;
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->multi_send_syn_sema_handle);
    return AT_OK;   
}

/* ============================================================================
 * WAPI Operation Functions
 * ============================================================================ */
#if 1
static void wapi_test(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), TEST);/* not echo */    
}

static void wapi_no_echo(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), SET_ECHO, 0);/* not echo */    
}

static void wapi_get_version(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), GET_VERSION);/* not echo */    
}

static void wapi_both_2p4_5g(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), SET_BAND, 3);/* 2.4G and 5G compatible */ 
}

static void wapi_reboot(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), AT_REBOOT);/* reboot after set band */ 
}

static void wapi_set_tx_pwr(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), SET_TX_PWR);/* diable low power model */
}

static void wapi_disable_low_pwr(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), SET_LOW_PWR, 0);/* diable low power model */
}

static void wapi_disconn_transect(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), DISCONN_TRANS);
}

static void wapi_set_net_config(m0804c_handler_t *const self)
{
    wapi_info_t *wapi_info = self->input_arg->data_provider->pf_get_wapi_info(self);
    AT_CMD_SEND(wapi_get_at_handler(self), SET_IP, 1, wapi_info->local_ip[0], \
                wapi_info->local_ip[1], wapi_info->local_ip[2], wapi_info->local_ip[3],\
                wapi_info->local_ip_mask[0], wapi_info->local_ip_mask[1],\
                wapi_info->local_ip_mask[2], wapi_info->local_ip_mask[3],\
                wapi_info->local_gateway[0], wapi_info->local_gateway[1],\
                wapi_info->local_gateway[2], wapi_info->local_gateway[3]);/* set net configuration */
}

#if IS_USE_CONN_BY_CERT
static void wapi_check_cert(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), CHECK_CERT);/* check certifacate */    
}

static void wapi_connect_by_cert(m0804c_handler_t *const self)
{
    wapi_info_t *wapi_info = self->input_arg->data_provider->pf_get_wapi_info(self);
    AT_CMD_SEND(wapi_get_at_handler(self), CONN_WAPI_BY_CERT, 0, &wapi_info->ssid);
}
#endif

#if IS_USE_CONN_BY_PWD
static void wapi_connect_by_pwd(m0804c_handler_t *const self)
{
    wapi_info_t *wapi_info = self->input_arg->data_provider->pf_get_wapi_info(self);
    AT_CMD_SEND(wapi_get_at_handler(self), CONN_WAPI_BY_PWD, 0, &wapi_info->ssid, &wapi_info->pwd);
}
#endif

static void wapi_check_link_layer_connect(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), CHECK_LINK_LAYER);
}

static void wapi_tcp_connect(m0804c_handler_t *const self)
{
    wapi_info_t *wapi_info = self->input_arg->data_provider->pf_get_wapi_info(self);
    AT_CMD_SEND(wapi_get_at_handler(self), TCP_UDP_CONN, "TCP", wapi_info->server_ip[0],\
                 wapi_info->server_ip[1], wapi_info->server_ip[2], wapi_info->server_ip[3],\
                 wapi_info->server_port, wapi_info->local_port, 1, 1, 1, 2, CUR_SOCKET);
}

static void wapi_tcp_disconnect(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), DISCONN_SOCKET, CUR_SOCKET);
}

static void wapi_recv_data(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), RECV_DATA, CUR_SOCKET, 1, 1);
}

static uint8_t nibble_to_hex_char(uint8_t nibble)
{
    if(nibble < 10)
    {
        return nibble + '0';
    }
    else if(nibble < 16)
    {
        return nibble + 'A' - 10;
    }
    else
    {
        return 'H';
    }
}

static void byte_array_to_hex_string(uint8_t *source, uint16_t source_len, uint8_t *dest, uint16_t *dest_len)
{
    uint16_t pos = source_len * 2 - 1;
    uint8_t byte_idx = source_len - 1;
    uint8_t nibble = 0;
    
    while(1)
    {
        if(pos % 2 != 0)
        {
            nibble = source[byte_idx] & 0x0F;
        }
        else
        {
            nibble = (source[byte_idx] >> 4) & 0x0F;
            if(byte_idx)
            {
                byte_idx--;
            }
        }

        dest[pos] = nibble_to_hex_char(nibble);
        if(0 == pos)
        {
            break;
        }
        pos--;
    }
    *dest_len = source_len * 2;
}

static void reset_wapi_state(m0804c_handler_t *self)
{
    at_reset_send_state(wapi_get_at_handler(self));    
}

static void check_connect(uint8_t *buf, uint16_t len, void *arg)
{            
    m0804c_handler_t *self = (m0804c_handler_t *)arg;
    if (!self)
        return;
     
    char string[] = "[ERR] Socket not in use!";
    
    /* Basic parameter validation (null pointer/invalid length) */
    if(!buf)
    {
        WAPI_DEBUG_ERR("Invalid parameter: buf is NULL");
        return;
    }
    
    /* Boundary check: substring length is 0 or longer than buf ¡ú impossible to contain */
    uint16_t string_len = strlen(string);
    if (0 == string_len || len < string_len)
    {
        return;
    }

    if (wapi_get_state(self) == WAPI_STATE_CONNECTED && find_substring_in_buffer(buf, len, string) >= 0)
    {
        WAPI_DEBUG_ERR("Socket error detected in CONNECTED state, reconnecting...");
        /* Transition to error state first, then restart */
        wapi_state_transition(self, WAPI_STATE_ERROR);         
        m0804c_init(self);/* restart init and connect process */  
#if IS_USE_CONN_BY_CERT
        if(CONN_BY_CERT == PRIV_DATA(self)->wapi_conn_mode)
            m0804c_use_cert_conn(self);
#endif
#if IS_USE_CONN_BY_PWD
        if(CONN_BY_PWD == PRIV_DATA(self)->wapi_conn_mode)
            m0804c_use_pwd_conn(self);
#endif
        return; /* Found substring, match successful */
    }
}

static void send_recv_cb(uint8_t *buf, uint16_t len, void *arg)
{
    pf_at_recv_parse_t recv_parse_cb = (pf_at_recv_parse_t)arg;
    if (recv_parse_cb)
        recv_parse_cb(buf, len, arg);
}

static wapi_status_t wapi_send_data(m0804c_handler_t *self, uint8_t *buf, uint16_t length,
                                    pf_at_recv_parse_t recv_parse_cb)
{
    if (!self || !buf || 0 == length)
        return WAPI_ERR_PARAM_INVALID;
    
    const char *cmd_format = "AT+NSEND,%d,1,";
    uint16_t command_len = snprintf((char*)PRIV_DATA(self)->wapi_send_buf, SEND_BUF_SIZE, cmd_format, CUR_SOCKET);
    
    if (command_len >= SEND_BUF_SIZE)
    {
        WAPI_DEBUG_ERR("Send buffer overflow: command too long (len=%u, max=%u)", command_len, SEND_BUF_SIZE);
        return WAPI_ERR_OTHERS;
    }
    
    uint16_t hex_len;
    byte_array_to_hex_string(buf, length, PRIV_DATA(self)->wapi_send_buf + command_len, &hex_len);
    
    uint16_t total_len = command_len + hex_len + 2; /* +2 for "\r\n" */
    if (total_len > SEND_BUF_SIZE)
    {
        WAPI_DEBUG_ERR("Send buffer overflow: total length exceeds limit (len=%u, max=%u)", total_len, SEND_BUF_SIZE);
        return WAPI_ERR_OTHERS;
    }
    
    PRIV_DATA(self)->wapi_send_buf[total_len - 2] = '\r';
    PRIV_DATA(self)->wapi_send_buf[total_len - 1] = '\n';
    
    /* Configure callback based on whether custom callback is provided */
    at_trans_callback_t callback;
    if (recv_parse_cb)
    {
        /* With custom response callback */
        callback = (at_trans_callback_t)
        {
            .recv_callbacks = {
                .items = {
                    {at_recv_parse_ok, self},
                    {send_recv_cb, (void *)recv_parse_cb}
                },
                .count = 2
            }
        };
    }
    else
    {
        /* Without custom response callback */
        callback = (at_trans_callback_t)
        {
            .recv_callbacks = {
                .items = {{at_recv_parse_ok, self}},
                .count = 1
            }
        };
    }
    
    at_status_t status = at_trans_send(wapi_get_at_handler(self), PRIV_DATA(self)->wapi_send_buf,
                                         total_len, &callback);
    return (status == AT_OK) ? WAPI_OK : WAPI_ERR_OTHERS;
}

static void wapi_upload_as_cert(m0804c_handler_t *const self)
{    
    AT_CMD_SEND(wapi_get_at_handler(self), UPLOAD_CERT_START, "AS");
}

static void wapi_upload_asue_cert(m0804c_handler_t *const self)
{
    AT_CMD_SEND(wapi_get_at_handler(self), UPLOAD_CERT_START, "ASUE");
}

/* Common function for uploading certificate files in segments */
static void wapi_upload_cert_file_common(m0804c_handler_t *self, file_att_t *file)
{
    const uint8_t seg_len = 64;
    uint8_t cnt = (file->file_append.file_len + seg_len - 1)/seg_len;
    
    for(uint8_t i=0; i<cnt; i++)
    {
        uint8_t send_len = seg_len;
        pf_at_recv_parse_t pf_at_recv_parse = sync_multi_send;
        
        if(cnt-1 == i)
        {
           pf_at_recv_parse = multi_send_complete_cb;
           send_len = file->file_append.file_len % seg_len;
        }
            
        uint32_t offset = seg_len * i;
        int32_t stat = AT_OS(self)->pf_sema_take(PRIV_DATA(self)->multi_send_syn_sema_handle,\
                                                 AT_TIMEOUT_TICK_STANDARD);
        if(0 == stat)
        {
            at_trans_callback_t callback = {
                .recv_callbacks = {
                    .items = {{pf_at_recv_parse, self}},
                    .count = 1
                }
            };
            at_trans_send(wapi_get_at_handler(self), file->file_payload + offset, send_len,
                             &callback);  
        }
        else   
        {
            multi_send_recv_timeout((void *)self);
            break;
        }
    }
}

static void wapi_upload_as_cert_file(m0804c_handler_t *const self)
{     
    file_att_t *file = &self->input_arg->data_provider->pf_get_cert_file(self)->as_file;
    wapi_upload_cert_file_common(self, file);
}

static void wapi_upload_asue_cert_file(m0804c_handler_t *const self)
{     
    file_att_t *file = &self->input_arg->data_provider->pf_get_cert_file(self)->asue_file;
    wapi_upload_cert_file_common(self, file);
}
#endif

/* ============================================================================
 * Process Core Functions
 * ============================================================================ */
static wapi_status_t table_process(m0804c_handler_t *const self, wapi_process_t *const wapi_process,\
                                     uint8_t table_num, int8_t *const err_index)
{
    if(!self || !wapi_process || !table_num || !err_index)
        return WAPI_ERR_PARAM_INVALID;
    uint8_t i=0;
    for(; i<table_num; i++)
    {
        int32_t ret = AT_OS(self)->pf_sema_take(PRIV_DATA(self)->process_syn_sema_handle, 0);
        if(0 != ret)
        {
            WAPI_DEBUG_ERR("Failed to acquire sync semaphore (index=%u, ret=%d)", i, ret);
            goto exit;
        } 
        bool is_success = false;
        for(uint8_t j=0; j<AT_ERR_REPEAT_CNT; j++)
        {
            if(wapi_process[i].pf_wapi_process_fun)
                wapi_process[i].pf_wapi_process_fun(self);
            else
            {
                WAPI_DEBUG_ERR("Process function pointer is NULL at index %u", i);
                goto exit;
            }
            /* release in response correct */
            wapi_recv_state_t wapi_recv_state;            
            ret = UP_OS(self)->pf_os_queue_get(PRIV_DATA(self)->recv_state_queue_handle,\
                                                 &wapi_recv_state, wapi_process[i].recv_timeout_tick);
            if(0 == ret)
            {
                if(wapi_recv_state.is_success)
                {
                    is_success = true;
                    if(wapi_process[i].pf_process_cpl)
                        wapi_process[i].pf_process_cpl(self, i, PROCESS_OK); 
                    if(wapi_process[i].process_success_interval_tick)
                        self->input_arg->os_interface->pf_os_delay_ms((int32_t)wapi_process[i].process_success_interval_tick);    
                    ret = AT_OS(self)->pf_sema_give(PRIV_DATA(self)->process_syn_sema_handle);
                    if(0 != ret)
                    {
                        WAPI_DEBUG_ERR("Failed to release sync semaphore (ret=%d)", ret);
                        goto exit;
                    }                      
                    break;
                }  
                else
                {
                    /* Queue returned error response - treat as command failure, will retry */
                    WAPI_DEBUG_ERR("AT command failed at step %u, retrying: attempt %u/%u", i, j + 1, AT_ERR_REPEAT_CNT);
                    if(wapi_process[i].process_fail_interval_tick)
                        self->input_arg->os_interface->pf_os_delay_ms((int32_t)wapi_process[i].process_fail_interval_tick);
                }
            }                
            else
            {
                /* Queue get timeout - AT command timeout or no response received */
                WAPI_DEBUG_ERR("AT command timeout at step %u, retrying: attempt %u/%u", i, j + 1, AT_ERR_REPEAT_CNT);
                /* Treat timeout as command failure and retry with delay */
                if(wapi_process[i].process_fail_interval_tick)
                    self->input_arg->os_interface->pf_os_delay_ms((int32_t)wapi_process[i].process_fail_interval_tick);
            }
        }   
        if(!is_success)
        {
            WAPI_DEBUG_ERR("WAPI process failed after %u retries at index %u", AT_ERR_REPEAT_CNT, i);
            goto exit;                          
        }          
    }    
    return WAPI_OK;
    exit:
        *err_index = i;
        if(wapi_process[i].pf_process_cpl)
            wapi_process[i].pf_process_cpl(self, i, PROCESS_ERR); 
        AT_OS(self)->pf_sema_give(PRIV_DATA(self)->process_syn_sema_handle);    
        return WAPI_ERR_OTHERS; 
}

static wapi_status_t generic_process(m0804c_handler_t *const self, wapi_process_t *const process_table,
                                      uint16_t table_size, const char *process_name)
{
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_PARAM_INVALID;
    
    int8_t err_index = -1;
    uint8_t table_num = table_size / sizeof(wapi_process_t);
    wapi_status_t ret = WAPI_ERR_OTHERS;
    
    for(uint8_t i = 0; i < WAPI_PROCESS_RETRY_MAX; i++)
    {
        ret = table_process(self, process_table, table_num, &err_index);
        if(WAPI_OK == ret)
        {
            WAPI_DEBUG_OUT("%s completed successfully on attempt %u", process_name, i + 1);
            break;
        }
        WAPI_DEBUG_ERR("%s failed: step %u, attempt %u/%u", process_name, err_index, i + 1, WAPI_PROCESS_RETRY_MAX);
    }
    
    return ret;
}

static wapi_status_t wapi_init_process(m0804c_handler_t *const self)
{
    return generic_process(self, wapi_process_init, 
                          sizeof(wapi_process_init), "init process");
}

#if IS_USE_CONN_BY_CERT
static wapi_status_t connect_by_cert_process(m0804c_handler_t *const self)
{
    return generic_process(self, wapi_process_use_cert,
                          sizeof(wapi_process_use_cert), "use cert process");
}
#endif

#if IS_USE_CONN_BY_PWD
static wapi_status_t connect_by_pwd_process(m0804c_handler_t *const self)
{
    return generic_process(self, wapi_process_use_pwd,
                          sizeof(wapi_process_use_pwd), "use pwd process");
}
#endif

static wapi_status_t connect_net_process(m0804c_handler_t *const self)
{
    return generic_process(self, wapi_process_conn_net,
                          sizeof(wapi_process_conn_net), "connect process");
}

static wapi_status_t cert_upload_process(m0804c_handler_t *const self)
{
    return generic_process(self, wapi_process_upload_certiface,
                          sizeof(wapi_process_upload_certiface), "upload cert process");               
}

static wapi_status_t disconn_process(m0804c_handler_t *const self)
{
    return generic_process(self, wapi_process_disconn,
                          sizeof(wapi_process_disconn), "close process");
}

/* ============================================================================
 * Thread Functions
 * ============================================================================ */
/* Generic thread function */
static void generic_wapi_thread(m0804c_handler_t *self, 
                                wapi_status_t (*process_fn)(m0804c_handler_t *),
                                wapi_process_callbacks_t *cbs)
{
    uint8_t fail_cnt = 0;
    while (1)
    {
        if(cbs && cbs->pf_process_start)
            cbs->pf_process_start(self);

        wapi_status_t ret = process_fn(self);
        
        if(WAPI_OK == ret)
        {
            fail_cnt = 0;
            
            /* Notify observers of successful process completion */
            wapi_event_t event = WAPI_EVENT_ERROR; /* Default to error */
            switch(cbs->process_type)
            {
                case PROCESS_INIT:
                    event = WAPI_EVENT_INIT_SUCCESS;
                    break;
                case PROCESS_CERT_AUTH:
                    event = WAPI_EVENT_CERT_AUTH_SUCCESS;
                    break;
                case PROCESS_PWD_AUTH:
                    event = WAPI_EVENT_PWD_AUTH_SUCCESS;
                    break;
                case PROCESS_CONNECT:
                    event = WAPI_EVENT_CONNECTED;
                    break;
                default:
                    break;
            }
            wapi_subject_notify(self, event);
            
            if(cbs && cbs->pf_process_success)
                cbs->pf_process_success(self);
        } 
        else if(fail_cnt < WAPI_PROCESS_FAIL_MAX)
        {
            fail_cnt++;
            WAPI_DEBUG_ERR("%s failed, retry_count=%u/%u", cbs->process_name, fail_cnt, WAPI_PROCESS_FAIL_MAX);
            if(cbs && cbs->pf_process_retry)
                cbs->pf_process_retry(self);
            else if(cbs && cbs->pf_process_start)
                cbs->pf_process_start(self);
        }
        else
        {
            WAPI_DEBUG_ERR("%s failed permanently after %u attempts", cbs->process_name, WAPI_PROCESS_FAIL_MAX);
            
            /* Notify observers of failed process */
            wapi_event_t event = WAPI_EVENT_ERROR; /* Default to error */
            switch(cbs->process_type)
            {
                case PROCESS_INIT:
                    event = WAPI_EVENT_INIT_FAILED;
                    break;
                case PROCESS_CERT_AUTH:
                    event = WAPI_EVENT_CERT_AUTH_FAILED;
                    break;
                case PROCESS_PWD_AUTH:
                    event = WAPI_EVENT_PWD_AUTH_FAILED;
                    break;
                case PROCESS_CONNECT:
                    event = WAPI_EVENT_CONNECT_FAILED;
                    break;
                default:
                    break;
            }
            wapi_subject_notify(self, event);
        }
    }
}

static void wapi_init_thread(void *arg)
{
    m0804c_handler_t *self = (m0804c_handler_t *)arg;    
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
    {
        WAPI_DEBUG_ERR("Init thread: invalid parameter or not initialized");
        return;
    }
    generic_wapi_thread(self, wapi_init_process,
                       &init_callbacks);
}

#if IS_USE_CONN_BY_CERT
static void wapi_use_cert_thread(void *arg)
{
    m0804c_handler_t *self = (m0804c_handler_t *)arg;    
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
    {
        WAPI_DEBUG_ERR("Cert thread: invalid parameter or not initialized");
        return;
    }   
    generic_wapi_thread(self, connect_by_cert_process,
                       &conn_cfg_by_cert_callbacks);
}
#endif

#if IS_USE_CONN_BY_PWD
static void wapi_use_pwd_thread(void *arg)
{
    m0804c_handler_t *self = (m0804c_handler_t *)arg;    
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
    {
        WAPI_DEBUG_ERR("Pwd thread: invalid parameter or not initialized");
        return;
    }    
    generic_wapi_thread(self, connect_by_pwd_process,
                       &conn_cfg_by_pwd_callbacks);
}
#endif

static void wapi_conn_thread(void *arg)
{
    m0804c_handler_t *self = (m0804c_handler_t *)arg;    
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
    {
        WAPI_DEBUG_ERR("Conn thread: invalid parameter or not initialized");
        return;
    }    
    generic_wapi_thread(self, connect_net_process,
                       &conn_callbacks);
}

static wapi_status_t m0804c_start_recv(m0804c_handler_t *const self)
{
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_HANDLER_NOT_READY;
    char send_buf[32] = {0};
    int total_len = sprintf(send_buf, "AT+NRECV,%d,1,1\r\n", CUR_SOCKET);
    at_trans_callback_t callback = {
        .recv_callbacks = {
            .items = {{at_recv_parse_ok, self}},
            .count = 1
        }
    };
    at_status_t status = at_trans_send(wapi_get_at_handler(self), (uint8_t *)send_buf,
                                         total_len, &callback);
    return (status == AT_OK) ? WAPI_OK : WAPI_ERR_OTHERS;
}

/* Forward declaration for wapi_subject_init (called from m0804c_inst) */
static wapi_status_t wapi_subject_init(m0804c_handler_t *const self);

wapi_status_t m0804c_inst(m0804c_handler_t *const self, wapi_m0804c_input_arg_t *const p_input_args)
{
    /* Comprehensive input validation */
    if(!self || !p_input_args ||
       !p_input_args->os_interface|| 
       !p_input_args->os_interface->pf_os_delay_ms||  
       !p_input_args->at_input_arg ||
       !p_input_args->pwr_ops ||
       !p_input_args->pwr_ops->pf_m0804c_open ||
       !p_input_args->pwr_ops->pf_m0804c_close ||
       !p_input_args->data_provider ||
       !p_input_args->data_provider->pf_get_wapi_info ||
       !p_input_args->data_provider->pf_get_cert_file)
    {
        return WAPI_ERR_PARAM_INVALID;
    }
    
    /* Allocate private data and at_handler for this instance */
    PRIV_DATA(self) = (m0804c_priv_data_t *)MALLOC(sizeof(m0804c_priv_data_t));
    if (!PRIV_DATA(self))
        return WAPI_ERR_OTHERS;

    memset(PRIV_DATA(self), 0, sizeof(m0804c_priv_data_t));
    
    /* Initialize state machine to UNINIT */
    PRIV_DATA(self)->current_state = WAPI_STATE_UNINIT;

    /* Allocate at_handler */
    PRIV_DATA(self)->at_handler = (at_handler_t *)MALLOC(sizeof(at_handler_t));
    if (!PRIV_DATA(self)->at_handler)
    {
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }   

    /* Use global AT command table (no need for instance-specific copy) */
    if(!p_input_args->at_input_arg->at_cmd_set_table)
    {
        p_input_args->at_input_arg->at_cmd_set_table = &g_m0804c_at_cmd_set_table;
        WAPI_DEBUG_OUT("Using built-in AT command table");
    }

    /* Set holder to self so AT handler knows which module instance owns it */
    p_input_args->at_input_arg->holder = self;

    at_input_arg_t *at_arg = p_input_args->at_input_arg;
    at_status_t at_status = at_inst(PRIV_DATA(self)->at_handler, at_arg);   
    if(AT_OK != at_status)
    {
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    self->input_arg = p_input_args;

    /* Create a mutable copy of AT command table with arg set to self for all callbacks */
    uint8_t cmd_table_len = g_m0804c_at_cmd_set_table.table_len;
    PRIV_DATA(self)->at_cmd_table_copy = (at_cmd_set_t *)MALLOC(cmd_table_len * sizeof(at_cmd_set_t));
    if (!PRIV_DATA(self)->at_cmd_table_copy)
    {
        WAPI_DEBUG_ERR("Failed to allocate AT command table copy");
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    
    /* Copy table and set arg to self for all callbacks */
    memcpy(PRIV_DATA(self)->at_cmd_table_copy, g_m0804c_at_cmd_set_table.table, cmd_table_len * sizeof(at_cmd_set_t));
    for (uint8_t i = 0; i < cmd_table_len; i++)
    {
        for (uint8_t j = 0; j < PRIV_DATA(self)->at_cmd_table_copy[i].recv_callbacks.count; j++)
        {
            PRIV_DATA(self)->at_cmd_table_copy[i].recv_callbacks.items[j].arg = self;
        }
    }
    
    /* Update at_input_arg to use our mutable table copy */
    at_arg->at_cmd_set_table->table = PRIV_DATA(self)->at_cmd_table_copy;

    int32_t ret = AT_OS(self)->pf_sema_binary_create(&PRIV_DATA(self)->process_syn_sema_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("process_syn_sema creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->process_syn_sema_handle);

    ret = AT_OS(self)->pf_sema_binary_create(&PRIV_DATA(self)->multi_send_syn_sema_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("multi_send_syn_sema creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->multi_send_syn_sema_handle);

    ret = UP_OS(self)->pf_os_queue_create(1, sizeof(wapi_recv_state_t), &PRIV_DATA(self)->recv_state_queue_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("recv_state_queue creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }

    ret = AT_OS(self)->pf_sema_binary_create(&PRIV_DATA(self)->init_start_sema_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("init_start_sema creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->init_start_sema_handle, 0);
    
    ret = AT_OS(self)->pf_sema_binary_create(&PRIV_DATA(self)->init_success_sema_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("init_success_sema creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->init_success_sema_handle, 0);
    
#if IS_USE_CONN_BY_CERT
    ret = AT_OS(self)->pf_sema_binary_create(&PRIV_DATA(self)->use_cert_sema_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("use_cert_sema creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->use_cert_sema_handle, 0);
    
    ret = UP_OS(self)->pf_os_thread_create("use_cert", wapi_use_cert_thread, WAPI_THREAD_STACK_SIZE, \
                WAPI_THREAD_PRIORITY, NULL, self);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("wapi_use_cert_thread creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
#endif
    
#if IS_USE_CONN_BY_PWD
    ret = AT_OS(self)->pf_sema_binary_create(&PRIV_DATA(self)->use_pwd_sema_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("use_pwd_sema creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->use_pwd_sema_handle, 0);
    
    ret = UP_OS(self)->pf_os_thread_create("use_pwd", wapi_use_pwd_thread, WAPI_THREAD_STACK_SIZE, \
                WAPI_THREAD_PRIORITY, NULL, self);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("wapi_use_pwd_thread creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
#endif
    
    ret = AT_OS(self)->pf_sema_binary_create(&PRIV_DATA(self)->connect_cfg_success_sema_handle);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("connect_cfg_success_sema creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }
    AT_OS(self)->pf_sema_take(PRIV_DATA(self)->connect_cfg_success_sema_handle, 0);

    ret = UP_OS(self)->pf_os_thread_create("wapi_init", wapi_init_thread, WAPI_THREAD_STACK_SIZE, \
                WAPI_THREAD_PRIORITY, NULL, self);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("wapi_init_thread creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }

    ret = UP_OS(self)->pf_os_thread_create("wapi_conn", wapi_conn_thread, WAPI_THREAD_STACK_SIZE, \
                WAPI_THREAD_PRIORITY, NULL, self);
    if(0 != ret)
    {
        WAPI_DEBUG_ERR("wapi_conn_thread creation failed (ret=%d)", ret);
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }

    /* Initialize observer subject for event notification system */
    if (WAPI_OK != wapi_subject_init(self))
    {
        WAPI_DEBUG_ERR("Observer subject initialization failed");
        FREE(PRIV_DATA(self)->at_cmd_table_copy);
        FREE(PRIV_DATA(self)->at_handler);
        if (PRIV_DATA(self)->observer_subject)
            FREE(PRIV_DATA(self)->observer_subject);
        FREE(PRIV_DATA(self));
        return WAPI_ERR_OTHERS;
    }

    PRIV_DATA(self)->is_inited = true;
    return WAPI_OK;
}

/* call in IDLE ISR */
void m0804c_at_notify_recv_isr_cb(m0804c_handler_t *const self)
{
    at_notify_recv_isr_cb(wapi_get_at_handler(self));
}
/* call in tx complete ISR */
void m0804c_at_send_complete_isr_cb(m0804c_handler_t *const self)
{
    at_send_complete_isr_cb(wapi_get_at_handler(self));
}
/* call in UART/DMA error ISR */
void m0804c_at_error_recv_isr_cb(m0804c_handler_t *const self)
{
    at_error_recv_isr_cb(wapi_get_at_handler(self));
}

wapi_status_t m0804c_init(m0804c_handler_t *const self)
{
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_HANDLER_NOT_READY;
    at_recv_hook_unregister(PRIV_DATA(self)->at_handler);
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->init_start_sema_handle);    
    return WAPI_OK;
}
#if IS_USE_CONN_BY_CERT
wapi_status_t m0804c_use_cert_conn(m0804c_handler_t *const self)
{

    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_HANDLER_NOT_READY;
    at_recv_hook_unregister(PRIV_DATA(self)->at_handler);
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->use_cert_sema_handle);    
    return WAPI_OK;
}
#endif

#if IS_USE_CONN_BY_PWD
wapi_status_t m0804c_use_pwd_conn(m0804c_handler_t *const self)
{

    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_HANDLER_NOT_READY;
    at_recv_hook_unregister(PRIV_DATA(self)->at_handler);
    AT_OS(self)->pf_sema_give(PRIV_DATA(self)->use_pwd_sema_handle);    
    return WAPI_OK;
}
#endif

wapi_status_t m0804c_send(m0804c_handler_t *const self, uint8_t *buf, uint16_t length,
                         pf_at_recv_parse_t recv_parse_cb)
{
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_HANDLER_NOT_READY;

    /* Only allow data transmission when in CONNECTED state (replaces trans_send_flag) */
    if(wapi_get_state(self) == WAPI_STATE_CONNECTED) 
    {
        return wapi_send_data(self, buf, length, recv_parse_cb);
    }        
    else
    {
        WAPI_DEBUG_ERR("Cannot send data: device not in CONNECTED state (current: %s)", 
                       wapi_get_state_name(wapi_get_state(self)));
        return WAPI_ERR_SEND_NOT_READY;
    }
}

wapi_status_t m0804c_cert_upload(m0804c_handler_t *const self)
{
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_HANDLER_NOT_READY;
    wapi_info_t *wapi_info = self->input_arg->data_provider->pf_get_wapi_info(self);  
    if(!wapi_info->is_exist_certicate)  
        return WAPI_ERR_MISS_CERT;
    reset_wapi_state(self);
    return cert_upload_process(self);      
}

wapi_status_t m0804c_disconn(m0804c_handler_t *const self)
{
    if(!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return WAPI_ERR_HANDLER_NOT_READY;
    return disconn_process(self);
}

/* Public State Query Functions --------------------------------------------- */
wapi_handler_state_t wapi_get_state(m0804c_handler_t *const self)
{
    if (!self || !PRIV_DATA(self))
        return WAPI_STATE_UNINIT;
    return PRIV_DATA(self)->current_state;
}

const char* wapi_get_state_name(wapi_handler_state_t state)
{
    static const char* state_names[] = 
    {
        [WAPI_STATE_UNINIT]          = "UNINIT",
        [WAPI_STATE_INITING]         = "INITING",
        [WAPI_STATE_INITED]          = "INITED",
        [WAPI_STATE_CONFIGURING_CONN] = "CONFIGURING_CONN",
        [WAPI_STATE_CONFIGURED]      = "CONFIGURED",
        [WAPI_STATE_CONNECTING]      = "CONNECTING",
        [WAPI_STATE_CONNECTED]       = "CONNECTED",
        [WAPI_STATE_DISCONNECTING]   = "DISCONNECTING",
        [WAPI_STATE_ERROR]           = "ERROR"
    };
    
    if (state < (sizeof(state_names) / sizeof(state_names[0])))
        return state_names[state];
    return "UNKNOWN";
}

/* return true when valid, others invalid */
bool is_wapi_info_valid(wapi_info_t *const wapi_info) 
{
    uint16_t payload_len = sizeof(wapi_info_t) - sizeof(uint16_t);
    uint16_t digest_candidate = wapi_info->digest;
    uint16_t digest = M0804C_DATA_INTEGRITY_ALGO((uint8_t *)wapi_info, payload_len);
    return(digest_candidate == digest);                     
}

void reset_wapi_info(wapi_info_t *const wapi_info)
{
    uint16_t digest = M0804C_DATA_INTEGRITY_ALGO((uint8_t *)&g_default_wapi_info,\
                                                 sizeof(wapi_info_t) - sizeof(uint16_t));
    g_default_wapi_info.digest = digest;
    memcpy(wapi_info, &g_default_wapi_info, sizeof(wapi_info_t));    
}

void validate_wapi_info(wapi_info_t *const wapi_info)
{
    uint16_t digest = M0804C_DATA_INTEGRITY_ALGO((uint8_t *)wapi_info, sizeof(wapi_info_t) - sizeof(uint16_t));
    wapi_info->digest = digest;
}

/* ============================================================================
 * Observer Pattern Implementation
 * ============================================================================
 * Implements the Observer design pattern for flexible event notification.
 * Multiple observers can register and receive notifications of WAPI events.
 */

/**
 * @brief Initialize the observer subject (event dispatcher)
 *
 * Allocates and initializes the observer subject structure inside m0804c_priv_data.
 * This should be called after m0804c_inst() to enable observer functionality.
 *
 * @param self Pointer to the WAPI handler instance
 * @return WAPI_OK on success, WAPI_ERR_PARAM_INVALID if self is NULL
 */
static wapi_status_t wapi_subject_init(m0804c_handler_t *const self)
{
    if (!self || !PRIV_DATA(self))
        return WAPI_ERR_PARAM_INVALID;
    
    /* Allocate the subject structure */
    PRIV_DATA(self)->observer_subject = (wapi_subject_t *)MALLOC(sizeof(wapi_subject_t));
    if (!PRIV_DATA(self)->observer_subject)
        return WAPI_ERR_OTHERS;
    
    /* Initialize linked list sentinel */
    t_list_init(&PRIV_DATA(self)->observer_subject->observers_sentinel);
    
    WAPI_DEBUG_OUT("Observer subject initialized");
    
    return WAPI_OK;
}

/**
 * @brief Attach an observer to receive WAPI events
 *
 * Registers an observer to receive notifications. The observer must have a valid
 * on_notify callback function.
 *
 * @param self Pointer to the WAPI handler instance
 * @param observer Pointer to the observer structure
 * @return WAPI_OK on success
 * @return WAPI_ERR_PARAM_INVALID if parameters invalid
 * @return WAPI_ERR_OTHERS if memory allocation fails
 */
wapi_status_t wapi_subject_attach(m0804c_handler_t *const self, wapi_observer_t *observer)
{
    if (!self || !PRIV_DATA(self) || !observer || !observer->on_notify)
        return WAPI_ERR_PARAM_INVALID;
    
    wapi_subject_t *subject = PRIV_DATA(self)->observer_subject;
    if (!subject)
        return WAPI_ERR_PARAM_INVALID;
    
    /* Allocate observer node */
    wapi_observer_node_t *node = (wapi_observer_node_t *)MALLOC(sizeof(wapi_observer_node_t));
    if (!node)
        return WAPI_ERR_OTHERS;
    
    /* Initialize node and attach observer */
    t_list_init(&node->list);
    node->observer = observer;
    
    /* Insert at end of list */
    UP_OS(self)->pf_os_enter_critical();
    t_list_insert_before(&subject->observers_sentinel, &node->list);
    UP_OS(self)->pf_os_exit_critical(0);
    
    WAPI_DEBUG_OUT("Observer attached successfully");
    
    return WAPI_OK;
}

/**
 * @brief Detach an observer from event notifications
 *
 * Unregisters an observer so it no longer receives notifications.
 *
 * @param self Pointer to the WAPI handler instance
 * @param observer Pointer to the observer to remove
 * @return WAPI_OK on success
 * @return WAPI_ERR_PARAM_INVALID if observer not found or invalid parameters
 */
wapi_status_t wapi_subject_detach(m0804c_handler_t *const self, wapi_observer_t *observer)
{
    if (!self || !PRIV_DATA(self) || !observer)
        return WAPI_ERR_PARAM_INVALID;
    
    wapi_subject_t *subject = PRIV_DATA(self)->observer_subject;
    if (!subject)
        return WAPI_ERR_PARAM_INVALID;
    
    /* Find and remove the matching observer node */
    UP_OS(self)->pf_os_enter_critical();
    
    t_list_t *head = &subject->observers_sentinel;
    for (t_list_t *current = head->next; current != head; current = current->next)
    {
        wapi_observer_node_t *node = T_LIST_ENTRY(current, wapi_observer_node_t, list);
        
        if (node->observer == observer)
        {
            t_list_remove(&node->list);
            UP_OS(self)->pf_os_exit_critical(0);
            
            FREE(node);
            WAPI_DEBUG_OUT("Observer detached successfully");
            return WAPI_OK;
        }
    }
    
    UP_OS(self)->pf_os_exit_critical(0);
    WAPI_DEBUG_ERR("Observer not found");
    return WAPI_ERR_PARAM_INVALID;
}

/**
 * @brief Notify all attached observers of a WAPI event
 *
 * Dispatches an event to all registered observers. Each observer's on_notify
 * callback is invoked synchronously in order.
 *
 * @param self Pointer to the WAPI handler instance
 * @param event The WAPI event to notify observers about
 */
void wapi_subject_notify(m0804c_handler_t *const self, wapi_event_t event)
{
    if (!self || !PRIV_DATA(self))
        return;
    
    wapi_subject_t *subject = PRIV_DATA(self)->observer_subject;
    if (!subject)
        return;
    
    WAPI_DEBUG_OUT("Notifying observers of event: %s", wapi_get_event_name(event));
    
    /* Notify all observers in the linked list */
    t_list_t *head = &subject->observers_sentinel;
    for (t_list_t *current = head->next; current != head; current = current->next)
    {
        wapi_observer_node_t *node = T_LIST_ENTRY(current, wapi_observer_node_t, list);
        
        if (node && node->observer && node->observer->on_notify)
        {
            node->observer->on_notify(node->observer, self, event);
        }
    }
}

/**
 * @brief Get human-readable name for a WAPI event
 *
 * @param event The event enumeration value
 * @return const char* Event name string
 */
const char* wapi_get_event_name(wapi_event_t event)
{
    static const char* event_names[] = 
    {
        [WAPI_EVENT_INIT_SUCCESS]      = "INIT_SUCCESS",
        [WAPI_EVENT_INIT_FAILED]       = "INIT_FAILED",
        [WAPI_EVENT_CERT_AUTH_SUCCESS] = "CERT_AUTH_SUCCESS",
        [WAPI_EVENT_CERT_AUTH_FAILED]  = "CERT_AUTH_FAILED",
        [WAPI_EVENT_PWD_AUTH_SUCCESS]  = "PWD_AUTH_SUCCESS",
        [WAPI_EVENT_PWD_AUTH_FAILED]   = "PWD_AUTH_FAILED",
        [WAPI_EVENT_CONNECTED]         = "CONNECTED",
        [WAPI_EVENT_CONNECT_FAILED]    = "CONNECT_FAILED",
        [WAPI_EVENT_DISCONNECTED]      = "DISCONNECTED",
        [WAPI_EVENT_ERROR]             = "ERROR"
    };
    
    if (event < (sizeof(event_names) / sizeof(event_names[0])))
        return event_names[event];
    return "UNKNOWN";
}

/**
 * @brief Get the current number of attached observers
 *
 * @param self Pointer to the WAPI handler instance
 * @return uint8_t Number of observers (0 if uninitialized)
 */
uint8_t wapi_subject_get_observer_count(m0804c_handler_t *const self)
{
    if (!self || !PRIV_DATA(self))
        return 0;
    
    wapi_subject_t *subject = PRIV_DATA(self)->observer_subject;
    if (!subject)
        return 0;
    
    /* Count nodes in linked list */
    uint8_t count = 0;
    t_list_t *head = &subject->observers_sentinel;
    for (t_list_t *current = head->next; current != head; current = current->next)
    {
        count++;
    }
    
    return count;
}


