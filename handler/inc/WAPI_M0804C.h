/**
 * @file WAPI_M0804C.h
 * @brief WAPI M0804C Module Handler (Header File)
 *
 * This header defines the core data structures, enumerations, macros, and public APIs
 * for the M0804C WAPI wireless module handler. It implements a complete interface for
 * managing M0804C device operations including certificate-based and password-based
 * authentication, network connectivity, and data transmission.
 * 
 * Key features:
 * - Support for dual authentication methods (certificate and password)
 * - Network configuration and connectivity management
 * - Certificate file handling and integrity verification
 * - Event-driven callback mechanism for connection state changes
 * - Hardware-independent OSAL interface for platform portability
 * - Debug output support via SEGGER RTT
 */

#ifndef __WAPI_M0804C_H__
#define __WAPI_M0804C_H__

#include "AT_handler.h"
#include "stdbool.h"

#include "algo_data_integrity.h"
#define M0804C_DATA_INTEGRITY_ALGO(buf, len)     checksum_16bit(buf, len)

#define IS_USE_CONN_BY_CERT             1
#define IS_USE_CONN_BY_PWD              1

#if (IS_USE_CONN_BY_CERT == 0) && (IS_USE_CONN_BY_PWD == 0)
#error "At least one connection method must be enabled: IS_USE_CONN_BY_CERT or IS_USE_CONN_BY_PWD"
#endif

#define WAPI_THREAD_PRIORITY            24
#define WAPI_THREAD_STACK_SIZE          2048//1024

#include "SEGGER_RTT.h"
extern int SEGGER_RTT_printf(unsigned BufferIndex, const char *sFormat, ...);
#define WAPI_DEBUG_OUT(fmt, ...)      SEGGER_RTT_printf(0, fmt "\r\n", ##__VA_ARGS__)  /* Output log to RTT buffer 0 */
#define WAPI_DEBUG_ERR(fmt, ...)      SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_RED fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)
#define WAPI_DEBUG_STRING(p_data, len)  \
    do {                                  \
        SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN);  /* Add red color control code at the beginning */ \
        SEGGER_RTT_Write(0, (const char*)p_data, len);        /* Print raw string/buffer */ \
        SEGGER_RTT_WriteString(0, RTT_CTRL_RESET "\r\n");     /* Reset format at the end */ \
    } while(0)

typedef enum
{
    WAPI_OK = 0,                /* Operation completed successfully */
    WAPI_ERR_PARAM_INVALID,     /* Invalid input parameter(s) (e.g., NULL pointer, wrong arg count) */
    WAPI_ERR_HANDLER_NOT_READY, /* AT handler not initialized (UART/command table unready) */
    WAPI_ERR_SEND_NOT_READY,
    WAPI_ERR_MISS_CERT,
    WAPI_ERR_CMD_NOT_FOUND,     /* Specified AT function ID not found in command table */
    WAPI_ERR_RECV_NOT_MATCH,
    WAPI_ERR_OTHERS             /* Unspecified error (e.g., UART transmission failure) */
} wapi_status_t;

typedef enum
{
    PROCESS_INIT = 0,
    PROCESS_CERT_AUTH,
    PROCESS_PWD_AUTH,
    PROCESS_CONNECT
}wapi_process_type_t;

/* ---------------- OSAL interface for M0804C handler ---------------- */
typedef struct
{
    void (*pf_os_delay_ms)(uint32_t ms);
} m0804c_os_interface_t;

typedef struct
{
    uint8_t server_ip[4];
    uint16_t server_port;
    uint16_t local_port;
    bool is_exist_certicate;
    uint8_t local_ip[4];
    uint8_t local_ip_mask[4];
    uint8_t local_gateway[4];
    char ssid[32]; 
    char pwd[16]; 
    uint16_t digest;       
}wapi_info_t;

typedef struct 
{
    uint16_t file_len;
    uint16_t digest;
}file_append_t;

typedef struct 
{    
    uint8_t *file_payload;
    file_append_t file_append;
}file_att_t;

typedef struct 
{
    file_att_t as_file;
    file_att_t asue_file;
}cert_file_t;

typedef struct m0804c_handler m0804c_handler_t;

typedef struct
{
    void (*pf_m0804c_open)(struct m0804c_handler *const self);
    void (*pf_m0804c_close)(struct m0804c_handler *const self);
} m0804c_pwr_ops_t;

typedef struct
{
    wapi_info_t *(*pf_get_wapi_info)(struct m0804c_handler *const self);/* return NULL means invalid */
    cert_file_t *(*pf_get_cert_file)(struct m0804c_handler *const self);/* return NULL means invalid */
} wapi_data_provider_t;

typedef struct
{
    void (*pf_process_success_cb)(struct m0804c_handler *const self, wapi_process_type_t process_type);
    void (*pf_process_err_cb)(struct m0804c_handler *const self, wapi_process_type_t process_type);    
} wapi_callback_t;

typedef struct
{
    /* at_cmd_set_table & parse_algo already built-in , so inject NULL */
    at_input_arg_t          *at_input_arg;  /* Pointer to AT handler input arguments */
    m0804c_os_interface_t   *os_interface;  /* OSAL interface for M0804C handler */
    m0804c_pwr_ops_t        *pwr_ops;       /* Power control operations */       
    wapi_data_provider_t    *data_provider; /* Data providers (info/cert) */
    wapi_callback_t         *callbacks;     /* Event callbacks */
}wapi_m0804c_input_arg_t;

typedef struct m0804c_priv_data m0804c_priv_data_t;

typedef struct m0804c_handler
{
    wapi_m0804c_input_arg_t  *input_arg;
    m0804c_priv_data_t       *priv_data;
}m0804c_handler_t;

wapi_status_t m0804c_inst(m0804c_handler_t *const self, wapi_m0804c_input_arg_t *const p_input_args);
/* call in IDLE ISR */
void m0804c_at_notify_recv_isr_cb(m0804c_handler_t *const self);
/* call in tx complete ISR */
void m0804c_at_send_complete_isr_cb(m0804c_handler_t *const self);
/* call in UART/DMA error ISR */
void m0804c_at_error_recv_isr_cb(m0804c_handler_t *const self);

wapi_status_t m0804c_init(m0804c_handler_t *const self);
#if IS_USE_CONN_BY_CERT
wapi_status_t m0804c_use_cert_conn(m0804c_handler_t *const self);
#endif
#if IS_USE_CONN_BY_PWD
wapi_status_t m0804c_use_pwd_conn(m0804c_handler_t *const self);
#endif
wapi_status_t m0804c_send(m0804c_handler_t *const self, uint8_t *buf, uint16_t length,\
                         pf_at_recv_parse_t recv_parse_cb);
wapi_status_t m0804c_send_without_response(m0804c_handler_t *const self, uint8_t *buf,\
                         uint16_t length);
wapi_status_t m0804c_cert_upload(m0804c_handler_t *const self);

/* return true when valid, others invalid */
bool is_wapi_info_valid(wapi_info_t *const wapi_info);
void reset_wapi_info(wapi_info_t *const wapi_info);
void validate_wapi_info(wapi_info_t *const wapi_info);
#endif /* __WAPI_M0804C_H__ */
