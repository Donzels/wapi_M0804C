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
#include "t_list.h"

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

/**
 * @enum wapi_handler_state_t
 * @brief WAPI M0804C handler state machine states
 * 
 * Defines all possible operational states for the WAPI module handler.
 * State transitions are controlled by process completion callbacks.
 */
typedef enum
{
    WAPI_STATE_UNINIT = 0,          /* Handler not initialized */
    WAPI_STATE_INITING,             /* Initialization process running */
    WAPI_STATE_INITED,              /* Initialization completed */
    WAPI_STATE_CONFIGURING_CONN,    /* Connection configuration in progress */
    WAPI_STATE_CONFIGURED,          /* Connection configured (cert/pwd) */
    WAPI_STATE_CONNECTING,          /* Network connection in progress */
    WAPI_STATE_CONNECTED,           /* Connected to network and server */
    WAPI_STATE_DISCONNECTING,       /* Disconnection in progress */
    WAPI_STATE_ERROR,               /* Error state */
    WAPI_STATE_COUNT                /* Total state count (for bounds checking) */
} wapi_handler_state_t;

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
/* ============================================================================
 * Observer Pattern Implementation for Event Notification System
 * ============================================================================
 * Provides flexible multi-observer support for WAPI events, allowing multiple
 * listeners to react to connection state changes without tight coupling.
 */

/**
 * @enum wapi_event_t
 * @brief WAPI system events
 *
 * Defines all possible events that occur during WAPI module lifecycle
 */
typedef enum
{
    WAPI_EVENT_INIT_SUCCESS = 0,      /* Module initialization completed successfully */
    WAPI_EVENT_INIT_FAILED,           /* Module initialization failed */
    WAPI_EVENT_CERT_AUTH_SUCCESS,     /* Certificate-based authentication succeeded */
    WAPI_EVENT_CERT_AUTH_FAILED,      /* Certificate-based authentication failed */
    WAPI_EVENT_PWD_AUTH_SUCCESS,      /* Password-based authentication succeeded */
    WAPI_EVENT_PWD_AUTH_FAILED,       /* Password-based authentication failed */
    WAPI_EVENT_CONNECTED,             /* Connected to network and server */
    WAPI_EVENT_CONNECT_FAILED,        /* Connection to network/server failed */
    WAPI_EVENT_DISCONNECTED,          /* Disconnected from server/network */
    WAPI_EVENT_ERROR,                 /* Critical error occurred */
    WAPI_EVENT_COUNT                  /* Total event count (for bounds checking) */
} wapi_event_t;

typedef struct m0804c_handler m0804c_handler_t;
/**
 * @struct wapi_observer_t
 * @brief Observer interface for WAPI events
 *
 * Describes an observer that wants to be notified of WAPI events.
 * Each observer has a notification callback and optional context data.
 */
typedef struct wapi_observer
{
    /**
     * Callback function invoked when an event is notified
     * @param observer Pointer to this observer instance
     * @param self Pointer to the WAPI handler that triggered the event
     * @param event The WAPI event that occurred
     */
    void (*on_notify)(struct wapi_observer *observer, 
                      struct m0804c_handler *const self,
                      wapi_event_t event);
    
    void *observer_context;  /* Optional context data for the observer */
} wapi_observer_t;

/**
 * @struct wapi_observer_node_t
 * @brief Observer node for linked list management
 *
 * Wraps observer data in a linked list node for dynamic observer management.
 * Similar to funcode_node_t in uart_proto for flexible extension.
 */
typedef struct
{
    t_list_t list;           /* Linked list node */
    wapi_observer_t *observer; /* Pointer to observer */
} wapi_observer_node_t;

/**
 * @struct wapi_subject_t
 * @brief Subject/Event dispatcher for managing multiple observers
 *
 * Manages a collection of observers using linked list and notifies them of WAPI events.
 * This implements the Subject part of the Observer pattern with unlimited observer capacity.
 */
typedef struct
{
    t_list_t observers_sentinel;  /* Sentinel head for linked list of observers */
} wapi_subject_t;

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
    /* at_cmd_set_table & parse_algo already built-in , so inject NULL */
    at_input_arg_t          *at_input_arg;  /* Pointer to AT handler input arguments */
    m0804c_os_interface_t   *os_interface;  /* OSAL interface for M0804C handler */
    m0804c_pwr_ops_t        *pwr_ops;       /* Power control operations */       
    wapi_data_provider_t    *data_provider; /* Data providers (info/cert) */
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
/* recv_parse_cb = NULL means send without response */
wapi_status_t m0804c_send(m0804c_handler_t *const self, uint8_t *buf, uint16_t length,\
                         pf_at_recv_parse_t recv_parse_cb);
                         
wapi_status_t m0804c_cert_upload(m0804c_handler_t *const self);

/* ============================================================================
 * Observer Pattern Public API
 * ============================================================================
 */

/**
 * @brief Attach an observer to receive WAPI events
 *
 * Registers an observer to receive notifications when WAPI events occur.
 * The same observer instance can only be attached once.
 *
 * @param self Pointer to the WAPI handler instance
 * @param observer Pointer to the observer structure with on_notify callback
 * @return WAPI_OK on success
 * @return WAPI_ERR_PARAM_INVALID if parameters are invalid or observer already attached
 * @return WAPI_ERR_OTHERS if maximum observers exceeded
 */
wapi_status_t wapi_subject_attach(m0804c_handler_t *const self, wapi_observer_t *observer);

/**
 * @brief Detach an observer from event notifications
 *
 * Unregisters an observer so it no longer receives WAPI event notifications.
 *
 * @param self Pointer to the WAPI handler instance
 * @param observer Pointer to the observer structure to remove
 * @return WAPI_OK on success
 * @return WAPI_ERR_PARAM_INVALID if observer not found or invalid parameters
 */
wapi_status_t wapi_subject_detach(m0804c_handler_t *const self, wapi_observer_t *observer);

/**
 * @brief Get current number of attached observers
 *
 * Returns the number of observers currently attached to the event system.
 *
 * @param self Pointer to the WAPI handler instance
 * @return uint8_t Number of observers (0 if self is NULL or uninitialized)
 */
uint8_t wapi_subject_get_observer_count(m0804c_handler_t *const self);


wapi_handler_state_t wapi_get_state(m0804c_handler_t *const self);
const char* wapi_get_event_name(wapi_event_t event);
const char* wapi_get_state_name(wapi_handler_state_t state);

/* return true when valid, others invalid */
bool is_wapi_info_valid(wapi_info_t *const wapi_info);
void reset_wapi_info(wapi_info_t *const wapi_info);
void validate_wapi_info(wapi_info_t *const wapi_info);
#endif /* __WAPI_M0804C_H__ */
