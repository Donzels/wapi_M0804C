/**
 * @file at_handler.h
 * @brief AT Command Handler for WAPI Module (Header File)
 *
 * This header defines the core data structures, enumerations, macros, and public APIs
 * for a table-driven AT command handler, specifically designed for WAPI (Wireless LAN 
 * Authentication and Privacy Infrastructure) modules. The handler implements a hardware-
 * agnostic architecture with UART operation callbacks, dynamic AT command lookup, 
 * and parameter validation. It supports flexible AT command transmission with variadic 
 * arguments and provides standardized status codes for error handling.
 * 
 * Key features:
 * - Table-driven AT command management (decouples command definitions from logic)
 * - Hardware-independent UART interface (supports different MCU/module UART implementations)
 * - Variadic argument support for AT commands with automatic NULL termination
 * - Standardized error codes for robust error handling
 */

#ifndef __AT_HANDLER_H__
#define __AT_HANDLER_H__

#include <stdint.h>
#include "uart_proto.h"

#include "SEGGER_RTT.h"
extern int SEGGER_RTT_printf(unsigned BufferIndex, const char *sFormat, ...);
#define AT_DEBUG_OUT(fmt, ...)      SEGGER_RTT_printf(0, fmt "\r\n", ##__VA_ARGS__)  /* Output log to RTT buffer 0 */
#define AT_DEBUG_ERR(fmt, ...)      SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_RED fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)
#define AT_DEBUG_STRING(p_data, len)  \
    do {                                  \
        SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW);  /* Add red color control code at the beginning */ \
        SEGGER_RTT_Write(0, (const char*)p_data, len);        /* Print raw string/buffer */ \
        SEGGER_RTT_WriteString(0, RTT_CTRL_RESET "\r\n");     /* Reset format and add newline at the end */ \
    } while(0)


/**
 * @def AT_CMD_LEN_MAX
 * @brief Maximum length of AT command buffer (in bytes)
 * 
 * Defines the maximum size of the buffer used to format AT commands, 
 * including variable parameters and termination characters (\r\n).
 */
#define AT_CMD_LEN_MAX                  128

#define AT_TIMEOUT_TICK                 500
#define TRANSPARANT_TIMEOUT_TICK        2000

/**
 * @def AT_CMD_END_MARKER
 * @brief Magic value to mark end of AT command variadic arguments
 *
 * Prevents numeric 0 from being misinterpreted as a termination signal.
 * Must be appended to the end of variadic arguments (auto-added by AT_CMD_SEND macro).
 */
#define AT_CMD_END_MARKER           ((void*)0xDEADBEEF)

#define MAX_RECV_CNT_OF_ONCE_SEND   1

/**
 * @enum at_status_t
 * @brief Standardized AT command operation status codes
 * 
 * Defines all possible return values for AT handler APIs to ensure consistent 
 * error handling across the application.
 */
typedef enum
{
    AT_OK = 0,                /* Operation completed successfully */
    AT_ERR_PARAM_INVALID,     /* Invalid input parameter(s) (e.g., NULL pointer, wrong arg count) */
    AT_ERR_ALREADY_INITED,
    AT_ERR_HANDLER_NOT_READY, /* AT handler not initialized (UART/command table unready) */
    AT_ERR_NOT_CONSUMED,
    AT_ERR_CMD_NOT_FOUND,     /* Specified AT function ID not found in command table */
    AT_ERR_RECV_NOT_MATCH,
    AT_ERR_OTHERS             /* Unspecified error (e.g., UART transmission failure) */
} at_status_t;

typedef at_status_t (*pf_at_recv_parse_t)(uint8_t *buf, uint16_t len, void *arg, void *holder);

/* ---------------- OSAL interface for AT handler (semaphore + timer) ---------------- */
typedef struct
{
    int32_t (*pf_sema_binary_create)(void **p_sema_handle);
    void    (*pf_sema_delete)(void *sema_handle);
    int32_t (*pf_sema_give)(void *sema_handle);
    int32_t (*pf_sema_take)(void *sema_handle, uint32_t timeout);

    int32_t (*pf_timer_create)(void **p_timer_handle, const char *timer_name, uint32_t timer_period,
                               uint8_t auto_reload, void (*timer_cb)(void *timer_handle, void *arg), void *arg);
    int32_t (*pf_timer_start)(void *timer_handle, uint32_t ticks_to_wait);
    int32_t (*pf_timer_stop)(void *timer_handle, uint32_t ticks_to_wait);
    int32_t (*pf_timer_delete)(void *timer_handle, uint32_t ticks_to_wait);

} at_os_interface_t;
/**
 * @struct at_cmd_set_t
 * @brief AT command table entry structure
 * 
 * Represents a single entry in the AT command lookup table, mapping an AT function ID
 * to its command string template and expected response string.
 */
typedef struct
{
    uint8_t     at_func;      /* Unique ID for the AT command function (e.g., TEST=0, WAPI_CONN=2) */
    char        *send;        /* AT command string template (supports %s placeholders for variables) */    
    uint8_t     receive_count;       /* must <= MAX_RECV_CNT_OF_ONCE_SEND */
    pf_at_recv_parse_t pf_at_recv_parse[MAX_RECV_CNT_OF_ONCE_SEND];
    void        *arg;         /* User context passed to parse algo callbacks */
}at_cmd_set_t;

/**
 * @struct at_cmd_set_table_t
 * @brief AT Command Table Container Structure
 * 
 * This structure serves as a unified container for managing the AT command lookup table,
 * encapsulating both the array of AT command entries and the total count of entries.
 * It provides a standardized way to reference and traverse the command table, enabling
 * dynamic lookup of AT command templates by function ID and simplifying table maintenance
 * (e.g., adding/removing commands, validating table integrity).
 * 
 * Designed to be module-agnostic (compatible with WAPI M0804 and other WAPI/AT modules),
 * it decouples the command table definition from the handler logic for flexible integration.
 */
typedef struct
{
    const at_cmd_set_t  *table;     /* Pointer to the AT command table array (each element is an at_cmd_set_t entry 
                                     defining a unique AT command function, template, and expected response) */
    uint8_t             table_len; 
    void                *holder;                                
} at_cmd_set_table_t;

/**
 * @struct at_input_arg_t
 * @brief AT handler initialization input arguments
 * 
 * Aggregates all required initialization parameters for the AT handler, including
 * UART operation callbacks and AT command table configuration.
 */
typedef struct
{
    /* parse_algo uses built-in function, so inject NULL */
    uart_proto_input_arg_t  *uart_proto_input_arg; /* UART protocol layer input arguments */
    at_cmd_set_table_t      *at_cmd_set_table;   /* Pointer to AT command table container */    
    at_os_interface_t       *at_os_interface;    /* OSAL for semaphore/timer */
} at_input_arg_t;

/**
 * @struct at_priv_data_t
 * @brief Forward declaration of AT handler private data
 * 
 * Private data structure (implementation-specific) for the AT handler (defined in .c file).
 * Hides internal state (e.g., command buffer, UART status) from the public interface.
 */
typedef struct at_priv_data at_priv_data_t;

/**
 * @struct at_handler
 * @brief AT handler core structure (object-oriented design)
 * 
 * Main AT handler instance structure implementing an object-oriented pattern.
 * Contains initialization arguments, private data, and function pointers for AT operations.
 */
typedef struct at_handler at_handler_t;
typedef struct at_handler
{
    at_input_arg_t  *at_input_arg;  /* Pointer to initialization input arguments (public config) */
    at_priv_data_t  *at_priv_data;  /* Pointer to private handler data (internal state) */
    /**
     * @brief AT command send function pointer (variadic arguments)
     * 
     * Sends an AT command to the WAPI module with variable parameters, validates input,
     * formats the command string, and transmits via UART.
     * @param self      Pointer to at_handler_t instance
     * @param at_func   AT command function ID (matches command table entry)
     * @param ...       Variable arguments (matches %s placeholders in command template)
     * @note Final argument must be AT_CMD_END_MARKER (automatically added by AT_CMD_SEND macro)
     * @return at_status_t Operation status code
     */
    // at_status_t (*pf_at_cmd_send)(at_handler_t *const self, uint8_t at_func, ...);
} at_handler_t;

/**
 * @brief AT handler initialization function
 * 
 * Initializes the AT handler instance with input arguments, sets up UART hardware,
 * validates command table, and binds function pointers. Must be called before using
 * any other AT handler APIs.
 * 
 * @param self          Pointer to at_handler_t instance (must be pre-allocated)
 * @param p_input_args  Pointer to initialization input arguments (UART/command table config)
 * @return at_status_t  Operation status: AT_OK on success, error code on failure
 */
at_status_t at_inst(at_handler_t *const self,
                                    at_input_arg_t *const p_input_args);

/* use AT_CMD_SEND without adding AT_CMD_END_MARKER manually */                                    
at_status_t at_cmd_send_impl(at_handler_t *const self, uint8_t at_func, ...);  

#define AT_CMD_SEND(self, at_func, ...)  \
    at_cmd_send_impl((self), (at_func), ##__VA_ARGS__, AT_CMD_END_MARKER)   
    
/* transparant send with receive callback */
at_status_t at_trans_send(at_handler_t *const self, uint8_t *const data, uint16_t len, pf_at_recv_parse_t pf_at_recv_parse, void *arg, void *holder); 

/* call in IDLE ISR */
void at_notify_recv_isr_cb(at_handler_t *const self);
/* call in tx complete ISR */
void at_send_complete_isr_cb(at_handler_t *const self);
/* call in UART/DMA error ISR */
void at_error_recv_isr_cb(at_handler_t *const self);
void at_reset_send_state(at_handler_t *const self);

#endif //__AT_HANDLER_H__
