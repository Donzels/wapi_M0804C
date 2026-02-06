/*
 ******************************************************************************
 * File Name          : AT_handler.c
 * Description        : AT Command Handler Implementation for WAPI Module
 *                      Implements table-driven AT command management, variadic
 *                      argument validation, UART transmission, and initialization
 *                      logic with FreeRTOS memory management integration.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "AT_handler.h"
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* Private Macros ------------------------------------------------------------*/
#include <stdlib.h>
#define MALLOC(size)        malloc(size)  
#define FREE(ptr)           free(ptr)

#define PRIV_DATA(p) (p)->at_priv_data                /* Private internal data */
#define UART_INTERFACE(p) (p)->at_input_arg->uart_proto_input_arg->uart_ops /* UART hardware interface */
#define UP_OS_IF(p) (p)->at_input_arg->uart_proto_input_arg->os_interface       /* Reuse UART proto OS iface for queue/thread */
#define OS_IF(p) (p)->at_input_arg->at_os_interface               /* AT handler OSAL interface */

#define ACQUIRE_SEND_FEEDBACK_SEMA(self, timeout) \
            OS_IF(self)->pf_sema_take(PRIV_DATA(self)->send_feedback_sema_handle, timeout) \

#define RELEASE_SEND_FEEDBACK_SEMA(self) \
    do { \
            OS_IF(self)->pf_sema_give(PRIV_DATA(self)->send_feedback_sema_handle); \
            AT_DEBUG_OUT("line=%d: Released send feedback semaphore", __LINE__); \
    } while(0)

#define TIMER_START(self, timeout_ms) \
    do { \
        int32_t ret = OS_IF(self)->pf_timer_start(PRIV_DATA(self)->timeout_timer, timeout_ms); \
        if (0 == ret) { \
            AT_DEBUG_OUT("line=%d: Timer started with timeout %d ms", __LINE__, timeout_ms); \
        } else { \
            AT_DEBUG_ERR("line=%d: Failed to start timer (ret=%d)", __LINE__, ret); \
        } \
    } while(0)    

#define TIMER_STOP(self) \
    do { \
        int32_t ret = OS_IF(self)->pf_timer_stop(PRIV_DATA(self)->timeout_timer, 0); \
        if (0 == ret) { \
            AT_DEBUG_OUT("line=%d: Timer stopped", __LINE__); \
        } else { \
            AT_DEBUG_ERR("line=%d: Failed to stop timer (ret=%d)", __LINE__, ret); \
        } \
    } while(0)    

/* Private Type Definitions -------------------------------------------------*/

typedef enum
{
    SEND_CMD = 0,
    SEND_TRANSPARENT
}at_send_type_t;

typedef struct
{
    const at_cmd_set_t *cmd_entry;
}cmd_event_t;

typedef struct
{
    at_trans_callback_t callback;
    /* Future extension fields can be added here (e.g., retry_count, priority) */
}transparent_event_t;


typedef struct
{
    at_send_type_t at_send_type;
    union
    {
        cmd_event_t cmd_event;
        transparent_event_t transparent_event;
    }u;
} send_info_t;


typedef struct at_priv_data
{
    bool is_inited; /* Initialization flag: true = handler ready, false = uninitialized */
    uint8_t remain_receive_count;
    send_info_t send_info;
    uart_proto_t *uart_proto_handle;
    void *send_feedback_sema_handle;
    void *send_queue_handle;
    void *timeout_timer;  
    uint8_t send_buf[AT_SEND_LEN_MAX];     
} at_priv_data_t;

typedef struct
{
    uint8_t *payload;     /* Pointer to the frame payload data */
    uint16_t payload_len; /* Length of the payload data (in bytes) */
} parse_info_t;


/* Private Function Implementations ------------------------------------------*/

/**
 * @brief Count the number of %s/%d placeholders in a string
 *
 * Parses the AT command template string to count the number of %s/%d format specifiers,
 * which determines the expected number of variadic arguments for the command.
 * Skips consecutive characters to avoid double-counting (e.g., "%%s" is ignored).
 *
 * @param str Pointer to the AT command template string (NULL = return 0)
 * @return uint8_t Number of %s/%d placeholders found
 */
static uint8_t count_placeholder(const char *str)
{
    uint8_t cnt = 0;
    if (str == NULL)
        return 0;

    while (*str != '\0')
    {
        /* Check for %s or %d placeholder pattern */
        if (*str == '%' && ((*(str + 1) == 's') || (*(str + 1) == 'd')))
        {
            cnt++;
            str++; /* Skip 's'/'d' to avoid re-counting the same placeholder */
        }
        str++;
    }
    return cnt;
}

/**
 * @brief Count valid variadic arguments (terminated by 0xDEADBEEF)
 *
 * Iterates through the variadic argument list to count valid parameters, stopping
 * when the magic end marker (0xDEADBEEF) is encountered. Supports both pointer (char*)
 * and numeric (int) arguments (cast to void* for counting).
 *
 * @param max_cnt Maximum expected arguments (from placeholder count)
 * @param args    va_list pointer to variadic argument list
 * @return uint8_t Actual number of valid arguments found (before end marker)
 * @note Numeric arguments (int) are cast to void*; 0 is treated as valid parameter
 */
static uint8_t count_va_args(uint8_t max_cnt, va_list args)
{
    uint8_t cnt = 0;
    void* arg;

    /* Iterate until end marker or max count (prevent overflow) */
    while (cnt < max_cnt + 1)
    {
        arg = va_arg(args, void*);
        /* Terminate if end marker is found */
        if (arg == AT_CMD_END_MARKER)
            break;
        cnt++;
    }
    return cnt;
}

at_status_t at_cmd_send_impl(at_handler_t *const self, uint32_t at_func, ...)
{
    va_list args;
    const at_cmd_set_t *cmd_entry = NULL;         /* Pointer to matched command table entry */
    volatile uint8_t expected_param_count = 0;         /* Expected args (from placeholder count) */
    volatile uint8_t actual_param_count = 0;           /* Actual args (from variadic list) */

    /* Basic input validation */
    if (!self)
        return AT_ERR_PARAM_INVALID;
    if(!PRIV_DATA(self)->is_inited)
        return AT_ERR_HANDLER_NOT_READY;
            
    /* Look up AT command template in command table */
    at_cmd_set_table_t *cmd_table = self->at_input_arg->at_cmd_set_table;
    uint8_t table_length = self->at_input_arg->at_cmd_set_table->table_len;
    for (uint8_t i = 0; i < table_length; i++)
    {
        if (cmd_table->table[i].at_func == at_func)
        {
            cmd_entry = &cmd_table->table[i];
            break;
        }
    }
    if (cmd_entry == NULL)  /* Command ID not found in table */
    {
        return AT_ERR_CMD_NOT_FOUND;
    }
    /* Get expected parameter count from command template placeholders */
    expected_param_count = count_placeholder(cmd_entry->send);

    /* Validate variadic argument count matches expected count */
    va_start(args, at_func);
    actual_param_count = count_va_args(expected_param_count, args);
    va_end(args);

    if (actual_param_count != expected_param_count)
    {
        AT_DEBUG_ERR("AT command parameter count mismatch: expected=%u, actual=%u", expected_param_count, actual_param_count);
        return AT_ERR_PARAM_INVALID;  /* Mismatch between expected/actual arguments */
    }
    /* Format AT command string with variadic arguments */
    va_start(args, at_func);

    int send_len = vsnprintf((char*)PRIV_DATA(self)->send_buf, AT_SEND_LEN_MAX, cmd_entry->send, args);
    va_end(args);
    
    /* Check for formatting errors or buffer overflow */
    if (send_len < 0 || send_len >= (AT_SEND_LEN_MAX))
    {
        return AT_ERR_OTHERS;
    } 

    if(0 != ACQUIRE_SEND_FEEDBACK_SEMA(self, 0))
    {
        AT_DEBUG_ERR("Previous AT command not consumed, send feedback semaphore unavailable");
        return AT_ERR_NOT_CONSUMED;
    }      
        
    PRIV_DATA(self)->send_info.at_send_type = SEND_CMD;
    PRIV_DATA(self)->send_info.u.cmd_event.cmd_entry = cmd_entry;
    PRIV_DATA(self)->remain_receive_count = cmd_entry->receive_count;
    AT_DEBUG_OUT("Send remaining receive count: %u", PRIV_DATA(self)->remain_receive_count);

    /* Transmit formatted command via UART (hardware-agnostic callback) */
    UART_INTERFACE(self)->pf_uart_write(PRIV_DATA(self)->send_buf, strlen((char*)PRIV_DATA(self)->send_buf));

    TIMER_START(self, AT_TIMEOUT_TICK);

    return AT_OK;
}

at_status_t at_trans_send(at_handler_t *const self, uint8_t *const data, uint16_t len, 
                          const at_trans_callback_t *callback)
{
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return AT_ERR_HANDLER_NOT_READY;

    if(0 != ACQUIRE_SEND_FEEDBACK_SEMA(self, 0))
    {
        AT_DEBUG_ERR("Previous transparent data not consumed, send feedback semaphore unavailable");
        return AT_ERR_NOT_CONSUMED;
    }
    
    /* Send with response - setup transparent event */
    if(callback && callback->pf_at_recv_parse[0])
    {
        uint8_t recv_count = callback->receive_count ? callback->receive_count : 1;
        if (recv_count > MAX_RECV_CNT_OF_TRANS_SEND)
        {
            RELEASE_SEND_FEEDBACK_SEMA(self);
            AT_DEBUG_ERR("Transparent receive_count overflow: %u > max=%u", recv_count, MAX_RECV_CNT_OF_TRANS_SEND);
            return AT_ERR_PARAM_INVALID;
        }
        
        /* Validate all callback function pointers up to recv_count */
        for (uint8_t i = 0; i < recv_count; i++)
        {
            if (!callback->pf_at_recv_parse[i])
            {
                RELEASE_SEND_FEEDBACK_SEMA(self);
                AT_DEBUG_ERR("Transparent callback is NULL at index %u", i);
                return AT_ERR_PARAM_INVALID;
            }
        }
        
        PRIV_DATA(self)->send_info.at_send_type = SEND_TRANSPARENT;
        PRIV_DATA(self)->send_info.u.transparent_event.callback = *callback;
        PRIV_DATA(self)->remain_receive_count = recv_count;
    }
    
    /* Prepare data transmission */
#if IS_ENABLE_SEND_BUF_PROTECTED
    memcpy(PRIV_DATA(self)->send_buf, data, len);
    UART_INTERFACE(self)->pf_uart_write(PRIV_DATA(self)->send_buf, len);
#else
    UART_INTERFACE(self)->pf_uart_write(data, len);
#endif

    /* Send without response */
    if(!callback || !callback->pf_at_recv_parse[0])
    {
        RELEASE_SEND_FEEDBACK_SEMA(self);
        return AT_OK;
    }   

    TIMER_START(self, TRANSPARANT_TIMEOUT_TICK);    
    return AT_OK;
}

/**
 * @brief Core AT command send implementation (variadic arguments)
 *
 * Validates input parameters, looks up the AT command template from the command table,
 * validates variadic argument count against placeholders, formats the command string,
 * appends standard AT termination characters (\r\n), and transmits via UART.
 * Supports both %s (string) and %d (integer) placeholders, with numeric 0 as valid input.
 *
 * @param self     Pointer to AT handler instance
 * @param at_func  AT command function ID (matches command table entry)
 * @param ...      Variadic arguments matching %s/%d placeholders + end marker
 * @return at_status_t Operation status:
 *         - AT_OK: Command sent successfully
 *         - AT_ERR_PARAM_INVALID: Invalid input/argument count mismatch
 *         - AT_ERR_HANDLER_NOT_READY: Handler not initialized
 *         - AT_ERR_CMD_NOT_FOUND: at_func not present in command table
 *         - AT_ERR_OTHERS: Command formatting/transmission failure
 */

static void at_parse_algo(uint8_t *const p_data, uint16_t data_len, void *arg)
{
    at_handler_t *const self = (at_handler_t *)arg;
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;
    send_info_t send_info;
    AT_DEBUG_STRING(p_data, data_len);
    if (0 != UP_OS_IF(self)->pf_os_queue_get(PRIV_DATA(self)->send_queue_handle, &send_info, 0))
    {
        AT_DEBUG_ERR("Received data but no corresponding send info in queue");  
        return;  
    }

    uint32_t timeout_tick = AT_TIMEOUT_TICK;
    if(SEND_CMD == send_info.at_send_type)    
    {
        const at_cmd_set_t *cmd_entry = send_info.u.cmd_event.cmd_entry;
        uint8_t parse_algo_index = cmd_entry->receive_count - PRIV_DATA(self)->remain_receive_count;
        if(parse_algo_index > cmd_entry->receive_count)
        {
            AT_DEBUG_ERR("Invalid parse algorithm index: %u > max_count=%u", parse_algo_index, cmd_entry->receive_count);
            return;
        }            
        if(cmd_entry->pf_at_recv_parse[parse_algo_index])
            cmd_entry->pf_at_recv_parse[parse_algo_index](p_data, data_len,\
                                 cmd_entry->arg, self->at_input_arg->at_cmd_set_table->holder);
        else
            AT_DEBUG_ERR("AT command parse callback is NULL at index %u", parse_algo_index);                
    }
    else if(SEND_TRANSPARENT == send_info.at_send_type)
    {
        timeout_tick = TRANSPARANT_TIMEOUT_TICK;
        uint8_t parse_algo_index = send_info.u.transparent_event.callback.receive_count - PRIV_DATA(self)->remain_receive_count;
        if(parse_algo_index >= send_info.u.transparent_event.callback.receive_count)
        {
            AT_DEBUG_ERR("Invalid transparent parse algorithm index: %u >= max_count=%u", parse_algo_index, send_info.u.transparent_event.callback.receive_count);
            return;
        }
        if(send_info.u.transparent_event.callback.pf_at_recv_parse[parse_algo_index])
            send_info.u.transparent_event.callback.pf_at_recv_parse[parse_algo_index](p_data, data_len,\
                             send_info.u.transparent_event.callback.arg, send_info.u.transparent_event.callback.holder);
        else
            AT_DEBUG_ERR("Transparent parse callback is NULL at index %u", parse_algo_index);
    }
    if (PRIV_DATA(self)->remain_receive_count > 0)
        PRIV_DATA(self)->remain_receive_count --;
    AT_DEBUG_OUT("Recv remaining receive count: %u, len=%u", PRIV_DATA(self)->remain_receive_count, data_len);
    if ((0 == PRIV_DATA(self)->remain_receive_count) ||
        ((SEND_CMD == send_info.at_send_type) && 
         (PRIV_DATA(self)->remain_receive_count > MAX_RECV_CNT_OF_CMD_SEND)) ||
        ((SEND_TRANSPARENT == send_info.at_send_type) && 
         (PRIV_DATA(self)->remain_receive_count > MAX_RECV_CNT_OF_TRANS_SEND)))
    {
        TIMER_STOP(self);
        RELEASE_SEND_FEEDBACK_SEMA(self);
        AT_DEBUG_OUT("AT command/transparent data reception completed" );
    }
    else
        /**
         * If there are remaining receive counts,
         * restart the timeout timer to receive next data.
         */
    {
        if (0 != UP_OS_IF(self)->pf_os_queue_put(PRIV_DATA(self)->send_queue_handle, &send_info, 0))
        {
            AT_DEBUG_ERR("Failed to re-queue send info for next receive");
            TIMER_STOP(self);
            RELEASE_SEND_FEEDBACK_SEMA(self);
            return;
        }
        TIMER_START(self, timeout_tick);
    }
}

static void timeout_callback(void *timer_handle, void *arg)
{
    (void)timer_handle;
    at_handler_t *const self = (at_handler_t *)arg;

    send_info_t send_info;
    if (0 != UP_OS_IF(self)->pf_os_queue_get(PRIV_DATA(self)->send_queue_handle, &send_info, 0))
    {
        AT_DEBUG_ERR("Timeout: received response but failed to get send info from queue");
    }
        
    AT_DEBUG_ERR("AT response reception timeout");
    RELEASE_SEND_FEEDBACK_SEMA(self);
}

/* Public Function Implementations -------------------------------------------*/
/**
 * @brief AT handler initialization function
 *
 * Performs comprehensive input validation, allocates private runtime data,
 * initializes UART hardware via callback, binds the command send function pointer,
 * and sets the initialization flag to mark the handler as ready for use.
 * Uses FreeRTOS memory management for thread-safe private data allocation.
 *
 * @param self          Pointer to AT handler instance (must be pre-allocated)
 * @param p_input_args  Pointer to initialization arguments (UART ops + command table)
 * @return at_status_t Operation status:
 *         - AT_OK: Initialization successful
 *         - AT_ERR_PARAM_INVALID: Invalid/NULL input parameter(s)
 *         - AT_ERR_OTHERS: Memory allocation failure
 */
at_status_t at_inst(at_handler_t *const self, at_input_arg_t *const p_input_args)
{
    /* Comprehensive input validation (prevent NULL pointer dereferencing) */
    if (!self || !p_input_args || !p_input_args->uart_proto_input_arg ||
        !p_input_args->at_cmd_set_table || !p_input_args->at_cmd_set_table->table ||
        !p_input_args->at_cmd_set_table->table_len ||
        !p_input_args->at_os_interface ||
        !p_input_args->at_os_interface->pf_sema_binary_create ||
        !p_input_args->at_os_interface->pf_sema_delete ||
        !p_input_args->at_os_interface->pf_sema_give ||
        !p_input_args->at_os_interface->pf_sema_take ||
        !p_input_args->at_os_interface->pf_timer_create ||
        !p_input_args->at_os_interface->pf_timer_start ||
        !p_input_args->at_os_interface->pf_timer_stop ||
        !p_input_args->at_os_interface->pf_timer_delete)
    {
        return AT_ERR_PARAM_INVALID;
    }
    if(PRIV_DATA(self) && PRIV_DATA(self)->is_inited)
    {
        return AT_ERR_ALREADY_INITED;
    }   
    /* check table validity */
    uint8_t table_length = p_input_args->at_cmd_set_table->table_len;
    const at_cmd_set_t  *table = p_input_args->at_cmd_set_table->table;
    for(uint8_t i=0; i<table_length; i++)
    {
        if(table[i].receive_count > MAX_RECV_CNT_OF_CMD_SEND || table[i].receive_count < 1)
            return AT_ERR_PARAM_INVALID;
        for(uint8_t j=0; j<table[i].receive_count; j++)
        {
            if(!table[i].pf_at_recv_parse[j])
                return AT_ERR_PARAM_INVALID;    
        }
    }

    /* Bind initialization arguments to handler instance */
    self->at_input_arg = p_input_args;

    /* Allocate private runtime data (FreeRTOS thread-safe malloc) */
    PRIV_DATA(self) = MALLOC(sizeof(at_priv_data_t));
    if (!PRIV_DATA(self)) /* Handle malloc failure */
    {
        return AT_ERR_OTHERS;
    }
    PRIV_DATA(self)->is_inited = false; /* Mark as uninitialized during setup */
    
    parse_algo_t *algo = self->at_input_arg->uart_proto_input_arg->frame_parse_att->parse_algo;
    if (algo)
    {
        AT_DEBUG_OUT("Using built-in AT handler parse algorithm");
    }
    algo = (parse_algo_t *)MALLOC(sizeof(parse_algo_t));
    if (!algo)
    {
        FREE(PRIV_DATA(self));
        return AT_ERR_OTHERS;
    }
    self->at_input_arg->uart_proto_input_arg->frame_parse_att->parse_algo = algo;

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    algo->algo_type = ALGO_TRANSPARENT;
#endif
    algo->u.transparent_algo.arg = self;
    algo->u.transparent_algo.pf_transparent_parse = at_parse_algo;

    /* Allocate and instantiate UART protocol handle */
    PRIV_DATA(self)->uart_proto_handle = (uart_proto_t *)MALLOC(sizeof(uart_proto_t));
    if (!PRIV_DATA(self)->uart_proto_handle)
    {
        FREE(PRIV_DATA(self));
        FREE(algo);
        return AT_ERR_OTHERS;
    }

    uart_proto_status_t uart_proto_status = uart_proto_inst(PRIV_DATA(self)->uart_proto_handle,
                                                            self->at_input_arg->uart_proto_input_arg);
    if (UART_PROTO_OK != uart_proto_status)
    {        
        FREE(PRIV_DATA(self));
        FREE(algo);
        FREE(PRIV_DATA(self)->uart_proto_handle);
        return AT_ERR_OTHERS;
    }

    OS_IF(self)->pf_sema_binary_create(&PRIV_DATA(self)->send_feedback_sema_handle);
    UP_OS_IF(self)->pf_os_queue_create(1, sizeof(send_info_t), &PRIV_DATA(self)->send_queue_handle);
    OS_IF(self)->pf_timer_create(&PRIV_DATA(self)->timeout_timer, "at_timeout", AT_TIMEOUT_TICK,\
                                 0, timeout_callback, self);
    RELEASE_SEND_FEEDBACK_SEMA(self);

    /* Mark handler as initialized and ready for use */
    PRIV_DATA(self)->is_inited = true;

    AT_DEBUG_OUT("AT handler initialized successfully");
    return AT_OK;
}

void at_notify_recv_isr_cb(at_handler_t *const self)
{
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;
    notify_isr_cb(PRIV_DATA(self)->uart_proto_handle);
}

void at_error_recv_isr_cb(at_handler_t *const self)
{
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;
    reset_rx_state(PRIV_DATA(self)->uart_proto_handle);
}

void at_send_complete_isr_cb(at_handler_t *const self)
{
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;

    UP_OS_IF(self)->pf_os_queue_put(PRIV_DATA(self)->send_queue_handle, &PRIV_DATA(self)->send_info, 0);
}

void at_reset_send_state(at_handler_t *const self)
{
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;
    RELEASE_SEND_FEEDBACK_SEMA(self);
    reset_rx_state(PRIV_DATA(self)->uart_proto_handle);
    AT_DEBUG_OUT("AT handler send state reset");
}

