/**
 * @file uart_proto.c
 * @brief UART Protocol Layer Implementation
 * @version 1.02
 * @date 2025-12-30
 * @author
 *   Donzel
 * Implements initialization, frame parsing, ISR notifications, and
 * callback subscription management for the UART protocol abstraction layer.
 *
 * The layer runs asynchronously in a dedicated task and uses a message queue
 * to deliver parsed frames from ISR context. It works with function-code,
 * transparent, or dual-mode configurations.
 */

#include "uart_proto.h"
#include <stdbool.h>
#include <stdlib.h>

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
#include "t_list.h"
#endif

/* -------------------------------------------------------------------------- */
/*                           Convenience Macros                               */
/* -------------------------------------------------------------------------- */
#include "FreeRTOS.h"
#define MALLOC(size) pvPortMalloc(size) /* FreeRTOS memory allocation */
#define FREE(ptr)           \
    do                      \
    {                       \
        if (ptr)            \
        {                   \
            vPortFree(ptr); \
            ptr = NULL;     \
        }                   \
    } while (0) /* Safe memory free */

#define OS_INTERFACE(p)         (p)->uart_proto_input_arg->os_interface
#define UART_INTERFACE(p)       (p)->uart_proto_input_arg->uart_ops
#define PARSE_INTERFACE(p)      (p)->uart_proto_input_arg->frame_parse_att
#define PRIV_DATA(p)            (p)->uart_proto_priv_data

#if (CUSTOM_RX_THREAD_ATT)
#define THREAD_ATT(p)       (p)->uart_proto_input_arg->thread_att
#define PARSE_THREAD_ATT(p) (p)->uart_proto_input_arg->thread_att->parse_thread_att
#endif

#if (CUSTOM_UART_PROTO_CONFIG)
#define UART_CFG(p)     (p)->uart_proto_input_arg->uart_proto_config
#endif

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE)
#define FUNCODE_ALGO(p) PARSE_INTERFACE(p)->parse_algo->u.funcoude_algo.pf_parse_funcode
#elif (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_TRANSPARENT)
#define TRANS_ALGO(p)   PARSE_INTERFACE(p)->parse_algo->u.transparent_algo.pf_transparent_parse
#define TRANS_ARG(p)    PARSE_INTERFACE(p)->parse_algo->u.transparent_algo.arg
#elif (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
#define PARSE_ALGO(p)   PARSE_INTERFACE(p)->parse_algo
#define ALGO_TYPE(p)    PARSE_INTERFACE(p)->parse_algo->algo_type
#define FUNCODE_ALGO(p) PARSE_INTERFACE(p)->parse_algo->u.funcoude_algo.pf_parse_funcode
#define TRANS_ALGO(p)   PARSE_INTERFACE(p)->parse_algo->u.transparent_algo.pf_transparent_parse
#define TRANS_ARG(p)    PARSE_INTERFACE(p)->parse_algo->u.transparent_algo.arg
#endif

#define RECV_BUF(p)         PARSE_INTERFACE(p)->recv_buf_att->recv_buf
#define RECV_BUF_SIZE(p)    PARSE_INTERFACE(p)->recv_buf_att->buffer_size

#define NON_COPY_WHEN_NON_WRAP

/* -------------------------------------------------------------------------- */
/*                         Internal Private Structures                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Private runtime data structure
 */
typedef struct uart_proto_priv_data
{
    bool is_inited;
    uint8_t num_notify_isr_cb_call;
    volatile uint16_t parse_fail_count;
    void *queue_handle;
    volatile uint16_t data_counter;
    volatile uint32_t header;
    volatile uint32_t tail;
    uint8_t *parse_buf;
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    t_list_t funcode_sentinel;
#endif
} uart_proto_priv_data_t;

/**
 * @brief Frame information passed between ISR and parsing task
 */
typedef struct
{
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    uint8_t fun_code;
#endif
    uint8_t *payload;
    uint16_t payload_length;
} parse_info_t;

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)

/**
 * @brief Function-code callback node in linked list
 */
typedef struct
{
    t_list_t list;
    uint8_t fun_code;
    void *arg;
    pf_fun_code_cb_t cb;
} funcode_node_t;

/* -------------------------------------------------------------------------- */
/*                         Function Code Helper APIs                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Allocate and initialize a function-code node
 */
static funcode_node_t *funcode_node_init(uart_proto_t *const self,
                                         subscribe_para_t *para)
{
    if (!self || !para)
        return NULL;

    funcode_node_t *node = MALLOC(sizeof(funcode_node_t));
    if (!node)
        return NULL;

    t_list_init(&node->list);
    node->arg = para->arg;
    node->fun_code = para->fun_code;
    node->cb = para->cb;
    return node;
}

/**
 * @brief Subscribe a callback for a function code
 */
static uart_proto_status_t subscribe_funcode(uart_proto_t *const self,
                                             subscribe_para_t *para,
                                             void **handle)
{
    if (!self || !para)
        return UART_PROTO_ERR_PARAM_INVALID;
    if (!PRIV_DATA(self)->is_inited)
        return UART_PROTO_ERR_HANDLER_NOT_READY;

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    if (ALGO_FUNCODE != ALGO_TYPE(self))
        return UART_PROTO_ERR_ALGO_TYPE;
#endif

    funcode_node_t *fn = funcode_node_init(self, para);
    if (!fn)
        return UART_PROTO_ERR_OTHERS;

    /* Insert node into sorted linked list by function code */
    OS_INTERFACE(self)->pf_os_enter_critical();
    t_list_t *head = &PRIV_DATA(self)->funcode_sentinel;
    t_list_t *current = head;
    while (current->next != head)
    {
        funcode_node_t *next = T_LIST_ENTRY(current->next, funcode_node_t, list);
        if (next->fun_code > para->fun_code)
            break;
        current = current->next;
    }
    t_list_insert_after(current, &fn->list);
    OS_INTERFACE(self)->pf_os_exit_critical(0);

    if (handle)
        *handle = fn;

    return UART_PROTO_OK;
}

/**
 * @brief Unsubscribe callback by handle
 */
static uart_proto_status_t unsubscribe_funcode(uart_proto_t *const self,
                                               void *handle)
{
    if (!self || !handle)
        return UART_PROTO_ERR_PARAM_INVALID;
    if (!PRIV_DATA(self)->is_inited)
        return UART_PROTO_ERR_HANDLER_NOT_READY;

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    if (ALGO_FUNCODE != ALGO_TYPE(self))
        return UART_PROTO_ERR_ALGO_TYPE;
#endif

    /* Remove node safely */
    OS_INTERFACE(self)->pf_os_enter_critical();
    funcode_node_t *node = (funcode_node_t *)handle;
    t_list_remove(&node->list);
    OS_INTERFACE(self)->pf_os_exit_critical(0);

    FREE(handle);
    return UART_PROTO_OK;
}

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
/**
 * @brief Change algorithm type dynamically (dual mode only)
 */
static uart_proto_status_t algo_strategy(uart_proto_t *const self,
                                         parse_algo_t *algo)
{
    if (!self || !algo)
        return UART_PROTO_ERR_PARAM_INVALID;
    if (!PRIV_DATA(self)->is_inited)
        return UART_PROTO_ERR_HANDLER_NOT_READY;

    PARSE_ALGO(self) = algo;
    return UART_PROTO_OK;
}
#endif
#endif /* function-code mode or dual mode */

/* -------------------------------------------------------------------------- */
/*                          Frame Parsing Logic                               */
/* -------------------------------------------------------------------------- */

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
/**
 * @brief Parse frames using function-code algorithm
 */
static uint16_t parse_function_code_mode(uart_proto_t *const self,
                                         uint8_t *addr,
                                         uint16_t length)
{
    uint16_t bytes_used = 0;
    frame_info_t info;
    algo_status_t status;

    /* Loop until all valid frames are parsed or error occurs */
    while ((status = FUNCODE_ALGO(self)(addr + bytes_used, length - bytes_used, &info)) == ALGO_OK ||
           status == ALGO_ERR_CRC || status == ALGO_ERR_LENGTH_INVALID ||
           status == ALGO_ERR_NOICE)
    {
        /* Prepare payload information for thread delivery */
        parse_info_t pi = {
            .fun_code = info.fun_code,
            .payload = addr + bytes_used + info.pre_payload_length,
            .payload_length = info.payload_length
        };

        /* Update parsed length including header/trailer */
        bytes_used += info.pre_payload_length + info.payload_length + info.post_payload_length;
        UP_DEBUG_OUT("Parsed frame length=%u", bytes_used);

        /* Queue successfully parsed frame to RTOS queue */
        if (status == ALGO_OK)
        {
            PRIV_DATA(self)->parse_fail_count = 0;
            OS_INTERFACE(self)->pf_os_queue_put(PRIV_DATA(self)->queue_handle, &pi, 0);
        }
    }

    /* On hard parser error, discard remaining data */
    if (status == ALGO_ERR_OTHERS)
        bytes_used = length;

    return bytes_used;
}

/**
 * @brief Deliver parsed frame to matching function-code subscribers
 */
static void handle_function_code_parse(uart_proto_t *const self,
                                       parse_info_t *info)
{
    t_list_t *head = &PRIV_DATA(self)->funcode_sentinel;

    for (t_list_t *current = head->next; current != head; current = current->next)
    {
        funcode_node_t *node = T_LIST_ENTRY(current, funcode_node_t, list);

        if (node->fun_code > info->fun_code)
            break;

        if (node->fun_code == info->fun_code && node->cb)
            node->cb(node->arg, info->payload, info->payload_length);
    }
}
#endif

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_TRANSPARENT || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
/**
 * @brief Parse received data in transparent mode
 */
static uint16_t parse_transparent_mode(uart_proto_t *const self,
                                       uint8_t *addr,
                                       uint16_t length)
{
    parse_info_t info = { .payload = addr, .payload_length = length };

    PRIV_DATA(self)->parse_fail_count = 0;
    /* Push data directly to queue */
    OS_INTERFACE(self)->pf_os_queue_put(PRIV_DATA(self)->queue_handle, &info, 0);
    return length;
}

/**
 * @brief Call registered transparent data callback
 */
static void handle_transparent_parse(uart_proto_t *const self,
                                     parse_info_t *pi)
{
    TRANS_ALGO(self)(pi->payload, pi->payload_length, TRANS_ARG(self));
}
#endif

/* -------------------------------------------------------------------------- */
/*                            Parsing Thread Task                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Background thread that processes parsed frames
 */
static void parse_thread(void *arg)
{
    uart_proto_t *self = (uart_proto_t *)arg;
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;

    while (1)
    {
        parse_info_t info;
        /* Wait for new parsed frame from ISR queue */
        OS_INTERFACE(self)->pf_os_queue_get(PRIV_DATA(self)->queue_handle, &info, OS_DELAY_MAX);
        UP_DEBUG_OUT("Frame parsed, dispatching callbacks, len=%d", info.payload_length);

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE)
        handle_function_code_parse(self, &info);
#elif (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_TRANSPARENT)
        handle_transparent_parse(self, &info);
#elif (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
        if (ALGO_FUNCODE == ALGO_TYPE(self))
            handle_function_code_parse(self, &info);
        else
            handle_transparent_parse(self, &info);
#endif
    }
}

/* -------------------------------------------------------------------------- */
/*                            Public API Functions                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the UART protocol instance
 */
uart_proto_status_t uart_proto_inst(uart_proto_t *const self,
                                    uart_proto_input_arg_t *const args)
{
    /* --- Parameter validation --- */
    if (!self || !args || !args->frame_parse_att ||
        !args->os_interface || !args->uart_ops)
        return UART_PROTO_ERR_PARAM_INVALID;

    if(self->uart_proto_priv_data && self->uart_proto_priv_data->is_inited)
    {
        return UART_PROTO_ERR_ALREADY_INITED;
    }     

    self->uart_proto_input_arg = args;

    if (!UART_INTERFACE(self)->pf_get_counter || !UART_INTERFACE(self)->pf_uart_init ||
        !UART_INTERFACE(self)->pf_uart_deinit || !UART_INTERFACE(self)->pf_uart_write)
        return UART_PROTO_ERR_PARAM_INVALID;

    if (!PARSE_INTERFACE(self)->parse_algo || !PARSE_INTERFACE(self)->recv_buf_att)
        return UART_PROTO_ERR_PARAM_INVALID;
    
    if(!RECV_BUF(self) || !RECV_BUF_SIZE(self))
        return UART_PROTO_ERR_PARAM_INVALID;
    
    if (!OS_INTERFACE(self)->pf_os_thread_create ||
        !OS_INTERFACE(self)->pf_os_thread_delete ||
        !OS_INTERFACE(self)->pf_os_queue_create ||
        !OS_INTERFACE(self)->pf_os_queue_put ||
        !OS_INTERFACE(self)->pf_os_queue_get ||
        !OS_INTERFACE(self)->pf_os_enter_critical ||
        !OS_INTERFACE(self)->pf_os_exit_critical)
        return UART_PROTO_ERR_PARAM_INVALID;     

    /* --- Resource allocation --- */
    self->uart_proto_priv_data = MALLOC(sizeof(uart_proto_priv_data_t));
    if (!PRIV_DATA(self))
        return UART_PROTO_ERR_OTHERS;

    PRIV_DATA(self)->is_inited = false;
    PRIV_DATA(self)->parse_fail_count = 0;
    PRIV_DATA(self)->header = PRIV_DATA(self)->tail = PRIV_DATA(self)->data_counter = 0;

    PRIV_DATA(self)->parse_buf = MALLOC(RECV_BUF_SIZE(self));
    if (!PRIV_DATA(self)->parse_buf)
        return UART_PROTO_ERR_OTHERS;

    /* --- Initialize hardware --- */
    UART_INTERFACE(self)->pf_uart_init();

    /* --- Create queue for parsed frames --- */
    uint32_t item_num = MAX_PARSE_NUM_ONCE_TRIGGER;
    PRIV_DATA(self)->num_notify_isr_cb_call = NUM_NOTIFY_ISR_CB_CALL;
#if (CUSTOM_UART_PROTO_CONFIG)
    if (UART_CFG(self))
    {
        item_num = UART_CFG(self)->max_parse_num_once_trigger;
        PRIV_DATA(self)->num_notify_isr_cb_call = UART_CFG(self)->num_notify_isr_cb_call;
    }
#endif
    OS_INTERFACE(self)->pf_os_queue_create(item_num, sizeof(parse_info_t),
                                    &PRIV_DATA(self)->queue_handle);

    /* --- Create background parsing thread --- */
    uint32_t stack_size = PARSE_THREAD_STACK_DEPTH;
    uint32_t priority = PARSE_THREAD_PRIORITY;
#if (CUSTOM_RX_THREAD_ATT)
    if (THREAD_ATT(self))
    {
        stack_size = PARSE_THREAD_ATT(self).stack_depth;
        priority = PARSE_THREAD_ATT(self).thread_priority;
    }
#endif
    OS_INTERFACE(self)->pf_os_thread_create("parse_thread",
                                     parse_thread,
                                     stack_size,
                                     priority,
                                     NULL,
                                     self);

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    t_list_init(&PRIV_DATA(self)->funcode_sentinel);
    self->pf_subscribe = subscribe_funcode;
    self->pf_unsubscribe = unsubscribe_funcode;
#endif
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    self->pf_algo_strategy = algo_strategy;
#endif

    PRIV_DATA(self)->is_inited = true;
    UP_DEBUG_OUT("UART proto instance initialized successfully");
    return UART_PROTO_OK;
}

/**
 * @brief Reset UART internal counters (e.g., after error)
 */
void reset_rx_state(uart_proto_t *const self)
{
    /* --- Sanity check --- */
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;
    PRIV_DATA(self)->parse_fail_count = PRIV_DATA(self)->header =
        PRIV_DATA(self)->tail = PRIV_DATA(self)->data_counter = 0;
    UART_INTERFACE(self)->pf_set_counter(RECV_BUF_SIZE(self));
}

/**
 * @brief UART receive ISR callback
 *
 * Performs ring-buffer index update, handles wrap cases,
 * and parses data using the configured mode.
 */
void notify_isr_cb(uart_proto_t *const self)
{
    UP_TRACE_ISR_ENTER();

    /* --- Sanity check --- */
    if (!self || !PRIV_DATA(self) || !PRIV_DATA(self)->is_inited)
        return;

    /* --- Lock shared state --- */
    uint32_t primask = OS_INTERFACE(self)->pf_os_enter_critical();

    PRIV_DATA(self)->parse_fail_count++;

    /* --- Calculate DMA ring-buffer data range --- */
    uint16_t previous_index = PRIV_DATA(self)->tail % RECV_BUF_SIZE(self);
    uint16_t current_index = RECV_BUF_SIZE(self) - UART_INTERFACE(self)->pf_get_counter();
    uint16_t length = (current_index - previous_index + RECV_BUF_SIZE(self)) % RECV_BUF_SIZE(self);

    /* Update header counter to detect overflow */
    PRIV_DATA(self)->header += (current_index - PRIV_DATA(self)->data_counter + RECV_BUF_SIZE(self)) %
                          RECV_BUF_SIZE(self);

    /* Exit if no new data */
    if (PRIV_DATA(self)->header == PRIV_DATA(self)->tail)
    {
        OS_INTERFACE(self)->pf_os_exit_critical(primask);
        UP_TRACE_ISR_EXTI();
        return;
    }

    /* --- Overflow protection --- */
    if (PRIV_DATA(self)->header - PRIV_DATA(self)->tail >= RECV_BUF_SIZE(self))
    {
        UP_DEBUG_ERR("RX buffer overflow detected, resetting state");
        reset_rx_state(self);
        OS_INTERFACE(self)->pf_os_exit_critical(primask);
        UP_TRACE_ISR_EXTI();
        return;
    }

    PRIV_DATA(self)->data_counter = current_index;

#ifdef NON_COPY_WHEN_NON_WRAP
    bool wrapped = false;
#endif

    /* --- Linearize ring-buffer data --- */
    if (length > 0)
    {
        if (previous_index <= current_index || current_index == 0)
        {
#ifndef NON_COPY_WHEN_NON_WRAP            
            /* Case 1: No buffer wrap-around - copy entire pending data in one segment */
            memcpy(PRIV_DATA(self)->parse_buf,
                RECV_BUF(self) + previous_index,
                length);
#endif
        }
        else
        {
            /* Case 2: Buffer wrap-around - copy in two segments to cover full pending data */
            uint16_t length_part1 = RECV_BUF_SIZE(self) - previous_index;
            uint16_t length_part2 = current_index;
            memcpy(PRIV_DATA(self)->parse_buf, RECV_BUF(self) + previous_index, length_part1);
            memcpy(PRIV_DATA(self)->parse_buf + length_part1, RECV_BUF(self), length_part2);
#ifdef NON_COPY_WHEN_NON_WRAP
            wrapped = true;
#endif
        }
    }

#ifdef NON_COPY_WHEN_NON_WRAP
    uint8_t *parse_addr = wrapped ? PRIV_DATA(self)->parse_buf : (RECV_BUF(self) + previous_index);
#else
    uint8_t *parse_addr = PRIV_DATA(self)->parse_buf;
#endif

    /* --- Call parsing function --- */
    uint16_t bytes_used = 0;
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE)
    bytes_used = parse_function_code_mode(self, parse_addr, length);
#elif (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_TRANSPARENT)
    bytes_used = parse_transparent_mode(self, parse_addr, length);
#elif (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    if (ALGO_FUNCODE == ALGO_TYPE(self))
        bytes_used = parse_function_code_mode(self, parse_addr, length);
    else
        bytes_used = parse_transparent_mode(self, parse_addr, length);
#endif

    PRIV_DATA(self)->tail += bytes_used;

    /* --- Error handling --- */
    if (PRIV_DATA(self)->parse_fail_count >= PRIV_DATA(self)->num_notify_isr_cb_call)
    {
        UP_DEBUG_ERR("Parse failure threshold exceeded, resetting state");
        reset_rx_state(self);
    }

    /* --- Release lock --- */
    OS_INTERFACE(self)->pf_os_exit_critical(primask);
    UP_TRACE_ISR_EXTI();
}
