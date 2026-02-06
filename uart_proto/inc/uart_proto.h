/**
 * @file uart_proto.h
 * @brief UART Protocol Layer Header
 * @version 1.02
 * @date 2025-12-30
 * @author
 *   Donzel
 * This header defines data types, macros, and APIs for the UART protocol layer.
 * The design supports:
 *   - Function-code-based parsing mode
 *   - Transparent (raw data) mode
 *   - Dual-mode runtime switching (function-code and transparent mode)
 *
 * The layer abstracts hardware and OS dependencies through generic interfaces,
 * enabling portability across different platforms.
 *
 * @par Version History
 * - V1.00 (2025-11-30): Initial version - basis
 * - V1.01 (2025-12-13): Added transparent mode and dual-mode support
 * - V1.02 (2025-12-30):
 *      - Switched to unified enum-style configuration (UART_PROTO_MODE_DEFAULT)
 *      - Updated OS interface function contracts
 *      - Updated `parse_algo_t` to unified layout (always includes union `u`)
 *        to improve structural consistency and forward compatibility
 */

#ifndef __UART_PROTO_H__
#define __UART_PROTO_H__

#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/*                            Mode Configuration                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief UART protocol working modes
 */
#define UART_PROTO_MODE_DUAL_STRATEGY   (0) /**< Dual-mode, dynamic switch */
#define UART_PROTO_MODE_FUNCTION_CODE   (1) /**< Function code parsing mode */
#define UART_PROTO_MODE_TRANSPARENT     (2) /**< Raw transparent transmission */

/**
 * @brief Select the active compilation mode
 *
 * Only one mode must be selected for UART_PROTO_MODE_DEFAULT.
 */
#define UART_PROTO_MODE_DEFAULT         UART_PROTO_MODE_DUAL_STRATEGY

#if (UART_PROTO_MODE_DEFAULT != UART_PROTO_MODE_DUAL_STRATEGY && \
     UART_PROTO_MODE_DEFAULT != UART_PROTO_MODE_FUNCTION_CODE && \
     UART_PROTO_MODE_DEFAULT != UART_PROTO_MODE_TRANSPARENT)
#error "Invalid UART_PROTO_MODE_DEFAULT value. Must be one of UART_PROTO_MODE_xxx."
#endif

/* Optional feature controls */
#define CUSTOM_RX_THREAD_ATT            1  /**< Enable custom thread attributes */
#define CUSTOM_UART_PROTO_CONFIG        0  /**< Enable custom UART protocol config */

/* -------------------------------------------------------------------------- */
/*                           Core Configuration                               */
/* -------------------------------------------------------------------------- */

/** @defgroup UART_PROTO_CONFIG Configuration Parameters
 *  @brief Basic compile-time settings
 *  @{
 */
#define NUM_NOTIFY_ISR_CB_CALL          1           /**< ISR callback retries */
#define MAX_PARSE_NUM_ONCE_TRIGGER      1           /**< Max frames parsed per ISR callback */
#define PARSE_THREAD_PRIORITY           25          /**< Parsing thread priority */
#define PARSE_THREAD_STACK_DEPTH        2048       /**< Parsing thread stack size */
#define OS_DELAY_MAX                    0xFFFFFFFF  /**< Max OS delay */
/** @} */

/* -------------------------------------------------------------------------- */
/*                                   Debug                                    */
/* -------------------------------------------------------------------------- */
#include "SEGGER_RTT.h"
extern int SEGGER_RTT_printf(unsigned BufferIndex, const char *sFormat, ...);
#define UP_DEBUG_OUT(fmt, ...)      SEGGER_RTT_printf(0, fmt "\r\n", ##__VA_ARGS__)  /* Output log to RTT buffer 0 */
#define UP_DEBUG_ERR(fmt, ...)      SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_RED fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)


#define UP_TRACE_ISR_ENTER()
#define UP_TRACE_ISR_EXTI()

/* -------------------------------------------------------------------------- */
/*                              Status Codes                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Status returned by UART protocol operations
 */
typedef enum
{
    UART_PROTO_OK = 0,                    /**< Operation successful */
    UART_PROTO_ERR_PARAM_INVALID,         /**< Invalid input parameter */
    UART_PROTO_ERR_ALREADY_INITED, 
    UART_PROTO_ERR_HANDLER_NOT_READY,     /**< Protocol not initialized */
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    UART_PROTO_ERR_ALGO_TYPE,             /**< Invalid algorithm type in dual mode */
#endif
    UART_PROTO_ERR_OTHERS                 /**< Generic failure */
} uart_proto_status_t;

/* -------------------------------------------------------------------------- */
/*                           Parser return codes                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parser status returned by frame parsing function
 */
typedef enum
{
    ALGO_ING = 1,                 /**< Parsing incomplete (need more data) */
    ALGO_OK = 0,                  /**< Frame parsed successfully */
    ALGO_ERR_LENGTH_INVALID = -1, /**< Invalid frame length */
    ALGO_ERR_CRC = -2,            /**< CRC check failed */
    ALGO_ERR_NOICE = -3,          /**< Noise or invalid data detected */
    ALGO_ERR_OTHERS = -4          /**< Other parsing error */
} algo_status_t;

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
/**
 * @brief Algorithm type for dual-mode parsing
 */
typedef enum
{
    ALGO_FUNCODE = 0,      /**< Function code parser */
    ALGO_TRANSPARENT        /**< Transparent parser */
} algo_type_t;
#endif

/* -------------------------------------------------------------------------- */
/*                             Data Structures                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief UART receive buffer information
 */
typedef struct
{
    uint8_t *recv_buf;       /**< Pointer to receive buffer memory */
    uint16_t buffer_size;    /**< Buffer size in bytes */
} recv_buf_att_t;

#if (CUSTOM_RX_THREAD_ATT)
/**
 * @brief Thread attribute information
 */
typedef struct
{
    const uint32_t stack_depth; /**< Thread stack size */
    uint32_t thread_priority;   /**< Thread priority */
} thread_att_t;

/**
 * @brief RX thread configuration container
 */
typedef struct
{
    thread_att_t parse_thread_att; /**< Parsing thread attributes */
} rx_thread_att_t;
#endif

#if (CUSTOM_UART_PROTO_CONFIG)
/**
 * @brief UART protocol behavior configuration
 */
typedef struct
{
    uint8_t num_notify_isr_cb_call;       /**< ISR callback retry count */
    uint8_t max_parse_num_once_trigger;   /**< Max frames processed per ISR trigger */
} uart_proto_config_t;
#endif

/* -------------------------------------------------------------------------- */
/*                    Frame fields and parser binding                         */
/* -------------------------------------------------------------------------- */

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
/**
 * @brief Basic frame metadata used by parser in function-code mode
 */
typedef struct
{
    uint8_t fun_code;           /**< Function code */
    uint16_t pre_payload_length;  /**< Header length in bytes */
    uint16_t post_payload_length; /**< Trailer length in bytes */
    uint16_t payload_length;      /**< Payload length in bytes */
} frame_info_t;

/**
 * @brief Function-code parsing function type
 */
typedef algo_status_t (*pf_parse_funcode_t)(uint8_t *const p_data,
                                            uint16_t data_len,
                                            frame_info_t *const frame_info);

/**
 * @brief Function-code parsing algorithm container
 */
typedef struct
{
    pf_parse_funcode_t pf_parse_funcode; /**< Parser callback */
} funcoude_algo_t;
#endif

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_TRANSPARENT || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
/**
 * @brief Transparent parsing algorithm (raw data transfer)
 */
typedef struct
{
    void *arg; /**< Custom context argument */
    void (*pf_transparent_parse)(uint8_t *const p_data, uint16_t data_len, void *arg);
} transparent_algo_t;
#endif

/**
 * @brief Unified parse algorithm interface
 */
typedef struct
{
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    algo_type_t algo_type;   /**< Current algorithm type (dual mode only) */
#endif

    union
    {
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
        funcoude_algo_t funcoude_algo;        /**< Function-code parsing handler */
#endif
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_TRANSPARENT || UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
        transparent_algo_t transparent_algo;  /**< Transparent parsing handler */
#endif
    } u;
} parse_algo_t;

/**
 * @brief Frame parsing attribute settings
 */
typedef struct
{
    recv_buf_att_t  *recv_buf_att; /**< Receive buffer info */
    parse_algo_t    *parse_algo;     /**< Linked parse algorithm */
} frame_parse_att_t;

/* -------------------------------------------------------------------------- */
/*                            UART HW interface                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief UART hardware abstraction callbacks
 */
typedef struct
{
    void (*pf_uart_init)(void);                           /**< Init hardware */
    void (*pf_uart_deinit)(void);                         /**< Deinit hardware */
    void (*pf_uart_write)(uint8_t *const data, uint16_t len); /**< Send UART data */
    uint16_t (*pf_get_counter)(void);                     /**< Get DMA remaining counter */
    void (*pf_set_counter)(uint16_t counter);             /**< Set DMA remaining counter */
} uart_ops_t;

/* -------------------------------------------------------------------------- */
/*                          OS abstraction layer                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief OS-level interface for threading and queues
 */
typedef struct
{
    int32_t (*pf_os_thread_create)(const char *name,
                                   void (*task)(void *),
                                   size_t stack_size,
                                   uint32_t priority,
                                   void **handle,
                                   void *arg);
    void (*pf_os_thread_delete)(void *const handle);
    int32_t (*pf_os_queue_create)(size_t num, size_t size, void **handle);
    int32_t (*pf_os_queue_put)(void *queue, const void *item, uint32_t timeout);
    int32_t (*pf_os_queue_get)(void *queue, const void *item, uint32_t timeout);
    uint32_t (*pf_os_enter_critical)(void);
    void (*pf_os_exit_critical)(uint32_t primask);
} uart_rx_os_interface_t;

/* -------------------------------------------------------------------------- */
/*                         Initialization arguments                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief UART protocol initialization arguments
 */
typedef struct
{
    frame_parse_att_t *frame_parse_att;  /**< Frame parsing configuration */
    uart_ops_t *uart_ops;                /**< Hardware interface */
    uart_rx_os_interface_t *os_interface;/**< OS interface */
#if (CUSTOM_RX_THREAD_ATT)
    rx_thread_att_t *thread_att;         /**< Optional thread attributes (NULL -> use default) */
#endif
#if (CUSTOM_UART_PROTO_CONFIG)
    uart_proto_config_t *uart_proto_config; /**< Optional protocol config (NULL -> use default) */
#endif
} uart_proto_input_arg_t;

#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
/**
 * @brief Function-code callback signature
 */
typedef void (*pf_fun_code_cb_t)(void *arg, uint8_t *const payload, uint16_t payload_length);

/**
 * @brief Function-code subscription information
 */
typedef struct
{
    uint8_t fun_code;     /**< Subscribed function code */
    void *arg;            /**< User context pointer */
    pf_fun_code_cb_t cb;  /**< Callback handler */
} subscribe_para_t;
#endif

/** Private forward declaration */
typedef struct uart_proto_priv_data uart_proto_priv_data_t;

/* -------------------------------------------------------------------------- */
/*                             Main UART handle                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Main UART protocol handle
 */
typedef struct uart_proto
{
    uart_proto_input_arg_t *uart_proto_input_arg;
    uart_proto_priv_data_t *uart_proto_priv_data;
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_FUNCTION_CODE || \
     UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    uart_proto_status_t (*pf_subscribe)(struct uart_proto *const self,
                                        subscribe_para_t *const para,
                                        void **const handle);
    uart_proto_status_t (*pf_unsubscribe)(struct uart_proto *const self,
                                          void *const handle);
#endif
#if (UART_PROTO_MODE_DEFAULT == UART_PROTO_MODE_DUAL_STRATEGY)
    uart_proto_status_t (*pf_algo_strategy)(struct uart_proto *const self,
                                            parse_algo_t *const algo);
#endif
} uart_proto_t;

/* -------------------------------------------------------------------------- */
/*                               Public APIs                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize UART protocol layer
 * @param self     UART protocol handle
 * @param args     Initialization arguments
 * @retval UART_PROTO_OK  Success
 * @retval UART_PROTO_ERR_xxx  Failure status
 */
uart_proto_status_t uart_proto_inst(uart_proto_t *const self,
                                    uart_proto_input_arg_t *const args);

/**
 * @brief UART RX ISR callback
 * @param self UART protocol handle
 *
 * Should be invoked in UART or DMA interrupt handlers
 * to notify protocol of received data.
 */
void notify_isr_cb(uart_proto_t *const self);

/**
 * @brief Reset UART internal state (e.g. on DMA/overflow error)
 */
void reset_rx_state(uart_proto_t *const self);

#endif /* __UART_PROTO_H__ */
