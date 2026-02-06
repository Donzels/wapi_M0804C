#ifndef __WAPI_COMMU_H__
#define __WAPI_COMMU_H__

#include "WAPI_M0804C.h"
#include "osal.h"
#include "upper_commu.h"



extern m0804c_handler_t g_wapi_handler_inst;

#include "SEGGER_RTT.h"
extern int SEGGER_RTT_printf(unsigned BufferIndex, const char *sFormat, ...);
#define WAPI_COMMU_DEBUG_OUT(fmt, ...)      SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)  /* Output log to RTT buffer 0 */
#define WAPI_COMMU_DEBUG_ERR(fmt, ...)      SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_RED fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)
#define WAPI_COMMU_DEBUG_STRING(p_data, len)  \
    do {                                  \
        SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED);  /* Add red color control code at the beginning */ \
        SEGGER_RTT_Write(0, (const char*)p_data, len);        /* Print raw string/buffer */ \
        SEGGER_RTT_WriteString(0, RTT_CTRL_RESET "\r\n");     /* Reset format and add newline at the end */ \
    } while(0)

#define WAPI_COMMU_PARSE_THREAD_STACK_DEPTH        2048
#define WAPI_COMMU_PARSE_THREAD_PRIORITY           24 /* WAPI_COMMU_PARSE_THREAD_PRIORITY must higher than UPP_COMMU_PARSE_THREAD_PRIORITY */
    
void wapi_commu_init(void);   

#endif /* __WAPI_COMMU_H__ */
