/**************************************************************************//**
 * @file      AC_defs_utils.h
 * @brief     General utilities and constants
 * @details   
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/
#ifndef _AC_DEFS_UTILS_H_
#define _AC_DEFS_UTILS_H_

/*****************************************************************************/
/*****************************************************************************/
/*                                UTILS DEFS                                 */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
// To avoid gcc/g++ warnings
/*****************************************************************************/
#define UNUSED(X) (void)X   

/*****************************************************************************/
/* Busy wait with Timeout and custom error action                            */
/*****************************************************************************/
#define BUSY_WAIT_TIMEOUT_ERROR_ACTION(busy_cond, timeout, errAction)   \
{                                                                       \
    uint32_t __time = timeout;                                            \
                                                                        \
    do                                                                  \
    {                                                                   \
        if (__time-- == 0)                                              \
        {                                                               \
            errAction;                                                  \
        }                                                               \
    } while (busy_cond);                                                \
}

/*****************************************************************************/
/* Busy wait with Timeout and custom return error                            */
/*****************************************************************************/
#define BUSY_WAIT_TIMEOUT_ERROR(busy_cond, timeout, err)    BUSY_WAIT_TIMEOUT_ERROR_ACTION(busy_cond, timeout, return err)

/*****************************************************************************/
/* Busy Wait with Timeout                                                    */
/*****************************************************************************/
#define BUSY_WAIT_TIMEOUT(busy_cond, timeout)               BUSY_WAIT_TIMEOUT_ERROR(busy_cond, timeout, AC_RESPONSE_TIMEOUT)

/*****************************************************************************/
/* Busy Wait without Timeout                                                 */
/*****************************************************************************/
#define BUSY_WAIT(busy_cond)                                            \
{                                                                       \
    do                                                                  \
    {                                                                   \
    } while (busy_cond);                                                \
}

/*****************************************************************************/
/* Empty itteration delay loop                                               */
/*****************************************************************************/
#define DELAY_LOOP(count)                                               \
{                                                                       \
    volatile uint64_t __i;                                                  \
    for (__i=0; __i<count; ++__i) {}                                    \
}

/*****************************************************************************/
/*****************************************************************************/
/*                             REGISTERS ACCESS                              */
/*****************************************************************************/
/*****************************************************************************/
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#define AC_IS_BIT_SET(REG, BIT)    (((REG) & (BIT)) == (BIT))
#define AC_IS_BIT_CLR(REG, BIT)    (((REG) & (BIT)) == 0U)


/*****************************************************************************/
/*****************************************************************************/
/*                           MISCELLANEOUS ACCESS                            */
/*****************************************************************************/
/*****************************************************************************/
typedef enum
{
    AC_REG_RESET = 0,
    AC_REG_SET = !AC_REG_RESET
} AC_reg_status_t;

/*****************************************************************************/
// FLASH_Latency FLASH Latency
/*****************************************************************************/
#define FLASH_LATENCY_0           FLASH_ACR_LATENCY_0WS     /*!< FLASH Zero wait state */
#define FLASH_LATENCY_1           FLASH_ACR_LATENCY_1WS     /*!< FLASH One wait state */
#define FLASH_LATENCY_2           FLASH_ACR_LATENCY_2WS     /*!< FLASH Two wait states */
#define FLASH_LATENCY_3           FLASH_ACR_LATENCY_3WS     /*!< FLASH Three wait states */
#define FLASH_LATENCY_4           FLASH_ACR_LATENCY_4WS     /*!< FLASH Four wait states */

#endif // _AC_DEFS_UTILS_H_
