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
// Max delay can be used in LL_mDelay
/*****************************************************************************/
#define AC_MAX_DELAY                  0xFFFFFFFFU

/*****************************************************************************/
// To avoid gcc/g++ warnings
/*****************************************************************************/
#define UNUSED(X) (void)X   

/*****************************************************************************/
/*****************************************************************************/
/*                                MACRO UTILS                                */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
// calculate maximum value
/*****************************************************************************/
#define MAX(a, b)               ((a)>(b) ? (a) : (b))

/*****************************************************************************/
/* calculate minimum value                                                   */
/*****************************************************************************/
#define MIN(a, b)               ((a)<(b) ? (a) : (b))

/*****************************************************************************/
/* calculate absolute value                                                  */
/*****************************************************************************/
#define ABS(a)                  ((a)>(0) ? (a) : -(a))

/*****************************************************************************/
/* calculate modulus                                                         */
/*****************************************************************************/
#define MOD(a,b)                ((a) - ((a)/(b))*(b))

/*****************************************************************************/
/* calculate integer division with rounding                                  */
/*****************************************************************************/
#define DIV_ROUND(a, b)         (((a) + ((b)/2)) / (b))

/*****************************************************************************/
/* calculate integer division with rounding up                               */
/*****************************************************************************/
#define DIV_CEILING(a, b)       (((a) + ((b)-1)) / (b))

/*****************************************************************************/
/* calculate average of two integers                                         */
/*****************************************************************************/
#define AVG(a,b)                DIV_ROUND(((a)+(b)),2)

/*****************************************************************************/
/* calculate square                                                          */
/*****************************************************************************/
#define SQR(a)                  ((a)*(a))

/*****************************************************************************/
/* Determine if number is even                                               */
/*****************************************************************************/
#define IS_EVEN(a)              ((a)%2 == 0)

/*****************************************************************************/
/* Determine if number is odd                                                */
/*****************************************************************************/
#define IS_ODD(a)               (!IS_EVEN(a))

/*****************************************************************************/
/* Calculate size of statically declared array                               */
/*****************************************************************************/
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)           (sizeof(a) / sizeof((a)[0]))
#endif

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
/* In Range                                                                  */
/*****************************************************************************/
#define IN_RANGE(x, range_start, range_size)            (((x) >= (range_start)) && ((x) < ((range_start)+(range_size))))


/*****************************************************************************/
/* Cyclic Counter Increment/Decriment with limit                             */
/*****************************************************************************/
#define CYCLIC_COUNTER_INC_VAL(x, y, limit)             ((((x) + (y)) >= (limit)) ? (((x) + (y)) - (limit)) : ((x) + (y)))
#define CYCLIC_COUNTER_INC(x, limit)                    CYCLIC_COUNTER_INC_VAL(x, 1, limit)

#define CYCLIC_COUNTER_DEC_VAL(x, y, limit)             (((x) >= (y)) ? ((x) - (y)) : (((x) + (limit)) - (y)))
#define CYCLIC_COUNTER_DEC(x, limit)                    CYCLIC_COUNTER_DEC_VAL(x, 1, limit)


/*****************************************************************************/
/*****************************************************************************/
/*                            MEASUREMENT UNITS                              */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/* Capacity (basic unit : byte)                                              */
/*****************************************************************************/
#define   _1B_            1UL
#define   _2B_            (   2 * _1B_   )
#define   _4B_            (   2 * _2B_   )
#define   _8B_            (   2 * _4B_   )
#define  _16B_            (   2 * _8B_   )
#define  _32B_            (   2 * _16B_  )
#define  _64B_            (   2 * _32B_  )
#define _128B_            (   2 * _64B_  )
#define _256B_            (   2 * _128B_ )
#define _512B_            (   2 * _256B_ )

#define   _1KB_           (1024 * _1B_    )
#define   _2KB_           (   2 * _1KB_   )
#define   _4KB_           (   2 * _2KB_   )
#define   _8KB_           (   2 * _4KB_   )
#define  _16KB_           (   2 * _8KB_   )
#define  _32KB_           (   2 * _16KB_  )
#define  _64KB_           (   2 * _32KB_  )
#define _128KB_           (   2 * _64KB_  )
#define _256KB_           (   2 * _128KB_ )
#define _512KB_           (   2 * _256KB_ )

#define   _1MB_           (1024 * _1KB_   )
#define   _2MB_           (   2 * _1MB_   )
#define   _4MB_           (   2 * _2MB_   )
#define   _8MB_           (   2 * _4MB_   )
#define  _16MB_           (   2 * _8MB_   )
#define  _32MB_           (   2 * _16MB_  )
#define  _64MB_           (   2 * _32MB_  )
#define _128MB_           (   2 * _64MB_  )
#define _256MB_           (   2 * _128MB_ )
#define _512MB_           (   2 * _256MB_ )

#define   _1GB_           (1024 * _1MB_   )
#define   _2GB_           (   2 * _1GB_   )
#define   _4GB_           (   2 * _2GB_   )
#define   _8GB_           (   2 * _4GB_   )
#define  _16GB_           (   2 * _8GB_   )
#define  _32GB_           (   2 * _16GB_  )
#define  _64GB_           (   2 * _32GB_  )
#define _128GB_           (   2 * _64GB_  )
#define _256GB_           (   2 * _128GB_ )
#define _512GB_           (   2 * _256GB_ )


/*****************************************************************************/
/* Frequency (basic unit : hertz)                                            */
/*****************************************************************************/
#define _1Hz_           1UL
#define _1KHz_          (1000 * _1Hz_ )
#define _1MHz_          (1000 * _1KHz_)
#define _1GHz_          (1000 * _1MHz_)

/*****************************************************************************/
/* Time                                                                      */
/*****************************************************************************/
#define _1USEC_IN_NSEC_     1000UL

#define _1MSEC_IN_USEC_     1000UL
#define _1MSEC_IN_NSEC_     ((_1MSEC_IN_USEC_) * (_1USEC_IN_NSEC_))

#define _1SEC_IN_MSEC_      1000UL
#define _1SEC_IN_USEC_      ((_1SEC_IN_MSEC_) * (_1MSEC_IN_USEC_))
#define _1SEC_IN_NSEC_      ((_1SEC_IN_MSEC_) * (_1MSEC_IN_NSEC_))

#define _1MIN_IN_SEC_       60UL
#define _1MIN_IN_MSEC_      ((_1MIN_IN_SEC_) * (_1SEC_IN_MSEC_))
#define _1MIN_IN_USEC_      ((_1MIN_IN_SEC_) * (_1SEC_IN_USEC_))
#define _1MIN_IN_NSEC_      ((_1MIN_IN_SEC_) * (_1SEC_IN_NSEC_))

#define _1HOUR_IN_MIN_      60UL
#define _1HOUR_IN_SEC_      ((_1HOUR_IN_MIN_) * (_1MIN_IN_SEC_))
#define _1HOUR_IN_MSEC_     ((_1HOUR_IN_MIN_) * (_1MIN_IN_MSEC_))
#define _1HOUR_IN_USEC_     ((_1HOUR_IN_MIN_) * (_1MIN_IN_USEC_))
#define _1HOUR_IN_NSEC_     ((_1HOUR_IN_MIN_) * (_1MIN_IN_NSEC_))

#define _1DAY_IN_HOURS_     24UL
#define _1DAY_IN_MIN_       ((_1DAY_IN_HOURS_) * (_1HOUR_IN_MIN_))
#define _1DAY_IN_SEC_       ((_1DAY_IN_HOURS_) * (_1HOUR_IN_SEC_))
#define _1DAY_IN_MSEC_      ((_1DAY_IN_HOURS_) * (_1HOUR_IN_MSEC_))
#define _1DAY_IN_USEC_      ((_1DAY_IN_HOURS_) * (_1HOUR_IN_USEC_))
#define _1DAY_IN_NSEC_      ((_1DAY_IN_HOURS_) * (_1HOUR_IN_NSEC_))


/*****************************************************************************/
/* Structs' tricks                                                           */
/*****************************************************************************/
#ifndef OFFSET_OF
#define OFFSET_OF(type, member)                 ((UINT32)&((type *)0)->member)
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr,cont_type,member)      ((cont_type *)( (UINT32)((INT8 *)ptr) - OFFSET_OF( cont_type,member ) ) )
#endif

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
