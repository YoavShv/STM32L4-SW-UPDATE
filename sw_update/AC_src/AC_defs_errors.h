/**************************************************************************//**
 * @file      AC_defs_errors.h
 * @brief     Standard error codes set
 * @details   
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/
#ifndef _AC_DEFS_ERRORS_H_
#define _AC_DEFS_ERRORS_H_

/*****************************************************************************/
/*****************************************************************************/
/*                            ERROR CODE MASKING                             */
/*****************************************************************************/
/*****************************************************************************/
#ifndef AC_MASK
    #ifndef AC_OK_MASK
        #define AC_OK_MASK             0
    #endif
    #ifndef AC_ERROR_MASK
        #define AC_ERROR_MASK          0
    #endif

    #ifndef AC_SEC_ERROR_MASK
        #define AC_SEC_ERROR_MASK      AC_ERROR_MASK
    #endif
#else
    #define AC_OK_MASK                 AC_MASK
    #define AC_ERROR_MASK              AC_MASK
    #define AC_SEC_ERROR_MASK          AC_MASK
#endif


/*****************************************************************************/
/*****************************************************************************/
/*                                ERROR CODES                                */
/*****************************************************************************/
/*****************************************************************************/
typedef enum
{
    AC_OK                                  = AC_OK_MASK    | 0x00,
    AC_FAIL                                = AC_ERROR_MASK | 0x01,

    /*************************************************************************/
    /* Parameters validity                                                   */
    /*************************************************************************/
    AC_INVALID_PARAMETER                   = AC_ERROR_MASK | 0x10,

    AC_INVALID_DATA_SIZE                   = AC_ERROR_MASK | 0x11,
    AC_PARAMETER_OUT_OF_RANGE              = AC_ERROR_MASK | 0x12,
    AC_INVALID_DATA_FIELD                  = AC_ERROR_MASK | 0x13,
    AC_CORRUPTED_VALUE                     = AC_ERROR_MASK | 0x14,
    AC_INVALID_NUMBER_OF_PARAMETERS        = AC_ERROR_MASK | 0x15,
    AC_INVALID_DATA_ALIGNMENT              = AC_ERROR_MASK | 0x16,

    /*************************************************************************/
    /* Response                                                              */
    /*************************************************************************/
    AC_RESPONSE_CANT_BE_PROVIDED           = AC_ERROR_MASK | 0x20,

    AC_SYSTEM_BUSY                         = AC_ERROR_MASK | 0x21,
    AC_SYSTEM_NOT_INITIALIZED              = AC_ERROR_MASK | 0x22,
    AC_SYSTEM_IN_INCORRECT_STATE           = AC_ERROR_MASK | 0x23,
    AC_RESPONSE_TIMEOUT                    = AC_ERROR_MASK | 0x24,
    AC_RESPONSE_ABORT                      = AC_ERROR_MASK | 0x25,

    /*************************************************************************/
    /* Security                                                              */
    /*************************************************************************/
    AC_SECURITY_ERROR                      = AC_SEC_ERROR_MASK | 0x30,

    AC_INSUFFICIENT_PRIVILEGES_LEVEL       = AC_SEC_ERROR_MASK | 0x31,
    AC_AUTHENTICATION_FAIL                 = AC_SEC_ERROR_MASK | 0x32,
    AC_BAD_SIGNATURE                       = AC_SEC_ERROR_MASK | 0x33,
    AC_LOCKED                              = AC_SEC_ERROR_MASK | 0x34,
    AC_INTEGRITY_FAILED                    = AC_SEC_ERROR_MASK | 0x35,
    AC_CHECKSUM_FAILED                     = AC_SEC_ERROR_MASK | 0x36,
    AC_ILLEGAL_ACCESS                      = AC_SEC_ERROR_MASK | 0x37,
    AC_ILLEGAL_CONFIGURATION               = AC_SEC_ERROR_MASK | 0x38,
    AC_PERMANENT_LOCK                      = AC_SEC_ERROR_MASK | 0x39,
   

    /*************************************************************************/
    /* Communication                                                         */
    /*************************************************************************/
    AC_COMMUNICATION_ERROR                 = AC_ERROR_MASK | 0x40,

    AC_NO_CONNECTION                       = AC_ERROR_MASK | 0x41,
    AC_CANT_OPEN_CONNECTION                = AC_ERROR_MASK | 0x42,
    AC_CONNECTION_ALREADY_OPEN             = AC_ERROR_MASK | 0x43,
    AC_COMMAND_ILLEGAL                     = AC_ERROR_MASK | 0x44,
    AC_COMMAND_UNKNOWN                     = AC_ERROR_MASK | 0x45,
    AC_COMMAND_IGNORED                     = AC_ERROR_MASK | 0x46,
    AC_COMMAND_FAILED                      = AC_ERROR_MASK | 0x47,
    
    AC_NO_COMMAND                          = AC_ERROR_MASK | 0x48,
    AC_COMMAND_READY                       = AC_ERROR_MASK | 0x49,

    /*************************************************************************/
    /* Hardware                                                              */
    /*************************************************************************/
    AC_HARDWARE_ERROR                      = AC_ERROR_MASK | 0x50,

    AC_IO_ERROR                            = AC_ERROR_MASK | 0x51,
    AC_CLK_ERROR                           = AC_ERROR_MASK | 0x52,

    /*************************************************************************/
    /* Implementation                                                        */
    /*************************************************************************/
    AC_NOT_IMPLEMENTED                     = AC_ERROR_MASK | 0x60,

    AC_IMPLEMENTATION_ERROR                = AC_ERROR_MASK | 0x61,
    AC_DEPRECATED_FUNCTION                 = AC_ERROR_MASK | 0x62,
    AC_NOT_SUPPORTED                       = AC_ERROR_MASK | 0x63,

    /*************************************************************************/
    /* File                                                                  */
    /*************************************************************************/
    AC_FILE_CANT_OPEN                      = AC_ERROR_MASK | 0x70,
    AC_FILE_NOT_FOUND                      = AC_ERROR_MASK | 0x71,

    /*************************************************************************/
    /* Memory                                                                */
    /*************************************************************************/
    AC_OUT_OF_MEMORY                       = AC_ERROR_MASK | 0x80,
    AC_BUFFER_IS_FULL                      = AC_ERROR_MASK | 0x81,
    AC_BUFFER_IS_EMPTY                     = AC_ERROR_MASK | 0x82,
    AC_BUFFER_OVERFLOW                     = AC_ERROR_MASK | 0x83,

    /*************************************************************************/
    /* Custom error codes                                                    */
    /*************************************************************************/
    AC_CUSTOM_ERROR_00                      = AC_ERROR_MASK | 0xF0,
    AC_CUSTOM_ERROR_01                      = AC_ERROR_MASK | 0xF1,
    AC_CUSTOM_ERROR_02                      = AC_ERROR_MASK | 0xF2,
    AC_CUSTOM_ERROR_03                      = AC_ERROR_MASK | 0xF3,
    AC_CUSTOM_ERROR_04                      = AC_ERROR_MASK | 0xF4,
    AC_CUSTOM_ERROR_05                      = AC_ERROR_MASK | 0xF5,
    AC_CUSTOM_ERROR_06                      = AC_ERROR_MASK | 0xF6,
    AC_CUSTOM_ERROR_07                      = AC_ERROR_MASK | 0xF7,
    AC_CUSTOM_ERROR_08                      = AC_ERROR_MASK | 0xF8,
    AC_CUSTOM_ERROR_09                      = AC_ERROR_MASK | 0xF9,
    AC_CUSTOM_ERROR_10                      = AC_ERROR_MASK | 0xFA,
    AC_CUSTOM_ERROR_11                      = AC_ERROR_MASK | 0xFB,
    AC_CUSTOM_ERROR_12                      = AC_ERROR_MASK | 0xFC,
    AC_CUSTOM_ERROR_13                      = AC_ERROR_MASK | 0xFD,
    AC_CUSTOM_ERROR_14                      = AC_ERROR_MASK | 0xFE,
    AC_CUSTOM_ERROR_15                      = AC_ERROR_MASK | 0xFF
} AC_status_t;


/*****************************************************************************/
/*****************************************************************************/
/*                            ERROR CODE STRINGS                             */
/*****************************************************************************/
/*****************************************************************************/
#define AC_ERROR_STRINGS                                                                               \
{                                                                                                               \
    /*************************************************************************/   \
    /* General                                                               */   \
    /*************************************************************************/   \
    "No error",                                                                                                 \
    "General error",                                                                                            \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* Parameters                                                            */   \
    /*************************************************************************/   \
    "Invalid parameter",                                                                                        \
    "Invalid data size",                                                                                        \
    "Parameter is out of range",                                                                                \
    "Invalid data field",                                                                                       \
    "Value is corrupted",                                                                                       \
    "Invalid number of parameters",                                                                             \
    "Invalid data alignment",                                                                                   \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* Response                                                              */   \
    /*************************************************************************/   \
    "Response cannot be provided",                                                                              \
    "System is busy",                                                                                           \
    "System is not initialized",                                                                                \
    "System is in incorrect state",                                                                             \
    "Response timeout occurred",                                                                                \
    "Response is aborted",                                                                                      \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* Security                                                              */   \
    /*************************************************************************/   \
    "Security error",                                                                                           \
    "Insufficient privileges level",                                                                            \
    "Authentication failed",                                                                                    \
    "Bad signature",                                                                                            \
    "System is locked",                                                                                         \
    "Integrity check failed",                                                                                   \
    "Checksum verification failed",                                                                             \
    "Illegal Access",                                                                                           \
    "Illegal Configuration",                                                                                    \
    "System is permanently locked",                                                                             \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* Communication                                                         */   \
    /*************************************************************************/   \
    "Communication error",                                                                                      \
    "No connection",                                                                                            \
    "Can't open connection",                                                                                    \
    "Connection is already open",                                                                               \
    "Illegal Command",                                                                                          \
    "Unknown Command",                                                                                          \
    "Command ignored",                                                                                          \
    "Command failed",                                                                                           \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* Hardware                                                              */   \
    /*************************************************************************/   \
    "Hardware error",                                                                                           \
    "IO error",                                                                                                 \
    "Clock error",                                                                                              \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* Implementation                                                        */   \
    /*************************************************************************/   \
    "Not implemented",                                                                                          \
    "Implementation error",                                                                                     \
    "Function is deprecated",                                                                                   \
    "Not supported",                                                                                            \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* File                                                                  */   \
    /*************************************************************************/   \
    "File cannot be open",                                                                                      \
    "File not found",                                                                                           \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    /*************************************************************************/   \
    /* Memory                                                                */   \
    /*************************************************************************/   \
    "Out of memory",                                                                                            \
    "Buffer is full",                                                                                           \
    "Buffer is empty",                                                                                          \
    "Buffer overflow",                                                                                          \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
    "",                                                                                                         \
                                                                                                                \
}


/*****************************************************************************/
/*****************************************************************************/
/*                           UTILITY ERROR MACROS                            */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/* Macro:           AC_RET_CHECK_ACTION                                      */
/*                                                                           */
/* Parameters:                                                               */
/*                  func      - function to check                            */
/*                  action    - action to execute on error                   */
/*                                                                           */
/* Returns:                                                                  */
/* Side effects:    none                                                     */
/* Description:                                                              */
/*                  This macro checks if given function returns              */
/*                  AC_(ERROR) and perform specified                         */
/*                  action                                                   */
/*****************************************************************************/
#define AC_RET_CHECK_ACTION(func, action)                           \
{                                                                   \
    AC_status_t ___ret;                                             \
                                                                    \
    if ((___ret = func) != AC_OK)                                   \
    {                                                               \
        AC_LOG_ERROR;                                               \
        action;                                                     \
    }                                                               \
    (void)___ret;                                                   \
}

/*****************************************************************************/
/* Macro:           AC_RET_CHECK_ACTION_NO_RET                               */
/*                                                                           */
/* Parameters:                                                               */
/*                  func      - function to check                            */
/*                  action    - action to execute on error                   */
/*                                                                           */     
/* Returns:                                                                  */
/* Side effects:    none                                                     */
/* Description:                                                              */
/*                  This macro checks if given function returns              */
/*                  AC_(ERROR) and perform specified                         */
/*                  action. In this macro no __ret variable is available for */
/*                  'action'                                                 */
/*****************************************************************************/
#define AC_RET_CHECK_ACTION_NO_RET(func, action)                    \
{                                                                   \
    if ((func) != AC_OK)                                            \
    {                                                               \
        AC_LOG_ERROR;                                               \
        action;                                                     \
    }                                                               \
}

/*****************************************************************************/
/* Macro:           AC_RET_ASSERT                                            */
/*                                                                           */
/* Parameters:                                                               */
/*                  func    - function to check                              */
/*                                                                           */
/* Returns:                                                                  */
/* Side effects:                                                             */
/* Description:                                                              */
/*                  This macro checks if given function returns              */
/*                  AC_status_t error and asserts FALSE if                   */
/*                  error occurs                                             */
/*****************************************************************************/
#define AC_RET_CHECK_ASSERT(func)                                   \
{                                                                   \
    if ((func) != AC_OK)                                            \
    {                                                               \
        AC_LOG_ERROR;                                               \
        ASSERT(FALSE)                                               \
    }                                                               \
}

/*****************************************************************************/
/* Macro:           AC_RET_CHECK_GOTO                                        */
/*                                                                           */
/* Parameters:                                                               */
/*                  func         - function to check                         */
/*                  err          - variable to hold error value              */
/*                  err_label    - error label                               */
/*                                                                           */
/* Returns:                                                                  */
/* Side effects:    none                                                     */
/* Description:                                                              */
/*                  This routine checks if given function returns            */
/*                  AC_status_t error, sets error variable                   */
/*                  and jumps to given label                                 */
/*****************************************************************************/
#define AC_RET_CHECK_GOTO(func, err, err_label)            AC_RET_CHECK_ACTION(func, {err=___ret; goto err_label;})


/*****************************************************************************/
/* Macro:           AC_RET_CHECK_VAL                                         */
/*                                                                           */
/* Parameters:                                                               */
/*                  func    - function to check                              */
/*                  retVal  - return value                                   */
/*                                                                           */
/* Returns:                                                                  */
/* Side effects:                                                             */
/* Description:                                                              */
/*                  This macro checks if give function returns               */
/*                  AC_status_t error, and returns the error immediately     */
/*                                                                           */
/* Example:                                                                  */
/*                                                                           */
/*    AC_status_t myFunc(INT p1, INT p2);                                    */
/*                                                                           */
/*    INT32 otherFunc                                                        */
/*    {                                                                      */
/*        ...                                           // Some code         */
/*        AC_RET_CHECK(myFunc(p1,p2), -1);              // Executing myFunc  */
/*        ...                                                                */
/*    }                                                                      */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#define AC_RET_CHECK_VAL(func, retVal)                     AC_RET_CHECK_ACTION_NO_RET(func, AC_RETURN_ACTION(retVal))


/*****************************************************************************/
/* Macro:           AC_COND_CHECK_ACTION                                     */
/*                                                                           */
/* Parameters:                                                               */
/*                  cond      - Condition to check                           */
/*                  action    - Action to execute if condition is not met    */
/*                                                                           */
/* Returns:                                                                  */
/* Side effects:    none                                                     */
/* Description:                                                              */
/*                  This routine checks if given condition is not met,       */
/*                  and performs specific action                             */
/*****************************************************************************/
#define AC_COND_CHECK_ACTION(cond, action)                          \
{                                                                   \
    if (!(cond))                                                    \
    {                                                               \
        AC_LOG_ERROR;                                               \
        action;                                                     \
    }                                                               \
}



/*****************************************************************************/
/*****************************************************************************/
/*                         CONTROLABLE ERROR MACROS                          */
/*                                                                           */
/*  These macros are controlled by following flags:                          */
/*  default                     - Each check macro returns error if FALSE condition met              */
/*  AC_CHECK_ASSERT    - If defined, each check generates ASSERT on FALSE condition         */
/*  AC_CHECK_EXCEPTION - If defined, each check generates exception on FALSE condition      */
/*  AC_CHECK_DISABLE   - if defined, no checks are performed at all                         */
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/
#if defined (AC_CHECK_DISABLE)
    #define AC_RETURN_ACTION(e)

#elif defined (AC_CHECK_EXCEPTION)
    #define AC_RETURN_ACTION(e)         throw e

#elif defined (AC_CHECK_ASSERT)
    #define AC_RETURN_ACTION(e)         ASSERT(FALSE)

#else
    #define AC_RETURN_ACTION(e)         return e
#endif


/*****************************************************************************/
/*                               ERROR LOGGING                               */
/*                                                                           */
/* User might override the definition of this macro to use custom logging    */
/*****************************************************************************/
#if defined  AC_LOG_ERROR_PC
#define AC_LOG_ERROR       { AC_LAST_INST = CURRENT_INSTRUCTION_ADDRESS();   }
#elif defined AC_LOG_FUNCTION
#define AC_LOG_ERROR       { AC_LOG_FUNCTION(__FUNCTION__, __LINE__, __FILE__, CURRET_INSTRUCTION_ADDRESS()); }
#else
#define AC_LOG_ERROR
#endif

/*****************************************************************************/
/* Macro:           AC_COND_CHECK                                            */
/*                                                                           */
/* Parameters:                                                               */
/*                  cond    - Condition to check                             */
/*                  err     - Error to through if condition is not met       */
/*                                                                           */
/* Returns:                                                                  */
/* Side effects:                                                             */
/* Description:                                                              */
/*                  This routine checks given condition and returns          */
/*                  the given error if condition was not met                 */
/*                                                                           */
/* Example:                                                                  */
/*                                                                           */
/*    AC_status_t myFunc(void* ptr)                                          */
/*    {                                                                      */
/*        AC_COND_CHECK(ptr, AC_INVALID_PARAMETER);                          */
/*        ...                                                                */
/*    }                                                                      */
/*                                                                           */
/*****************************************************************************/
#define AC_COND_CHECK(cond, err)   AC_COND_CHECK_ACTION(cond,  AC_RETURN_ACTION(err))

/*****************************************************************************/
/* Macro:           AC_RET_CHECK                                             */
/*                                                                           */
/* Parameters:                                                               */
/*                  func    - function to check                              */
/*                                                                           */
/* Returns:                                                                  */
/* Side effects:                                                             */
/* Description:                                                              */
/*                  This macro checks if give function returns               */
/*                  AC_status_t error, and returns the error immediately     */
/*                                                                           */
/* Example:                                                                  */
/*                                                                           */
/*    AC_status_t myFunc(INT p1, INT p2);                                    */
/*                                                                           */
/*    AC_status_t otherFunc                                                  */
/*    {                                                                      */
/*        ...                                         // Some code           */
/*        AC_RET_CHECK(myFunc(p1,p2));                // Executing myFunc    */
/*        ...                                                                */
/*    }                                                                      */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#define AC_RET_CHECK(func) AC_RET_CHECK_ACTION(func, AC_RETURN_ACTION(___ret))

#endif // _AC_DEFS_ERRORS_H_

