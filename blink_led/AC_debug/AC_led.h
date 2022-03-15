/**************************************************************************//**
 * @file      AC_led.h
 * @brief     LED controller - Header file
 * @details   
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/
#ifndef _AC_LED_H_
#define _AC_LED_H_

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include "AC_defs.h"
#include "stdio.h"

/*****************************************************************************/
/*****************************************************************************/
/*                           DEFINITIONS & TYPES                             */
/*****************************************************************************/
/*****************************************************************************/
typedef enum{
    LED_OFF         = 0,
    LED_BLINK_SLOW  = 1,
    LED_BLINK_FAST  = 2,
    LED_ON          = 3
} led_state_t;

/*****************************************************************************/
/*****************************************************************************/
/*                      INTERFACE FUNCTIONS DECLARATION                      */
/*****************************************************************************/
/*****************************************************************************/
AC_status_t LED_init(void);
AC_status_t LED_set_state(led_state_t state);

#endif /* _AC_LED_H_ */