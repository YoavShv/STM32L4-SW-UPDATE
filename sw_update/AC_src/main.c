/**************************************************************************//**
 * @file      main.c
 * @brief     Start point sw update example
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include <string.h>
#include <inttypes.h>
#include "AC_system.h"
#include "AC_led.h"
#include "AC_debug.h"
#include "AC_sw_update.h"
#include "AC_new_sw_example.h"

/*****************************************************************************/
/*****************************************************************************/
/*                               DEFINITIONS                                 */
/*****************************************************************************/
/*****************************************************************************/
#define CURR_APP_ADDR   SU_ADDR_FLASH_PAGE_0
#define CURR_APP_SIZE   30000
#define NEW_APP_ADDR    SU_ADDR_FLASH_PAGE_30

/*****************************************************************************/
/*****************************************************************************/
/*                    INTERFACE FUNCTIONS IMPLEMENTATION                     */
/*****************************************************************************/
/*****************************************************************************/
void handle_err_L(void);
void print_buff_L(uint64_t* buff, uint32_t size);

/*****************************************************************************/
/*****************************************************************************/
/*                              SW ENTRY POINT                               */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   SW entry point
 * 
 * @param   none
 * @return  none
 *****************************************************************************/
int main(void)
{   
    /*************************************************************************/
    // init system
    /*************************************************************************/
    AC_SYS_init();

    /*************************************************************************/
    // init debug printf & scanf
    /*************************************************************************/
    AC_DEBUG_uart_retarget_init();

    /*************************************************************************/
    // init led
    /*************************************************************************/
    LED_init();

    /*************************************************************************/
    // start led
    /*************************************************************************/
    LED_set_state(LED_ON);
    
    /*************************************************************************/
    // run sw update example
    /*************************************************************************/
    printf("########################\r\n");
    printf("RUN SW UPDATE EXAMPLE:\r\n");
    printf("########################\r\n");
    
    /*************************************************************************/
    // init sw update
    /*************************************************************************/
    if (SW_UPDATE_OK != SW_UPDATE_init(CURR_APP_ADDR, CURR_APP_SIZE))
    {
        handle_err_L();
    }

    /*************************************************************************/
    // sw update begin
    /*************************************************************************/
    if (SW_UPDATE_OK != SW_UPDATE_begin(NEW_APP_ADDR, sizeof(BLINK_LED_SW)))
    {
        handle_err_L();
    }

    /*************************************************************************/
    // sw update program
    /*************************************************************************/
    uint32_t NEW_APP_BUFF_SIZE = sizeof(BLINK_LED_SW)/sizeof(BLINK_LED_SW[0]);
    
    if (SW_UPDATE_OK != SW_UPDATE_next((uint64_t*)BLINK_LED_SW, NEW_APP_BUFF_SIZE))
    {
        handle_err_L();
    }

    /*************************************************************************/
    // sw update end
    /*************************************************************************/
    if (SW_UPDATE_OK != SW_UPDATE_end())
    {
        handle_err_L();
    }
    
    /*************************************************************************/
    // turn led off
    /*************************************************************************/
    LED_set_state(LED_OFF);

    /*************************************************************************/
    // jump to new app
    /*************************************************************************/
    if (SW_UPDATE_OK != SW_UPDATE_jump_to_new_app())
    {
        handle_err_L();
    }

    printf("AFTER THE JUMP. THIS SHOULD NOT BE PRINTED!!!\r\n");
    while(1);
}

/*****************************************************************************/
/*****************************************************************************/
/*                      LOCAL FUNCTIONS IMPLEMENTATION                       */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   Handle errors
 *          
 * @param   none
 * @return  none
 *****************************************************************************/
void handle_err_L(void)
{
    printf("ERROR!!!\r\n");
    while(1);
}

/**************************************************************************//**
 * @brief   print buffer
 *          
 * @param   [IN] buff   - buffer start address
 * @param   [IN] size   - buffer's size (number of elements in buffer,
 *                        each element is 64bits)
 * 
 * @return  none
 *****************************************************************************/
void print_buff_L(uint64_t* buff, uint32_t size)
{
    for (uint32_t i = 0; i<size; i++)
    {
        printf("buff[%ld] = 0x%08lX_0x%08lX\r\n", i, (uint32_t)(buff[i] >> 32), (uint32_t)buff[i]);
    }
}