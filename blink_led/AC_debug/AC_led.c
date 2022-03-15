/**************************************************************************//**
 * @file      AC_led.c
 * @brief     LED control implementation
 * @details   
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include "AC_led.h"

/*****************************************************************************/
/*****************************************************************************/
/*                        LOCAL FUNCTIONS DECLARATION                        */
/*****************************************************************************/
/*****************************************************************************/
__STATIC_INLINE void LL_AHB2_GRP1_EnableClock_L(uint32_t Periphs);
__STATIC_INLINE void LL_GPIO_SetPinMode_L(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode);
__STATIC_INLINE void LL_GPIO_SetOutputPin_L(GPIO_TypeDef *GPIOx, uint32_t PinMask);
__STATIC_INLINE void LL_GPIO_ResetOutputPin_L(GPIO_TypeDef *GPIOx, uint32_t PinMask);
__STATIC_INLINE void LL_GPIO_TogglePin_L(GPIO_TypeDef *GPIOx, uint32_t PinMask);

/*****************************************************************************/
/*****************************************************************************/
/*                           DEFINITIONS & TYPES                             */
/*****************************************************************************/
/*****************************************************************************/
#define LED4_GPIO_CLK_ENABLE()    LL_AHB2_GRP1_EnableClock_L(RCC_AHB2ENR_GPIOBEN)
#define LED4_GPIO_PORT            GPIOB
#define LED4_PIN                  0x00002000 /* LL_GPIO_PIN_13 */
#define LL_GPIO_MODE_OUTPUT       0x00000001 

#define BLINK_NUM                 10 

/*****************************************************************************/
/*****************************************************************************/
/*                    INTERFACE FUNCTIONS IMPLEMENTATION                     */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   Initialize the User LED on the board
 * 
 * @param   none
 * @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
AC_status_t LED_init(void)
{
    /*************************************************************************/
    // enable the LED4 Clock
    /*************************************************************************/
    LED4_GPIO_CLK_ENABLE();
    
    /*************************************************************************/
    // configure IO in output push-pull mode to drive external LED2
    /*************************************************************************/
    LL_GPIO_SetPinMode_L(LED4_GPIO_PORT, LED4_PIN, LL_GPIO_MODE_OUTPUT);
    return (AC_OK);
}

/**************************************************************************//**
 * @brief   Set the LED state
 * 
 * @param   [IN] state - the LED state to set (ON, BLINK_SLOW, BLINK_FAST, OFF)
 * @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
AC_status_t LED_set_state(led_state_t state)
{
    uint32_t delay = 200000;
    
    /*************************************************************************/
    // run LED state
    /*************************************************************************/
    switch (state)
    {
        case (LED_ON):
            LL_GPIO_SetOutputPin_L(LED4_GPIO_PORT, LED4_PIN);
            break;
        case (LED_OFF):
            LL_GPIO_ResetOutputPin_L(LED4_GPIO_PORT, LED4_PIN);
            break;
        case (LED_BLINK_SLOW):
            delay *= 4;
        case (LED_BLINK_FAST):
            for(uint32_t i = 0; i < 2*BLINK_NUM; i++)
            {
                /*************************************************************/
                // Toggle LED4 
                /*************************************************************/
                LL_GPIO_TogglePin_L(LED4_GPIO_PORT, LED4_PIN);

                /*************************************************************/
                // Delay
                /*************************************************************/
                DELAY_LOOP(delay);
            }
            break;
        default:
            return (AC_FAIL);
    }

    return (AC_OK);
}

/*****************************************************************************/
/*****************************************************************************/
/*                      LOCAL FUNCTIONS IMPLEMENTATION                       */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief  Enable AHB2 peripherals clock.
 * @param  Periphs This parameter can be a combination of the following values:
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOA
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOB
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOC
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOD (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOE (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOF (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOG (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOH
 *         @arg @ref LL_AHB2_GRP1_PERIPH_GPIOI (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_OTGFS (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_ADC
 *         @arg @ref LL_AHB2_GRP1_PERIPH_DCMI (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_AES (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_HASH (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_RNG
 *         @arg @ref LL_AHB2_GRP1_PERIPH_OSPIM (*)
 *         @arg @ref LL_AHB2_GRP1_PERIPH_SDMMC1 (*)
 *
 *         (*) value not defined in all devices.
 *  @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
__STATIC_INLINE void LL_AHB2_GRP1_EnableClock_L(uint32_t Periphs)
{
    __IO uint32_t tmpreg;
    SET_BIT(RCC->AHB2ENR, Periphs);
    
    /* Delay after an RCC peripheral clock enabling */
    tmpreg = READ_BIT(RCC->AHB2ENR, Periphs);
    (void)tmpreg;
}

/**************************************************************************//**
 * @brief  Configure gpio mode for a dedicated pin on dedicated port.
 * @note   I/O mode can be Input mode, General purpose output, Alternate function mode or Analog.
 * @note   Warning: only one pin can be passed as parameter.

 * @param  [IN] GPIOx GPIO Port
 * @param  [IN] Pin This parameter can be one of the following values:
 *         @arg @ref LL_GPIO_PIN_0
 *         @arg @ref LL_GPIO_PIN_1
 *         @arg @ref LL_GPIO_PIN_2
 *         @arg @ref LL_GPIO_PIN_3
 *         @arg @ref LL_GPIO_PIN_4
 *         @arg @ref LL_GPIO_PIN_5
 *         @arg @ref LL_GPIO_PIN_6
 *         @arg @ref LL_GPIO_PIN_7
 *         @arg @ref LL_GPIO_PIN_8
 *         @arg @ref LL_GPIO_PIN_9
 *         @arg @ref LL_GPIO_PIN_10
 *         @arg @ref LL_GPIO_PIN_11
 *         @arg @ref LL_GPIO_PIN_12
 *         @arg @ref LL_GPIO_PIN_13
 *         @arg @ref LL_GPIO_PIN_14
 *         @arg @ref LL_GPIO_PIN_15
 * @param  [IN] Mode This parameter can be one of the following values:
 *         @arg @ref LL_GPIO_MODE_INPUT
 *         @arg @ref LL_GPIO_MODE_OUTPUT
 *         @arg @ref LL_GPIO_MODE_ALTERNATE
 *         @arg @ref LL_GPIO_MODE_ANALOG
 * @retval None
 */
__STATIC_INLINE void LL_GPIO_SetPinMode_L(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
    MODIFY_REG(GPIOx->MODER, (GPIO_MODER_MODE0 << (POSITION_VAL(Pin) * 2U)), (Mode << (POSITION_VAL(Pin) * 2U)));
}

/**
  * @brief  Set several pins to high level on dedicated gpio port.
  * @rmtoll BSRR         BSy           LL_GPIO_SetOutputPin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void LL_GPIO_SetOutputPin_L(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
    WRITE_REG(GPIOx->BSRR, PinMask);
}

/**
  * @brief  Set several pins to low level on dedicated gpio port.
  * @rmtoll BRR          BRy           LL_GPIO_ResetOutputPin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void LL_GPIO_ResetOutputPin_L(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
    WRITE_REG(GPIOx->BRR, PinMask);
}

/**
  * @brief  Toggle data value for several pin of dedicated port.
  * @rmtoll ODR          ODy           LL_GPIO_TogglePin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
__STATIC_INLINE void LL_GPIO_TogglePin_L(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
    uint32_t odr = READ_REG(GPIOx->ODR);
    WRITE_REG(GPIOx->BSRR, ((odr & PinMask) << 16u) | (~odr & PinMask));
}
