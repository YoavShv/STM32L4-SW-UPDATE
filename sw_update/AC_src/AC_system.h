/**************************************************************************//**
 * @file      AC_system.h
 * @brief     System control - Header file
 * @details   
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/
#ifndef _AC_SYSTEM_H_
#define _AC_SYSTEM_H_

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include "AC_defs.h"

/*****************************************************************************/
/*****************************************************************************/
/*                                CONSTANTS                                  */
/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************/
/*****************************************************************************/
/*                                  MACROS                                   */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
// Clock values [Hz]
/*****************************************************************************/
#define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#define HSE_VALUE    ((uint32_t)8000000U)  /*!< Value of the External oscillator in Hz */

/*****************************************************************************/
// PWREx_Regulator_Voltage_Scale  PWR Regulator voltage scale
/*****************************************************************************/
#define PWR_REGULATOR_VOLTAGE_SCALE1    PWR_CR1_VOS_0       /*!< Voltage scaling range 1 normal mode */
#define PWR_REGULATOR_VOLTAGE_SCALE2    PWR_CR1_VOS_1       /*!< Voltage scaling range 2             */

/*****************************************************************************/
/*****************************************************************************/
/*                                  MACROS                                   */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief  Set the FLASH Latency.
 * @param  __LATENCY__ FLASH Latency
 *         This parameter can be one of the following values :
 *     @arg FLASH_LATENCY_0: FLASH Zero wait state
 *     @arg FLASH_LATENCY_1: FLASH One wait state
 *     @arg FLASH_LATENCY_2: FLASH Two wait states
 *     @arg FLASH_LATENCY_3: FLASH Three wait states
 *     @arg FLASH_LATENCY_4: FLASH Four wait states
 * @retval None
 *****************************************************************************/
#define AC_FLASH_SET_LATENCY(__LATENCY__)    (MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (__LATENCY__)))

/**************************************************************************//**
 * @brief  Get the FLASH Latency.
 * @retval FLASH Latency
 *         This parameter can be one of the following values :
 *     @arg FLASH_LATENCY_0: FLASH Zero wait state
 *     @arg FLASH_LATENCY_1: FLASH One wait state
 *     @arg FLASH_LATENCY_2: FLASH Two wait states
 *     @arg FLASH_LATENCY_3: FLASH Three wait states
 *     @arg FLASH_LATENCY_4: FLASH Four wait states
 *****************************************************************************/
#define AC_FLASH_GET_LATENCY()               READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY)

/*****************************************************************************/
/*****************************************************************************/
/*                      INTERFACE FUNCTIONS DECLARATION                      */
/*****************************************************************************/
/*****************************************************************************/

extern uint32_t SystemCoreClock;            /*!< System Clock Frequency (Core Clock) */

extern const uint8_t  AHBPrescTable[16];    /*!< AHB prescalers table values */
extern const uint8_t  APBPrescTable[8];     /*!< APB prescalers table values */
extern const uint32_t MSIRangeTable[12];    /*!< MSI ranges table values     */

extern void SystemInit(void);
AC_status_t AC_SYS_init(void);

#endif /* _AC_SYSTEM_H_ */
