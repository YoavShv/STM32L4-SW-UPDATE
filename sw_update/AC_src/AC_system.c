/**************************************************************************//**
 * @file      AC_system.c
 * @brief     System control implementation
 * @details   
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).

 *   This file provides one function and one global variable to be called from
 *   user application:
 *      - SystemInit(): This function is called at startup just after reset and
 *                      before branch to main program. This call is made inside
 *                      the "startup_stm32l4xx.s" file.
 *
 *   After each device reset the MSI (4 MHz) is used as system clock source.
 *   Then SystemInit() function is called, in "startup_stm32l4xx.s" file, to
 *   configure the system clock before to branch to main program.
 *
 *   This file configures the system clock as follows:
 *=============================================================================
 *-----------------------------------------------------------------------------
 *        System Clock source                    | MSI
 *-----------------------------------------------------------------------------
 *        SYSCLK(Hz)                             | 4000000
 *-----------------------------------------------------------------------------
 *        HCLK(Hz)                               | 4000000
 *-----------------------------------------------------------------------------
 *        AHB Prescaler                          | 1
 *-----------------------------------------------------------------------------
 *        APB1 Prescaler                         | 1
 *-----------------------------------------------------------------------------
 *        APB2 Prescaler                         | 1
 *-----------------------------------------------------------------------------
 *        PLL_M                                  | 1
 *-----------------------------------------------------------------------------
 *        PLL_N                                  | 8
 *-----------------------------------------------------------------------------
 *        PLL_P                                  | 7
 *-----------------------------------------------------------------------------
 *        PLL_Q                                  | 2
 *-----------------------------------------------------------------------------
 *        PLL_R                                  | 2
 *-----------------------------------------------------------------------------
 *        PLLSAI1_P                              | NA
 *-----------------------------------------------------------------------------
 *        PLLSAI1_Q                              | NA
 *-----------------------------------------------------------------------------
 *        PLLSAI1_R                              | NA
 *-----------------------------------------------------------------------------
 *        PLLSAI2_P                              | NA
 *-----------------------------------------------------------------------------
 *        PLLSAI2_Q                              | NA
 *-----------------------------------------------------------------------------
 *        PLLSAI2_R                              | NA
 *-----------------------------------------------------------------------------
 *        Require 48MHz for USB OTG FS,          | Disabled
 *        SDIO and RNG clock                     |
 *-----------------------------------------------------------------------------
 *****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include "AC_system.h"
#include "AC_rcc.h"

/*****************************************************************************/
/*****************************************************************************/
/*                   TYPES & LOCAL VARIABLES & CONSTANTS                     */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
// CORTEX_Preemption_Priority_Group CORTEX Preemption Priority Group
/*****************************************************************************/
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */

#define PWR_FLAG_SETTING_DELAY_US       50UL                /*!< Time out value for REGLPF and VOSF flags setting */


/* system timeout*/
#define SYS_TIMEOUT                  ((uint32_t)(1000000))

/*****************************************************************************/
/*****************************************************************************/
/*                        LOCAL FUNCTIONS DECLARATION                        */
/*****************************************************************************/
/*****************************************************************************/
static AC_status_t system_clock_config_L(void);
static void error_handler_L(void);
static AC_status_t PWREx_ControlVoltageScaling_L(uint32_t VoltageScaling);
static AC_status_t NVIC_SetPriorityGrouping_L(uint32_t PriorityGroup);

/*****************************************************************************/
/*****************************************************************************/
/*                    INTERFACE FUNCTIONS IMPLEMENTATION                     */
/*****************************************************************************/
/*****************************************************************************/

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */
/******************************************************************************/
  uint32_t SystemCoreClock = 4000000U;

  const uint8_t  AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
  const uint8_t  APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
  const uint32_t MSIRangeTable[12] = {100000U,   200000U,   400000U,   800000U,  1000000U,  2000000U, \
                                      4000000U, 8000000U, 16000000U, 24000000U, 32000000U, 48000000U};

/**************************************************************************//**
 * @brief   Setup the microcontroller system.
 * @param   None
 * @return  None
 *****************************************************************************/
void SystemInit(void)
{
    /* FPU settings ------------------------------------------------------------*/
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
      SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
    #endif

    /* Reset the RCC clock configuration to the default reset state ------------*/
    /* Set MSION bit */
    RCC->CR |= RCC_CR_MSION;

    /* Reset CFGR register */
    RCC->CFGR = 0x00000000U;

    /* Reset HSEON, CSSON , HSION, and PLLON bits */
    RCC->CR &= 0xEAF6FFFFU;

    /* Reset PLLCFGR register */
    RCC->PLLCFGR = 0x00001000U;

    /* Reset HSEBYP bit */
    RCC->CR &= 0xFFFBFFFFU;

    /* Disable all interrupts */
    RCC->CIER = 0x00000000U;

    /* Configure the Vector Table location add offset address ------------------*/
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
}

/**************************************************************************//**
 * @brief   Initialize system NVIC & clocks
 * @note    This function is called at the beginning of program after reset.
 *
 * @param   none
 * @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
AC_status_t AC_SYS_init(void)
{

    /*************************************************************************/
    // set Interrupt Group Priority
    /*************************************************************************/
    if (AC_OK != NVIC_SetPriorityGrouping_L(NVIC_PRIORITYGROUP_4))
    {
        return AC_FAIL;
    }

    /*************************************************************************/
    // configure clocks
    /*************************************************************************/
    if (AC_OK != system_clock_config_L())
    {
        return AC_FAIL;
    }

    return AC_OK;
}

/*****************************************************************************/
/*****************************************************************************/
/*                      LOCAL FUNCTIONS IMPLEMENTATION                       */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   Configure the system clocks
 *
 * @param   none
 * @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
static AC_status_t system_clock_config_L(void)
{
    
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    //RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /*************************************************************************/
    // enable system clocks
    /*************************************************************************/
    AC_RCC_SYSCFG_CLK_ENABLE();
    AC_RCC_PWR_CLK_ENABLE();

    /*************************************************************************/
    // Configure LSE Drive Capability:
    //  - Enable access to the backup domain (RTC registers, RTC backup data registers).
    //  - After reset, the backup domain is protected against possible unwanted write accesses.
    //  - RTCSEL that sets the RTC clock source selection is in the RTC back-up domain.
    //  - In order to set or modify the RTC clock, the backup domain access must be disabled.
    //  - LSEON bit that switches on and off the LSE crystal belongs as well to the back-up domain.
    /*************************************************************************/
    SET_BIT(PWR->CR1, PWR_CR1_DBP);
    AC_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
    
    /*************************************************************************/
    // Configure the main internal regulator output voltage
    /*************************************************************************/
    if (PWREx_ControlVoltageScaling_L(PWR_REGULATOR_VOLTAGE_SCALE1) != AC_OK)
    {
        error_handler_L();
    }

    /*************************************************************************/
    // Initializes the RCC Oscillators according to the specified parameters
    // in the RCC_OscInitTypeDef structure.
    /*************************************************************************/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                                |RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (AC_RCC_OscConfig(&RCC_OscInitStruct) != AC_OK)
    {
        error_handler_L();
    }

    /*************************************************************************/
    // Initializes the CPU, AHB and APB buses clocks
    /*************************************************************************/
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (AC_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != AC_OK)
    {
        error_handler_L();
    }

    /*************************************************************************/
    // Enable the PLL-mode of the MSI.
    // Prior to enable the PLL-mode of the MSI for automatic hardware
    // calibration LSE oscillator is to be enabled with HAL_RCC_OscConfig().
    /*************************************************************************/
    SET_BIT(RCC->CR, RCC_CR_MSIPLLEN) ;

    return AC_OK;
}

/**************************************************************************//**
 * @brief   This function is executed in case of error occurrence.

 * @param   none
 * @return  none
 *****************************************************************************/
static void error_handler_L(void)
{
    __disable_irq();
    while(1);
}

/**************************************************************************//**
 * @brief Configure the main internal regulator output voltage.
 * @param  VoltageScaling specifies the regulator output voltage to achieve
 *         a tradeoff between performance and power consumption.
 *          This parameter can be one of the following values:
 *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE1 Regulator voltage output range 1 mode,
 *                                                typical output voltage at 1.2 V,
 *                                                system frequency up to 80 MHz.
 *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE2 Regulator voltage output range 2 mode,
 *                                                typical output voltage at 1.0 V,
 *                                                system frequency up to 26 MHz.
 * @note  When moving from Range 1 to Range 2, the system frequency must be decreased to
 *        a value below 26 MHz before calling HAL_PWREx_ControlVoltageScaling() API.
 *        When moving from Range 2 to Range 1, the system frequency can be increased to
 *        a value up to 80 MHz after calling HAL_PWREx_ControlVoltageScaling() API. For
 *        some devices, the system frequency can be increased up to 120 MHz.
 * @note  When moving from Range 2 to Range 1, the API waits for VOSF flag to be
 *        cleared before returning the status. If the flag is not cleared within
 *        50 microseconds, HAL_TIMEOUT status is reported.
 * @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
static AC_status_t PWREx_ControlVoltageScaling_L(uint32_t VoltageScaling)
{
    uint32_t wait_loop_index;

    /*************************************************************************/
    // set Range 1 
    /*************************************************************************/
    if (VoltageScaling == PWR_REGULATOR_VOLTAGE_SCALE1)
    {
        if (READ_BIT(PWR->CR1, PWR_CR1_VOS) != PWR_REGULATOR_VOLTAGE_SCALE1)
        {
            /* Set Range 1 */
            MODIFY_REG(PWR->CR1, PWR_CR1_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);

            /* Wait until VOSF is cleared */
            wait_loop_index = ((PWR_FLAG_SETTING_DELAY_US * SystemCoreClock) / 1000000U) + 1U;
            while ((AC_IS_BIT_SET(PWR->SR2, PWR_SR2_VOSF)) && (wait_loop_index != 0U))
            {
              wait_loop_index--;
            }
            if (AC_IS_BIT_SET(PWR->SR2, PWR_SR2_VOSF))
            {
              return AC_RESPONSE_TIMEOUT;
            }
        }
    }

    /*************************************************************************/
    // set Range 2 
    /*************************************************************************/
    else if (VoltageScaling == PWR_REGULATOR_VOLTAGE_SCALE2)
    {
        if (READ_BIT(PWR->CR1, PWR_CR1_VOS) != PWR_REGULATOR_VOLTAGE_SCALE2)
        {
          /* Set Range 2 */
          MODIFY_REG(PWR->CR1, PWR_CR1_VOS, PWR_REGULATOR_VOLTAGE_SCALE2);
          /* No need to wait for VOSF to be cleared for this transition */
        }
    }

    /*************************************************************************/
    // no valid range
    /*************************************************************************/
    else
    {
        return AC_FAIL;
    }

    return AC_OK;
}

/**************************************************************************//**
 * @brief  Set the priority grouping field (pre-emption priority and subpriority)
 *         using the required unlock sequence.
 * @param  PriorityGroup: The priority grouping bits length.
 *         This parameter can be one of the following values:
 *         @arg : 0 bit  for pre-emption priority,
 *                                    4 bits for subpriority
 *         @arg NVIC_PRIORITYGROUP_1: 1 bit  for pre-emption priority,
 *                                    3 bits for subpriority
 *         @arg NVIC_PRIORITYGROUP_2: 2 bits for pre-emption priority,
 *                                    2 bits for subpriority
 *         @arg NVIC_PRIORITYGROUP_3: 3 bits for pre-emption priority,
 *                                    1 bit  for subpriority
 *         @arg NVIC_PRIORITYGROUP_4: 4 bits for pre-emption priority,
 *                                    0 bit  for subpriority
 * @note   When the NVIC_PriorityGroup_0 is selected, IRQ pre-emption is no more possible.
 *         The pending IRQ priority will be managed only by the subpriority.
 * @retval None
 *****************************************************************************/
static AC_status_t NVIC_SetPriorityGrouping_L(uint32_t PriorityGroup)
{

    /*************************************************************************/
    // check parameters
    /*************************************************************************/
    if  (
        (PriorityGroup != NVIC_PRIORITYGROUP_0) &&
        (PriorityGroup != NVIC_PRIORITYGROUP_1) &&
        (PriorityGroup != NVIC_PRIORITYGROUP_2) &&
        (PriorityGroup != NVIC_PRIORITYGROUP_3) &&
        (PriorityGroup != NVIC_PRIORITYGROUP_4)
        )
    {
        return AC_INVALID_PARAMETER;
    }

    /*************************************************************************/
    // Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value
    /*************************************************************************/
    NVIC_SetPriorityGrouping(PriorityGroup);

    return AC_OK;
}
