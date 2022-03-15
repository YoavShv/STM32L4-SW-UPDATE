/**************************************************************************//**
 * @file    AC_rcc.c
 * @brief   Reset & Clock Controller implementation
 *          This file provides firmware functions to manage the following
 *          functionalities of the Reset and Clock Control (RCC) peripheral:
 *           + Initialization and de-initialization functions
 *           + Peripheral Control functions
 *
 @verbatim
 ==============================================================================
                     ##### RCC specific features #####
 ==============================================================================
    [..]
      After reset the device is running from Multiple Speed Internal oscillator
      (4 MHz) with Flash 0 wait state. Flash prefetch buffer, D-Cache
      and I-Cache are disabled, and all peripherals are off except internal
      SRAM, Flash and JTAG.

      (+) There is no prescaler on High speed (AHBs) and Low speed (APBs) busses:
          all peripherals mapped on these busses are running at MSI speed.
      (+) The clock for all peripherals is switched off, except the SRAM and FLASH.
      (+) All GPIOs are in analog mode, except the JTAG pins which
          are assigned to be used for debug purpose.

    [..]
      Once the device started from reset, the user application has to:
      (+) Configure the clock source to be used to drive the System clock
          (if the application needs higher frequency/performance)
      (+) Configure the System clock frequency and Flash settings
      (+) Configure the AHB and APB busses prescalers
      (+) Enable the clock for the peripheral(s) to be used
      (+) Configure the clock source(s) for peripherals which clocks are not
          derived from the System clock (SAIx, RTC, ADC, USB OTG FS/SDMMC1/RNG)

 @endverbatim
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include "AC_rcc.h"
#include "AC_system.h"

/*****************************************************************************/
/*****************************************************************************/
/*                   TYPES & LOCAL VARIABLES & CONSTANTS                     */
/*****************************************************************************/
/*****************************************************************************/
#define RCC_DELAY 10000

/*****************************************************************************/
/*****************************************************************************/
/*                                  MACROS                                   */
/*****************************************************************************/
/*****************************************************************************/


/*****************************************************************************/
/*****************************************************************************/
/*                        LOCAL FUNCTIONS DECLARATION                        */
/*****************************************************************************/
/*****************************************************************************/
static AC_status_t RCC_SetFlashLatencyFromMSIRange_L(uint32_t msirange);

/*****************************************************************************/
/*****************************************************************************/
/*                    INTERFACE FUNCTIONS IMPLEMENTATION                     */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   Initialize the RCC Oscillators according to the specified parameters in the
 *          RCC_OscInitTypeDef.
 * @param   RCC_OscInitStruct  pointer to an RCC_OscInitTypeDef structure that
 *          contains the configuration information for the RCC Oscillators.
 * @note    The PLL is not disabled when used as system clock.
 * @note    The PLL source is not updated when used as PLLSAI(s) clock source.
 * @note    Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not
 *          supported by this macro. User should request a transition to LSE Off
 *          first and then LSE On or LSE Bypass.
 * @note    Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
 *          supported by this macro. User should request a transition to HSE Off
 *          first and then HSE On or HSE Bypass.
 * @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
AC_status_t AC_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
    uint32_t sysclk_source, pll_config;

    /*************************************************************************/
    // Check Null pointer
    /*************************************************************************/
    if(RCC_OscInitStruct == NULL)
    {
        return AC_FAIL;
    }

    sysclk_source = AC_RCC_GET_SYSCLK_SOURCE();
    pll_config = AC_RCC_GET_PLL_OSCSOURCE();

    /*************************************************************************/
    /*                         MSI Configuration                             */
    /*************************************************************************/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_MSI) == RCC_OSCILLATORTYPE_MSI)
    {

        /*********************************************************************/
        // Check if MSI is used as system clock or as PLL source when PLL is selected as system clock 
        /*********************************************************************/
        if((sysclk_source == RCC_CFGR_SWS_MSI) ||
           ((sysclk_source == RCC_CFGR_SWS_PLL) && (pll_config == RCC_PLLSOURCE_MSI)))
        {
            if((READ_BIT(RCC->CR, RCC_CR_MSIRDY) != 0U) && (RCC_OscInitStruct->MSIState == RCC_MSI_OFF))
            {
              return AC_FAIL;
            }

            /*****************************************************************/
            // Otherwise, just the calibration and MSI range change are allowed
            /*****************************************************************/
            else
            {
                /*************************************************************/
                // To correctly read data from FLASH memory, the number of wait states (LATENCY)
                // must be correctly programmed according to the frequency of the CPU clock
                // (HCLK) and the supply voltage of the device.
                /*************************************************************/
                if(RCC_OscInitStruct->MSIClockRange > AC_RCC_GET_MSI_RANGE())
                {
                    /*********************************************************/
                    // First increase number of wait states update if necessary 
                    /*********************************************************/
                    if(RCC_SetFlashLatencyFromMSIRange_L(RCC_OscInitStruct->MSIClockRange) != AC_OK)
                    {
                      return AC_FAIL;
                    }

                    /*********************************************************/
                    // Selects the Multiple Speed oscillator (MSI) clock range
                    /*********************************************************/
                    AC_RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);

                    /*********************************************************/
                    // Adjusts the Multiple Speed oscillator (MSI) calibration value.
                    /*********************************************************/
                    AC_RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);
                }
                else
                {
                    /*********************************************************/
                    // Else, keep current flash latency while decreasing applies
                    // Selects the Multiple Speed oscillator (MSI) clock range. 
                    /*********************************************************/
                    AC_RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);

                    /*********************************************************/
                    // Adjusts the Multiple Speed oscillator (MSI) calibration value.
                    /*********************************************************/
                    AC_RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);

                    /*********************************************************/
                    // Decrease number of wait states update if necessary 
                    // Only possible when MSI is the System clock source  
                    /*********************************************************/
                    if(sysclk_source == RCC_CFGR_SWS_MSI)
                    {
                        if(RCC_SetFlashLatencyFromMSIRange_L(RCC_OscInitStruct->MSIClockRange) != AC_OK)
                        {
                          return AC_FAIL;
                        }
                    }
                }

                /*************************************************************/
                // Update the SystemCoreClock global variable 
                /*************************************************************/
                SystemCoreClock = AC_RCC_GetSysClockFreq() >> (AHBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos] & 0x1FU);

            }
        }
        else
        {
            /*****************************************************************/
            // Check the MSI State
            /*****************************************************************/
            if(RCC_OscInitStruct->MSIState != RCC_MSI_OFF)
            {
                /*************************************************************/
                // Enable the Internal High Speed oscillator (MSI).
                /*************************************************************/
                AC_RCC_MSI_ENABLE();

                /* Wait till oscillator is ready */
                DELAY_LOOP(RCC_DELAY);

                /*************************************************************/
                // Selects the Multiple Speed oscillator (MSI) clock range
                /*************************************************************/
                AC_RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
                
                /*************************************************************/
                // Adjusts the Multiple Speed oscillator (MSI) calibration value
                /*************************************************************/
                AC_RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);

            }
            else
            {
                /*************************************************************/
                // Disable the Internal High Speed oscillator (MSI).
                /*************************************************************/
                AC_RCC_MSI_DISABLE();

                /* Wait till oscillator is ready */
                DELAY_LOOP(RCC_DELAY);
                
            }
        }
    }
    
    /*************************************************************************/
    /*                         HSE Configuration                             */
    /*************************************************************************/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
    {

        /*************************************************************************/
        // When the HSE is used as system clock or clock source for PLL in these cases it is not allowed to be disabled
        /*************************************************************************/
        if((sysclk_source == RCC_CFGR_SWS_HSE) ||
           ((sysclk_source == RCC_CFGR_SWS_PLL) && (pll_config == RCC_PLLSOURCE_HSE)))
        {
            if((READ_BIT(RCC->CR, RCC_CR_HSERDY) != 0U) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
            {
                return AC_FAIL;
            }
        }
        else
        {
            /*************************************************************************/
            // Set the new HSE configuration
            /*************************************************************************/
            AC_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

            /*************************************************************************/
            // Check the HSE State
            /*************************************************************************/
            if(RCC_OscInitStruct->HSEState != RCC_HSE_OFF)
            {
                /* Wait till oscillator is ready */
                DELAY_LOOP(RCC_DELAY);
            }
            else
            {
                /* Wait till oscillator is ready */
                DELAY_LOOP(RCC_DELAY);
            }
        }
    }

    /*************************************************************************/
    /*                         HSI Configuration                             */
    /*************************************************************************/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
    {

        /*********************************************************************/
        // Check if HSI is used as system clock or as PLL source when PLL is selected as system clock
        /*********************************************************************/
        if((sysclk_source == RCC_CFGR_SWS_HSI) ||
           ((sysclk_source == RCC_CFGR_SWS_PLL) && (pll_config == RCC_PLLSOURCE_HSI)))
        {

            /*****************************************************************/
            // When HSI is used as system clock it will not be disabled
            /*****************************************************************/
            if((READ_BIT(RCC->CR, RCC_CR_HSIRDY) != 0U) && (RCC_OscInitStruct->HSIState == RCC_HSI_OFF))
            {
                return AC_FAIL;
            }

            /*****************************************************************/
            // Otherwise, just the calibration is allowed 
            /*****************************************************************/
            else
            {
                /*************************************************************/
                // Adjusts the Internal High Speed oscillator (HSI) calibration value.
                /*************************************************************/
                AC_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
            }
        }
        else
        {
            /*****************************************************************/
            // Check the HSI State
            /*****************************************************************/
            if(RCC_OscInitStruct->HSIState != RCC_HSI_OFF)
            {
                /*************************************************************/
                // Enable the Internal High Speed oscillator (HSI).
                /*************************************************************/
                AC_RCC_HSI_ENABLE();

                /* Wait till oscillator is ready */
                DELAY_LOOP(RCC_DELAY);
                
                /*************************************************************/
                // Adjusts the Internal High Speed oscillator (HSI) calibration value.
                /*************************************************************/
                AC_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
            }

            else
            {
                /*************************************************************/
                // Disable the Internal High Speed oscillator (HSI).
                /*************************************************************/
                AC_RCC_HSI_DISABLE();

                /* Wait till oscillator is ready */
                DELAY_LOOP(RCC_DELAY);
                
            }
        }
    }
    
    /*************************************************************************/
    /*                         LSI Configuration                             */
    /*************************************************************************/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
    {

        /*********************************************************************/
        // Check the LSI State
        /*********************************************************************/
        if(RCC_OscInitStruct->LSIState != RCC_LSI_OFF)
        {

            uint32_t csr_temp = RCC->CSR;

            if (RCC_OscInitStruct->LSIDiv != (csr_temp & RCC_CSR_LSIPREDIV))
            {
                
                if (((csr_temp & RCC_CSR_LSIRDY) == RCC_CSR_LSIRDY) && \
                    ((csr_temp & RCC_CSR_LSION) != RCC_CSR_LSION))
                {
                    /*********************************************************/
                    // If LSIRDY is set while LSION is not enabled, 
                    // LSIPREDIV can't be updated  
                    /*********************************************************/
                    return AC_FAIL;
                }

                /*************************************************************/
                // Turn off LSI before changing RCC_CSR_LSIPREDIV
                /*************************************************************/
                if ((csr_temp & RCC_CSR_LSION) == RCC_CSR_LSION)
                {
                    AC_RCC_LSI_DISABLE();

                    /* Wait till oscillator is ready */
                    DELAY_LOOP(RCC_DELAY);
                    
                }

                /*************************************************************/
                // Set LSI division factor
                /*************************************************************/
                MODIFY_REG(RCC->CSR, RCC_CSR_LSIPREDIV, RCC_OscInitStruct->LSIDiv);
            }

            /*****************************************************************/
            // Enable the Internal Low Speed oscillator (LSI).
            /*****************************************************************/
            AC_RCC_LSI_ENABLE();

            /* Wait till oscillator is ready */
            DELAY_LOOP(RCC_DELAY);
            
        }
        else
        {
            /*****************************************************************/
            // Disable the Internal Low Speed oscillator (LSI).
            /*****************************************************************/
            AC_RCC_LSI_DISABLE();

            /* Wait till oscillator is ready */
            DELAY_LOOP(RCC_DELAY);
            
        }
    }

    /*************************************************************************/
    /*                         LSE Configuration                             */
    /*************************************************************************/
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
    {
        AC_reg_status_t pwrclkchanged = AC_REG_RESET;

        /*********************************************************************/
        // Update LSE configuration in Backup Domain control register   
        // Requires to enable write access to Backup Domain of necessary
        /*********************************************************************/
        if(AC_IS_BIT_CLR(RCC->APB1ENR1, RCC_APB1ENR1_PWREN))
        {
            AC_RCC_PWR_CLK_ENABLE();
            pwrclkchanged = AC_REG_SET;
        }

        if(AC_IS_BIT_CLR(PWR->CR1, PWR_CR1_DBP))
        {
            
            /*****************************************************************/
            // Enable write access to Backup domain
            /*****************************************************************/
            SET_BIT(PWR->CR1, PWR_CR1_DBP);

            /* Wait for Backup domain Write protection disable */
            DELAY_LOOP(RCC_DELAY);
        }

        /*********************************************************************/
        // Set the new LSE configuration 
        /*********************************************************************/
#if defined(RCC_BDCR_LSESYSDIS)
        if((RCC_OscInitStruct->LSEState & RCC_BDCR_LSEON) != 0U)
        {
            /*****************************************************************/
            // Set LSESYSDIS bit according to LSE propagation option (enabled or disabled) 
            /*****************************************************************/
            MODIFY_REG(RCC->BDCR, RCC_BDCR_LSESYSDIS, (RCC_OscInitStruct->LSEState & RCC_BDCR_LSESYSDIS));

            if((RCC_OscInitStruct->LSEState & RCC_BDCR_LSEBYP) != 0U)
            {
                /*************************************************************/
                // LSE oscillator bypass enable
                /*************************************************************/
                SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
                SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
            }
            else
            {
                /*************************************************************/
                // LSE oscillator enable
                /*************************************************************/
                SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
            }
        }
        else
        {
            CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
            CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
        }
#else
        __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
#endif /* RCC_BDCR_LSESYSDIS */

        /*********************************************************************/
        // Check the LSE State
        /*********************************************************************/
        if(RCC_OscInitStruct->LSEState != RCC_LSE_OFF)
        {

            /* Wait till LSE is ready */
            DELAY_LOOP(RCC_DELAY);
            
        }
        else
        {
            /* Wait till LSE is disabled */
            DELAY_LOOP(RCC_DELAY);
            
#if defined(RCC_BDCR_LSESYSDIS)
            /* By default, stop disabling LSE propagation */
            CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSESYSDIS);
#endif /* RCC_BDCR_LSESYSDIS */
        }

        /* Restore clock configuration if changed */
        if(pwrclkchanged == AC_REG_SET)
        {
            AC_RCC_PWR_CLK_DISABLE();
        }
    }


    /*************************************************************************/
    /*                       HSI48 Configuration                             */
    /*************************************************************************/  
#if defined(RCC_HSI48_SUPPORT)
    if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI48) == RCC_OSCILLATORTYPE_HSI48)
    {

        /*********************************************************************/
        // Check the LSI State
        /*********************************************************************/
        if(RCC_OscInitStruct->HSI48State != RCC_HSI48_OFF)
        {
            /*****************************************************************/
            // Enable the Internal Low Speed oscillator (HSI48).
            /*****************************************************************/
            AC_RCC_HSI48_ENABLE();

            /* Wait till HSI48 is ready */
            DELAY_LOOP(RCC_DELAY);
            
        }
        else
        {
            /*****************************************************************/
            // Disable the Internal Low Speed oscillator (HSI48).
            /*****************************************************************/
            AC_RCC_HSI48_DISABLE();

            /* Wait till HSI48 is disabled */
            DELAY_LOOP(RCC_DELAY);
            
        }
    }
#endif /* RCC_HSI48_SUPPORT */

    /*************************************************************************/
    /*                         PLL Configuration                             */
    /*************************************************************************/
    if(RCC_OscInitStruct->PLL.PLLState != RCC_PLL_NONE)
    {
      
        /*********************************************************************/
        //Check PLL is ON
        /*********************************************************************/
        if(RCC_OscInitStruct->PLL.PLLState == RCC_PLL_ON)
        {

            /*****************************************************************/
            // Do nothing if PLL configuration is the unchanged
            /*****************************************************************/
            pll_config = RCC->PLLCFGR;
            if((READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC)  != RCC_OscInitStruct->PLL.PLLSource) ||
               (READ_BIT(pll_config, RCC_PLLCFGR_PLLM)    != ((RCC_OscInitStruct->PLL.PLLM - 1U) << RCC_PLLCFGR_PLLM_Pos)) ||
               (READ_BIT(pll_config, RCC_PLLCFGR_PLLN)    != (RCC_OscInitStruct->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)) ||
               (READ_BIT(pll_config, RCC_PLLCFGR_PLLQ)    != ((((RCC_OscInitStruct->PLL.PLLQ) >> 1U) - 1U) << RCC_PLLCFGR_PLLQ_Pos)) ||
               (READ_BIT(pll_config, RCC_PLLCFGR_PLLR)    != ((((RCC_OscInitStruct->PLL.PLLR) >> 1U) - 1U) << RCC_PLLCFGR_PLLR_Pos)))
            {
            
                /*************************************************************/
                /* Check if the PLL is used as system clock or not */
                /*************************************************************/
                if(sysclk_source != RCC_CFGR_SWS_PLL)
                {
                    {
                        /*****************************************************/
                        // Disable the main PLL.
                        /*****************************************************/
                        AC_RCC_PLL_DISABLE();

                        /* Wait till PLL is ready */
                        DELAY_LOOP(RCC_DELAY);
                        
                        /*****************************************************/
                        // Configure the main PLL clock source, multiplication
                        // and division factors. 
                        /*****************************************************/
                        AC_RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource,
                                             RCC_OscInitStruct->PLL.PLLM,
                                             RCC_OscInitStruct->PLL.PLLN,
                                             RCC_OscInitStruct->PLL.PLLQ,
                                             RCC_OscInitStruct->PLL.PLLR);

                        /*****************************************************/
                        // Enable the main PLL. 
                        /*****************************************************/
                        AC_RCC_PLL_ENABLE();

                        /*****************************************************/
                        // Enable PLL System Clock output. 
                        /*****************************************************/
                        AC_RCC_PLLCLKOUT_ENABLE(RCC_PLL_SYSCLK);

                        /* Wait till PLL is ready */
                        DELAY_LOOP(RCC_DELAY);
                        
                    }
                }
                else
                {
                    /*********************************************************/
                    /* PLL is already used as System core clock */
                    /*********************************************************/
                    return AC_FAIL;
                }
            }
            else
            {
                /*************************************************************/
                // PLL configuration is unchanged 
                // Re-enable PLL if it was disabled (ie. low power mode)
                /*************************************************************/
                if(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U)
                {
             
                    /*********************************************************/
                    // Enable the main PLL.
                    /*********************************************************/
                    AC_RCC_PLL_ENABLE();

                    /*********************************************************/
                    // Enable PLL System Clock output.
                    /*********************************************************/
                    AC_RCC_PLLCLKOUT_ENABLE(RCC_PLL_SYSCLK);

                    /* Wait till PLL is ready */
                    DELAY_LOOP(RCC_DELAY);
                    
                }
            }
        }
        else
        {

            /*****************************************************************/
            // Check that PLL is not used as system clock or not
            /*****************************************************************/
            if(sysclk_source != RCC_CFGR_SWS_PLL)
            {
                
                /*************************************************************/
                // Disable the main PLL. 
                /*************************************************************/
                AC_RCC_PLL_DISABLE();

                /*************************************************************/
                // Disable all PLL outputs to save power if no PLLs on 
                /*************************************************************/
                MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLSOURCE_NONE);

                AC_RCC_PLLCLKOUT_DISABLE(RCC_PLL_SYSCLK | RCC_PLL_48M1CLK);

                /* Wait till PLL is disabled */
                DELAY_LOOP(RCC_DELAY);
                
            }
            else
            {
                /*************************************************************/
                // PLL is already used as System core clock 
                /*************************************************************/
                return AC_FAIL;
            }
        }
    }
    return AC_OK;
}

/**************************************************************************//**
 * @brief  Initialize the CPU, AHB and APB busses clocks according to the specified
 *         parameters in the RCC_ClkInitStruct.
 * @param  RCC_ClkInitStruct  pointer to an RCC_OscInitTypeDef structure that
 *         contains the configuration information for the RCC peripheral.
 * @param  FLatency  FLASH Latency
 *          This parameter can be one of the following values:
 *            @arg FLASH_LATENCY_0   FLASH 0 Latency cycle
 *            @arg FLASH_LATENCY_1   FLASH 1 Latency cycle
 *            @arg FLASH_LATENCY_2   FLASH 2 Latency cycles
 *            @arg FLASH_LATENCY_3   FLASH 3 Latency cycles
 *            @arg FLASH_LATENCY_4   FLASH 4 Latency cycles
 *
 * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
 *         and updated by HAL_RCC_GetHCLKFreq() function called within this function
 *
 * @note   The MSI is used by default as system clock source after
 *         startup from Reset, wake-up from STANDBY mode. After restart from Reset,
 *         the MSI frequency is set to its default value 4 MHz.
 *
 * @note   The HSI can be selected as system clock source after
 *         from STOP modes or in case of failure of the HSE used directly or indirectly
 *         as system clock (if the Clock Security System CSS is enabled).
 *
 * @note   A switch from one clock source to another occurs only if the target
 *         clock source is ready (clock stable after startup delay or PLL locked).
 *         If a clock source which is not yet ready is selected, the switch will
 *         occur when the clock source is ready.
 *
 * @note   You can use HAL_RCC_GetClockConfig() function to know which clock is
 *         currently used as system clock source.
 *
 * @note   Depending on the device voltage range, the software has to set correctly
 *         HPRE[3:0] bits to ensure that HCLK not exceed the maximum allowed frequency
 *         (for more details refer to section above "Initialization/de-initialization functions")
 * 
 * @retval AC status
 *****************************************************************************/
AC_status_t AC_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{

    /*************************************************************************/
    // Check Null pointer
    /*************************************************************************/
    if(RCC_ClkInitStruct == NULL)
    {
        return AC_FAIL;
    }

    /*************************************************************************/
    // To correctly read data from FLASH memory, the number of wait states (LATENCY)
    // must be correctly programmed according to the frequency of the CPU clock
    // (HCLK) and the supply voltage of the device. */
    /*************************************************************************/

    /*************************************************************************/
    // Increasing the number of wait states because of higher CPU frequency
    /*************************************************************************/
    if(FLatency > AC_FLASH_GET_LATENCY())
    {
        
        /*********************************************************************/
        // Program the new number of wait states to the LATENCY bits in the FLASH_ACR register
        /*********************************************************************/
        AC_FLASH_SET_LATENCY(FLatency);

        /*********************************************************************/
        // Check that the new number of wait states is taken into account to access the Flash
        // memory by reading the FLASH_ACR register
        /*********************************************************************/
        if(AC_FLASH_GET_LATENCY() != FLatency)
        {
            return AC_FAIL;
        }
    }

    /*************************************************************************/
    /*                       SYSCLK Configuration                            */
    /*************************************************************************/
    if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
    {

        /*********************************************************************/
        // PLL is selected as System Clock Source
        /*********************************************************************/
        if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
        {
            /*****************************************************************/
            // Check the PLL ready flag
            /*****************************************************************/
            if(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U)
            {
                return AC_FAIL;
            }
        }
        else
        {
            /*****************************************************************/
            // HSE is selected as System Clock Source 
            /*****************************************************************/
            if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
            {
                /* Check the HSE ready flag */
                if(READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0U)
                {
                    return AC_FAIL;
                }
            }

            /*****************************************************************/
            // MSI is selected as System Clock Source
            /*****************************************************************/
            else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_MSI)
            {
                /*************************************************************/
                // Check the MSI ready flag
                /*************************************************************/
                if(READ_BIT(RCC->CR, RCC_CR_MSIRDY) == 0U)
                {
                    return AC_FAIL;
                }
            }

            /*****************************************************************/
            // HSI is selected as System Clock Source
            /*****************************************************************/
            else
            {
                /*************************************************************/
                // Check the HSI ready flag
                /*************************************************************/
                if(READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0U)
                {
                    return AC_FAIL;
                }
            }
        }

        MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_ClkInitStruct->SYSCLKSource);

        DELAY_LOOP(RCC_DELAY);
        
    }

    /*************************************************************************/
    /*                         HCLK Configuration                            */
    /*************************************************************************/
    if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
    {
        MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
    }

    /* Decreasing the number of wait states because of lower CPU frequency */
    if(FLatency < AC_FLASH_GET_LATENCY())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        AC_FLASH_SET_LATENCY(FLatency);

        /* Check that the new number of wait states is taken into account to access the Flash
        memory by reading the FLASH_ACR register */
        if(AC_FLASH_GET_LATENCY() != FLatency)
        {
            return AC_FAIL;
        }
    }

    /*************************************************************************/
    /*                       PCLK1 Configuration                             */
    /*************************************************************************/
    if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
    {
        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
    }

    /*************************************************************************/
    /*                       PCLK2 Configuration                             */
    /*************************************************************************/
    if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
    {
        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3U));
    }

    /*************************************************************************/
    // Update the SystemCoreClock global variable
    /*************************************************************************/
    SystemCoreClock = AC_RCC_GetSysClockFreq() >> (AHBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos] & 0x1FU);

    return AC_OK;
}


/**************************************************************************//**
 * @brief  Return the SYSCLK frequency.
 *
 * @note   The system frequency computed by this function is not the real
 *         frequency in the chip. It is calculated based on the predefined
 *         constant and the selected clock source:
 * @note     If SYSCLK source is MSI, function returns values based on MSI
 *             Value as defined by the MSI range.
 * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
 * @note     If SYSCLK source is HSE, function returns values based on HSE_VALUE(**)
 * @note     If SYSCLK source is PLL, function returns values based on HSE_VALUE(**),
 *           HSI_VALUE(*) or MSI Value multiplied/divided by the PLL factors.
 * @note     (*) HSI_VALUE is a constant defined in stm32l4xx_hal_conf.h file (default value
 *               16 MHz) but the real value may vary depending on the variations
 *               in voltage and temperature.
 * @note     (**) HSE_VALUE is a constant defined in stm32l4xx_hal_conf.h file (default value
 *                8 MHz), user has to ensure that HSE_VALUE is same as the real
 *                frequency of the crystal used. Otherwise, this function may
 *                have wrong result.
 *
 * @note   The result of this function could be not correct when using fractional
 *         value for HSE crystal.
 *
 * @note   This function can be used by the user application to compute the
 *         baudrate for the communication peripherals or configure other parameters.
 *
 * @note   Each time SYSCLK changes, this function must be called to update the
 *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
 *
 *
 * @retval SYSCLK frequency
 *****************************************************************************/
uint32_t AC_RCC_GetSysClockFreq(void)
{
    uint32_t msirange = 0U, sysclockfreq = 0U;
    uint32_t pllvco, pllsource, pllr, pllm;    /* no init needed */
    uint32_t sysclk_source, pll_oscsource;

    sysclk_source = AC_RCC_GET_SYSCLK_SOURCE();
    pll_oscsource = AC_RCC_GET_PLL_OSCSOURCE();

    if((sysclk_source == RCC_CFGR_SWS_MSI) ||
       ((sysclk_source == RCC_CFGR_SWS_PLL) && (pll_oscsource == RCC_PLLSOURCE_MSI)))
    {
        /*********************************************************************/
        // MSI or PLL with MSI source used as system clock source 
        /*************************************************************/

        /*************************************************************/
        // Get SYSCLK source
        /*************************************************************/
        if(READ_BIT(RCC->CR, RCC_CR_MSIRGSEL) == 0U)
        { 
            /* MSISRANGE from RCC_CSR applies */
            msirange = READ_BIT(RCC->CSR, RCC_CSR_MSISRANGE) >> RCC_CSR_MSISRANGE_Pos;
        }
        else
        { 
            /* MSIRANGE from RCC_CR applies */
            msirange = READ_BIT(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
        }
        
        /*************************************************************/
        // MSI frequency range in HZ
        /*************************************************************/
        msirange = MSIRangeTable[msirange];

        if(sysclk_source == RCC_CFGR_SWS_MSI)
        {
            /* MSI used as system clock source */
            sysclockfreq = msirange;
        }
    }
    else if(sysclk_source == RCC_CFGR_SWS_HSI)
    {
        /*************************************************************/
        // HSI used as system clock source 
        /*************************************************************/
        sysclockfreq = HSI_VALUE;
    }
    else if(sysclk_source == RCC_CFGR_SWS_HSE)
    {
        /*************************************************************/
        // HSE used as system clock source
        /*************************************************************/
        sysclockfreq = HSE_VALUE;
    }
    else
    {
        /* unexpected case: sysclockfreq at 0 */
    }

    if(sysclk_source == RCC_CFGR_SWS_PLL)
    {
        /*************************************************************/
        // PLL used as system clock  source 
        /*************************************************************/

        /* PLL_VCO = (HSE_VALUE or HSI_VALUE or MSI_VALUE) * PLLN / PLLM
        SYSCLK = PLL_VCO / PLLR
        */
        pllsource = READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);

        switch (pllsource)
        {
            case RCC_PLLSOURCE_HSI:  /* HSI used as PLL clock source */
              pllvco = HSI_VALUE;
              break;
    
            case RCC_PLLSOURCE_HSE:  /* HSE used as PLL clock source */
              pllvco = HSE_VALUE;
              break;
    
            case RCC_PLLSOURCE_MSI:  /* MSI used as PLL clock source */
            default:
              pllvco = msirange;
              break;
        }
        pllm = (READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U ;
        pllvco = (pllvco * (READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)) / pllm;
        pllr = ((READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1U ) * 2U;
        sysclockfreq = pllvco / pllr;
    }   

    return sysclockfreq;
}

/**************************************************************************//**
  * @brief  Initialize the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit  pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains a field PeriphClockSelection which can be a combination of the following values:
  *            @arg @ref RCC_PERIPHCLK_RTC  RTC peripheral clock
  *            @arg @ref RCC_PERIPHCLK_ADC  ADC peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C1  I2C1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C2  I2C2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C3  I2C3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPTIM1  LPTIM1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPTIM2  LPTIM2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPUART1  LPUART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_RNG  RNG peripheral clock
  *            @arg @ref RCC_PERIPHCLK_SAI1  SAI1 peripheral clock (only for devices with SAI1)
  *            @arg @ref RCC_PERIPHCLK_SDMMC1  SDMMC1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART1  USART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART2  USART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USART3  USART1 peripheral clock
  *
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source: in this case the access to Backup domain is enabled.
  *
  * @retval AC status
 *****************************************************************************/
AC_status_t AC_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tmpregister;
  AC_status_t ret = AC_OK;      /* Intermediate status */

    /*************************************************************************/
    /*                  RTC clock source configuration                       */
    /*************************************************************************/
    if((PeriphClkInit->PeriphClockSelection & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC)
    {
        AC_reg_status_t pwrclkchanged = AC_REG_RESET;

        /*********************************************************************/
        // Enable Power Clock
        /*********************************************************************/
        if(AC_RCC_PWR_IS_CLK_DISABLED() != 0U)
        {
            AC_RCC_PWR_CLK_ENABLE();
            pwrclkchanged = AC_REG_SET;
        }

        /*********************************************************************/
        // Enable write access to Backup domain
        /*********************************************************************/
        SET_BIT(PWR->CR1, PWR_CR1_DBP);

        /* Wait for Backup domain Write protection disable */
        DELAY_LOOP(RCC_DELAY);
        
        if(ret == AC_OK)
        {
            /*****************************************************************/
            // Reset the Backup domain only if 
            // the RTC Clock source selection is modified from default 
            /*****************************************************************/
            tmpregister = READ_BIT(RCC->BDCR, RCC_BDCR_RTCSEL);

            if((tmpregister != RCC_RTCCLKSOURCE_NONE) && (tmpregister != PeriphClkInit->RTCClockSelection))
            {
                /*************************************************************/
                // Store the content of BDCR register before the reset of Backup Domain
                /*************************************************************/
                tmpregister = READ_BIT(RCC->BDCR, ~(RCC_BDCR_RTCSEL));

                /*************************************************************/
                // RTC Clock selection can be changed only if the Backup Domain is reset
                /*************************************************************/
                AC_RCC_BACKUPRESET_FORCE();
                AC_RCC_BACKUPRESET_RELEASE();
                
                /*************************************************************/
                // Restore the Content of BDCR register
                /*************************************************************/
                RCC->BDCR = tmpregister;
            }

            /*****************************************************************/
            // Wait for LSE reactivation if LSE was enable prior to Backup Domain reset
            /*****************************************************************/
            if (AC_IS_BIT_SET(tmpregister, RCC_BDCR_LSEON))
            {
                /* Wait till LSE is ready */
                DELAY_LOOP(RCC_DELAY);
            }

            if(ret == AC_OK)
            {
                /*************************************************************/
                // Apply new RTC clock source selection
                /*************************************************************/
                AC_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
                
            }
            else
            {
                /* set overall return value */
                return AC_FAIL;
            }
        }
        else
        {
            /* set overall return value */
            return AC_FAIL;
        }

        /*********************************************************************/
        // Restore clock configuration if changed
        /*********************************************************************/
        if(pwrclkchanged == AC_REG_SET)
        {
            AC_RCC_PWR_CLK_DISABLE();
        }
    }

    /*************************************************************************/
    /*                 USART1 clock source configuration                     */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART1) == RCC_PERIPHCLK_USART1)
    {
        AC_RCC_USART1_CONFIG(PeriphClkInit->Usart1ClockSelection);
    }

    /*************************************************************************/
    /*                 USART2 clock source configuration                     */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART2) == RCC_PERIPHCLK_USART2)
    {
        AC_RCC_USART2_CONFIG(PeriphClkInit->Usart2ClockSelection);
    }

    /*************************************************************************/
    /*                 USART3 clock source configuration                     */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART3) == RCC_PERIPHCLK_USART3)
    {
        AC_RCC_USART3_CONFIG(PeriphClkInit->Usart3ClockSelection);
    }

    /*************************************************************************/
    /*                LPUART1 clock source configuration                     */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPUART1) == RCC_PERIPHCLK_LPUART1)
    {
        AC_RCC_LPUART1_CONFIG(PeriphClkInit->Lpuart1ClockSelection);
    }

    /*************************************************************************/
    /*                LPTIM1 clock source configuration                      */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM1) == (RCC_PERIPHCLK_LPTIM1))
    {
        AC_RCC_LPTIM1_CONFIG(PeriphClkInit->Lptim1ClockSelection);
    }

    /*************************************************************************/
    /*                LPTIM2 clock source configuration                      */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM2) == (RCC_PERIPHCLK_LPTIM2))
    {
        AC_RCC_LPTIM2_CONFIG(PeriphClkInit->Lptim2ClockSelection);
    }

    /*************************************************************************/
    /*                 I2C1 clock source configuration                       */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C1) == RCC_PERIPHCLK_I2C1)
    {
        AC_RCC_I2C1_CONFIG(PeriphClkInit->I2c1ClockSelection);
    }

    /*************************************************************************/
    /*                 I2C2 clock source configuration                       */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C2) == RCC_PERIPHCLK_I2C2)
    {
        AC_RCC_I2C2_CONFIG(PeriphClkInit->I2c2ClockSelection);
    }

    /*************************************************************************/
    /*                 I2C3 clock source configuration                       */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C3) == RCC_PERIPHCLK_I2C3)
    {
        AC_RCC_I2C3_CONFIG(PeriphClkInit->I2c3ClockSelection);
    }

    /*************************************************************************/
    /*                  USB clock source configuration                       */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == (RCC_PERIPHCLK_USB))
    {
        AC_RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);

        if(PeriphClkInit->UsbClockSelection == RCC_USBCLKSOURCE_PLL)
        {
            /* Enable PLL48M1CLK output clock */
            AC_RCC_PLLCLKOUT_ENABLE(RCC_PLL_48M1CLK);
        }
    }

    /*************************************************************************/
    /*                  RNG clock source configuration                       */
    /*************************************************************************/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RNG) == (RCC_PERIPHCLK_RNG))
    {
        AC_RCC_RNG_CONFIG(PeriphClkInit->RngClockSelection);

        if(PeriphClkInit->RngClockSelection == RCC_RNGCLKSOURCE_PLL)
        {
          /* Enable PLL48M1CLK output clock */
          AC_RCC_PLLCLKOUT_ENABLE(RCC_PLL_48M1CLK);
        }
    }

  return AC_OK;
}

/*****************************************************************************/
/*****************************************************************************/
/*                      LOCAL FUNCTIONS IMPLEMENTATION                       */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief  Update number of Flash wait states in line with MSI range and current
            voltage range.
 * @param  msirange  MSI range value from RCC_MSIRANGE_0 to RCC_MSIRANGE_11
 * @return  AC_OK on success or AC_ERR_[ERROR] otherwise
 *****************************************************************************/
static AC_status_t RCC_SetFlashLatencyFromMSIRange_L(uint32_t msirange)
{
    uint32_t vos;
    uint32_t latency = FLASH_LATENCY_0;  /* default value 0WS */

    if(AC_RCC_PWR_IS_CLK_ENABLED())
    {
        vos = (PWR->CR1 & PWR_CR1_VOS); /* Return Voltage Scaling Range */
    }
    else
    {
        AC_RCC_PWR_CLK_ENABLE();
        vos = (PWR->CR1 & PWR_CR1_VOS); /* Return Voltage Scaling Range */
        AC_RCC_PWR_CLK_DISABLE();
    }

    if(vos == PWR_REGULATOR_VOLTAGE_SCALE1)
    {
        if(msirange > RCC_MSIRANGE_8)
        {
            /* MSI > 16Mhz */
            if(msirange > RCC_MSIRANGE_10)
            {
                /* MSI 48Mhz */
                latency = FLASH_LATENCY_2; /* 2WS */
            }
            else
            {
                /* MSI 24Mhz or 32Mhz */
                latency = FLASH_LATENCY_1; /* 1WS */
            }
        }
        /* else MSI <= 16Mhz default FLASH_LATENCY_0 0WS */
    }
    else
    {
        if(msirange > RCC_MSIRANGE_8)
        {
              /* MSI > 16Mhz */
              latency = FLASH_LATENCY_3; /* 3WS */
        }
        else
        {
              if(msirange == RCC_MSIRANGE_8)
              {
                    /* MSI 16Mhz */
                    latency = FLASH_LATENCY_2; /* 2WS */
              }
              else if(msirange == RCC_MSIRANGE_7)
              {
                    /* MSI 8Mhz */
                    latency = FLASH_LATENCY_1; /* 1WS */
              }
              /* else MSI < 8Mhz default FLASH_LATENCY_0 0WS */
        }
    }

    AC_FLASH_SET_LATENCY(latency);

    /* Check that the new number of wait states is taken into account to access the Flash
       memory by reading the FLASH_ACR register */
    if(AC_FLASH_GET_LATENCY() != latency)
    {
        return AC_FAIL;
    }

    return AC_OK;
}