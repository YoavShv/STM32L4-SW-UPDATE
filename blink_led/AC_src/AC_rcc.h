/**************************************************************************//**
 * @file      AC_rcc.h
 * @brief     Reset & Clock Controller - Header file
 * @details   
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/
#ifndef _AC_RCC_H_
#define _AC_RCC_H_

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include "AC_defs.h"
#include <stddef.h>

/*****************************************************************************/
/*****************************************************************************/
/*                                CONSTANTS                                  */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
// RCC_PLL_Config PLL Config
/*****************************************************************************/
#define RCC_PLL_NONE                   0x00000000U   /*!< PLL configuration unchanged */
#define RCC_PLL_OFF                    0x00000001U   /*!< PLL deactivation */
#define RCC_PLL_ON                     0x00000002U   /*!< PLL activation */

/*****************************************************************************/
// RCC_PLL_Clock_Source PLL Clock Source
/*****************************************************************************/
#define RCC_PLLSOURCE_NONE             0x00000000U             /*!< No clock selected as PLL entry clock source  */
#define RCC_PLLSOURCE_MSI              RCC_PLLCFGR_PLLSRC_MSI  /*!< MSI clock selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSI              RCC_PLLCFGR_PLLSRC_HSI  /*!< HSI clock selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSE              RCC_PLLCFGR_PLLSRC_HSE  /*!< HSE clock selected as PLL entry clock source */

/*****************************************************************************/
// RCC_PLLQ_Clock_Divider PLLQ Clock Divider
/*****************************************************************************/
#define RCC_PLLQ_DIV2                  0x00000002U   /*!< PLLQ division factor = 2 */
#define RCC_PLLQ_DIV4                  0x00000004U   /*!< PLLQ division factor = 4 */
#define RCC_PLLQ_DIV6                  0x00000006U   /*!< PLLQ division factor = 6 */
#define RCC_PLLQ_DIV8                  0x00000008U   /*!< PLLQ division factor = 8 */

/*****************************************************************************/
// RCC_PLLR_Clock_Divider PLLR Clock Divider
/*****************************************************************************/  
#define RCC_PLLR_DIV2                  0x00000002U   /*!< PLLR division factor = 2 */
#define RCC_PLLR_DIV4                  0x00000004U   /*!< PLLR division factor = 4 */
#define RCC_PLLR_DIV6                  0x00000006U   /*!< PLLR division factor = 6 */
#define RCC_PLLR_DIV8                  0x00000008U   /*!< PLLR division factor = 8 */

/*****************************************************************************/
// RCC_Oscillator_Type Oscillator Type
/*****************************************************************************/
#define RCC_OSCILLATORTYPE_NONE        0x00000000U   /*!< Oscillator configuration unchanged */
#define RCC_OSCILLATORTYPE_HSE         0x00000001U   /*!< HSE to configure */
#define RCC_OSCILLATORTYPE_HSI         0x00000002U   /*!< HSI to configure */
#define RCC_OSCILLATORTYPE_LSE         0x00000004U   /*!< LSE to configure */
#define RCC_OSCILLATORTYPE_LSI         0x00000008U   /*!< LSI to configure */
#define RCC_OSCILLATORTYPE_MSI         0x00000010U   /*!< MSI to configure */
#define RCC_OSCILLATORTYPE_HSI48       0x00000020U   /*!< HSI48 to configure */

/*****************************************************************************/
// RCC_HSE_Config HSE Config
/*****************************************************************************/
#define RCC_HSE_OFF                    0x00000000U                    /*!< HSE clock deactivation */
#define RCC_HSE_ON                     RCC_CR_HSEON                   /*!< HSE clock activation */
#define RCC_HSE_BYPASS                 (RCC_CR_HSEBYP | RCC_CR_HSEON) /*!< External clock source for HSE clock */

/*****************************************************************************/
// RCC_LSE_Config LSE Config
/*****************************************************************************/  
#define RCC_LSE_OFF                    0x00000000U                           /*!< LSE clock deactivation */
#define RCC_LSE_ON                     RCC_BDCR_LSEON                        /*!< LSE clock activation */
#define RCC_LSE_BYPASS                 (RCC_BDCR_LSEBYP | RCC_BDCR_LSEON)    /*!< External clock source for LSE clock */
#define RCC_LSE_ON_RTC_ONLY            (RCC_BDCR_LSESYSDIS | RCC_BDCR_LSEON) /*!< LSE clock activation without propagation to system */
#define RCC_LSE_BYPASS_RTC_ONLY        (RCC_BDCR_LSEBYP | RCC_BDCR_LSESYSDIS | RCC_BDCR_LSEON) /*!< External clock source for LSE clock without propagation to system */

/*****************************************************************************/
// RCC_HSI_Config HSI Config
/*****************************************************************************/
#define RCC_HSI_OFF                    0x00000000U   /*!< HSI clock deactivation */
#define RCC_HSI_ON                     RCC_CR_HSION  /*!< HSI clock activation */

#define RCC_HSICALIBRATION_DEFAULT     0x40U         /*!< Default HSI calibration trimming value 64 on devices other than STM32L47x/STM32L48x */

/*****************************************************************************/
// RCC_LSI_Config LSI Config
/*****************************************************************************/
#define RCC_LSI_OFF                    0x00000000U   /*!< LSI clock deactivation */
#define RCC_LSI_ON                     RCC_CSR_LSION /*!< LSI clock activation */

/*****************************************************************************/
// @defgroup RCC_LSI_Div LSI Div
/*****************************************************************************/
#define RCC_LSI_DIV1                   0x00000000U          /*!< LSI clock not divided    */
#define RCC_LSI_DIV128                 RCC_CSR_LSIPREDIV    /*!< LSI clock divided by 128 */

/*****************************************************************************/
// RCC_MSI_Config MSI Config
/*****************************************************************************/
#define RCC_MSI_OFF                    0x00000000U   /*!< MSI clock deactivation */
#define RCC_MSI_ON                     RCC_CR_MSION  /*!< MSI clock activation */

#define RCC_MSICALIBRATION_DEFAULT     0U            /*!< Default MSI calibration trimming value */

/*****************************************************************************/
// RCC_MSI_Clock_Range MSI Clock Range
/*****************************************************************************/
#define RCC_MSIRANGE_0                 RCC_CR_MSIRANGE_0  /*!< MSI = 100 KHz  */
#define RCC_MSIRANGE_1                 RCC_CR_MSIRANGE_1  /*!< MSI = 200 KHz  */
#define RCC_MSIRANGE_2                 RCC_CR_MSIRANGE_2  /*!< MSI = 400 KHz  */
#define RCC_MSIRANGE_3                 RCC_CR_MSIRANGE_3  /*!< MSI = 800 KHz  */
#define RCC_MSIRANGE_4                 RCC_CR_MSIRANGE_4  /*!< MSI = 1 MHz    */
#define RCC_MSIRANGE_5                 RCC_CR_MSIRANGE_5  /*!< MSI = 2 MHz    */
#define RCC_MSIRANGE_6                 RCC_CR_MSIRANGE_6  /*!< MSI = 4 MHz    */
#define RCC_MSIRANGE_7                 RCC_CR_MSIRANGE_7  /*!< MSI = 8 MHz    */
#define RCC_MSIRANGE_8                 RCC_CR_MSIRANGE_8  /*!< MSI = 16 MHz   */
#define RCC_MSIRANGE_9                 RCC_CR_MSIRANGE_9  /*!< MSI = 24 MHz   */
#define RCC_MSIRANGE_10                RCC_CR_MSIRANGE_10 /*!< MSI = 32 MHz   */
#define RCC_MSIRANGE_11                RCC_CR_MSIRANGE_11 /*!< MSI = 48 MHz   */

/*****************************************************************************/
// RCC_HSI48_Config HSI48 Config
/*****************************************************************************/
#define RCC_HSI48_OFF                  0x00000000U       /*!< HSI48 clock deactivation */
#define RCC_HSI48_ON                   RCC_CRRCR_HSI48ON /*!< HSI48 clock activation */

/*****************************************************************************/
// RCC_System_Clock_Type System Clock Type
/*****************************************************************************/
#define RCC_CLOCKTYPE_SYSCLK           0x00000001U   /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK             0x00000002U   /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1            0x00000004U   /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2            0x00000008U   /*!< PCLK2 to configure */

/*****************************************************************************/
// RCC_System_Clock_Source System Clock Source
/*****************************************************************************/
#define RCC_SYSCLKSOURCE_MSI           RCC_CFGR_SW_MSI    /*!< MSI selection as system clock */
#define RCC_SYSCLKSOURCE_HSI           RCC_CFGR_SW_HSI    /*!< HSI selection as system clock */
#define RCC_SYSCLKSOURCE_HSE           RCC_CFGR_SW_HSE    /*!< HSE selection as system clock */
#define RCC_SYSCLKSOURCE_PLLCLK        RCC_CFGR_SW_PLL    /*!< PLL selection as system clock */

/*****************************************************************************/
// RCC_AHB_Clock_Source AHB Clock Source
/*****************************************************************************/
#define RCC_SYSCLK_DIV1                RCC_CFGR_HPRE_DIV1   /*!< SYSCLK not divided */
#define RCC_SYSCLK_DIV2                RCC_CFGR_HPRE_DIV2   /*!< SYSCLK divided by 2 */
#define RCC_SYSCLK_DIV4                RCC_CFGR_HPRE_DIV4   /*!< SYSCLK divided by 4 */
#define RCC_SYSCLK_DIV8                RCC_CFGR_HPRE_DIV8   /*!< SYSCLK divided by 8 */
#define RCC_SYSCLK_DIV16               RCC_CFGR_HPRE_DIV16  /*!< SYSCLK divided by 16 */
#define RCC_SYSCLK_DIV64               RCC_CFGR_HPRE_DIV64  /*!< SYSCLK divided by 64 */
#define RCC_SYSCLK_DIV128              RCC_CFGR_HPRE_DIV128 /*!< SYSCLK divided by 128 */
#define RCC_SYSCLK_DIV256              RCC_CFGR_HPRE_DIV256 /*!< SYSCLK divided by 256 */
#define RCC_SYSCLK_DIV512              RCC_CFGR_HPRE_DIV512 /*!< SYSCLK divided by 512 */

/*****************************************************************************/
// RCC_APB1_APB2_Clock_Source APB1 APB2 Clock Source
/*****************************************************************************/
#define RCC_HCLK_DIV1                  RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                  RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define RCC_HCLK_DIV4                  RCC_CFGR_PPRE1_DIV4  /*!< HCLK divided by 4 */
#define RCC_HCLK_DIV8                  RCC_CFGR_PPRE1_DIV8  /*!< HCLK divided by 8 */
#define RCC_HCLK_DIV16                 RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */

/*****************************************************************************/
// RCC_RTC_Clock_Source RTC Clock Source
/*****************************************************************************/
#define RCC_RTCCLKSOURCE_NONE          0x00000000U          /*!< No clock used as RTC clock */
#define RCC_RTCCLKSOURCE_LSE           RCC_BDCR_RTCSEL_0    /*!< LSE oscillator clock used as RTC clock */
#define RCC_RTCCLKSOURCE_LSI           RCC_BDCR_RTCSEL_1    /*!< LSI oscillator clock used as RTC clock */
#define RCC_RTCCLKSOURCE_HSE_DIV32     RCC_BDCR_RTCSEL      /*!< HSE oscillator clock divided by 32 used as RTC clock */

/*****************************************************************************/
// RCC_PLL_Clock_Output PLL Clock Output
/*****************************************************************************/
#define RCC_PLL_48M1CLK                RCC_PLLCFGR_PLLQEN      /*!< PLL48M1CLK selection from main PLL */
#define RCC_PLL_SYSCLK                 RCC_PLLCFGR_PLLREN      /*!< PLLCLK selection from main PLL */

/*****************************************************************************/
// RCC_LSEDrive_Config LSE Drive Config
/*****************************************************************************/
#define RCC_LSEDRIVE_LOW                 0x00000000U        /*!< LSE low drive capability */
#define RCC_LSEDRIVE_MEDIUMLOW           RCC_BDCR_LSEDRV_0  /*!< LSE medium low drive capability */
#define RCC_LSEDRIVE_MEDIUMHIGH          RCC_BDCR_LSEDRV_1  /*!< LSE medium high drive capability */
#define RCC_LSEDRIVE_HIGH                RCC_BDCR_LSEDRV    /*!< LSE high drive capability */

/*****************************************************************************/
// RCCEx_Periph_Clock_Selection Periph Clock Selection
/*****************************************************************************/
#define RCC_PERIPHCLK_USART1           0x00000001U
#define RCC_PERIPHCLK_USART2           0x00000002U
#define RCC_PERIPHCLK_USART3           0x00000004U
#define RCC_PERIPHCLK_LPUART1          0x00000020U
#define RCC_PERIPHCLK_I2C1             0x00000040U
#define RCC_PERIPHCLK_I2C2             0x00000080U
#define RCC_PERIPHCLK_I2C3             0x00000100U
#define RCC_PERIPHCLK_LPTIM1           0x00000200U
#define RCC_PERIPHCLK_LPTIM2           0x00000400U
#define RCC_PERIPHCLK_USB              0x00002000U
#define RCC_PERIPHCLK_ADC              0x00004000U
#define RCC_PERIPHCLK_RTC              0x00020000U
#define RCC_PERIPHCLK_RNG              0x00040000U

/*****************************************************************************/
// RCCEx_USART1_Clock_Source USART1 Clock Source
/*****************************************************************************/
#define RCC_USART1CLKSOURCE_PCLK2      0x00000000U
#define RCC_USART1CLKSOURCE_SYSCLK     RCC_CCIPR_USART1SEL_0
#define RCC_USART1CLKSOURCE_HSI        RCC_CCIPR_USART1SEL_1
#define RCC_USART1CLKSOURCE_LSE        (RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1)

/*****************************************************************************/
// RCCEx_USART2_Clock_Source USART2 Clock Source
/*****************************************************************************/
#define RCC_USART2CLKSOURCE_PCLK1      0x00000000U
#define RCC_USART2CLKSOURCE_SYSCLK     RCC_CCIPR_USART2SEL_0
#define RCC_USART2CLKSOURCE_HSI        RCC_CCIPR_USART2SEL_1
#define RCC_USART2CLKSOURCE_LSE        (RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1)

/*****************************************************************************/
// RCCEx_USART3_Clock_Source USART3 Clock Source
/*****************************************************************************/
#define RCC_USART3CLKSOURCE_PCLK1      0x00000000U
#define RCC_USART3CLKSOURCE_SYSCLK     RCC_CCIPR_USART3SEL_0
#define RCC_USART3CLKSOURCE_HSI        RCC_CCIPR_USART3SEL_1
#define RCC_USART3CLKSOURCE_LSE        (RCC_CCIPR_USART3SEL_0 | RCC_CCIPR_USART3SEL_1)

/*****************************************************************************/
// RCCEx_LPUART1_Clock_Source LPUART1 Clock Source
/*****************************************************************************/
#define RCC_LPUART1CLKSOURCE_PCLK1     0x00000000U
#define RCC_LPUART1CLKSOURCE_SYSCLK    RCC_CCIPR_LPUART1SEL_0
#define RCC_LPUART1CLKSOURCE_HSI       RCC_CCIPR_LPUART1SEL_1
#define RCC_LPUART1CLKSOURCE_LSE       (RCC_CCIPR_LPUART1SEL_0 | RCC_CCIPR_LPUART1SEL_1)

/*****************************************************************************/
// RCCEx_I2C1_Clock_Source I2C1 Clock Source
/*****************************************************************************/
#define RCC_I2C1CLKSOURCE_PCLK1        0x00000000U
#define RCC_I2C1CLKSOURCE_SYSCLK       RCC_CCIPR_I2C1SEL_0
#define RCC_I2C1CLKSOURCE_HSI          RCC_CCIPR_I2C1SEL_1

/*****************************************************************************/
// RCCEx_I2C2_Clock_Source I2C2 Clock Source
/*****************************************************************************/
#define RCC_I2C2CLKSOURCE_PCLK1        0x00000000U
#define RCC_I2C2CLKSOURCE_SYSCLK       RCC_CCIPR_I2C2SEL_0
#define RCC_I2C2CLKSOURCE_HSI          RCC_CCIPR_I2C2SEL_1

/*****************************************************************************/
// RCCEx_I2C3_Clock_Source I2C3 Clock Source
/*****************************************************************************/
#define RCC_I2C3CLKSOURCE_PCLK1        0x00000000U
#define RCC_I2C3CLKSOURCE_SYSCLK       RCC_CCIPR_I2C3SEL_0
#define RCC_I2C3CLKSOURCE_HSI          RCC_CCIPR_I2C3SEL_1

/*****************************************************************************/
// RCCEx_LPTIM1_Clock_Source LPTIM1 Clock Source
/*****************************************************************************/
#define RCC_LPTIM1CLKSOURCE_PCLK1      0x00000000U
#define RCC_LPTIM1CLKSOURCE_LSI        RCC_CCIPR_LPTIM1SEL_0
#define RCC_LPTIM1CLKSOURCE_HSI        RCC_CCIPR_LPTIM1SEL_1
#define RCC_LPTIM1CLKSOURCE_LSE        RCC_CCIPR_LPTIM1SEL

/*****************************************************************************/
// RCCEx_LPTIM2_Clock_Source LPTIM2 Clock Source
/*****************************************************************************/
#define RCC_LPTIM2CLKSOURCE_PCLK1      0x00000000U
#define RCC_LPTIM2CLKSOURCE_LSI        RCC_CCIPR_LPTIM2SEL_0
#define RCC_LPTIM2CLKSOURCE_HSI        RCC_CCIPR_LPTIM2SEL_1
#define RCC_LPTIM2CLKSOURCE_LSE        RCC_CCIPR_LPTIM2SEL

/*****************************************************************************/
// RCCEx_RNG_Clock_Source RNG Clock Source
/*****************************************************************************/
#define RCC_RNGCLKSOURCE_HSI48         0x00000000U
#define RCC_RNGCLKSOURCE_PLL           RCC_CCIPR_CLK48SEL_1
#define RCC_RNGCLKSOURCE_MSI           RCC_CCIPR_CLK48SEL

/*****************************************************************************/
// RCCEx_USB_Clock_Source USB Clock Source
/*****************************************************************************/
#define RCC_USBCLKSOURCE_HSI48         0x00000000U
#define RCC_USBCLKSOURCE_PLL           RCC_CCIPR_CLK48SEL_1
#define RCC_USBCLKSOURCE_MSI           RCC_CCIPR_CLK48SEL

/*****************************************************************************/
// RCCEx_ADC_Clock_Source ADC Clock Source
/*****************************************************************************/
#define RCC_ADCCLKSOURCE_NONE         0x00000000U
#define RCC_ADCCLKSOURCE_SYSCLK       0x30000000U

/*****************************************************************************/
// RCCEx_EXTI_LINE_LSECSS  RCC LSE CSS external interrupt line
/*****************************************************************************/
#define RCC_EXTI_LINE_LSECSS           EXTI_IMR1_IM19        /*!< External interrupt line 19 connected to the LSE CSS EXTI Line */

/*****************************************************************************/
// RCCEx_CRS_Status RCCEx CRS Status
/*****************************************************************************/
#define RCC_CRS_NONE                   0x00000000U
#define RCC_CRS_TIMEOUT                0x00000001U
#define RCC_CRS_SYNCOK                 0x00000002U
#define RCC_CRS_SYNCWARN               0x00000004U
#define RCC_CRS_SYNCERR                0x00000008U
#define RCC_CRS_SYNCMISS               0x00000010U
#define RCC_CRS_TRIMOVF                0x00000020U

/*****************************************************************************/
// RCCEx_CRS_SynchroSource RCCEx CRS SynchroSource
/*****************************************************************************/
#define RCC_CRS_SYNC_SOURCE_GPIO       0x00000000U             /*!< Synchro Signal source GPIO */
#define RCC_CRS_SYNC_SOURCE_LSE        CRS_CFGR_SYNCSRC_0      /*!< Synchro Signal source LSE */
#define RCC_CRS_SYNC_SOURCE_USB        CRS_CFGR_SYNCSRC_1      /*!< Synchro Signal source USB SOF (default)*/

/*****************************************************************************/
// RCCEx_CRS_SynchroDivider RCCEx CRS SynchroDivider
/*****************************************************************************/
#define RCC_CRS_SYNC_DIV1        0x00000000U                               /*!< Synchro Signal not divided (default) */
#define RCC_CRS_SYNC_DIV2        CRS_CFGR_SYNCDIV_0                        /*!< Synchro Signal divided by 2 */
#define RCC_CRS_SYNC_DIV4        CRS_CFGR_SYNCDIV_1                        /*!< Synchro Signal divided by 4 */
#define RCC_CRS_SYNC_DIV8        (CRS_CFGR_SYNCDIV_1 | CRS_CFGR_SYNCDIV_0) /*!< Synchro Signal divided by 8 */
#define RCC_CRS_SYNC_DIV16       CRS_CFGR_SYNCDIV_2                        /*!< Synchro Signal divided by 16 */
#define RCC_CRS_SYNC_DIV32       (CRS_CFGR_SYNCDIV_2 | CRS_CFGR_SYNCDIV_0) /*!< Synchro Signal divided by 32 */
#define RCC_CRS_SYNC_DIV64       (CRS_CFGR_SYNCDIV_2 | CRS_CFGR_SYNCDIV_1) /*!< Synchro Signal divided by 64 */
#define RCC_CRS_SYNC_DIV128      CRS_CFGR_SYNCDIV                          /*!< Synchro Signal divided by 128 */

/*****************************************************************************/
// RCCEx_CRS_SynchroPolarity RCCEx CRS SynchroPolarity
/*****************************************************************************/
#define RCC_CRS_SYNC_POLARITY_RISING   0x00000000U         /*!< Synchro Active on rising edge (default) */
#define RCC_CRS_SYNC_POLARITY_FALLING  CRS_CFGR_SYNCPOL    /*!< Synchro Active on falling edge */

/*****************************************************************************/
// @defgroup RCCEx_CRS_ReloadValueDefault RCCEx CRS ReloadValueDefault
/*****************************************************************************/
#define RCC_CRS_RELOADVALUE_DEFAULT    0x0000BB7FU   /*!< The reset value of the RELOAD field corresponds
                                                          to a target frequency of 48 MHz and a synchronization signal frequency of 1 kHz (SOF signal from USB). */
/*****************************************************************************/
// @defgroup RCCEx_CRS_ErrorLimitDefault RCCEx CRS ErrorLimitDefault
/*****************************************************************************/  
#define RCC_CRS_ERRORLIMIT_DEFAULT     0x00000022U   /*!< Default Frequency error limit */

/*****************************************************************************/  
// RCCEx_CRS_HSI48CalibrationDefault RCCEx CRS HSI48CalibrationDefault
/*****************************************************************************/  
#define RCC_CRS_HSI48CALIBRATION_DEFAULT 0x00000040U /*!< The default value is 64, which corresponds to the middle of the trimming interval.
                                                          The trimming step is specified in the product datasheet. A higher TRIM value
                                                          corresponds to a higher output frequency */

/*****************************************************************************/  
// RCCEx_CRS_FreqErrorDirection RCCEx CRS FreqErrorDirection
/*****************************************************************************/  
#define RCC_CRS_FREQERRORDIR_UP        0x00000000U   /*!< Upcounting direction, the actual frequency is above the target */
#define RCC_CRS_FREQERRORDIR_DOWN      CRS_ISR_FEDIR /*!< Downcounting direction, the actual frequency is below the target */

/*****************************************************************************/  
// RCCEx_CRS_Interrupt_Sources RCCEx CRS Interrupt Sources
/*****************************************************************************/  
#define RCC_CRS_IT_SYNCOK              CRS_CR_SYNCOKIE       /*!< SYNC event OK */
#define RCC_CRS_IT_SYNCWARN            CRS_CR_SYNCWARNIE     /*!< SYNC warning */
#define RCC_CRS_IT_ERR                 CRS_CR_ERRIE          /*!< Error */
#define RCC_CRS_IT_ESYNC               CRS_CR_ESYNCIE        /*!< Expected SYNC */
#define RCC_CRS_IT_SYNCERR             CRS_CR_ERRIE          /*!< SYNC error */
#define RCC_CRS_IT_SYNCMISS            CRS_CR_ERRIE          /*!< SYNC missed */
#define RCC_CRS_IT_TRIMOVF             CRS_CR_ERRIE           /*!< Trimming overflow or underflow */

/*****************************************************************************/  
// RCCEx_CRS_Flags RCCEx CRS Flags
/*****************************************************************************/  
#define RCC_CRS_FLAG_SYNCOK            CRS_ISR_SYNCOKF       /*!< SYNC event OK flag     */
#define RCC_CRS_FLAG_SYNCWARN          CRS_ISR_SYNCWARNF     /*!< SYNC warning flag      */
#define RCC_CRS_FLAG_ERR               CRS_ISR_ERRF          /*!< Error flag        */
#define RCC_CRS_FLAG_ESYNC             CRS_ISR_ESYNCF        /*!< Expected SYNC flag     */
#define RCC_CRS_FLAG_SYNCERR           CRS_ISR_SYNCERR       /*!< SYNC error */
#define RCC_CRS_FLAG_SYNCMISS          CRS_ISR_SYNCMISS      /*!< SYNC missed*/
#define RCC_CRS_FLAG_TRIMOVF           CRS_ISR_TRIMOVF       /*!< Trimming overflow or underflow */

/*****************************************************************************/
/*****************************************************************************/
/*                                  TYPES                                    */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
//  RCC PLL configuration structure definition
/*****************************************************************************/
typedef struct
{
    uint32_t PLLState;   /*!< The new state of the PLL.
                              This parameter can be a value of @ref RCC_PLL_Config                      */

    uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                              This parameter must be a value of @ref RCC_PLL_Clock_Source               */

    uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                              This parameter must be a number between Min_Data = 1 and Max_Data = 16 on STM32L4Rx/STM32L4Sx devices.
                              This parameter must be a number between Min_Data = 1 and Max_Data = 8 on the other devices */

    uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                              This parameter must be a number between Min_Data = 8 and Max_Data = 86    */

    uint32_t PLLQ;       /*!< PLLQ: Division factor for SDMMC1, RNG and USB clocks.
                              This parameter must be a value of @ref RCC_PLLQ_Clock_Divider             */

    uint32_t PLLR;       /*!< PLLR: Division for the main system clock.
                              User have to set the PLLR parameter correctly to not exceed max frequency 120MHZ
                              on STM32L4Rx/STM32L4Sx devices else 80MHz on the other devices.
                              This parameter must be a value of @ref RCC_PLLR_Clock_Divider             */

}RCC_PLLInitTypeDef;

/*****************************************************************************/
// RCC Internal/External Oscillator (HSE, HSI, MSI, LSE and LSI) configuration structure definition
/*****************************************************************************/
typedef struct
{
    uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                        This parameter can be a value of @ref RCC_Oscillator_Type                   */

    uint32_t HSEState;             /*!< The new state of the HSE.
                                        This parameter can be a value of @ref RCC_HSE_Config                        */

    uint32_t LSEState;             /*!< The new state of the LSE.
                                        This parameter can be a value of @ref RCC_LSE_Config                        */

    uint32_t HSIState;             /*!< The new state of the HSI.
                                        This parameter can be a value of @ref RCC_HSI_Config                        */

    uint32_t HSICalibrationValue;  /*!< The calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                        This parameter must be a number between Min_Data = 0 and Max_Data = 31 on
                                        STM32L43x/STM32L44x/STM32L47x/STM32L48x devices.
                                        This parameter must be a number between Min_Data = 0 and Max_Data = 127 on
                                        the other devices */

    uint32_t LSIState;             /*!< The new state of the LSI.
                                        This parameter can be a value of @ref RCC_LSI_Config                        */

    uint32_t LSIDiv;               /*!< The division factor of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Div                           */

    uint32_t MSIState;             /*!< The new state of the MSI.
                                        This parameter can be a value of @ref RCC_MSI_Config */

    uint32_t MSICalibrationValue;  /*!< The calibration trimming value (default is RCC_MSICALIBRATION_DEFAULT).
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF */

    uint32_t MSIClockRange;        /*!< The MSI frequency range.
                                        This parameter can be a value of @ref RCC_MSI_Clock_Range  */

    uint32_t HSI48State;             /*!< The new state of the HSI48 (only applicable to STM32L43x/STM32L44x/STM32L49x/STM32L4Ax devices).
                                          This parameter can be a value of @ref RCC_HSI48_Config */

    RCC_PLLInitTypeDef PLL;        /*!< Main PLL structure parameters                                               */

}RCC_OscInitTypeDef;

/*****************************************************************************/
// RCC System, AHB and APB busses clock configuration structure definition
/*****************************************************************************/
typedef struct
{
    uint32_t ClockType;             /*!< The clock to be configured.
                                         This parameter can be a value of @ref RCC_System_Clock_Type      */

    uint32_t SYSCLKSource;          /*!< The clock source used as system clock (SYSCLK).
                                         This parameter can be a value of @ref RCC_System_Clock_Source    */

    uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                         This parameter can be a value of @ref RCC_AHB_Clock_Source       */

    uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                         This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

    uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                         This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

}RCC_ClkInitTypeDef;


/**
  * @brief  RCC extended clocks structure definition
  */
typedef struct
{
  uint32_t PeriphClockSelection;   /*!< The Extended Clock to be configured.
                                        This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */

  uint32_t Usart1ClockSelection;   /*!< Specifies USART1 clock source.
                                        This parameter can be a value of @ref RCCEx_USART1_Clock_Source */

  uint32_t Usart2ClockSelection;   /*!< Specifies USART2 clock source.
                                        This parameter can be a value of @ref RCCEx_USART2_Clock_Source */

  uint32_t Usart3ClockSelection;   /*!< Specifies USART3 clock source.
                                        This parameter can be a value of @ref RCCEx_USART3_Clock_Source */

  uint32_t Lpuart1ClockSelection;  /*!< Specifies LPUART1 clock source.
                                        This parameter can be a value of @ref RCCEx_LPUART1_Clock_Source */

  uint32_t I2c1ClockSelection;     /*!< Specifies I2C1 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C1_Clock_Source */

  uint32_t I2c2ClockSelection;     /*!< Specifies I2C2 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C2_Clock_Source */

  uint32_t I2c3ClockSelection;     /*!< Specifies I2C3 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C3_Clock_Source */

  uint32_t I2c4ClockSelection;     /*!< Specifies I2C4 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C4_Clock_Source */

  uint32_t Lptim1ClockSelection;   /*!< Specifies LPTIM1 clock source.
                                        This parameter can be a value of @ref RCCEx_LPTIM1_Clock_Source */

  uint32_t Lptim2ClockSelection;   /*!< Specifies LPTIM2 clock source.
                                        This parameter can be a value of @ref RCCEx_LPTIM2_Clock_Source */

  uint32_t UsbClockSelection;      /*!< Specifies USB clock source (warning: same source for SDMMC1 and RNG).
                                        This parameter can be a value of @ref RCCEx_USB_Clock_Source */

  uint32_t RngClockSelection;      /*!< Specifies RNG clock source (warning: same source for USB and SDMMC1).
                                        This parameter can be a value of @ref RCCEx_RNG_Clock_Source */

  uint32_t RTCClockSelection;      /*!< Specifies RTC clock source.
                                        This parameter can be a value of @ref RCC_RTC_Clock_Source */
}RCC_PeriphCLKInitTypeDef;

/*****************************************************************************/
/*****************************************************************************/
/*                                  MACROS                                   */
/*****************************************************************************/
/*****************************************************************************/

#define AC_RCC_PWR_CLK_DISABLE()            CLEAR_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN)
#define AC_RCC_PWR_IS_CLK_ENABLED()         (READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN) != 0U)
#define AC_RCC_PWR_IS_CLK_DISABLED()        (READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN) == 0U)

#define AC_RCC_ADC_IS_CLK_ENABLED()         (READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_ADCEN) != 0U)

/**************************************************************************//**
 * @brief  Enable or disable the APB1 peripheral clock.
 * @note   After reset, the peripheral clock (used for registers read/write access)
 *         is disabled and the application software has to enable this clock before
 *         using it.
 *****************************************************************************/
#define AC_RCC_PWR_CLK_ENABLE()             do { \
                                                __IO uint32_t tmpreg; \
                                                SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN); \
                                                /* Delay after an RCC peripheral clock enabling */ \
                                                tmpreg = READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN); \
                                                UNUSED(tmpreg); \
                                            } while(0)

/**************************************************************************//**
 * @brief  Enable or disable the APB2 peripheral clock.
 * @note   After reset, the peripheral clock (used for registers read/write access)
 *         is disabled and the application software has to enable this clock before
 *         using it.
 *****************************************************************************/
#define AC_RCC_SYSCFG_CLK_ENABLE()          do { \
                                                __IO uint32_t tmpreg; \
                                                SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN); \
                                                /* Delay after an RCC peripheral clock enabling */ \
                                                tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN); \
                                                UNUSED(tmpreg); \
                                            } while(0)

/**************************************************************************//**
 * @brief  Macro to configure the External Low Speed oscillator (LSE) drive capability.
 * @note   As the LSE is in the Backup domain and write access is denied to
 *         this domain after reset, you have to enable write access using
 *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
 *         (to be done once after reset).
 * @param  __LSEDRIVE__ specifies the new state of the LSE drive capability.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_LSEDRIVE_LOW  LSE oscillator low drive capability.
 *            @arg @ref RCC_LSEDRIVE_MEDIUMLOW  LSE oscillator medium low drive capability.
 *            @arg @ref RCC_LSEDRIVE_MEDIUMHIGH  LSE oscillator medium high drive capability.
 *            @arg @ref RCC_LSEDRIVE_HIGH  LSE oscillator high drive capability.
 * @retval None
 *****************************************************************************/
#define AC_RCC_LSEDRIVE_CONFIG(__LSEDRIVE__) \
                  MODIFY_REG(RCC->BDCR, RCC_BDCR_LSEDRV, (__LSEDRIVE__))


/**************************************************************************//**
 * @brief  Macro to get the clock source used as system clock.
 * @retval The clock source used as system clock. The returned value can be one
 *         of the following:
 *              - RCC_SYSCLKSOURCE_STATUS_MSI: MSI used as system clock.
 *              - RCC_SYSCLKSOURCE_STATUS_HSI: HSI used as system clock.
 *              - RCC_SYSCLKSOURCE_STATUS_HSE: HSE used as system clock.
 *              - RCC_SYSCLKSOURCE_STATUS_PLLCLK: PLL used as system clock.
 *****************************************************************************/
#define AC_RCC_GET_SYSCLK_SOURCE() (READ_BIT(RCC->CFGR, RCC_CFGR_SWS))

/**************************************************************************//**
 * @brief  Macro to get the clock source used as system clock.
 * @retval The clock source used as system clock. The returned value can be one
 *         of the following:
 *              - RCC_SYSCLKSOURCE_STATUS_MSI: MSI used as system clock.
 *              - RCC_SYSCLKSOURCE_STATUS_HSI: HSI used as system clock.
 *              - RCC_SYSCLKSOURCE_STATUS_HSE: HSE used as system clock.
 *              - RCC_SYSCLKSOURCE_STATUS_PLLCLK: PLL used as system clock.
 *****************************************************************************/
#define AC_RCC_GET_SYSCLK_SOURCE() (READ_BIT(RCC->CFGR, RCC_CFGR_SWS))

/**************************************************************************//**
 * @brief  Macro to get the oscillator used as PLL clock source.
 * @retval The oscillator used as PLL clock source. The returned value can be one
 *         of the following:
 *              - RCC_PLLSOURCE_NONE: No oscillator is used as PLL clock source.
 *              - RCC_PLLSOURCE_MSI: MSI oscillator is used as PLL clock source.
 *              - RCC_PLLSOURCE_HSI: HSI oscillator is used as PLL clock source.
 *              - RCC_PLLSOURCE_HSE: HSE oscillator is used as PLL clock source.
 *****************************************************************************/
#define AC_RCC_GET_PLL_OSCSOURCE() (READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC))

/**************************************************************************//**
 * @brief  Macro to get the Internal Multi Speed oscillator (MSI) clock range in run mode
 * @retval MSI clock range.
 *         This parameter must be one of the following values:
 *            @arg @ref RCC_MSIRANGE_0  MSI clock is around 100 KHz
 *            @arg @ref RCC_MSIRANGE_1  MSI clock is around 200 KHz
 *            @arg @ref RCC_MSIRANGE_2  MSI clock is around 400 KHz
 *            @arg @ref RCC_MSIRANGE_3  MSI clock is around 800 KHz
 *            @arg @ref RCC_MSIRANGE_4  MSI clock is around 1 MHz
 *            @arg @ref RCC_MSIRANGE_5  MSI clock is around 2 MHz
 *            @arg @ref RCC_MSIRANGE_6  MSI clock is around 4 MHz (default after Reset)
 *            @arg @ref RCC_MSIRANGE_7  MSI clock is around 8 MHz
 *            @arg @ref RCC_MSIRANGE_8  MSI clock is around 16 MHz
 *            @arg @ref RCC_MSIRANGE_9  MSI clock is around 24 MHz
 *            @arg @ref RCC_MSIRANGE_10  MSI clock is around 32 MHz
 *            @arg @ref RCC_MSIRANGE_11  MSI clock is around 48 MHz
 *****************************************************************************/
#define AC_RCC_GET_MSI_RANGE()                                              \
                  ((READ_BIT(RCC->CR, RCC_CR_MSIRGSEL) != 0U) ?             \
                   READ_BIT(RCC->CR, RCC_CR_MSIRANGE) :                     \
                   (READ_BIT(RCC->CSR, RCC_CSR_MSISRANGE) >> 4U))

/**************************************************************************//**
 * @brief  Macro configures the Internal Multi Speed oscillator (MSI) clock range in run mode
 * @note     After restart from Reset , the MSI clock is around 4 MHz.
 *           After stop the startup clock can be MSI (at any of its possible
 *           frequencies, the one that was used before entering stop mode) or HSI.
 *          After Standby its frequency can be selected between 4 possible values
 *          (1, 2, 4 or 8 MHz).
 * @note     MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready
 *          (MSIRDY=1).
 * @note    The MSI clock range after reset can be modified on the fly.
 * @param  __MSIRANGEVALUE__ specifies the MSI clock range.
 *         This parameter must be one of the following values:
 *            @arg @ref RCC_MSIRANGE_0  MSI clock is around 100 KHz
 *            @arg @ref RCC_MSIRANGE_1  MSI clock is around 200 KHz
 *            @arg @ref RCC_MSIRANGE_2  MSI clock is around 400 KHz
 *            @arg @ref RCC_MSIRANGE_3  MSI clock is around 800 KHz
 *            @arg @ref RCC_MSIRANGE_4  MSI clock is around 1 MHz
 *            @arg @ref RCC_MSIRANGE_5  MSI clock is around 2 MHz
 *            @arg @ref RCC_MSIRANGE_6  MSI clock is around 4 MHz (default after Reset)
 *            @arg @ref RCC_MSIRANGE_7  MSI clock is around 8 MHz
 *            @arg @ref RCC_MSIRANGE_8  MSI clock is around 16 MHz
 *            @arg @ref RCC_MSIRANGE_9  MSI clock is around 24 MHz
 *            @arg @ref RCC_MSIRANGE_10  MSI clock is around 32 MHz
 *            @arg @ref RCC_MSIRANGE_11  MSI clock is around 48 MHz
 * @retval None
 *****************************************************************************/
#define AC_RCC_MSI_RANGE_CONFIG(__MSIRANGEVALUE__) \
                  do {                                                         \
                    SET_BIT(RCC->CR, RCC_CR_MSIRGSEL);                         \
                    MODIFY_REG(RCC->CR, RCC_CR_MSIRANGE, (__MSIRANGEVALUE__)); \
                  } while(0)

/**************************************************************************//**
 * @brief  Macro Adjusts the Internal Multi Speed oscillator (MSI) calibration value.
 * @note   The calibration is used to compensate for the variations in voltage
 *         and temperature that influence the frequency of the internal MSI RC.
 *         Refer to the Application Note AN3300 for more details on how to
 *         calibrate the MSI.
 * @param  __MSICALIBRATIONVALUE__ specifies the calibration trimming value
 *         (default is RCC_MSICALIBRATION_DEFAULT).
 *         This parameter must be a number between 0 and 255.
 * @retval None
 *****************************************************************************/
#define AC_RCC_MSI_CALIBRATIONVALUE_ADJUST(__MSICALIBRATIONVALUE__) \
                  MODIFY_REG(RCC->ICSCR, RCC_ICSCR_MSITRIM, (__MSICALIBRATIONVALUE__) << RCC_ICSCR_MSITRIM_Pos)

/**************************************************************************//**
 * @brief  Macro to adjust the Internal High Speed 16MHz oscillator (HSI) calibration value.
 * @note   The calibration is used to compensate for the variations in voltage
 *         and temperature that influence the frequency of the internal HSI RC.
 * @param  __HSICALIBRATIONVALUE__ specifies the calibration trimming value
 *         (default is RCC_HSICALIBRATION_DEFAULT).
 *         This parameter must be a number between 0 and 31 on STM32L43x/STM32L44x/STM32L47x/STM32L48x
 *         or between 0 and 127 on other devices.
 * @retval None
 *****************************************************************************/
#define AC_RCC_HSI_CALIBRATIONVALUE_ADJUST(__HSICALIBRATIONVALUE__) \
                  MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSITRIM, (__HSICALIBRATIONVALUE__) << RCC_ICSCR_HSITRIM_Pos)

/**************************************************************************//**
 * @brief  Macros to enable or disable the Internal Multi Speed oscillator (MSI).
 * @note     The MSI is stopped by hardware when entering STOP and STANDBY modes.
 *           It is used (enabled by hardware) as system clock source after
 *           startup from Reset, wakeup from STOP and STANDBY mode, or in case
 *           of failure of the HSE used directly or indirectly as system clock
 *           (if the Clock Security System CSS is enabled).
 * @note     MSI can not be stopped if it is used as system clock source.
 *           In this case, you have to select another source of the system
 *           clock then stop the MSI.
 * @note     After enabling the MSI, the application software should wait on
 *           MSIRDY flag to be set indicating that MSI clock is stable and can
 *           be used as system clock source.
 * @note   When the MSI is stopped, MSIRDY flag goes low after 6 MSI oscillator
 *         clock cycles.
 * @retval None
 *****************************************************************************/
#define AC_RCC_MSI_ENABLE()  SET_BIT(RCC->CR, RCC_CR_MSION)
#define AC_RCC_MSI_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_MSION)

/**************************************************************************//**
 * @brief  Macros to enable or disable the Internal High Speed 16MHz oscillator (HSI).
 * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
 *         It is used (enabled by hardware) as system clock source after startup
 *         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
 *         of the HSE used directly or indirectly as system clock (if the Clock
 *         Security System CSS is enabled).
 * @note   HSI can not be stopped if it is used as system clock source. In this case,
 *         you have to select another source of the system clock then stop the HSI.
 * @note   After enabling the HSI, the application software should wait on HSIRDY
 *         flag to be set indicating that HSI clock is stable and can be used as
 *         system clock source.
 *         This parameter can be: ENABLE or DISABLE.
 * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
 *         clock cycles.
 * @retval None
 *****************************************************************************/
#define AC_RCC_HSI_ENABLE()  SET_BIT(RCC->CR, RCC_CR_HSION)
#define AC_RCC_HSI_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_HSION)

/**************************************************************************//**
 * @brief  Macros to enable or disable the Internal Low Speed oscillator (LSI).
 * @note   After enabling the LSI, the application software should wait on
 *         LSIRDY flag to be set indicating that LSI clock is stable and can
 *         be used to clock the IWDG and/or the RTC.
 * @note   LSI can not be disabled if the IWDG is running.
 * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
 *         clock cycles.
 * @retval None
 *****************************************************************************/
#define AC_RCC_LSI_ENABLE()     SET_BIT(RCC->CSR, RCC_CSR_LSION)
#define AC_RCC_LSI_DISABLE()    CLEAR_BIT(RCC->CSR, RCC_CSR_LSION)

/**************************************************************************//**
 * @brief  Macros to enable or disable the Internal High Speed 48MHz oscillator (HSI48).
 * @note   The HSI48 is stopped by hardware when entering STOP and STANDBY modes.
 * @note   After enabling the HSI48, the application software should wait on HSI48RDY
 *         flag to be set indicating that HSI48 clock is stable.
 *         This parameter can be: ENABLE or DISABLE.
 * @retval None
 *****************************************************************************/
#define AC_RCC_HSI48_ENABLE()  SET_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON)
#define AC_RCC_HSI48_DISABLE() CLEAR_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON)

/**************************************************************************//**
 * @brief  Macros to enable or disable the main PLL.
 * @note   After enabling the main PLL, the application software should wait on
 *         PLLRDY flag to be set indicating that PLL clock is stable and can
 *         be used as system clock source.
 * @note   The main PLL can not be disabled if it is used as system clock source
 * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
 * @retval None
 *****************************************************************************/
#define AC_RCC_PLL_ENABLE()         SET_BIT(RCC->CR, RCC_CR_PLLON)
#define AC_RCC_PLL_DISABLE()        CLEAR_BIT(RCC->CR, RCC_CR_PLLON)

/**************************************************************************//**
 * @brief  Enable or disable each clock output (RCC_PLL_SYSCLK, RCC_PLL_48M1CLK, RCC_PLL_SAI3CLK)
 * @note   Enabling/disabling clock outputs RCC_PLL_SAI3CLK and RCC_PLL_48M1CLK can be done at anytime
 *         without the need to stop the PLL in order to save power. But RCC_PLL_SYSCLK cannot
 *         be stopped if used as System Clock.
 * @param  __PLLCLOCKOUT__ specifies the PLL clock to be output.
 *          This parameter can be one or a combination of the following values:
 *            @arg @ref RCC_PLL_SAI3CLK  This clock is used to generate an accurate clock to achieve
 *                                   high-quality audio performance on SAI interface in case.
 *            @arg @ref RCC_PLL_48M1CLK  This Clock is used to generate the clock for the USB OTG FS (48 MHz),
 *                                   the random analog generator (<=48 MHz) and the SDMMC1 (<= 48 MHz).
 *            @arg @ref RCC_PLL_SYSCLK  This Clock is used to generate the high speed system clock (up to 80MHz)
 * @retval None
 *****************************************************************************/
#define AC_RCC_PLLCLKOUT_ENABLE(__PLLCLOCKOUT__)   SET_BIT(RCC->PLLCFGR, (__PLLCLOCKOUT__))
#define AC_RCC_PLLCLKOUT_DISABLE(__PLLCLOCKOUT__)  CLEAR_BIT(RCC->PLLCFGR, (__PLLCLOCKOUT__))

/**************************************************************************//**
 * @brief  Macro to configure the External High Speed oscillator (HSE).
 * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
 *         supported by this macro. User should request a transition to HSE Off
 *         first and then HSE On or HSE Bypass.
 * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
 *         software should wait on HSERDY flag to be set indicating that HSE clock
 *         is stable and can be used to clock the PLL and/or system clock.
 * @note   HSE state can not be changed if it is used directly or through the
 *         PLL as system clock. In this case, you have to select another source
 *         of the system clock then change the HSE state (ex. disable it).
 * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
 * @note   This function reset the CSSON bit, so if the clock security system(CSS)
 *         was previously enabled you have to enable it again after calling this
 *         function.
 * @param  __STATE__ specifies the new state of the HSE.
 *         This parameter can be one of the following values:
 *            @arg @ref RCC_HSE_OFF  Turn OFF the HSE oscillator, HSERDY flag goes low after
 *                              6 HSE oscillator clock cycles.
 *            @arg @ref RCC_HSE_ON  Turn ON the HSE oscillator.
 *            @arg @ref RCC_HSE_BYPASS  HSE oscillator bypassed with external clock.
 * @retval None
 *****************************************************************************/
#define AC_RCC_HSE_CONFIG(__STATE__)                      \
                    do {                                     \
                      if((__STATE__) == RCC_HSE_ON)          \
                      {                                      \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);      \
                      }                                      \
                      else if((__STATE__) == RCC_HSE_BYPASS) \
                      {                                      \
                        SET_BIT(RCC->CR, RCC_CR_HSEBYP);     \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);      \
                      }                                      \
                      else                                   \
                      {                                      \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);    \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);   \
                      }                                      \
                    } while(0)


/**************************************************************************//**
 * @brief  Macro to configure the main PLL clock source, multiplication and division factors.
 * @note   This function must be used only when the main PLL is disabled.
 *
 * @param  __PLLSOURCE__ specifies the PLL entry clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_PLLSOURCE_NONE  No clock selected as PLL clock entry
 *            @arg @ref RCC_PLLSOURCE_MSI  MSI oscillator clock selected as PLL clock entry
 *            @arg @ref RCC_PLLSOURCE_HSI  HSI oscillator clock selected as PLL clock entry
 *            @arg @ref RCC_PLLSOURCE_HSE  HSE oscillator clock selected as PLL clock entry
 * @note   This clock source is common for the main PLL and audio PLL (PLLSAI1 and PLLSAI2).
 *
 * @param  __PLLM__ specifies the division factor for PLL VCO input clock.
 *          This parameter must be a number between Min_Data = 1 and Max_Data = 16 on STM32L4Rx/STM32L4Sx devices.
 *          This parameter must be a number between Min_Data = 1 and Max_Data = 8 on other devices.
 * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
 *         frequency ranges from 4 to 16 MHz. It is recommended to select a frequency
 *         of 16 MHz to limit PLL jitter.
 *
 * @param  __PLLN__ specifies the multiplication factor for PLL VCO output clock.
 *          This parameter must be a number between 8 and 86.
 * @note   You have to set the PLLN parameter correctly to ensure that the VCO
 *         output frequency is between 64 and 344 MHz.
 *
 * @param  __PLLP__ specifies the division factor for SAI clock when SAI available on device.
 *          This parameter must be a number in the range (7 or 17) for STM32L47x/STM32L48x
 *          else (2 to 31).
 *
 * @param  __PLLQ__ specifies the division factor for OTG FS, SDMMC1 and RNG clocks.
 *          This parameter must be in the range (2, 4, 6 or 8).
 * @note   If the USB OTG FS is used in your application, you have to set the
 *         PLLQ parameter correctly to have 48 MHz clock for the USB. However,
 *         the SDMMC1 and RNG need a frequency lower than or equal to 48 MHz to work
 *         correctly.
 * @param  __PLLR__ specifies the division factor for the main system clock.
 * @note   You have to set the PLLR parameter correctly to not exceed 80MHZ.
 *          This parameter must be in the range (2, 4, 6 or 8).
 * @retval None
 *****************************************************************************/
#define AC_RCC_PLL_CONFIG(__PLLSOURCE__, __PLLM__, __PLLN__, __PLLQ__,__PLLR__ ) \
                            MODIFY_REG(RCC->PLLCFGR, \
                             (RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | \
                              RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLR), \
                             ((__PLLSOURCE__) | \
                              (((__PLLM__) - 1U) << RCC_PLLCFGR_PLLM_Pos) | \
                              ((__PLLN__) << RCC_PLLCFGR_PLLN_Pos) | \
                              ((((__PLLQ__) >> 1U) - 1U) << RCC_PLLCFGR_PLLQ_Pos) | \
                              ((((__PLLR__) >> 1U) - 1U) << RCC_PLLCFGR_PLLR_Pos)))

/**************************************************************************//**
 * @brief  Macros to force or release the Backup domain reset.
 * @note   This function resets the RTC peripheral (including the backup registers)
 *         and the RTC clock source selection in RCC_CSR register.
 * @note   The BKPSRAM is not affected by this reset.
 * @retval None
*****************************************************************************/
#define AC_RCC_BACKUPRESET_FORCE()      SET_BIT(RCC->BDCR, RCC_BDCR_BDRST)
#define AC_RCC_BACKUPRESET_RELEASE()    CLEAR_BIT(RCC->BDCR, RCC_BDCR_BDRST)

/**************************************************************************//**
 * @brief  Macros to configure the RTC clock (RTCCLK).
 * @note   As the RTC clock configuration bits are in the Backup domain and write
 *         access is denied to this domain after reset, you have to enable write
 *         access using the Power Backup Access macro before to configure
 *         the RTC clock source (to be done once after reset).
 * @note   Once the RTC clock is configured it cannot be changed unless the
 *         Backup domain is reset using AC_RCC_BACKUPRESET_FORCE() macro, or by
 *         a Power On Reset (POR).
 *
 * @param  __RTC_CLKSOURCE__ specifies the RTC clock source.
 *         This parameter can be one of the following values:
 *            @arg @ref RCC_RTCCLKSOURCE_NONE  No clock selected as RTC clock.
 *            @arg @ref RCC_RTCCLKSOURCE_LSE  LSE selected as RTC clock.
 *            @arg @ref RCC_RTCCLKSOURCE_LSI  LSI selected as RTC clock.
 *            @arg @ref RCC_RTCCLKSOURCE_HSE_DIV32  HSE clock divided by 32 selected
 *
 * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
 *         work in STOP and STANDBY modes, and can be used as wakeup source.
 *         However, when the HSE clock is used as RTC clock source, the RTC
 *         cannot be used in STOP and STANDBY modes.
 * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
 *         RTC clock source).
 * @retval None
 *****************************************************************************/
#define AC_RCC_RTC_CONFIG(__RTC_CLKSOURCE__)  \
                  MODIFY_REG( RCC->BDCR, RCC_BDCR_RTCSEL, (__RTC_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the RTC clock source.
 * @retval The returned value can be one of the following:
 *            @arg @ref RCC_RTCCLKSOURCE_NONE  No clock selected as RTC clock.
 *            @arg @ref RCC_RTCCLKSOURCE_LSE  LSE selected as RTC clock.
 *            @arg @ref RCC_RTCCLKSOURCE_LSI  LSI selected as RTC clock.
 *            @arg @ref RCC_RTCCLKSOURCE_HSE_DIV32  HSE clock divided by 32 selected
 *****************************************************************************/
#define  AC_RCC_GET_RTC_SOURCE() (READ_BIT(RCC->BDCR, RCC_BDCR_RTCSEL))

/**************************************************************************//**
 * @brief  Macro to configure the I2C1 clock (I2C1CLK).
 *
 * @param  __I2C1_CLKSOURCE__ specifies the I2C1 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_I2C1CLKSOURCE_PCLK1  PCLK1 selected as I2C1 clock
 *            @arg @ref RCC_I2C1CLKSOURCE_HSI  HSI selected as I2C1 clock
 *            @arg @ref RCC_I2C1CLKSOURCE_SYSCLK  System Clock selected as I2C1 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_I2C1_CONFIG(__I2C1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C1SEL, (__I2C1_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the I2C1 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_I2C1CLKSOURCE_PCLK1  PCLK1 selected as I2C1 clock
 *            @arg @ref RCC_I2C1CLKSOURCE_HSI  HSI selected as I2C1 clock
 *            @arg @ref RCC_I2C1CLKSOURCE_SYSCLK  System Clock selected as I2C1 clock
 *****************************************************************************/
#define AC_RCC_GET_I2C1_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_I2C1SEL))

/**************************************************************************//**
 * @brief  Macro to configure the I2C2 clock (I2C2CLK).
 *
 * @param  __I2C2_CLKSOURCE__ specifies the I2C2 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_I2C2CLKSOURCE_PCLK1  PCLK1 selected as I2C2 clock
 *            @arg @ref RCC_I2C2CLKSOURCE_HSI  HSI selected as I2C2 clock
 *            @arg @ref RCC_I2C2CLKSOURCE_SYSCLK  System Clock selected as I2C2 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_I2C2_CONFIG(__I2C2_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C2SEL, (__I2C2_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the I2C2 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_I2C2CLKSOURCE_PCLK1  PCLK1 selected as I2C2 clock
 *            @arg @ref RCC_I2C2CLKSOURCE_HSI  HSI selected as I2C2 clock
 *            @arg @ref RCC_I2C2CLKSOURCE_SYSCLK  System Clock selected as I2C2 clock
 *****************************************************************************/
#define AC_RCC_GET_I2C2_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_I2C2SEL))

/**************************************************************************//**
 * @brief  Macro to configure the I2C3 clock (I2C3CLK).
 *
 * @param  __I2C3_CLKSOURCE__ specifies the I2C3 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_I2C3CLKSOURCE_PCLK1  PCLK1 selected as I2C3 clock
 *            @arg @ref RCC_I2C3CLKSOURCE_HSI  HSI selected as I2C3 clock
 *            @arg @ref RCC_I2C3CLKSOURCE_SYSCLK  System Clock selected as I2C3 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_I2C3_CONFIG(__I2C3_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C3SEL, (__I2C3_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the I2C3 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_I2C3CLKSOURCE_PCLK1  PCLK1 selected as I2C3 clock
 *            @arg @ref RCC_I2C3CLKSOURCE_HSI  HSI selected as I2C3 clock
 *            @arg @ref RCC_I2C3CLKSOURCE_SYSCLK  System Clock selected as I2C3 clock
 *****************************************************************************/
#define AC_RCC_GET_I2C3_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_I2C3SEL))

/**************************************************************************//**
 * @brief  Macro to configure the USART1 clock (USART1CLK).
 *
 * @param  __USART1_CLKSOURCE__ specifies the USART1 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_USART1CLKSOURCE_PCLK2  PCLK2 selected as USART1 clock
 *            @arg @ref RCC_USART1CLKSOURCE_HSI  HSI selected as USART1 clock
 *            @arg @ref RCC_USART1CLKSOURCE_SYSCLK  System Clock selected as USART1 clock
 *            @arg @ref RCC_USART1CLKSOURCE_LSE  SE selected as USART1 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_USART1_CONFIG(__USART1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART1SEL, (__USART1_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the USART1 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_USART1CLKSOURCE_PCLK2  PCLK2 selected as USART1 clock
 *            @arg @ref RCC_USART1CLKSOURCE_HSI  HSI selected as USART1 clock
 *            @arg @ref RCC_USART1CLKSOURCE_SYSCLK  System Clock selected as USART1 clock
 *            @arg @ref RCC_USART1CLKSOURCE_LSE  LSE selected as USART1 clock
 *****************************************************************************/
#define AC_RCC_GET_USART1_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL))

/**************************************************************************//**
 * @brief  Macro to configure the USART2 clock (USART2CLK).
 *
 * @param  __USART2_CLKSOURCE__ specifies the USART2 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_USART2CLKSOURCE_PCLK1  PCLK1 selected as USART2 clock
 *            @arg @ref RCC_USART2CLKSOURCE_HSI  HSI selected as USART2 clock
 *            @arg @ref RCC_USART2CLKSOURCE_SYSCLK  System Clock selected as USART2 clock
 *            @arg @ref RCC_USART2CLKSOURCE_LSE  LSE selected as USART2 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_USART2_CONFIG(__USART2_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART2SEL, (__USART2_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the USART2 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_USART2CLKSOURCE_PCLK1  PCLK1 selected as USART2 clock
 *            @arg @ref RCC_USART2CLKSOURCE_HSI  HSI selected as USART2 clock
 *            @arg @ref RCC_USART2CLKSOURCE_SYSCLK  System Clock selected as USART2 clock
 *            @arg @ref RCC_USART2CLKSOURCE_LSE  LSE selected as USART2 clock
 *****************************************************************************/
#define AC_RCC_GET_USART2_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_USART2SEL))

/**************************************************************************//**
 * @brief  Macro to configure the USART3 clock (USART3CLK).
 *
 * @param  __USART3_CLKSOURCE__ specifies the USART3 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_USART3CLKSOURCE_PCLK1  PCLK1 selected as USART3 clock
 *            @arg @ref RCC_USART3CLKSOURCE_HSI  HSI selected as USART3 clock
 *            @arg @ref RCC_USART3CLKSOURCE_SYSCLK  System Clock selected as USART3 clock
 *            @arg @ref RCC_USART3CLKSOURCE_LSE  LSE selected as USART3 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_USART3_CONFIG(__USART3_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART3SEL, (__USART3_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the USART3 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_USART3CLKSOURCE_PCLK1  PCLK1 selected as USART3 clock
 *            @arg @ref RCC_USART3CLKSOURCE_HSI  HSI selected as USART3 clock
 *            @arg @ref RCC_USART3CLKSOURCE_SYSCLK  System Clock selected as USART3 clock
 *            @arg @ref RCC_USART3CLKSOURCE_LSE  LSE selected as USART3 clock
 *****************************************************************************/
#define AC_RCC_GET_USART3_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_USART3SEL))

/**************************************************************************//**
 * @brief  Macro to configure the LPUART1 clock (LPUART1CLK).
 *
 * @param  __LPUART1_CLKSOURCE__ specifies the LPUART1 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1  PCLK1 selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_HSI  HSI selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK  System Clock selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_LSE  LSE selected as LPUART1 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_LPUART1_CONFIG(__LPUART1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPUART1SEL, (__LPUART1_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the LPUART1 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1  PCLK1 selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_HSI  HSI selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK  System Clock selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_LSE  LSE selected as LPUART1 clock
 *****************************************************************************/
#define AC_RCC_GET_LPUART1_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_LPUART1SEL))

/**************************************************************************//**
 * @brief  Macro to configure the LPTIM1 clock (LPTIM1CLK).
 *
 * @param  __LPTIM1_CLKSOURCE__ specifies the LPTIM1 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_LPTIM1CLKSOURCE_PCLK1  PCLK1 selected as LPTIM1 clock
 *            @arg @ref RCC_LPTIM1CLKSOURCE_LSI  HSI selected as LPTIM1 clock
 *            @arg @ref RCC_LPTIM1CLKSOURCE_HSI  LSI selected as LPTIM1 clock
 *            @arg @ref RCC_LPTIM1CLKSOURCE_LSE  LSE selected as LPTIM1 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_LPTIM1_CONFIG(__LPTIM1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL, (__LPTIM1_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the LPTIM1 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_LPTIM1CLKSOURCE_PCLK1  PCLK1 selected as LPUART1 clock
 *            @arg @ref RCC_LPTIM1CLKSOURCE_LSI  HSI selected as LPUART1 clock
 *            @arg @ref RCC_LPTIM1CLKSOURCE_HSI  System Clock selected as LPUART1 clock
 *            @arg @ref RCC_LPTIM1CLKSOURCE_LSE  LSE selected as LPUART1 clock
 *****************************************************************************/
#define AC_RCC_GET_LPTIM1_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL))

/**************************************************************************//**
 * @brief  Macro to configure the LPTIM2 clock (LPTIM2CLK).
 *
 * @param  __LPTIM2_CLKSOURCE__ specifies the LPTIM2 clock source.
 *          This parameter can be one of the following values:
 *            @arg @ref RCC_LPTIM2CLKSOURCE_PCLK1  PCLK1 selected as LPTIM2 clock
 *            @arg @ref RCC_LPTIM2CLKSOURCE_LSI  HSI selected as LPTIM2 clock
 *            @arg @ref RCC_LPTIM2CLKSOURCE_HSI  LSI selected as LPTIM2 clock
 *            @arg @ref RCC_LPTIM2CLKSOURCE_LSE  LSE selected as LPTIM2 clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_LPTIM2_CONFIG(__LPTIM2_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM2SEL, (__LPTIM2_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the LPTIM2 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_LPTIM2CLKSOURCE_PCLK1  PCLK1 selected as LPUART1 clock
 *            @arg @ref RCC_LPTIM2CLKSOURCE_LSI  HSI selected as LPUART1 clock
 *            @arg @ref RCC_LPTIM2CLKSOURCE_HSI  System Clock selected as LPUART1 clock
 *            @arg @ref RCC_LPTIM2CLKSOURCE_LSE  LSE selected as LPUART1 clock
 *****************************************************************************/
#define AC_RCC_GET_LPTIM2_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_LPTIM2SEL))

/**************************************************************************//**
 * @brief  Macro to configure the RNG clock.
 *
 * @note  USB, RNG and SDMMC1 peripherals share the same 48MHz clock source.
 *
 * @param  __RNG_CLKSOURCE__ specifies the RNG clock source.
 *         This parameter can be one of the following values:
 *            @arg @ref RCC_RNGCLKSOURCE_MSI  MSI selected as RNG clock
 *            @arg @ref RCC_RNGCLKSOURCE_PLLSAI1  PLLSAI1 Clock selected as RNG clock
 *            @arg @ref RCC_RNGCLKSOURCE_PLL  PLL Clock selected as RNG clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_RNG_CONFIG(__RNG_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_CLK48SEL, (__RNG_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the RNG clock.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_RNGCLKSOURCE_MSI  MSI selected as RNG clock
 *            @arg @ref RCC_RNGCLKSOURCE_PLLSAI1  PLLSAI1 "Q" clock (PLL48M2CLK) selected as RNG clock
 *            @arg @ref RCC_RNGCLKSOURCE_PLL  PLL "Q" clock (PLL48M1CLK) selected as RNG clock
 *****************************************************************************/
#define AC_RCC_GET_RNG_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_CLK48SEL))

/**************************************************************************//**
 * @brief  Macro to configure the USB clock (USBCLK).
 *
 * @note  USB, RNG and SDMMC1 peripherals share the same 48MHz clock source.
 *
 * @param  __USB_CLKSOURCE__ specifies the USB clock source.
 *         This parameter can be one of the following values:
 @if STM32L486xx
 *            @arg @ref RCC_USBCLKSOURCE_NONE  No clock selected as 48MHz clock for devices without HSI48
 @endif
 @if STM32L443xx
 *            @arg @ref RCC_USBCLKSOURCE_HSI48  HSI48 selected as 48MHz clock for devices with HSI48
 @endif
 *            @arg @ref RCC_USBCLKSOURCE_MSI  MSI selected as USB clock
 *            @arg @ref RCC_USBCLKSOURCE_PLLSAI1  PLLSAI1 "Q" clock (PLL48M2CLK) selected as USB clock
 *            @arg @ref RCC_USBCLKSOURCE_PLL  PLL "Q" clock (PLL48M1CLK) selected as USB clock
 * @retval None
 *****************************************************************************/
#define AC_RCC_USB_CONFIG(__USB_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_CLK48SEL, (__USB_CLKSOURCE__))

/**************************************************************************//**
 * @brief  Macro to get the USB clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_USBCLKSOURCE_MSI  MSI selected as USB clock
 *            @arg @ref RCC_USBCLKSOURCE_PLLSAI1  PLLSAI1 "Q" clock (PLL48M2CLK) selected as USB clock
 *            @arg @ref RCC_USBCLKSOURCE_PLL  PLL "Q" clock (PLL48M1CLK) selected as USB clock
*****************************************************************************/
#define AC_RCC_GET_USB_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_CLK48SEL))

/**************************************************************************//**
 * @brief  Macro to get the ADC clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_ADCCLKSOURCE_NONE  No clock selected as ADC clock
 *            @arg @ref RCC_ADCCLKSOURCE_SYSCLK  System Clock selected as ADC clock
 *****************************************************************************/
#define AC_RCC_GET_ADC_SOURCE() ((AC_RCC_ADC_IS_CLK_ENABLED() != 0U) ? RCC_ADCCLKSOURCE_SYSCLK : RCC_ADCCLKSOURCE_NONE)

/*****************************************************************************/
/*****************************************************************************/
/*                      INTERFACE FUNCTIONS DECLARATION                      */
/*****************************************************************************/
/*****************************************************************************/
AC_status_t AC_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
AC_status_t AC_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency);
uint32_t AC_RCC_GetSysClockFreq(void);
AC_status_t AC_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);


#endif /* _AC_RCC_H_ */

