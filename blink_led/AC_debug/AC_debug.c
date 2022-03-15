/**************************************************************************//**
 * @file    AC_debug.c
 * @brief   AC debug module definitions.
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include "AC_debug.h"
#include "AC_system.h"

#if !defined(OS_USE_SEMIHOSTING)

/*****************************************************************************/
/*****************************************************************************/
/*                                  TYPES                                    */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
// lock type
/*****************************************************************************/
typedef enum
{
    DEBUG_UNLOCKED = 0x00,
    DEBUG_LOCKED   = 0x01
}DEBUG_LockTypeDef;

/*****************************************************************************/
// UART Init Structure definition
/*****************************************************************************/
typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the UART communication baud rate.
                                           The baud rate register is computed using the following formula:
                                           LPUART:
                                           =======
                                              Baud Rate Register = ((256 * lpuart_ker_ckpres) / ((huart->Init.BaudRate)))
                                           where lpuart_ker_ck_pres is the UART input clock (divided by a prescaler if applicable)
                                           UART:
                                           =====
                                           - If oversampling is 16 or in LIN mode,
                                              Baud Rate Register = ((uart_ker_ckpres) / ((huart->Init.BaudRate)))
                                           - If oversampling is 8,
                                              Baud Rate Register[15:4] = ((2 * uart_ker_ckpres) / ((huart->Init.BaudRate)))[15:4]
                                              Baud Rate Register[3] =  0
                                              Baud Rate Register[2:0] =  (((2 * uart_ker_ckpres) / ((huart->Init.BaudRate)))[3:0]) >> 1
                                           where uart_ker_ck_pres is the UART input clock (divided by a prescaler if applicable) */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UARTEx_Word_Length. */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_Stop_Bits. */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Mode. */

  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref UART_Hardware_Flow_Control. */

  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to f_PCLK/8).
                                           This parameter can be a value of @ref UART_Over_Sampling. */

  uint32_t OneBitSampling;            /*!< Specifies whether a single sample or three samples' majority vote is selected.
                                           Selecting the single sample method increases the receiver tolerance to clock
                                           deviations. This parameter can be a value of @ref UART_OneBit_Sampling. */
} UART_InitTypeDef;

typedef uint32_t HAL_UART_StateTypeDef;

/*****************************************************************************/
// UART handle Structure definition
/*****************************************************************************/
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef            *Instance;                /*!< UART registers base address        */

  UART_InitTypeDef         Init;                     /*!< UART communication parameters      */

  uint8_t                  *pTxBuffPtr;              /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                 TxXferSize;               /*!< UART Tx Transfer size              */

  __IO uint16_t            TxXferCount;              /*!< UART Tx Transfer Counter           */

  uint8_t                  *pRxBuffPtr;              /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                 RxXferSize;               /*!< UART Rx Transfer size              */

  __IO uint16_t            RxXferCount;              /*!< UART Rx Transfer Counter           */

  uint16_t                 Mask;                     /*!< UART Rx RDR register mask          */

  void (*RxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Rx IRQ handler */

  void (*TxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Tx IRQ handler */

  DEBUG_LockTypeDef           Lock;                    /*!< Locking object                     */

  __IO HAL_UART_StateTypeDef    gState;              /*!< UART state information related to global Handle management
                                                          and also related to Tx operations.
                                                          This parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO HAL_UART_StateTypeDef    RxState;             /*!< UART state information related to Rx operations.
                                                          This parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO uint32_t                 ErrorCode;           /*!< UART Error code                    */

} DEBUG_UART_HandleTypeDef;

/*****************************************************************************/
// GPIO Init structure definition
/*****************************************************************************/
typedef struct
{
  uint32_t Pin;        /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins */

  uint32_t Mode;       /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode */

  uint32_t Pull;       /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull */

  uint32_t Speed;      /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins
                            This parameter can be a value of @ref GPIOEx_Alternate_function_selection */
}DEBUG_GPIO_InitTypeDef;

/*****************************************************************************/
/*****************************************************************************/
/*                        LOCAL FUNCTIONS DECLARATION                        */
/*****************************************************************************/
/*****************************************************************************/
static AC_status_t UART_Init_L(DEBUG_UART_HandleTypeDef *huart);
static AC_status_t UART_SetConfig_L(DEBUG_UART_HandleTypeDef *huart);
static AC_status_t UART_CheckIdleState_L(DEBUG_UART_HandleTypeDef *huart);
static AC_status_t UART_Transmit_L(DEBUG_UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
static AC_status_t UART_Receive_L(DEBUG_UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
static void HAL_UART_MspInit_L(DEBUG_UART_HandleTypeDef* huart);
static void GPIO_Init_L(GPIO_TypeDef  *GPIOx, DEBUG_GPIO_InitTypeDef *GPIO_Init);

/*****************************************************************************/
/*****************************************************************************/
/*                    TYPES & LOCAL VARIABLES & CONSTANTS                    */
/*****************************************************************************/
/*****************************************************************************/
#define DEBUG_UART_DELAY 10000

/*****************************************************************************/
// STD definitions
/*****************************************************************************/
#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

/*****************************************************************************/
// UART configuration
/*****************************************************************************/
#define DEBUG_UART                          USART3
#define DEBUG_BAUDRATE                      115200
#define DEBUG_UART_WORDLENGTH_8B            0x00000000U    /*!< 8-bit long UART frame */
#define DEBUG_UART_STOPBITS_1               0x00000000U    /*!< UART frame with 1 stop bit    */
#define DEBUG_UART_PARITY_NONE              0x00000000U    /*!< No parity   */
#define DEBUG_UART_MODE_TX_RX               (USART_CR1_TE |USART_CR1_RE)    /*!< RX and TX mode */
#define DEBUG_UART_HWCONTROL_NONE           0x00000000U    /*!< No hardware control       */
#define DEBUG_UART_OVERSAMPLING_16          0x00000000U    /*!< Oversampling by 16 */
#define DEBUG_UART_ONE_BIT_SAMPLE_DISABLE   0x00000000U    /*!< One-bit sampling disable */
#define DEBUG_UART_ADVFEATURE_NO_INIT       0x00000000U    /*!< No advanced feature initialization       */

/*****************************************************************************/
// UART State
/*****************************************************************************/
#define  DEBUG_UART_STATE_RESET         0x00000000U    /*!< Peripheral is not initialized
                                                          Value is allowed for gState and RxState */
#define  DEBUG_UART_STATE_READY         0x00000020U    /*!< Peripheral Initialized and ready for use
                                                          Value is allowed for gState and RxState */
#define  DEBUG_UART_STATE_BUSY          0x00000024U    /*!< an internal process is ongoing
                                                          Value is allowed for gState only */
#define  DEBUG_UART_STATE_BUSY_TX       0x00000021U    /*!< Data Transmission process is ongoing
                                                          Value is allowed for gState only */
#define  DEBUG_UART_STATE_BUSY_RX       0x00000022U    /*!< Data Reception process is ongoing
                                                          Value is allowed for RxState only */
#define  DEBUG_UART_STATE_BUSY_TX_RX    0x00000023U    /*!< Data Transmission and Reception process is ongoing
                                                          Not to be used for neither gState nor RxState.
                                                          Value is result of combination (Or) between gState and RxState values */
#define  DEBUG_UART_STATE_TIMEOUT       0x000000A0U    /*!< Timeout state
                                                          Value is allowed for gState only */
#define  DEBUG_UART_STATE_ERROR         0x000000E0U    /*!< Error 
                                                          Value is allowed for gState only */
/*****************************************************************************/
// UART Error Definition
/*****************************************************************************/
#define  DEBUG_UART_ERROR_NONE             ((uint32_t)0x00000000U)    /*!< No error                */
#define  DEBUG_UART_ERROR_PE               ((uint32_t)0x00000001U)    /*!< Parity error            */
#define  DEBUG_UART_ERROR_NE               ((uint32_t)0x00000002U)    /*!< Noise error             */
#define  DEBUG_UART_ERROR_FE               ((uint32_t)0x00000004U)    /*!< Frame error             */
#define  DEBUG_UART_ERROR_ORE              ((uint32_t)0x00000008U)    /*!< Overrun error           */
#define  DEBUG_UART_ERROR_DMA              ((uint32_t)0x00000010U)    /*!< DMA transfer error      */
#define  DEBUG_UART_ERROR_RTO              ((uint32_t)0x00000020U)    /*!< Receiver Timeout error  */

/*****************************************************************************/
// UART extended register fields
/*****************************************************************************/
#define USART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | \
                                      USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8 )) /*!< UART or USART CR1 fields of parameters set by UART_SetConfig API */
#define USART_CR3_FIELDS  ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT))  /*!< UART or USART CR3 fields of parameters set by UART_SetConfig API */

/*****************************************************************************/
// GPIO pins
/*****************************************************************************/
#define DEBUG_GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define DEBUG_GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define DEBUG_GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define DEBUG_GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define DEBUG_GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define DEBUG_GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define DEBUG_GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define DEBUG_GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define DEBUG_GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define DEBUG_GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define DEBUG_GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define DEBUG_GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define DEBUG_GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define DEBUG_GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define DEBUG_GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define DEBUG_GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define DEBUG_GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */

#define DEBUG_GPIO_PIN_MASK              (0x0000FFFFu) /* PIN mask for assert test */

#define DEBUG_GPIO_MODE_AF_PP            (0x00000002u)   /*!< Alternate Function Push Pull Mode     */
#define DEBUG_GPIO_NOPULL                (0x00000000u)   /*!< No Pull-up or Pull-down activation  */
#define DEBUG_GPIO_SPEED_FREQ_VERY_HIGH  (0x00000003u)   /*!< range 50 MHz to 80 MHz, please refer to the product datasheet */
#define DEBUG_GPIO_AF8_LPUART1           ((uint8_t)0x08)  /* LPUART1 Alternate Function mapping */
#define DEBUG_GPIO_AF7_USART3            ((uint8_t)0x07)  /* USART3 Alternate Function mapping */
#define DEBUG_GPIO_OUTPUT_TYPE           (0x00000010u)
#define DEBUG_GPIO_MODE                  (0x00000003u)


#define UART_BRR_MIN    0x10U        /* UART BRR minimum authorized value */
#define UART_BRR_MAX    0x0000FFFFU  /* UART BRR maximum authorized value */

#define DEBUG_MAX_DELAY      0xFFFFFFFFU

#define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#define LSE_VALUE    32768U /*!< Value of the External oscillator in Hz*/

#define UART_FLAG_TXE                       USART_ISR_TXE           /*!< UART transmit data register empty         */
#define UART_FLAG_TC                        USART_ISR_TC            /*!< UART transmission complete                */
#define UART_FLAG_RXNE                      USART_ISR_RXNE          /*!< UART read data register not empty         */

/*****************************************************************************/
// UART clock sources definition
/*****************************************************************************/
typedef enum
{
    UART_CLOCKSOURCE_PCLK1      = 0x00U,    /*!< PCLK1 clock source  */
    UART_CLOCKSOURCE_PCLK2      = 0x01U,    /*!< PCLK2 clock source  */
    UART_CLOCKSOURCE_HSI        = 0x02U,    /*!< HSI clock source    */
    UART_CLOCKSOURCE_SYSCLK     = 0x04U,    /*!< SYSCLK clock source */
    UART_CLOCKSOURCE_LSE        = 0x08U,    /*!< LSE clock source       */
    UART_CLOCKSOURCE_UNDEFINED  = 0x10U     /*!< Undefined clock source */
} UART_ClockSourceTypeDef;

/*****************************************************************************/
// RCCEx_USART1_Clock_Source USART1 Clock Source
/*****************************************************************************/
#define RCC_USART1CLKSOURCE_PCLK2      0x00000000U
#define RCC_USART1CLKSOURCE_SYSCLK     RCC_CCIPR_USART1SEL_0
#define RCC_USART1CLKSOURCE_HSI        RCC_CCIPR_USART1SEL_1
#define RCC_USART1CLKSOURCE_LSE        (RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1)

/*****************************************************************************/
// RCCEx_USART2_Clock_Source USART1 Clock Source
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
// UART clock sources definition
/*****************************************************************************/
typedef enum
{
    DEBUG_UART_CLOCKSOURCE_PCLK1      = 0x00U,    /*!< PCLK1 clock source  */
    DEBUG_UART_CLOCKSOURCE_PCLK2      = 0x01U,    /*!< PCLK2 clock source  */
    DEBUG_UART_CLOCKSOURCE_HSI        = 0x02U,    /*!< HSI clock source    */
    DEBUG_UART_CLOCKSOURCE_SYSCLK     = 0x04U,    /*!< SYSCLK clock source */
    DEBUG_UART_CLOCKSOURCE_LSE        = 0x08U,    /*!< LSE clock source       */
    DEBUG_UART_CLOCKSOURCE_UNDEFINED  = 0x10U     /*!< Undefined clock source */
} DEBUG_UART_ClockSourceTypeDef;

/*****************************************************************************/
// Global variables
/*****************************************************************************/
static DEBUG_UART_HandleTypeDef UART_HANDLE_DEBUG;

/*****************************************************************************/
/*****************************************************************************/
/*                                  MACROS                                   */
/*****************************************************************************/
/*****************************************************************************/
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define DEBUG_UART_ENABLE(__HANDLE__)       ((__HANDLE__)->Instance->CR1 |= USART_CR1_UE)
#define DEBUG_UART_DISABLE(__HANDLE__)      ((__HANDLE__)->Instance->CR1 &= ~USART_CR1_UE)

#define DEBUG_LOCK(__HANDLE__)                                              \
                                do{                                         \
                                    if((__HANDLE__)->Lock == DEBUG_LOCKED)  \
                                    {                                       \
                                       return AC_SYSTEM_BUSY;               \
                                    }                                       \
                                    else                                    \
                                    {                                       \
                                       (__HANDLE__)->Lock = DEBUG_LOCKED;   \
                                    }                                       \
                                  }while (0)

#define DEBUG_UNLOCK(__HANDLE__)                                            \
                                  do{                                       \
                                      (__HANDLE__)->Lock = DEBUG_UNLOCKED;  \
                                    }while (0)

#define DEBUG_RCC_LPUART1_CLK_ENABLE()         do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->APB1ENR2, RCC_APB1ENR2_LPUART1EN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->APB1ENR2, RCC_APB1ENR2_LPUART1EN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)

#define DEBUG_RCC_GPIOA_CLK_ENABLE()           do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)

#define DEBUG_RCC_GPIOB_CLK_ENABLE()           do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)

#define DEBUG_RCC_GPIOC_CLK_ENABLE()           do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)

#define DEBUG_RCC_USART1_CLK_ENABLE()          do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)

#define DEBUG_RCC_USART3_CLK_ENABLE()          do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART3EN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART3EN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)

/**************************************************************************//**
 * @brief  Macro to get the LPUART1 clock source.
 * @retval The clock source can be one of the following values:
 *            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1  PCLK1 selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_HSI  HSI selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK  System Clock selected as LPUART1 clock
 *            @arg @ref RCC_LPUART1CLKSOURCE_LSE  LSE selected as LPUART1 clock
 *****************************************************************************/
#define DEBUG_RCC_GET_LPUART1_SOURCE() (READ_BIT(RCC->CCIPR, RCC_CCIPR_LPUART1SEL))

/**************************************************************************//**
 * @brief  Report the UART clock source.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __CLOCKSOURCE__ output variable.
 * @retval UART clocking source, written in __CLOCKSOURCE__.
 *****************************************************************************/
#define DEBUG_UART_GETCLOCKSOURCE(__HANDLE__,__CLOCKSOURCE__) \
  do {                                                        \
    if((__HANDLE__)->Instance == USART1)                      \
    {                                                         \
      switch((READ_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL)))     \
      {                                                       \
        case RCC_USART1CLKSOURCE_PCLK2:                       \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK2;         \
          break;                                              \
        case RCC_USART1CLKSOURCE_HSI:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_USART1CLKSOURCE_SYSCLK:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_USART1CLKSOURCE_LSE:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
    }                                                         \
    else if((__HANDLE__)->Instance == USART2)                 \
    {                                                         \
      switch((READ_BIT(RCC->CCIPR, RCC_CCIPR_USART2SEL)))     \
      {                                                       \
        case RCC_USART2CLKSOURCE_PCLK1:                       \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK1;         \
          break;                                              \
        case RCC_USART2CLKSOURCE_HSI:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_USART2CLKSOURCE_SYSCLK:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_USART2CLKSOURCE_LSE:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
    }                                                         \
    else if((__HANDLE__)->Instance == USART3)                 \
    {                                                         \
      switch((READ_BIT(RCC->CCIPR, RCC_CCIPR_USART3SEL)))     \
      {                                                       \
        case RCC_USART3CLKSOURCE_PCLK1:                       \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK1;         \
          break;                                              \
        case RCC_USART3CLKSOURCE_HSI:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_USART3CLKSOURCE_SYSCLK:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_USART3CLKSOURCE_LSE:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
    }                                                         \
    else if((__HANDLE__)->Instance == LPUART1)                \
    {                                                         \
      switch(DEBUG_RCC_GET_LPUART1_SOURCE())                  \
      {                                                       \
        case RCC_LPUART1CLKSOURCE_PCLK1:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK1;         \
          break;                                              \
        case RCC_LPUART1CLKSOURCE_HSI:                        \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_LPUART1CLKSOURCE_SYSCLK:                     \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_LPUART1CLKSOURCE_LSE:                        \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
    }                                                         \
    else                                                      \
    {                                                         \
      (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;         \
    }                                                         \
  } while(0U)

/** @brief  Report the UART mask to apply to retrieve the received data
  *         according to the word length and to the parity bits activation.
  * @note   If PCE = 1, the parity bit is not included in the data extracted
  *         by the reception API().
  *         This masking operation is not carried out in the case of
  *         DMA transfers.
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None, the mask to apply to UART RDR register is stored in (__HANDLE__)->Mask field.
  */
#define DEBUG_UART_MASK_COMPUTATION(__HANDLE__)                       \
  do {                                                                \
    if ((__HANDLE__)->Init.WordLength == DEBUG_UART_WORDLENGTH_8B)    \
    {                                                                 \
      if ((__HANDLE__)->Init.Parity == DEBUG_UART_PARITY_NONE)        \
      {                                                               \
        (__HANDLE__)->Mask = 0x00FFU ;                                \
      }                                                               \
      else                                                            \
      {                                                               \
        (__HANDLE__)->Mask = 0x007FU ;                                \
      }                                                               \
    }                                                                 \
    else                                                              \
    {                                                                 \
      (__HANDLE__)->Mask = 0x0000U;                                   \
    }                                                                 \
  } while(0U)

/**************************************************************************//**
 * @brief  BRR division operation to set BRR register in 16-bit oversampling mode.
 * @param  __PCLK__ UART clock.
 * @param  __BAUD__ Baud rate set by the user.
 * @retval Division result
 *****************************************************************************/
#define DEBUG_UART_DIV_SAMPLING16(__PCLK__, __BAUD__)  (((__PCLK__) + ((__BAUD__)/2U)) / (__BAUD__))

#define DEBUG_IS_LPUART_INSTANCE(INSTANCE)    ((INSTANCE) == LPUART1)

/**************************************************************************//**
 * @brief  Check whether or not UART instance is Low Power UART.
 * @param  __HANDLE__ specifies the UART Handle.
 * @retval AC_REG_SET (instance is LPUART) or AC_REG_RESET (instance isn't LPUART)
 *****************************************************************************/
#define DEBUG_UART_INSTANCE_LOWPOWER(__HANDLE__) (DEBUG_IS_LPUART_INSTANCE((__HANDLE__)->Instance))

/**************************************************************************//**
 * @brief  Return the PCLK1 frequency.
 * @note   Each time PCLK1 changes, this function must be called to update the
 *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
 * @retval PCLK1 frequency in Hz
 *****************************************************************************/
uint32_t DEBUG_RCC_GetPCLK1Freq(void)
{
  /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
  return (SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU));
}

/**************************************************************************//**
 * @brief  Return the PCLK2 frequency.
 * @note   Each time PCLK2 changes, this function must be called to update the
 *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
 * @retval PCLK2 frequency in Hz
 *****************************************************************************/
uint32_t DEBUG_RCC_GetPCLK2Freq(void)
{
  /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
  return (SystemCoreClock>> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] & 0x1FU));
}

/**************************************************************************//**
 * @brief  Check whether the specified UART flag is set or not.
 * @param  __HANDLE__ specifies the UART Handle.
 * @param  __FLAG__ specifies the flag to check.
 *        This parameter can be one of the following values:
 *            @arg @ref UART_FLAG_TXFT  TXFIFO threshold flag
 *            @arg @ref UART_FLAG_RXFT  RXFIFO threshold flag
 *            @arg @ref UART_FLAG_RXFF  RXFIFO Full flag
 *            @arg @ref UART_FLAG_TXFE  TXFIFO Empty flag
 *            @arg @ref UART_FLAG_REACK Receive enable acknowledge flag
 *            @arg @ref UART_FLAG_TEACK Transmit enable acknowledge flag
 *            @arg @ref UART_FLAG_WUF   Wake up from stop mode flag
 *            @arg @ref UART_FLAG_RWU   Receiver wake up flag (if the UART in mute mode)
 *            @arg @ref UART_FLAG_SBKF  Send Break flag
 *            @arg @ref UART_FLAG_CMF   Character match flag
 *            @arg @ref UART_FLAG_BUSY  Busy flag
 *            @arg @ref UART_FLAG_ABRF  Auto Baud rate detection flag
 *            @arg @ref UART_FLAG_ABRE  Auto Baud rate detection error flag
 *            @arg @ref UART_FLAG_CTS   CTS Change flag
 *            @arg @ref UART_FLAG_LBDF  LIN Break detection flag
 *            @arg @ref UART_FLAG_TXE   Transmit data register empty flag
 *            @arg @ref UART_FLAG_TXFNF UART TXFIFO not full flag
 *            @arg @ref UART_FLAG_TC    Transmission Complete flag
 *            @arg @ref UART_FLAG_RXNE  Receive data register not empty flag
 *            @arg @ref UART_FLAG_RXFNE UART RXFIFO not empty flag
 *            @arg @ref UART_FLAG_RTOF  Receiver Timeout flag
 *            @arg @ref UART_FLAG_IDLE  Idle Line detection flag
 *            @arg @ref UART_FLAG_ORE   Overrun Error flag
 *            @arg @ref UART_FLAG_NE    Noise Error flag
 *            @arg @ref UART_FLAG_FE    Framing Error flag
 *            @arg @ref UART_FLAG_PE    Parity Error flag
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 *****************************************************************************/
#define DEBUG_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->ISR & (__FLAG__)) == (__FLAG__))

/*****************************************************************************/
/*****************************************************************************/
/*                    INTERFACE FUNCTIONS IMPLEMENTATION                     */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   Retargeting the UART handler (for printf & scanf use)
 * @param   None
 * @return  None
 *****************************************************************************/
AC_status_t AC_DEBUG_uart_retarget_init(void) 
{
    /*************************************************************************/
    // UART3 init
    /*************************************************************************/
    UART_HANDLE_DEBUG.Instance = DEBUG_UART;
    UART_HANDLE_DEBUG.Init.BaudRate = DEBUG_BAUDRATE;
    UART_HANDLE_DEBUG.Init.WordLength = DEBUG_UART_WORDLENGTH_8B;
    UART_HANDLE_DEBUG.Init.StopBits = DEBUG_UART_STOPBITS_1;
    UART_HANDLE_DEBUG.Init.Parity = DEBUG_UART_PARITY_NONE;
    UART_HANDLE_DEBUG.Init.Mode = DEBUG_UART_MODE_TX_RX;
    UART_HANDLE_DEBUG.Init.HwFlowCtl = DEBUG_UART_HWCONTROL_NONE;
    UART_HANDLE_DEBUG.Init.OverSampling = DEBUG_UART_OVERSAMPLING_16;
    UART_HANDLE_DEBUG.Init.OneBitSampling = DEBUG_UART_ONE_BIT_SAMPLE_DISABLE;
    if (UART_Init_L(&UART_HANDLE_DEBUG) != AC_OK) 
    {
        return AC_FAIL;
    }

    /*************************************************************************/
    // Disable I/O buffering for STDOUT stream, so that
    // chars are sent out as soon as they are printed.
    /*************************************************************************/
    setvbuf(stdout, NULL, _IONBF, 0);

    return AC_OK;
}

int _isatty(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

int _write(int fd, char* ptr, int len) {
  AC_status_t hstatus;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    hstatus = UART_Transmit_L(&UART_HANDLE_DEBUG, (uint8_t *) ptr, len, DEBUG_MAX_DELAY);
    if (hstatus == AC_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _close(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

int _lseek(int fd, int ptr, int dir) {
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {
  AC_status_t hstatus;

  if (fd == STDIN_FILENO) {
    hstatus = UART_Receive_L(&UART_HANDLE_DEBUG, (uint8_t *) ptr, 1, DEBUG_MAX_DELAY);
    if (hstatus == AC_OK)
      return 1;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat* st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}


/*****************************************************************************/
/*****************************************************************************/
/*                      LOCAL FUNCTIONS IMPLEMENTATION                       */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used for UART debug
 * @param huart: UART handle pointer
 * @retval None
 *****************************************************************************/
static void HAL_UART_MspInit_L(DEBUG_UART_HandleTypeDef* huart)
{
    DEBUG_GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance==USART3)
    {

        /*********************************************************************/
        // Peripheral clock enable
        /*********************************************************************/
        DEBUG_RCC_USART3_CLK_ENABLE();
        DEBUG_RCC_GPIOC_CLK_ENABLE();
      
        /*********************************************************************/
        // USART3 GPIO Configuration
        //PC10     ------> USART3_TX
        //PC11     ------> USART3_RX
        /*********************************************************************/
        GPIO_InitStruct.Pin = DEBUG_GPIO_PIN_10|DEBUG_GPIO_PIN_11;
        GPIO_InitStruct.Mode = DEBUG_GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = DEBUG_GPIO_NOPULL;
        GPIO_InitStruct.Speed = DEBUG_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = DEBUG_GPIO_AF7_USART3;
        GPIO_Init_L(GPIOC, &GPIO_InitStruct);
    }
}

/**************************************************************************//**
 * @brief  Initialize the GPIOx peripheral according to the specified parameters in the GPIO_Init.
 * @param  GPIOx where x can be (A..H) to select the GPIO peripheral for STM32L4 family
 * @param  GPIO_Init pointer to a GPIO_InitTypeDef structure that contains
 *         the configuration information for the specified GPIO peripheral.
 * @retval None
 *****************************************************************************/
static void GPIO_Init_L(GPIO_TypeDef  *GPIOx, DEBUG_GPIO_InitTypeDef *GPIO_Init)
{
    uint32_t position = 0x00u;
    uint32_t iocurrent;
    uint32_t temp;

    /* Configure the port pins */
    while (((GPIO_Init->Pin) >> position) != 0x00u)
    {
        /* Get current io position */
        iocurrent = (GPIO_Init->Pin) & (1uL << position);

        if (iocurrent != 0x00u)
        {
            /*--------------------- GPIO Mode Configuration ------------------------*/
            /* Configure the IO Speed */
            temp = GPIOx->OSPEEDR;
            temp &= ~(GPIO_OSPEEDR_OSPEED0 << (position * 2u));
            temp |= (GPIO_Init->Speed << (position * 2u));
            GPIOx->OSPEEDR = temp;
    
            /* Configure the IO Output Type */
            temp = GPIOx->OTYPER;
            temp &= ~(GPIO_OTYPER_OT0 << position) ;
            temp |= (((GPIO_Init->Mode & DEBUG_GPIO_OUTPUT_TYPE) >> 4u) << position);
            GPIOx->OTYPER = temp;
        
            /* Activate the Pull-up or Pull down resistor for the current IO */
            temp = GPIOx->PUPDR;
            temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2u));
            temp |= ((GPIO_Init->Pull) << (position * 2u));
            GPIOx->PUPDR = temp;
        
            /* Configure Alternate function mapped with the current IO */
            temp = GPIOx->AFR[position >> 3u];
            temp &= ~(0xFu << ((position & 0x07u) * 4u));
            temp |= ((GPIO_Init->Alternate) << ((position & 0x07u) * 4u));
            GPIOx->AFR[position >> 3u] = temp;
        
            /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
            temp = GPIOx->MODER;
            temp &= ~(GPIO_MODER_MODE0 << (position * 2u));
            temp |= ((GPIO_Init->Mode & DEBUG_GPIO_MODE) << (position * 2u));
            GPIOx->MODER = temp;
        }

        position++;
    }
}

/**************************************************************************//**
 * @brief   Initialize the UART mode according to the specified
 *          parameters in the UART_InitTypeDef and initialize the associated handle.
 * @param   huart UART handle.
 * @return  status
 *****************************************************************************/
static AC_status_t UART_Init_L(DEBUG_UART_HandleTypeDef *huart)
{
    /* Check the UART handle allocation */
    if (huart == NULL)
    {
        return AC_FAIL;
    }

    if (huart->gState == DEBUG_UART_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        huart->Lock = DEBUG_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK */
        HAL_UART_MspInit_L(huart);
    }

    huart->gState = DEBUG_UART_STATE_BUSY;

    DEBUG_UART_DISABLE(huart);

    /* Set the UART Communication parameters */
    if (UART_SetConfig_L(huart) == AC_FAIL)
    //if (UART_SetConfig(huart) == HAL_ERROR)
    {
        return AC_FAIL;
    }

    /* In asynchronous mode, the following bits must be kept cleared:
    - LINEN and CLKEN bits in the USART_CR2 register,
    - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
    CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

    DEBUG_UART_ENABLE(huart);

    /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
    return (UART_CheckIdleState_L(huart));
}

/**************************************************************************//**
 * @brief   Configure the UART peripheral.
 * @param   huart UART handle.
 * @retval  status
 *****************************************************************************/
static AC_status_t UART_SetConfig_L(DEBUG_UART_HandleTypeDef *huart)
{
    uint32_t tmpreg;
    UART_ClockSourceTypeDef clocksource;
    uint32_t usartdiv;
    AC_status_t ret = AC_OK;
    uint32_t pclk;

    /*-------------------------- USART CR1 Configuration -----------------------*/
    /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
    *  the UART Word Length, Parity, Mode and oversampling:
    *  set the M bits according to huart->Init.WordLength value
    *  set PCE and PS bits according to huart->Init.Parity value
    *  set TE and RE bits according to huart->Init.Mode value
    *  set OVER8 bit according to huart->Init.OverSampling value */
    tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling ;
    MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, tmpreg);

    /*-------------------------- USART CR2 Configuration -----------------------*/
    /* Configure the UART Stop Bits: Set STOP[13:12] bits according
    * to huart->Init.StopBits value */
    MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

    /*-------------------------- USART CR3 Configuration -----------------------*/
    /* Configure
    * - UART HardWare Flow Control: set CTSE and RTSE bits according
    *   to huart->Init.HwFlowCtl value
    * - one-bit sampling method versus three samples' majority rule according
    *   to huart->Init.OneBitSampling (not applicable to LPUART) */
    tmpreg = (uint32_t)huart->Init.HwFlowCtl;

    if (!(DEBUG_UART_INSTANCE_LOWPOWER(huart)))
    {
        tmpreg |= huart->Init.OneBitSampling;
    }
    MODIFY_REG(huart->Instance->CR3, USART_CR3_FIELDS, tmpreg);

  /*-------------------------- USART BRR Configuration -----------------------*/
  DEBUG_UART_GETCLOCKSOURCE(huart, clocksource);

    switch (clocksource)
    {
        case DEBUG_UART_CLOCKSOURCE_PCLK1:
              pclk = DEBUG_RCC_GetPCLK1Freq();
              break;
        case DEBUG_UART_CLOCKSOURCE_PCLK2:
              pclk = DEBUG_RCC_GetPCLK2Freq();
              break;
        case DEBUG_UART_CLOCKSOURCE_HSI:
              pclk = (uint32_t) HSI_VALUE;
              break;
        case DEBUG_UART_CLOCKSOURCE_SYSCLK:
              //pclk = HAL_RCC_GetSysClockFreq();
              pclk = 0;
              break;
        case DEBUG_UART_CLOCKSOURCE_LSE:
              pclk = (uint32_t) LSE_VALUE;
              break;
        default:
              pclk = 0U;
              ret = AC_FAIL;
              break;
    }

    if (pclk != 0U)
    {
      /* USARTDIV must be greater than or equal to 0d16 */
      usartdiv = (uint16_t)(DEBUG_UART_DIV_SAMPLING16(pclk, huart->Init.BaudRate));
      if ((usartdiv >= UART_BRR_MIN) && (usartdiv <= UART_BRR_MAX))
      {
        huart->Instance->BRR = usartdiv;
      }
      else
      {
        ret = AC_FAIL;
      }
    }
  

    /* Clear ISR function pointers */
    huart->RxISR = NULL;
    huart->TxISR = NULL;

    return ret;
}

/**************************************************************************//**
 * @brief   Check the UART Idle State.
 * @param   huart UART handle.
 * @retval  status
 *****************************************************************************/
static AC_status_t UART_CheckIdleState_L(DEBUG_UART_HandleTypeDef *huart)
{

    /* Initialize the UART ErrorCode */
    huart->ErrorCode = DEBUG_UART_ERROR_NONE;

    DELAY_LOOP(DEBUG_UART_DELAY);

    /* Initialize the UART State */
    huart->gState = DEBUG_UART_STATE_READY;
    huart->RxState = DEBUG_UART_STATE_READY;
    
    DEBUG_UNLOCK(huart);
    
    return AC_OK;
}

/**************************************************************************//**
 * @brief Send an amount of data in blocking mode.
 * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
 *         the sent data is handled as a set of u16. In this case, Size must indicate the number
 *         of u16 provided through pData.
 * @note When FIFO mode is enabled, writing a data in the TDR register adds one
 *       data to the TXFIFO. Write operations to the TDR register are performed
 *       when TXFNF flag is set. From hardware perspective, TXFNF flag and
 *       TXE are mapped on the same bit-field.
 * @param huart   UART handle.
 * @param pData   Pointer to data buffer (u8 or u16 data elements).
 * @param Size    Amount of data elements (u8 or u16) to be sent.
 * @param Timeout Timeout duration.
 * @retval HAL status
 *****************************************************************************/
static AC_status_t UART_Transmit_L(DEBUG_UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t  *pdata8bits;
    uint16_t *pdata16bits;

    /* Check that a Tx process is not already ongoing */
    if (huart->gState == DEBUG_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  AC_FAIL;
        }

        DEBUG_LOCK(huart);

        huart->ErrorCode = DEBUG_UART_ERROR_NONE;
        huart->gState = DEBUG_UART_STATE_BUSY_TX;

        huart->TxXferSize  = Size;
        huart->TxXferCount = Size;

        pdata8bits  = pData;
        pdata16bits = NULL;

        DEBUG_UNLOCK(huart);

        while (huart->TxXferCount > 0U)
        {
            /* Wait until flag is set */
            while ((DEBUG_UART_GET_FLAG(huart, UART_FLAG_TXE) ? AC_REG_SET : AC_REG_RESET) == AC_REG_RESET);
          
            if (pdata8bits == NULL)
            {
              huart->Instance->TDR = (uint16_t)(*pdata16bits & 0x01FFU);
              pdata16bits++;
            }
            else
            {
              huart->Instance->TDR = (uint8_t)(*pdata8bits & 0xFFU);
              pdata8bits++;
            }
            huart->TxXferCount--;
        }

        /* Wait until flag is set */
        while ((DEBUG_UART_GET_FLAG(huart, UART_FLAG_TC) ? AC_REG_SET : AC_REG_RESET) == AC_REG_RESET);

        /* At end of Tx process, restore huart->gState to Ready */
        huart->gState = DEBUG_UART_STATE_READY;

        return AC_OK;
    }
    else
    {
        return AC_SYSTEM_BUSY;
    }
}   

/**************************************************************************//**
 * @brief Receive an amount of data in blocking mode.
 * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
 *         the received data is handled as a set of u16. In this case, Size must indicate the number
 *         of u16 available through pData.
 * @note When FIFO mode is enabled, the RXFNE flag is set as long as the RXFIFO
 *       is not empty. Read operations from the RDR register are performed when
 *       RXFNE flag is set. From hardware perspective, RXFNE flag and
 *       RXNE are mapped on the same bit-field.
 * @param huart   UART handle.
 * @param pData   Pointer to data buffer (u8 or u16 data elements).
 * @param Size    Amount of data elements (u8 or u16) to be received.
 * @param Timeout Timeout duration.
 * @retval HAL status
 *****************************************************************************/
static AC_status_t UART_Receive_L(DEBUG_UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t  *pdata8bits;
    uint16_t *pdata16bits;
    uint16_t uhMask;

    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == DEBUG_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  AC_FAIL;
        }

        DEBUG_LOCK(huart);

        huart->ErrorCode = DEBUG_UART_ERROR_NONE;
        huart->RxState = DEBUG_UART_STATE_BUSY_RX;

        huart->RxXferSize  = Size;
        huart->RxXferCount = Size;

        /* Computation of UART mask to apply to RDR register */
        DEBUG_UART_MASK_COMPUTATION(huart);
        uhMask = huart->Mask;

        pdata8bits  = pData;
        pdata16bits = NULL;

        DEBUG_UNLOCK(huart);

        /* as long as data have to be received */
        while (huart->RxXferCount > 0U)
        {

            /* Wait until flag is set */
            while ((DEBUG_UART_GET_FLAG(huart, UART_FLAG_RXNE) ? AC_REG_SET : AC_REG_RESET) == AC_REG_RESET);

            if (pdata8bits == NULL)
            {
                *pdata16bits = (uint16_t)(huart->Instance->RDR & uhMask);
                pdata16bits++;
            }
            else
            {
                *pdata8bits = (uint8_t)(huart->Instance->RDR & (uint8_t)uhMask);
                pdata8bits++;
            }
            huart->RxXferCount--;
        }

        /* At end of Rx process, restore huart->RxState to Ready */
        huart->RxState = DEBUG_UART_STATE_READY;

        return AC_OK;
    }
    else
    {
        return AC_SYSTEM_BUSY;
    }
}

#endif //#if !defined(OS_USE_SEMIHOSTING)