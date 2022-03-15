/**************************************************************************//**
 * @file    AC_sw_update.c
 * @brief   AC SW updates handler.
 * @details Add this source code to your project in order to give your ST MCU
 *          the capability to flash & run any new SW.
 *****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include "AC_sw_update.h"

/*****************************************************************************/
/*****************************************************************************/
/*                             TYPES & CONSTANTS                             */
/*****************************************************************************/
/*****************************************************************************/
#define SU_FLASH_FLAG_EOP           FLASH_SR_EOP                         /*!< FLASH End of operation flag */
#define SU_FLASH_FLAG_OPERR         FLASH_SR_OPERR                       /*!< FLASH Operation error flag */
#define SU_FLASH_FLAG_PROGERR       FLASH_SR_PROGERR                     /*!< FLASH Programming error flag */
#define SU_FLASH_FLAG_WRPERR        FLASH_SR_WRPERR                      /*!< FLASH Write protection error flag */
#define SU_FLASH_FLAG_PGAERR        FLASH_SR_PGAERR                      /*!< FLASH Programming alignment error flag */
#define SU_FLASH_FLAG_SIZERR        FLASH_SR_SIZERR                      /*!< FLASH Size error flag  */
#define SU_FLASH_FLAG_PGSERR        FLASH_SR_PGSERR                      /*!< FLASH Programming sequence error flag */
#define SU_FLASH_FLAG_MISERR        FLASH_SR_MISERR                      /*!< FLASH Fast programming data miss error flag */
#define SU_FLASH_FLAG_FASTERR       FLASH_SR_FASTERR                     /*!< FLASH Fast programming error flag */
#define SU_FLASH_FLAG_RDERR         FLASH_SR_RDERR                       /*!< FLASH PCROP read error flag */
#define SU_FLASH_FLAG_OPTVERR       FLASH_SR_OPTVERR                     /*!< FLASH Option validity error flag  */
#define SU_FLASH_FLAG_BSY           FLASH_SR_BSY                         /*!< FLASH Busy flag */
#define SU_FLASH_FLAG_PEMPTY        FLASH_SR_PEMPTY                      /*!< FLASH Program empty */
#define SU_FLASH_FLAG_SR_ERRORS     (SU_FLASH_FLAG_OPERR   | SU_FLASH_FLAG_PROGERR | SU_FLASH_FLAG_WRPERR | \
                                     SU_FLASH_FLAG_PGAERR  | SU_FLASH_FLAG_SIZERR  | SU_FLASH_FLAG_PGSERR | \
                                     SU_FLASH_FLAG_MISERR  | SU_FLASH_FLAG_FASTERR | SU_FLASH_FLAG_RDERR  | \
                                     SU_FLASH_FLAG_OPTVERR | SU_FLASH_FLAG_PEMPTY)


#define SU_FLASH_FLAG_ECCR_ERRORS       (FLASH_ECCR_ECCD | FLASH_ECCR_ECCC)
#define SU_FLASH_KEY1  0x45670123U      /*!< Flash key1 */
#define SU_FLASH_KEY2  0xCDEF89ABU      /*!< Flash key2: used with FLASH_KEY1
                                        to unlock the FLASH registers access */

#define SU_SW_UPDATE_TIMEOUT        100     

typedef void (*pFunction)(void);        /*!< Function pointer definition */

/*****************************************************************************/
/*****************************************************************************/
/*                                  MACROS                                   */
/*****************************************************************************/
/*****************************************************************************/

/**
  * @brief  Check whether the specified FLASH flag is set or not.
  * @param  __FLAG__ specifies the FLASH flag to check.
  *   This parameter can be one of the following values:
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag
  *     @arg FLASH_FLAG_OPERR: FLASH Operation error flag
  *     @arg FLASH_FLAG_PROGERR: FLASH Programming error flag
  *     @arg FLASH_FLAG_WRPERR: FLASH Write protection error flag
  *     @arg FLASH_FLAG_PGAERR: FLASH Programming alignment error flag
  *     @arg FLASH_FLAG_SIZERR: FLASH Size error flag
  *     @arg FLASH_FLAG_PGSERR: FLASH Programming sequence error flag
  *     @arg FLASH_FLAG_MISERR: FLASH Fast programming data miss error flag
  *     @arg FLASH_FLAG_FASTERR: FLASH Fast programming error flag
  *     @arg FLASH_FLAG_RDERR: FLASH PCROP read  error flag
  *     @arg FLASH_FLAG_OPTVERR: FLASH Option validity error flag
  *     @arg FLASH_FLAG_BSY: FLASH write/erase operations in progress flag
  *     @arg FLASH_FLAG_PEMPTY : FLASH Boot from not programmed flash (apply only for STM32L43x/STM32L44x devices)
  *     @arg FLASH_FLAG_ECCC: FLASH one ECC error has been detected and corrected
  *     @arg FLASH_FLAG_ECCD: FLASH two ECC errors have been detected
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
#define SU_FLASH_GET_FLAG(__FLAG__)         ((((__FLAG__) & SU_FLASH_FLAG_ECCR_ERRORS) != 0U)     ? \
                                            (READ_BIT(FLASH->ECCR, (__FLAG__)) != 0U) : \
                                            (READ_BIT(FLASH->SR,   (__FLAG__)) != 0U))

/**
  * @brief  Clear the FLASH's pending flags.
  * @param  __FLAG__ specifies the FLASH flags to clear.
  *   This parameter can be any combination of the following values:
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag
  *     @arg FLASH_FLAG_OPERR: FLASH Operation error flag
  *     @arg FLASH_FLAG_PROGERR: FLASH Programming error flag
  *     @arg FLASH_FLAG_WRPERR: FLASH Write protection error flag
  *     @arg FLASH_FLAG_PGAERR: FLASH Programming alignment error flag
  *     @arg FLASH_FLAG_SIZERR: FLASH Size error flag
  *     @arg FLASH_FLAG_PGSERR: FLASH Programming sequence error flag
  *     @arg FLASH_FLAG_MISERR: FLASH Fast programming data miss error flag
  *     @arg FLASH_FLAG_FASTERR: FLASH Fast programming error flag
  *     @arg FLASH_FLAG_RDERR: FLASH PCROP read  error flag
  *     @arg FLASH_FLAG_OPTVERR: FLASH Option validity error flag
  *     @arg FLASH_FLAG_ECCC: FLASH one ECC error has been detected and corrected
  *     @arg FLASH_FLAG_ECCD: FLASH two ECC errors have been detected
  *     @arg FLASH_FLAG_ALL_ERRORS: FLASH All errors flags
  * @retval None
  */
#define SU_FLASH_CLEAR_FLAG(__FLAG__)        do { if(((__FLAG__) & SU_FLASH_FLAG_ECCR_ERRORS) != 0U) { SET_BIT(FLASH->ECCR, ((__FLAG__) & SU_FLASH_FLAG_ECCR_ERRORS)); }\
                                                     if(((__FLAG__) & ~(SU_FLASH_FLAG_ECCR_ERRORS)) != 0U) { WRITE_REG(FLASH->SR, ((__FLAG__) & ~(SU_FLASH_FLAG_ECCR_ERRORS))); }\
                                                   } while(0)

/*****************************************************************************/
/*****************************************************************************/
/*                            LOCAL VARIABLES                                */
/*****************************************************************************/
/*****************************************************************************/
static uint32_t current_sw_start_address_g = 0;
static uint32_t current_sw_size_g = 0;
static uint32_t new_sw_start_address_g = 0;
static uint32_t new_sw_address_g = 0;
static uint32_t new_sw_size_g = 0;

/*****************************************************************************/
/*****************************************************************************/
/*                        LOCAL FUNCTIONS DECLARATION                        */
/*****************************************************************************/
/*****************************************************************************/
static uint32_t get_page_L(uint32_t addr);
static SW_UPDATE_status_t wait_for_last_operation_L(uint32_t timeout);
static SW_UPDATE_status_t delet_page_L(uint32_t page_index);
static uint64_t swap_long_L(uint64_t a);

/*****************************************************************************/
/*****************************************************************************/
/*                    INTERFACE FUNCTIONS IMPLEMENTATION                     */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   Initialize SW update handler
 * @details The function erases all flash except where the sw sits.
 *          
 * @param   [IN] current_sw_start_address      - start address of the current 
 *                                                running SW
 * @param   [IN] current_sw_size               - the size of the current 
 *                                                running SW
 * 
 * @return  SW_UPDATE_OK on success or SU_[ERROR] otherwise
 *****************************************************************************/
SW_UPDATE_status_t SW_UPDATE_init(uint32_t current_sw_start_address, uint32_t current_sw_size)
{
    SW_UPDATE_status_t rc = SW_UPDATE_OK;
    uint32_t current_sw_start_page = 0;
    uint32_t current_sw_end_page = 0;

    /*************************************************************************/
    // check parameters
    /*************************************************************************/
    if ((current_sw_start_address < SU_FLASH_START_ADDRESS) || 
        (current_sw_start_address > SU_FLASH_END_ADDRESS))
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }

    if (current_sw_start_address + current_sw_size > SU_FLASH_END_ADDRESS)
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }

    if (0 == current_sw_size)
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }
    
    /*************************************************************************/
    // unlock the flash control register (FLASH_CR)
    /*************************************************************************/
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
    {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, SU_FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, SU_FLASH_KEY2);

        /* Verify Flash is unlocked */
        if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
        {
            return SW_UPDATE_FAIL;
        }
    }

    /*************************************************************************/
    // get the start & end pages where the current SW is located 
    /*************************************************************************/
    current_sw_start_page  = get_page_L(current_sw_start_address);
    current_sw_end_page    = get_page_L(current_sw_start_address + current_sw_size);

    /*************************************************************************/
    // erase flash except the SW location (erase all unused pages in flash)
    /*************************************************************************/
    for (uint32_t page_index = 0; page_index < SU_FLASH_PAGES_NUM; page_index++)
    {
        /* delete only pages that are not used */
        if ((page_index < current_sw_start_page) || (current_sw_end_page < page_index))
        {   
            rc = delet_page_L(page_index);
            if (SW_UPDATE_OK != rc)
            {
                return rc;
            }
        }
    }

    /*************************************************************************/
    // set globals 
    /*************************************************************************/
    current_sw_start_address_g  = current_sw_start_address;
    current_sw_size_g           = current_sw_size;

    return SW_UPDATE_OK;
}

/**************************************************************************//**
 * @brief   Sets address & size for the new SW
 *          
 * @param   [IN] new_sw_start_address      - start address of the new SW
 * @param   [IN] new_sw_size               - the size of the new SW
 * 
 * @return  SW_UPDATE_OK on success or SU_[ERROR] otherwise
 *****************************************************************************/
SW_UPDATE_status_t SW_UPDATE_begin(uint32_t new_sw_start_address, uint32_t new_sw_size)
{
    uint32_t current_sw_start_page = 0;
    uint32_t current_sw_end_page = 0;
    uint32_t new_sw_start_page = 0;
    uint32_t new_sw_end_page = 0;

    /*************************************************************************/
    // check parameters
    /*************************************************************************/
    if ((new_sw_start_address < SU_FLASH_START_ADDRESS) || 
        (new_sw_start_address > SU_FLASH_END_ADDRESS))
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }

    if (new_sw_start_address + new_sw_size > SU_FLASH_END_ADDRESS)
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }

    if (0 == new_sw_size)
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }
    
    /*************************************************************************/
    // check the new_sw_start_address is a base address of a page
    /*************************************************************************/
    bool addr_is_a_starting_address_of_a_page = false;
    for (uint32_t   page_start_addr = SU_FLASH_START_ADDRESS; 
                    page_start_addr < SU_FLASH_END_ADDRESS; 
                    page_start_addr += SU_FLASH_PAGE_SIZE
        )
    {
        if (new_sw_start_address == page_start_addr)
        {
            addr_is_a_starting_address_of_a_page = true;
            break;
        }
    }

    if (addr_is_a_starting_address_of_a_page == false)
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }

    /*************************************************************************/
    // calc the start & end pages where the current & new SW will be located 
    /*************************************************************************/
    current_sw_start_page   = get_page_L(current_sw_start_address_g);
    current_sw_end_page     = get_page_L(current_sw_start_address_g + current_sw_size_g);
    new_sw_start_page       = get_page_L(new_sw_start_address);
    new_sw_end_page         = get_page_L(new_sw_start_address + new_sw_size);

    /*************************************************************************/
    // check the new sw will be located in unused pages
    /*************************************************************************/
    if (!
        (((new_sw_start_page < current_sw_start_page) && (new_sw_end_page < current_sw_start_page))
        ||
        ((new_sw_start_page > current_sw_end_page) && (new_sw_end_page > current_sw_end_page)))
        )
    {
        return SW_UPDATE_FAIL;
    }

    /*************************************************************************/
    // set globals 
    /*************************************************************************/
    new_sw_start_address_g  = new_sw_start_address;
    new_sw_address_g        = new_sw_start_address;
    new_sw_size_g           = new_sw_size;

    return SW_UPDATE_OK;
}

/**************************************************************************//**
 * @brief   Perform programming by repeatedly calling this function
 * @details This function automatically increases the new_sw_address_g where 
 *          the data is being written
 *          
 * @param   [IN] data   - data buffer
 * @param   [IN] size   - buffer's size (number of elements in buffer,
 *                        each element is 64bits)
 * 
 * @return  SW_UPDATE_OK on success or SU_[ERROR] otherwise
 *****************************************************************************/
SW_UPDATE_status_t SW_UPDATE_next(uint64_t* data, uint32_t size)
{
    SW_UPDATE_status_t rc = SW_UPDATE_OK;
    uint64_t swap_bytes = 0;

    /*************************************************************************/
    // check parameters
    /*************************************************************************/
    if ((NULL == data) || (0 == size))
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }

    /*************************************************************************/
    // check FLASH is not busy
    /*************************************************************************/
    rc = wait_for_last_operation_L(SU_SW_UPDATE_TIMEOUT);
    if (SW_UPDATE_OK != rc)
    {
        return rc;
    }

    /*************************************************************************/
    // set standard program  
    /*************************************************************************/
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    /*************************************************************************/
    // disable the caches (Instruction & Data caches)
    /*************************************************************************/
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN);

    /*************************************************************************/
    // program flash with data  
    /*************************************************************************/
    for (uint32_t i = 0; i<size; i++)
    {

        /*********************************************************************/
        // check FLASH is not busy
        /*********************************************************************/
        rc = wait_for_last_operation_L(SU_SW_UPDATE_TIMEOUT);
        if (SW_UPDATE_OK != rc)
        {
            return rc;
        }

        /*********************************************************************/
        // switch bytes order
        /*********************************************************************/
        swap_bytes = swap_long_L(data[i]);

        /*********************************************************************/
        // Program first word
        /*********************************************************************/
        *(__IO uint32_t*)new_sw_address_g = (uint32_t)swap_bytes;
    
        /*********************************************************************/
        /* Barrier to ensure programming is performed in 2 steps, in right order
          (independently of compiler optimization behavior) */
        /*********************************************************************/
        __ISB();
    
        /*********************************************************************/
        // Program second word
        /*********************************************************************/
        *(__IO uint32_t*)(new_sw_address_g+4U) = (uint32_t)(swap_bytes >> 32);        

        /*********************************************************************/
        // check data was written correctly
        /*********************************************************************/        
        if(*(uint64_t*)new_sw_address_g != swap_bytes)
        {
            return SW_UPDATE_FAIL;
        }

        /*********************************************************************/
        // update address
        /*********************************************************************/
        new_sw_address_g += 8;
    }

    /*************************************************************************/
    // check FLASH is not busy
    /*************************************************************************/
    rc = wait_for_last_operation_L(SU_SW_UPDATE_TIMEOUT);
    if (SW_UPDATE_OK != rc)
    {
        return rc;
    }

    /*************************************************************************/
    // clear standard program  
    /*************************************************************************/
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

    /*************************************************************************/
    // flush the caches (Instruction & Data caches)
    /*************************************************************************/
    /* reset Instruction cache*/
    SET_BIT(FLASH->ACR, FLASH_ACR_ICRST);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICRST);

    /* reset Data cache */
    SET_BIT(FLASH->ACR, FLASH_ACR_DCRST);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCRST);

    /*************************************************************************/
    // enable the caches (Instruction & Data caches)
    /*************************************************************************/
    SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
    SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);
    
    return SW_UPDATE_OK;
}

/**************************************************************************//**
 * @brief   End the SW update flash process
 * @details This function locks...
 *          
 * @param   none
 * 
 * @return  SW_UPDATE_OK on success or SU_[ERROR] otherwise
 *****************************************************************************/
SW_UPDATE_status_t SW_UPDATE_end(void)
{
    /*************************************************************************/
    // lock the flash control register (FLASH_CR)
    /*************************************************************************/
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    /*************************************************************************/
    // verify control register is locked
    /*************************************************************************/
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) == 0U)
    {
        return SW_UPDATE_FAIL;
    }

    return SW_UPDATE_OK;
}

/**************************************************************************//**
 * @brief   Jump to new SW
 * @details The function carries out the following operations:
 *          - Sets the stack pointer location
 *          - Perform the jump
 * @param   none
 * 
 * @return  SU_[ERROR] when error occurs. Else no return.
 *****************************************************************************/
SW_UPDATE_status_t SW_UPDATE_jump_to_new_app(void)
{
    uint32_t JumpAddress = *(__IO uint32_t*)(new_sw_start_address_g + 4);
    pFunction Jump       = (pFunction)JumpAddress;

    /*************************************************************************/
    // set the stack poiner
    /*************************************************************************/
    __set_MSP(*(__IO uint32_t*)new_sw_start_address_g);

    /*************************************************************************/
    // jump to new app
    /*************************************************************************/
    Jump();

    return SW_UPDATE_OK;
}

/*****************************************************************************/
/*****************************************************************************/
/*                      LOCAL FUNCTIONS IMPLEMENTATION                       */
/*****************************************************************************/
/*****************************************************************************/

/**************************************************************************//**
 * @brief   Gets the page of a given address
 *          
 * @param   [IN] addr - address of the FLASH Memory
 * 
 * @return  The page of a given address
 *****************************************************************************/
static uint32_t get_page_L(uint32_t addr)
{
    return ((addr - SU_FLASH_START_ADDRESS) / SU_FLASH_PAGE_SIZE);
}

/**************************************************************************//**
 * @brief   Wait for a FLASH operation to complete.
 *          
 * @param   [IN] timeout - maximum flash operation timeout
 * 
 * @return  SW_UPDATE_OK on success or SU_[ERROR] otherwise
 *****************************************************************************/
static SW_UPDATE_status_t wait_for_last_operation_L(uint32_t timeout)
{
    uint32_t errors = 0;

    /*************************************************************************/
    // wait for flash to be not busy
    /*************************************************************************/
    BUSY_WAIT_TIMEOUT((SU_FLASH_GET_FLAG(SU_FLASH_FLAG_BSY) == 1), timeout);

    /*************************************************************************/
    // check if error occured
    /*************************************************************************/
    errors = (READ_REG(FLASH->SR) & SU_FLASH_FLAG_SR_ERRORS);

    if (errors != 0)
    {
        SU_FLASH_CLEAR_FLAG(errors);
        return SW_UPDATE_FAIL;
    }

    /*************************************************************************/
    // check FLASH End of Operation flag
    /*************************************************************************/
    if (SU_FLASH_GET_FLAG(SU_FLASH_FLAG_EOP) == 1)
    {
        /* Clear FLASH End of Operation pending bit */
        SU_FLASH_CLEAR_FLAG(SU_FLASH_FLAG_EOP);
    }

    return SW_UPDATE_OK;
}

/**************************************************************************//**
 * @brief   Delete a specified page
 *          
 * @param   [IN] page_index - page to delete
 * 
 * @return  SW_UPDATE_OK on success or SU_[ERROR] otherwise
 *****************************************************************************/
static SW_UPDATE_status_t delet_page_L(uint32_t page_index)
{
    SW_UPDATE_status_t rc = SW_UPDATE_OK;

    /*************************************************************************/
    // check parameters
    /*************************************************************************/
    if (page_index >= SU_FLASH_PAGES_NUM)
    {
        return SW_UPDATE_INVALID_PARAMETER;
    }

    /*************************************************************************/
    // check FLASH is not busy
    /*************************************************************************/
    rc = wait_for_last_operation_L(SU_SW_UPDATE_TIMEOUT);
    if (SW_UPDATE_OK != rc)
    {
        return rc;
    }

    /*************************************************************************/
    // disable the caches (Instruction & Data caches)
    /*************************************************************************/
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN);

    /*************************************************************************/
    // select a page to erase
    /*************************************************************************/
    MODIFY_REG(FLASH->CR, FLASH_CR_PNB, ((page_index & 0xFFU) << FLASH_CR_PNB_Pos));
    SET_BIT(FLASH->CR, FLASH_CR_PER);
  
    /*************************************************************************/
    // start erase operation
    /*************************************************************************/
    SET_BIT(FLASH->CR, FLASH_CR_STRT);

    /*************************************************************************/
    // check FLASH is not busy & no errors occured
    /*************************************************************************/
    rc = wait_for_last_operation_L(SU_SW_UPDATE_TIMEOUT);
    if (SW_UPDATE_OK != rc)
    {
        return rc;
    }

    /*************************************************************************/
    // set FLASH CONTROL register to reset values
    /*************************************************************************/
    CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));

    /*************************************************************************/
    // flush the caches (Instruction & Data caches)
    /*************************************************************************/
    /* reset Instruction cache*/
    SET_BIT(FLASH->ACR, FLASH_ACR_ICRST);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICRST);

    /* reset Data cache */
    SET_BIT(FLASH->ACR, FLASH_ACR_DCRST);
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCRST);

    /*************************************************************************/
    // enable the caches (Instruction & Data caches)
    /*************************************************************************/
    SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
    SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);

    return SW_UPDATE_OK;
}

/**************************************************************************//**
 * @brief   swap uint64_t bytes 
 *          
 * @param   [IN] a - a long uint64_t variable
 * 
 * @return  swaped bytes of input
 *****************************************************************************/
static uint64_t swap_long_L(uint64_t a)
{
    uint64_t x = a;
    x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
    x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
    x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
    return x;
}