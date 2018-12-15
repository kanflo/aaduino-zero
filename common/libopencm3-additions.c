
#include <flash.h>
#include "libopencm3-additions.h"
#include "dbg_printf.h"

/** @todo: these should go into lib/stm32/common/flash_common_l01.c */

/** @brief Wait until Last Operation has Ended

This loops indefinitely until an operation (write or erase) has completed by
testing the busy flag.
*/

void flash_wait_for_last_operation(void)
{
    while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY) ;
}

/** @brief Write a word to flash, inloging and locking is not taken care of here
 * @param address assumed to be in the eeprom space, no checking
 * @param data word to write
 */
void flash_program_word(uint32_t address, uint32_t data)
{
    flash_wait_for_last_operation();

    *((volatile uint32_t*) address) = data;

    flash_wait_for_last_operation();

    uint32_t sr = FLASH_SR;
    if ((sr & FLASH_SR_EOP) != 0) {
        FLASH_SR = FLASH_SR_EOP;
    } else {
        dbg_printf("Flash program failed: 0x%08x\n", sr);
        /** @todo: handle error */
    }
}


/** @todo: these should go into  lib/stm32/l0/flash.c */


/*---------------------------------------------------------------------------*/
/** @brief Read All Status Flags

The programming error, end of operation, write protect error and busy flags
are returned in the order of appearance in the status register.

@returns uint32_t. bit 0: busy, bit 2: programming error, bit 4: write protect
error, bit 5: end of operation.
*/

uint32_t flash_get_status_flags(void)
{
    return FLASH_SR & (FLASH_SR_OPTVERR |
        FLASH_SR_SIZEERR |
        FLASH_SR_PGAERR |
        FLASH_SR_WRPERR |
        FLASH_SR_EOP |
        FLASH_SR_BSY);
}


/*---------------------------------------------------------------------------*/
/** @brief Erase a Page of FLASH

This performs all operations necessary to erase a page in FLASH memory.
The page should be checked to ensure that it was properly erased. A page must
first be fully erased before attempting to program it.

Note that the page sizes differ between devices. See the reference manual or
the FLASH programming manual for details.

@param[in] page_address Full address of flash page to be erased.
*/
void flash_erase_page(uint32_t page_address)
{
#if 1
    flash_wait_for_last_operation();

    FLASH_PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;
    *((volatile uint32_t*) page_address) = 0;

    flash_wait_for_last_operation();

    if ((FLASH_SR & FLASH_SR_EOP) != 0) {
        FLASH_SR = FLASH_SR_EOP;
    } else {
        /** @todo: handle flash errors */
    }
    FLASH_PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_PROG);
#else
    flash_wait_for_last_operation();

    FLASH_CR |= FLASH_CR_PER;
    FLASH_AR = page_address;
    FLASH_CR |= FLASH_CR_STRT;

    flash_wait_for_last_operation();

    FLASH_CR &= ~FLASH_CR_PER;
#endif
}
