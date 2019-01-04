#ifndef __LIBOPENCM3_ADDITIONS_H__
#define __LIBOPENCM3_ADDITIONS_H__

/** libopencm3 currently lack support for writing flash on STM32L0 so these
 *  are needed until I (or someone else) get around to open a PR */


/** @todo: these should go into libopencm3/stm32/common/flash_common_l01.h */
void flash_wait_for_last_operation(void);
uint32_t flash_get_status_flags(void);
void flash_program_word(uint32_t address, uint32_t data);
void flash_erase_page(uint32_t page_address);

#endif // __LIBOPENCM3_ADDITIONS_H__
