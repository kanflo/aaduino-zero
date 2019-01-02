/* Boot linker script for STM32L052, 32K flash, 8K RAM. */

flash_base = 0x08000000;
flash_size = 32k;
flash_block_size = 128; /* Specific for STM32L052 */
ram_base = 0x20000000;
ram_size   = 8k;

boot_size = 8k; /* Will change as the bootloader is implemented and optimized */
bootcom_size = 64;
vector_size = 336;
past_start = 0x08080000; /** EEPROM */
past_block_size = 1024;
app_start = flash_base + boot_size;
app_size = flash_size - boot_size;

/* Define memory regions. */
MEMORY
{
    rom           (rx) : ORIGIN = flash_base, LENGTH = boot_size
    app           (rx) : ORIGIN = app_start, LENGTH = app_size
    past           (r) : ORIGIN = past_start, LENGTH = 2*past_block_size
    ram_vect     (rwx) : ORIGIN = ram_base, LENGTH = vector_size
    ram          (rwx) : ORIGIN = ram_base + vector_size, LENGTH = ram_size - vector_size - bootcom_size
    bootcom_ram  (rwx) : ORIGIN = ram_base + ram_size - bootcom_size, LENGTH = bootcom_size
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

SECTIONS {
    .app : {
        . = . + app_size;
     } >app

    .past : {
        _past_start = .;
        . = . + 2*past_block_size;
        _past_end = .;
     } >past
    
    .bootcom : {
        bootcom_start = .;
        . = . + bootcom_size;
        bootcom_end = .;
     } >bootcom_ram

   .ram_vect : {
        _ram_vect_start = .;
        . = . + vector_size;
        _ram_vect_end = .;
     } >ram_vect
}
