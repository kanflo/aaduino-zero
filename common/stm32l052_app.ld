/* App linker script for STM32L052, 32K flash, 8K RAM. */

flash_size = 32k;
ram_size   = 8k;

boot_size = 0; /* TBD */
bootcom_size = 16;
vector_size = 336;
past_start = 0x08080000;
past_block_size = 1024;
app_size = flash_size - boot_size;

/* Define memory regions. */
MEMORY
{
/*    boot          (rx) : ORIGIN = 0x08000000, LENGTH = boot_size */
    rom           (rx) : ORIGIN = 0x08000000 + boot_size, LENGTH = app_size
    past           (r) : ORIGIN = past_start, LENGTH = 2*past_block_size
    ram_vect     (rwx) : ORIGIN = 0x20000000, LENGTH = vector_size
    ram          (rwx) : ORIGIN = 0x20000000 + vector_size, LENGTH = ram_size - vector_size - bootcom_size
    bootcom_ram  (rwx) : ORIGIN = 0x20001FF0, LENGTH = bootcom_size
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

SECTIONS {
    .past : {
        _past_start = .;
        . = . + 2*past_block_size;
        _past_end = .;
     } >past
    
    .bootcom : {
        _bootcom_start = .;
        . = . + bootcom_size;
        _bootcom_end = .;
     } >bootcom_ram

   .ram_vect : {
        _ram_vect_start = .;
        . = . + vector_size;
        _ram_vect_end = .;
     } >ram_vect
}
