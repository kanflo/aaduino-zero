target remote localhost:3333
monitor reset halt
file dpsboot.elf
load
b main
c
