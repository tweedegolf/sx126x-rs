/*
 * Linker script for the STM32F303VCT6. 
 * These definitions below tell Rust where to put the code,
 * and provide the length and location of each memory type.
 */
MEMORY
{
  CCRAM : ORIGIN = 0x10000000, LENGTH = 8K
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 40K
}

_stack_start = ORIGIN(CCRAM) + LENGTH(CCRAM);