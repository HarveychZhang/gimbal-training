/* STM32H723VG: 1 MiB flash and 128 KiB of AXI SRAM available in either RAM partition. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   : ORIGIN = 0x24000000, LENGTH = 128K
}

_stext = ALIGN(ADDR(.vector_table) + SIZEOF(.vector_table), 8);
