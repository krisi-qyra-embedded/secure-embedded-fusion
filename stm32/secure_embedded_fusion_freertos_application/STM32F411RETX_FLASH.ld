/*
 ******************************************************************************
 **
 ** Linker script for the STM32F411RE application image.
 **
 ** Application Flash allocation:
 **   Sector 5: 0x08020000 - 0x0803FFFF (128 KiB)
 **   Sector 6: 0x08040000 - 0x0805FFFF (128 KiB)
 **   Total:    0x08020000 - 0x0805FFFF (256 KiB)
 **
 ** Sector 7 begins at 0x08060000 and is deliberately excluded so the
 ** application cannot overlap the secure-boot manifest/signature region.
 **
 ******************************************************************************
 */

/* Entry Point */
ENTRY(Reset_Handler)

/* Memories definition */
MEMORY
{
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH (rx)  : ORIGIN = 0x08020000, LENGTH = 256K
}

/* Highest address of the user-mode stack. */
_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

/* Exported application boundaries. The limit is exclusive. */
__app_flash_start__ = ORIGIN(FLASH);
__app_flash_limit__ = ORIGIN(FLASH) + LENGTH(FLASH);
__app_flash_last__  = __app_flash_limit__ - 1;

SECTIONS
{
  /* The application vector table must be the first content in Flash. */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Program code and executable read-only content. */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* Constant data. */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Flash load address used by startup code to initialize .data in RAM. */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT>FLASH

  . = ALIGN(4);
  .bss (NOLOAD) :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a (*)
    libm.a (*)
    libgcc.a (*)
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }

  /* End of all bytes that must be present in the application Flash image. */
  __app_flash_image_end__ = LOADADDR(.data) + SIZEOF(.data);
}

/* Hard failures for an incorrect or oversized application layout. */
ASSERT(ADDR(.isr_vector) == 0x08020000,
       "Application vector table is not at 0x08020000")

ASSERT(__app_flash_limit__ == 0x08060000,
       "Application Flash allocation does not stop at Sector 7")

ASSERT(__app_flash_image_end__ <= __app_flash_limit__,
       "Application image overlaps Sector 7 manifest region")

