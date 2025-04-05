ENTRY(Reset_Handler)

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* Memories definition */
MEMORY
{
  SRAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 96K
  FLASH    (rx)    : ORIGIN = 0x08000000,   LENGTH = 1024K
}

/* Sections */
SECTIONS
{
  .text :
  {  
    KEEP(*(.isr_vector))
    *(.text)
    *(.rodata)

    . = ALIGN(4);
    _etext = .;
  }> FLASH

  .data : 
  {
    _sdata = .;
    
    *(.data)

    . = ALIGN(4);
    _edata = .;
  }> SRAM AT>FLASH

  .bss :
  {
    _sbss = .;
    *(.bss)
    . = ALIGN(4);
    _ebss = .;
  }> SRAM

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
}
