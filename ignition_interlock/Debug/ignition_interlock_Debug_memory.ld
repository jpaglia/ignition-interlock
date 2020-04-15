/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2020
 * Generated linker script file for LPC802
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v11.1.1 [Build 3241] [2020-03-02] on 15-Apr-2020 1:57:35 PM
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x3f80 /* 16256 bytes (alias Flash) */  
  BOOT_FLASH (rx) : ORIGIN = 0x3f80, LENGTH = 0x80 /* 128 bytes (alias Flash2) */  
  SRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 0x7e0 /* 2016 bytes (alias RAM) */  
  IAP_SRAM (rwx) : ORIGIN = 0x100007e0, LENGTH = 0x20 /* 32 bytes (alias RAM2) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x3f80 ; /* 16256 bytes */  
  __top_Flash = 0x0 + 0x3f80 ; /* 16256 bytes */  
  __base_BOOT_FLASH = 0x3f80  ; /* BOOT_FLASH */  
  __base_Flash2 = 0x3f80 ; /* Flash2 */  
  __top_BOOT_FLASH = 0x3f80 + 0x80 ; /* 128 bytes */  
  __top_Flash2 = 0x3f80 + 0x80 ; /* 128 bytes */  
  __base_SRAM = 0x10000000  ; /* SRAM */  
  __base_RAM = 0x10000000 ; /* RAM */  
  __top_SRAM = 0x10000000 + 0x7e0 ; /* 2016 bytes */  
  __top_RAM = 0x10000000 + 0x7e0 ; /* 2016 bytes */  
  __base_IAP_SRAM = 0x100007e0  ; /* IAP_SRAM */  
  __base_RAM2 = 0x100007e0 ; /* RAM2 */  
  __top_IAP_SRAM = 0x100007e0 + 0x20 ; /* 32 bytes */  
  __top_RAM2 = 0x100007e0 + 0x20 ; /* 32 bytes */  
