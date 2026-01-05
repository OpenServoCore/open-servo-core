MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  /* Reserve last 4KB for EEPROM storage (sequential-storage) */
  FLASH  : ORIGIN = 0x08000000, LENGTH = 60K
  EEPROM : ORIGIN = 0x0800F000, LENGTH = 4K
  RAM    : ORIGIN = 0x20000000, LENGTH = 16K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* Export EEPROM region bounds for flash driver */
_eeprom_start = ORIGIN(EEPROM);
_eeprom_end = ORIGIN(EEPROM) + LENGTH(EEPROM);
