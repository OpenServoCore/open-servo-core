/* APP slot under the preserved 8K WCH IAP loader. The loader executes in
   the LOW flash alias and enters the APP at 0x00002000, not 0x08002000:
   an image linked in the 0x08 alias computes every PC-relative address
   0x08000000 too high (la sp -> 0x18008000, trap on the first stack push). */
MEMORY
{
    FLASH : ORIGIN = 0x00002000, LENGTH = 120K
    RAM   : ORIGIN = 0x20000000, LENGTH =  32K
}
REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
