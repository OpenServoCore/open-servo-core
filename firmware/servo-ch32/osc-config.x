/* osc-servo-ch32 saved-config slot bases (protocol sec 9.4), resolved against the
   board's CONFIG_A/CONFIG_B memory regions -- SAVE uses page 0 of each 512 B
   window, page 1 spare. Shipped into the linker search path by osc-servo-ch32's
   build.rs; a board's memory.x pulls it in with `INCLUDE osc-config.x`.
   Consumed by providers/config_store.rs. */
_config_a = ORIGIN(CONFIG_A);
_config_b = ORIGIN(CONFIG_B);
/* Calib slots (own A/B image, two 256 B pages each) at the CALIB region
   front, then the pot LUT slots (own A/B image, three pages each); the
   last 1280 B of the 4K region stay spare. */
_calib_a = ORIGIN(CALIB);
_calib_b = ORIGIN(CALIB) + 512;
_lut_a = ORIGIN(CALIB) + 1024;
_lut_b = ORIGIN(CALIB) + 2048;
