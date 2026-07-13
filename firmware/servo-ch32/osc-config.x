/* osc-servo-ch32 saved-config slot bases (protocol sec 9.4), resolved against the
   board's CONFIG_A/CONFIG_B memory regions -- SAVE uses page 0 of each 512 B
   window, page 1 spare. Shipped into the linker search path by osc-servo-ch32's
   build.rs; a board's memory.x pulls it in with `INCLUDE osc-config.x`.
   Consumed by providers/config_store.rs. */
_config_a = ORIGIN(CONFIG_A);
_config_b = ORIGIN(CONFIG_B);
