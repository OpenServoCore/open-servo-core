_crash = ORIGIN(CRASH);
ASSERT(LENGTH(CRASH) >= 32, "CRASH region smaller than the crash record");
ASSERT(_ebss <= ORIGIN(CRASH), "bss reaches into the CRASH region");
