#include "reset.h"

#if defined(PICO_RP2040) && PICO_RP2040
#include "hardware/regs/m0plus.h"
#elif defined(PICO_RP2350) && PICO_RP2350
#include "hardware/regs/m33.h"
#endif

#include "select.h"

void __attribute__((noreturn)) reset_jump_to_booster(void) {
  select_coreWaitPushDisable();
  (void)save_and_disable_interrupts();

#if defined(PICO_RP2040) && PICO_RP2040
  __asm__ __volatile__(
      "mov r0, %[start]\n"
      "ldr r1, =%[vtable]\n"
      "str r0, [r1]\n"
      "ldmia r0, {r0, r1}\n"
      "msr msp, r0\n"
      "bx r1\n"
      :
      : [start] "r"((unsigned int)&_booster_app_flash_start + 256),
        [vtable] "X"(PPB_BASE + M0PLUS_VTOR_OFFSET)
      : "r0", "r1", "memory");
#elif defined(PICO_RP2350) && PICO_RP2350
  __asm__ __volatile__(
      "mov r0, %[start]\n"
      "ldr r1, =%[vtable]\n"
      "str r0, [r1]\n"
      "ldmia r0, {r0, r1}\n"
      "msr msp, r0\n"
      "bx r1\n"
      :
      : [start] "r"((unsigned int)&_booster_app_flash_start),
        [vtable] "X"(PPB_BASE + M33_VTOR_OFFSET)
      : "r0", "r1", "memory");
#else
#error "Booster jump is only supported on RP2040/RP2350 builds"
#endif

  DPRINTF("You should never reach this point\n");
  while (true) {
    tight_loop_contents();
  }
}

void reset_device() {
  DPRINTF("Resetting the device\n");

  save_and_disable_interrupts();
  // watchdog_enable(RESET_WATCHDOG_TIMEOUT, 0);
  watchdog_reboot(0, 0, RESET_WATCHDOG_TIMEOUT);
  // 20 ms timeout, for example, then the chip will reset
  while (1) {
    // Wait for the reset
    DPRINTF("Waiting for the device to reset\n");
  }
  DPRINTF("You should never reach this point\n");
}

void reset_deviceAndEraseFlash() {
  // Erase the settings
  DPRINTF("Erasing the flash memory\n");
  settings_erase(gconfig_getContext());
  DPRINTF("Erasing the app lookup table\n");
  sleep_ms(SEC_TO_MS);

  // Reset the device
  DPRINTF("Resetting the device\n");
  reset_device();
}
