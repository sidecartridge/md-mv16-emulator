#include "blink.h"

static bool blinkState = false;
static absolute_time_t blinkTime = {0};

#if defined(CYW43_WL_GPIO_LED_PIN)
static bool blinkCyw43Initialized = false;

static bool blinkEnsureCyw43Initialized(void) {
  if (blinkCyw43Initialized) {
    return true;
  }

  if (cyw43_arch_init() != 0) {
    return false;
  }

  blinkCyw43Initialized = true;
  return true;
}
#endif

void blink_on() {
#if defined(CYW43_WL_GPIO_LED_PIN)
  if (blinkEnsureCyw43Initialized()) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  }
#else
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif
}

void blink_off() {
#if defined(CYW43_WL_GPIO_LED_PIN)
  if (blinkEnsureCyw43Initialized()) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  }
#else
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif
}

void blink_toogle() {
  // Blink when timeout
  if ((absolute_time_diff_us(get_absolute_time(), blinkTime) < 0)) {
    blinkState = !blinkState;
    blinkTime = make_timeout_time_ms(CHARACTER_GAP_MS);
    if (blinkState) {
      blink_on();
    } else {
      blink_off();
    }
  }
}
