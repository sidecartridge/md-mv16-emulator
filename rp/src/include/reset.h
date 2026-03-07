/**
 * File: reset.h
 * Author: Diego Parrilla Santamaría
 * Date: December 2025, February 2026
 * Copyright: 2024-2026 - GOODDATA LABS SL
 * Description: Header file for RESET functions of the booster app
 */

#ifndef RESET_H
#define RESET_H

#include "constants.h"
#include "debug.h"
#include "gconfig.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "settings.h"

#define RESET_WATCHDOG_TIMEOUT 20  // 20 ms

/**
 * @brief Reset the current app and jump to the Booster app in flash
 *
 *
 * Do not use to reboot the device, because it does not jump to the start of the
 * Flash
 *
 * @note This function should not return. If it does, an error message is
 * printed.
 */
void __attribute__((noreturn)) reset_jump_to_booster(void);

/**
 * @brief Reset the app and reentry in the main device app in flash
 *
 *
 * Use to reboot the device, it jumps to the start of the Flash
 *
 * @note This function should not return. If it does, an error message is
 * printed.
 */
void reset_device();

/**
 * @brief Reset the app and reentry in the main device app in flash.
 *       Erase the flash memory before rebooting
 *
 *
 * Use to reboot the device to a fabric configuration, it jumps to the start of
 * the Flash
 *
 * @note This function should not return. If it does, an error message is
 * printed.
 */
void reset_deviceAndEraseFlash();

#endif  // RESET_H
