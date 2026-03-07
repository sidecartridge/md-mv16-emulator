/**
 * File: emul.c
 * Author: Diego Parrilla Santamaría
 * Date: February 2025, February 2026
 * Copyright: 2025-2026 - GOODDATA LABS
 * Description: Template code for the core emulation
 */

#include "emul.h"

#include <stdint.h>

// inclusw in the C file to avoid multiple definitions
#include "a2dp.h"
#include "aconfig.h"
#include "constants.h"
#include "debug.h"
#include "display.h"
#include "memfunc.h"
#include "pico/stdlib.h"
#include "reset.h"
#include "romemul.h"
#include "select.h"
#include "target_firmware.h"  // Include the target firmware binary
#include "term.h"

#define SLEEP_LOOP_MS 100

// Command handlers
static void cmdExit(const char *arg);
static void cmdBooster(const char *arg);
static void cmdA2dp(const char *arg);

// Command table
static const Command commands[] = {
    {"s", cmdA2dp},    {"S", cmdA2dp}, {"x", cmdBooster},
    {"X", cmdBooster}, {"e", cmdExit}, {"E", cmdExit},
};

// Number of commands in the table
static const size_t numCommands = sizeof(commands) / sizeof(commands[0]);

// Keep active loop or exit
static bool keepActive = true;
static bool menuScreenActive = false;
static absolute_time_t menuRefreshTime;

#define MENU_REFRESH_TIME_MS 1000

// Should we reset the device, or jump to the booster app?
// By default, we reset the device.
static bool resetDeviceAtBoot = true;

static void showTitle() {
  term_printString(
      "\x1B"
      "E"
      "MV16 Emulator - " RELEASE_VERSION "\n");
}

static void menu(void) {
  menuScreenActive = true;
  showTitle();
  term_printString("\n\n");
  term_printString("Starting scan by default...\n");
  term_printString("[S] Start scan\n");
  term_printString("[X] Launch booster\n");
  term_printString("[E] Exit to desktop\n");
  term_printString("\nSelect an option: ");
  term_markMenuPromptCursor();
  menuRefreshTime = make_timeout_time_ms(MENU_REFRESH_TIME_MS);
}

void cmdExit(const char *arg) {
  (void)arg;
  menuScreenActive = false;
  keepActive = false;
  term_printString("Exiting terminal...\n");
  // Send continue to desktop command
  SEND_COMMAND_TO_DISPLAY(DISPLAY_COMMAND_CONTINUE);
}

void cmdBooster(const char *arg) {
  (void)arg;
  menuScreenActive = false;
  term_printString("Launching Booster app...\n");
  term_printString("The computer will boot shortly...\n\n");
  term_printString("If it doesn't boot, power it on and off.\n");
  resetDeviceAtBoot = false;  // Jump to the booster app
  keepActive = false;         // Exit the active loop
}

void cmdA2dp(const char *arg) {
  (void)arg;
  menuScreenActive = false;

  term_printString("Launching scan mode...\n");
  term_printString("Press X to launch booster or E to exit desktop.\n");

  a2dp_source_demo_exit_action_t action = a2dp_source_demo_launch();
  switch (action) {
    case A2DP_SOURCE_DEMO_EXIT_BOOSTER:
      cmdBooster(NULL);
      break;
    case A2DP_SOURCE_DEMO_EXIT_DESKTOP:
      cmdExit(NULL);
      break;
    case A2DP_SOURCE_DEMO_EXIT_NONE:
    default:
      menu();
      break;
  }
}

// This section contains the functions that are called from the main loop

static bool getKeepActive() { return keepActive; }

static bool getResetDevice() { return resetDeviceAtBoot; }

static void preinit() {
  // Initialize the terminal
  term_init();

  // Clear the screen
  term_clearScreen();

  // Show the title
  showTitle();
  term_printString("\n\n");

  display_refresh();
}

void failure(const char *message) {
  // Initialize the terminal
  term_init();

  // Clear the screen
  term_clearScreen();

  // Show the title
  showTitle();
  term_printString("\n\n");
  term_printString(message);

  display_refresh();
}

static void init(void) {
  // Set the command table
  term_setCommands(commands, numCommands);

  // Clear the screen
  term_clearScreen();

  // Display the menu
  menu();
  cmdA2dp(NULL);

  // Example 1: Move the cursor up one line.
  // VT52 sequence: ESC A (moves cursor up)
  // The escape sequence "\x1BA" will move the cursor up one line.
  // term_printString("\x1B" "A");
  // After moving up, print text that overwrites part of the previous line.
  // term_printString("Line 2 (modified by ESC A)\n");

  // Example 2: Move the cursor right one character.
  // VT52 sequence: ESC C (moves cursor right)
  // term_printString("\x1B" "C");
  // term_printString(" <-- Moved right with ESC C\n");

  // Example 3: Direct cursor addressing.
  // VT52 direct addressing uses ESC Y <row> <col>, where:
  //   row_char = row + 0x20, col_char = col + 0x20.
  // For instance, to move the cursor to row 0, column 10:
  //   row: 0 -> 0x20 (' ')
  //   col: 10 -> 0x20 + 10 = 0x2A ('*')
  // term_printString("\x1B" "Y" "\x20" "\x2A");
  // term_printString("Text at row 0, column 10 via ESC Y\n");

  // term_printString("\x1B" "Y" "\x2A" "\x20");

  display_refresh();
}

void emul_start() {
  // The anatomy of an app or microfirmware is as follows:
  // - The driver code running in the remote device (the computer)
  // - the driver code running in the host device (the rp2040/rp2350)
  //
  // The driver code running in the remote device is responsible for:
  // 1. Perform the emulation of the device (ex: a ROM cartridge)
  // 2. Handle the communication with the host device
  // 3. Handle the configuration of the driver (ex: the ROM file to load)
  // 4. Handle the communication with the user (ex: the terminal)
  //
  // The driver code running in the host device is responsible for:
  // 1. Handle the communication with the remote device
  // 2. Handle the configuration of the driver (ex: the ROM file to load)
  // 3. Handle the communication with the user (ex: the terminal)
  //
  // Hence, we effectively have two drivers running in two different devices
  // with different architectures and capabilities.
  //
  // Please read the documentation to learn to use the communication protocol
  // between the two devices in the tprotocol.h file.
  //

  // As a rule of thumb, the remote device (the computer) driver code must
  // be copied to the RAM of the host device where the emulation will take
  // place.
  // The code is stored as an array in the target_firmware.h file
  //
  // Copy the terminal firmware to RAM
  COPY_FIRMWARE_TO_RAM((uint16_t *)target_firmware, target_firmware_length);

  // Initialize the terminal emulator PIO programs
  // The communication between the remote (target) computer and the RP2040 is
  // done using a command protocol over the cartridge bus
  // term_dma_irq_handler_lookup is the implementation of the terminal emulator
  // using the command protocol.
  // Hence, if you want to implement your own app or microfirmware, you should
  // implement your own command handler using this protocol.
  init_romemul(NULL, term_dma_irq_handler_lookup, false);

  // After this point, the remote computer can execute the code

  // 4. During the setup/configuration mode, the driver code must interact
  // with the user to configure the device. To simplify the process, the
  // terminal emulator is used to interact with the user.
  // The terminal emulator is a simple text-based interface that allows the
  // user to configure the device using text commands.
  // If you want to use a custom app in the remote computer, you can do it.
  // But it's easier to debug and code in the rp2040

  // Initialize the display
  display_setupU8g2();

  // Show startup menu and wait for user action.
  preinit();
  init();

  while (getKeepActive()) {
    sleep_ms(SLEEP_LOOP_MS);
    select_checkPushReset();
    term_loop();
  }

  sleep_ms(SLEEP_LOOP_MS);
  // We must reset the computer
  SEND_COMMAND_TO_DISPLAY(DISPLAY_COMMAND_RESET);
  sleep_ms(SLEEP_LOOP_MS);
  if (getResetDevice()) {
    // Reset the device
    reset_device();
  } else {
    // Jump to the booster app
    DPRINTF("Jumping to the booster app...\n");
    reset_jump_to_booster();
  }
  return;
}
