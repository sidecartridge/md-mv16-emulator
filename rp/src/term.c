/**
 * File: term.c
 * Author: Diego Parrilla Santamaría
 * Date: January 2025, February 2026
 * Copyright: 2025-2026 - GOODDATA LABS
 * Description: Online terminal
 */

#include "term.h"

#include <ctype.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "a2dp.h"
#include "constants.h"
#include "debug.h"
#include "display.h"
#include "display_term.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "memfunc.h"
#include "romemul.h"
#include "tprotocol.h"

static TransmissionProtocol protocolBuffers[2];
static volatile uint8_t protocolReadIndex = 0;
static volatile uint8_t protocolWriteIndex = 1;
static volatile bool protocolBufferReady = false;
static volatile uint32_t protocolOverwriteCount = 0;
static uint8_t lookupDmaChannelCached = 0u;
static bool lookupDmaChannelCachedValid = false;

static uint32_t memorySharedAddress = 0;
static uint32_t memoryRandomTokenAddress = 0;
static uint32_t memoryRandomTokenSeedAddress = 0;

// Command table
static const Command *commands;

// Number of commands in the table
static size_t numCommands = 0;

// Setter for commands and numCommands
void term_setCommands(const Command *cmds, size_t count) {
  commands = cmds;
  numCommands = count;
}

/**
 * @brief Callback that handles the protocol command received.
 *
 * This callback copy the content of the protocol to the last_protocol
 * structure. The last_protocol_valid flag is set to true to indicate that the
 * last_protocol structure contains a valid protocol. We return to the
 * dma_irq_handler_lookup function to continue asap with the next
 *
 * @param protocol The TransmissionProtocol structure containing the protocol
 * information.
 */
static inline void __not_in_flash_func(handle_protocol_command)(
    const TransmissionProtocol *protocol) {
  uint8_t writeIndex = protocolWriteIndex;
  TransmissionProtocol *writeBuffer = &protocolBuffers[writeIndex];

  // Copy the content of protocol to the active write buffer.
  writeBuffer->command_id = protocol->command_id;
  writeBuffer->payload_size = protocol->payload_size;
  writeBuffer->bytes_read = protocol->bytes_read;
  writeBuffer->final_checksum = protocol->final_checksum;

  // Sanity check: clamp payload_size to avoid overflow
  uint16_t size = protocol->payload_size;
  if (size > MAX_PROTOCOL_PAYLOAD_SIZE) {
    size = MAX_PROTOCOL_PAYLOAD_SIZE;
  }

  // Copy only used payload bytes
  memcpy(writeBuffer->payload, protocol->payload, size);

  if (protocolBufferReady) {
    protocolOverwriteCount++;
  }

  // Publish atomically by swapping read/write roles.
  uint8_t readIndex = protocolReadIndex;
  protocolReadIndex = writeIndex;
  protocolWriteIndex = readIndex;
  protocolBufferReady = true;
};

static inline void __not_in_flash_func(handle_protocol_checksum_error)(
    const TransmissionProtocol *protocol) {
  DPRINTF("Checksum error detected (ID=%u, Size=%u)\n", protocol->command_id,
          protocol->payload_size);
}

static inline bool __not_in_flash_func(term_get_lookup_dma_channel_isr)(
    uint *lookupChannelOut) {
  if (lookupChannelOut == NULL) {
    return false;
  }

  if (__builtin_expect(lookupDmaChannelCachedValid, 1)) {
    *lookupChannelOut = (uint)lookupDmaChannelCached;
    return true;
  }

  int lookupChannel = romemul_getLookupDataRomDmaChannel();
  if ((lookupChannel < 0) || ((uint)lookupChannel >= NUM_DMA_CHANNELS)) {
    return false;
  }

  lookupDmaChannelCached = (uint8_t)lookupChannel;
  lookupDmaChannelCachedValid = true;
  *lookupChannelOut = (uint)lookupDmaChannelCached;
  return true;
}

// Interrupt handler for DMA completion
void __not_in_flash_func(term_dma_irq_handler_lookup)(void) {
  uint lookupChannel = 0u;
  if (!term_get_lookup_dma_channel_isr(&lookupChannel)) {
    return;
  }

  // Read the rom3 signal and if so then process the command
  dma_hw->ints1 = 1u << lookupChannel;

  // Read once to avoid redundant hardware access
  uint32_t addr = dma_hw->ch[lookupChannel].al3_read_addr_trig;

  // Check ROM3 signal (bit 16).
  if (__builtin_expect(addr & 0x00010000u, 0)) {
    // Invert highest bit of low word to get 16-bit address.
    uint16_t addrLsb = (uint16_t)(addr ^ ADDRESS_HIGH_BIT);
    tprotocol_parse(addrLsb, handle_protocol_command,
                    handle_protocol_checksum_error);
  }
}

void __not_in_flash_func(stream_dma_irq_handler_lookup)(void) {
  uint lookupChannel = 0u;
  if (!term_get_lookup_dma_channel_isr(&lookupChannel)) {
    return;
  }

  dma_hw->ints1 = 1u << lookupChannel;
  uint32_t addr = dma_hw->ch[lookupChannel].al3_read_addr_trig;

  // Interpret lookup payload as unsigned 12-bit sample (0..4095) and map to
  // signed PCM16 centered at 0.
  uint16_t raw12 = (uint16_t)(addr & 0x1FFEu);
  int16_t sample = (raw12 - 2048) << 3;
  (void)a2dp_source_demo_pcm_write_mono(&sample, 1u);
}

static char screen[TERM_SCREEN_SIZE];
static uint8_t cursorX = 0;
static uint8_t cursorY = 0;

// Store previous cursor position for block removal
static uint8_t prevCursorX = 0;
static uint8_t prevCursorY = 0;

// Buffer to keep track of chars entered between newlines
static char inputBuffer[TERM_INPUT_BUFFER_SIZE];
static size_t inputLength = 0;
static term_raw_key_handler_t termRawKeyHandler = NULL;

#define TERM_SCAN_1 0x02
#define TERM_SCAN_0 0x0B
#define TERM_SCAN_BACKSPACE 0x0E
#define TERM_SCAN_ENTER 0x1C
#define TERM_SCAN_C 0x2E
#define TERM_SCAN_KP_7 0x67
#define TERM_SCAN_KP_0 0x70
#define TERM_SCAN_KP_ENTER 0x72

void term_setRawKeyHandler(term_raw_key_handler_t handler) {
  termRawKeyHandler = handler;
}

// Clears entire screen buffer and resets cursor
void term_clearScreen(void) {
  memset(screen, 0, TERM_SCREEN_SIZE);
  cursorX = 0;
  cursorY = 0;
  display_termClear();
}

// Clears the input buffer
void term_clearInputBuffer(void) {
  memset(inputBuffer, 0, TERM_INPUT_BUFFER_SIZE);
  inputLength = 0;
}

// Custom scrollup function to scroll except for the last row
void termScrollupBuffer(uint16_t blankBytes) {
  // blank bytes is the number of bytes to blank out at the bottom of the
  // screen

  unsigned char *u8g2Buffer = u8g2_GetBufferPtr(display_getU8g2Ref());
  memmove(u8g2Buffer, u8g2Buffer + blankBytes,
          DISPLAY_BUFFER_SIZE - blankBytes -
              TERM_SCREEN_SIZE_X * DISPLAY_TERM_CHAR_HEIGHT);
  memset(u8g2Buffer + DISPLAY_BUFFER_SIZE - blankBytes -
             TERM_SCREEN_SIZE_X * DISPLAY_TERM_CHAR_HEIGHT,
         0, blankBytes);
}

// Scrolls the screen up by one row
static void termScrollUp(void) {
  memmove(screen, screen + TERM_SCREEN_SIZE_X,
          TERM_SCREEN_SIZE - TERM_SCREEN_SIZE_X);
  memset(screen + TERM_SCREEN_SIZE - TERM_SCREEN_SIZE_X, 0, TERM_SCREEN_SIZE_X);
  termScrollupBuffer(TERM_DISPLAY_ROW_BYTES);
}

// Prints a character to the screen, handles scrolling
static void termPutChar(char chr) {
  screen[cursorY * TERM_SCREEN_SIZE_X + cursorX] = chr;
  display_termChar(cursorX, cursorY, chr);
  cursorX++;
  if (cursorX >= TERM_SCREEN_SIZE_X) {
    cursorX = 0;
    cursorY++;
    if (cursorY >= TERM_SCREEN_SIZE_Y) {
      termScrollUp();
      cursorY = TERM_SCREEN_SIZE_Y - 1;
    }
  }
}

// Renders a single character, with special handling for newline and carriage
// return
static void termRenderChar(char chr) {
  // First, remove the old block by restoring the character
  display_termChar(prevCursorX, prevCursorY, ' ');
  if (chr == '\n' || chr == '\r') {
    // Move to new line
    cursorX = 0;
    cursorY++;
    if (cursorY >= TERM_SCREEN_SIZE_Y) {
      termScrollUp();
      cursorY = TERM_SCREEN_SIZE_Y - 1;
    }
  } else if (chr != '\0') {
    termPutChar(chr);
  }

  // Draw a block at the new cursor position
  display_termCursor(cursorX, cursorY);
  // Update previous cursor coords
  prevCursorX = cursorX;
  prevCursorY = cursorY;
}

/**
 * @brief Processes a complete VT52 escape sequence.
 *
 * This function interprets the VT52 sequence stored in `seq` (with given
 * length) and performs the corresponding cursor movements. Modify the TODO
 * sections to implement additional features as needed.
 *
 * @param seq Pointer to the escape sequence buffer.
 * @param length The length of the escape sequence.
 */
static void vt52ProcessSequence(const char *seq, size_t length) {
  // Ensure we have at least an ESC and a command character.
  if (length < 2) return;

  char command = seq[1];
  switch (command) {
    case 'A':  // Move cursor up
      // TODO(diego): Improve behavior if needed.
      if (cursorY > 0) {
        cursorY--;
      }
      termRenderChar('\0');
      break;
    case 'B':  // Move cursor down
      if (cursorY < TERM_SCREEN_SIZE_Y - 1) {
        cursorY++;
      }
      termRenderChar('\0');
      break;
    case 'C':  // Move cursor right
      if (cursorX < TERM_SCREEN_SIZE_X - 1) {
        cursorX++;
      }
      termRenderChar('\0');
      break;
    case 'D':  // Move cursor left
      if (cursorX > 0) {
        cursorX--;
      }
      termRenderChar('\0');
      break;
    case 'E':  // Clear screen and place cursor at top left corner
      cursorX = 0;
      cursorY = 0;
      termRenderChar('\0');
      for (int posY = 0; posY < TERM_SCREEN_SIZE_Y; posY++) {
        for (int posX = 0; posX < TERM_SCREEN_SIZE_X; posX++) {
          screen[posY * TERM_SCREEN_SIZE_X + posX] = 0;
          display_termChar(posX, posY, ' ');
        }
      }
      break;
    case 'H':  // Cursor home
      cursorX = 0;
      cursorY = 0;
      termRenderChar('\0');
      break;
    case 'J':  // Erases from the current cursor position to the end of the
               // screen
      for (int posY = cursorY; posY < TERM_SCREEN_SIZE_Y; posY++) {
        for (int posX = cursorX; posX < TERM_SCREEN_SIZE_X; posX++) {
          screen[posY * TERM_SCREEN_SIZE_X + posX] = 0;
          display_termChar(posX, posY, ' ');
        }
      }
      break;
    case 'K':  // Clear to end of line
      for (int posX = cursorX; posX < TERM_SCREEN_SIZE_X; posX++) {
        screen[cursorY * TERM_SCREEN_SIZE_X + posX] = 0;
        display_termChar(posX, cursorY, ' ');
      }
      break;
    case 'Y':  // Direct cursor addressing: ESC Y <row> <col>
      if (length == 4) {
        int row = seq[2] - TERM_POS_Y;
        int col = seq[3] - TERM_POS_X;
        if (row >= 0 && row < TERM_SCREEN_SIZE_Y && col >= 0 &&
            col < TERM_SCREEN_SIZE_X) {
          cursorY = row;
          cursorX = col;
        }
        termRenderChar('\0');
      }
      break;
    default:
      // Unrecognized sequence. Optionally, print or ignore.
      // For now, we'll ignore it.
      break;
  }
}

void term_printString(const char *str) {
  enum { STATE_NORMAL, STATE_ESC } state = STATE_NORMAL;
  char escBuffer[TERM_ESC_BUFFLINE_SIZE];
  size_t escLen = 0;

  while (*str) {
    char chr = *str;
    if (state == STATE_NORMAL) {
      if (chr == TERM_ESC_CHAR) {  // ESC character detected
        state = STATE_ESC;
        escLen = 0;
        escBuffer[escLen++] = chr;
      } else {
        termRenderChar(chr);
      }
    } else {  // STATE_ESC: we're accumulating an escape sequence
      escBuffer[escLen++] = chr;
      // Check for sequence completion:
      // Most VT52 sequences are two characters (ESC + command)...
      if (escLen == 2) {
        if (escBuffer[1] == 'Y') {
          // ESC Y requires two more characters (for row and col)
          // Do nothing now—wait until esc_len == 4.
        } else {
          // Sequence complete (ESC + single command)
          vt52ProcessSequence(escBuffer, escLen);
          state = STATE_NORMAL;
        }
      } else if (escBuffer[1] == 'Y' && escLen == 4) {
        // ESC Y <row> <col> sequence complete
        vt52ProcessSequence(escBuffer, escLen);
        state = STATE_NORMAL;
      }
      // In case the buffer gets too long, flush it as normal text
      if (escLen >= sizeof(escBuffer)) {
        for (size_t i = 0; i < escLen; i++) {
          termRenderChar(escBuffer[i]);
        }
        state = STATE_NORMAL;
      }
    }
    str++;
  }
  // If the string ends while still in ESC state, flush the buffered
  // characters as normal text.
  if (state == STATE_ESC) {
    for (size_t i = 0; i < escLen; i++) {
      termRenderChar(escBuffer[i]);
    }
  }
  display_termRefresh();
}

// Called whenever a character is entered by the user
// This is the single point of entry for user input
static void termInputChar(char chr) {
  if ((termRawKeyHandler != NULL) && termRawKeyHandler(chr)) {
    return;
  }

  // Check for backspace
  if (chr == '\b') {
    display_termChar(prevCursorX, prevCursorY, ' ');

    // If we have chars in input_buffer, remove last char
    if (inputLength > 0) {
      inputLength--;
      inputBuffer[inputLength] = '\0';  // Null-terminate the string
      // Also reflect on screen
      // same as term_backspace
      if (cursorX == 0) {
        if (cursorY > 0) {
          cursorY--;
          cursorX = TERM_SCREEN_SIZE_X - 1;
        } else {
          // already top-left corner
          return;
        }
      } else {
        cursorX--;
      }
      screen[cursorY * TERM_SCREEN_SIZE_X + cursorX] = 0;
      display_termChar(cursorX, cursorY, ' ');
    }

    display_termCursor(cursorX, cursorY);
    prevCursorX = cursorX;
    prevCursorY = cursorY;
    display_termRefresh();
    return;
  }

  // If it's newline or carriage return, finalize the line
  if (chr == '\n' || chr == '\r') {
    // Render newline on screen
    termRenderChar('\n');

    // Process input_buffer
    // Split the input into command and argument
    char command[TERM_INPUT_BUFFER_SIZE] = {0};
    char arg[TERM_INPUT_BUFFER_SIZE] = {0};
    sscanf(inputBuffer, "%63s %63[^\n]", command,
           arg);  // Split at the first space

    bool commandFound = false;
    for (size_t i = 0; i < numCommands; i++) {
      if (strcmp(command, commands[i].command) == 0) {
        commands[i].handler(arg);  // Pass the argument to the handler
        commandFound = true;
      }
    }
    if ((!commandFound) && (strlen(command) > 0)) {
      // The custom unknown command manager is called when the command is empty
      // in the command table. This is useful to manage custom entries.
      for (size_t i = 0; i < numCommands; i++) {
        if (strlen(commands[i].command) == 0) {
          commands[i].handler(inputBuffer);  // Pass the argument to the handler
        }
      }
    }

    // Reset input buffer
    memset(inputBuffer, 0, TERM_INPUT_BUFFER_SIZE);
    inputLength = 0;

    term_printString("> ");
    display_termRefresh();
    return;
  }

  // If it's a normal character
  // Add it to input_buffer if there's space
  if (inputLength < TERM_INPUT_BUFFER_SIZE - 1) {
    inputBuffer[inputLength++] = chr;
    // Render char on screen
    termRenderChar(chr);

    // show block cursor

    display_termRefresh();
  } else {
    // Buffer full, ignore or beep?
  }
}

static char termNormalizeKeystroke(char keystroke, uint8_t scanCode) {
  if (keystroke != 0) {
    return keystroke;
  }

  if ((scanCode >= TERM_SCAN_1) && (scanCode <= TERM_SCAN_0)) {
    if (scanCode == TERM_SCAN_0) {
      return '0';
    }
    return (char)('1' + (scanCode - TERM_SCAN_1));
  }

  switch (scanCode) {
    case 0x6D:
      return '1';
    case 0x6E:
      return '2';
    case 0x6F:
      return '3';
    case 0x6A:
      return '4';
    case 0x6B:
      return '5';
    case 0x6C:
      return '6';
    case TERM_SCAN_KP_7:
      return '7';
    case 0x68:
      return '8';
    case 0x69:
      return '9';
    case TERM_SCAN_KP_0:
      return '0';
    case TERM_SCAN_ENTER:
    case TERM_SCAN_KP_ENTER:
      return '\r';
    case TERM_SCAN_BACKSPACE:
      return '\b';
    case TERM_SCAN_C:
      return 'c';
    default:
      return 0;
  }
}

void term_init(void) {
  int lookupChannel = romemul_getLookupDataRomDmaChannel();
  if ((lookupChannel >= 0) && ((uint)lookupChannel < NUM_DMA_CHANNELS)) {
    lookupDmaChannelCached = (uint8_t)lookupChannel;
    lookupDmaChannelCachedValid = true;
  }

  // Memory shared address
  memorySharedAddress = (unsigned int)&__rom_in_ram_start__;
  memoryRandomTokenAddress = memorySharedAddress + TERM_RANDOM_TOKEN_OFFSET;
  memoryRandomTokenSeedAddress =
      memorySharedAddress + TERM_RANDON_TOKEN_SEED_OFFSET;
  SET_SHARED_VAR(TERM_HARDWARE_TYPE, 0, memorySharedAddress,
                 TERM_SHARED_VARIABLES_OFFSET);  // Clean the hardware type
  SET_SHARED_VAR(TERM_HARDWARE_VERSION, 0, memorySharedAddress,
                 TERM_SHARED_VARIABLES_OFFSET);  // Clean the hardware version

  // Initialize the random seed (add this line)
  srand(time(NULL));
  // Init the random token seed in the shared memory for the next command
  uint32_t newRandomSeedToken = rand();  // Generate a new random 32-bit value
  TPROTO_SET_RANDOM_TOKEN(memoryRandomTokenSeedAddress, newRandomSeedToken);

  // Initialize the welcome messages
  term_clearScreen();
  display_refresh();
}

// Invoke this function to process the commands from the active loop in the
// main function
void __not_in_flash_func(term_loop)() {
  TransmissionProtocol protocolSnapshot = {0};
  bool protocolReady = false;
  uint32_t overwriteCountSnapshot = 0;

  // Minimal critical section: atomically snapshot the latest published slot.
  // Easy to revert if we switch to a queue/ring-buffer approach later.
  uint32_t irqState = save_and_disable_interrupts();
  if (protocolBufferReady) {
    uint8_t readIndex = protocolReadIndex;
    protocolSnapshot = protocolBuffers[readIndex];
    protocolBufferReady = false;
    protocolReady = true;
  }
  overwriteCountSnapshot = protocolOverwriteCount;
  restore_interrupts(irqState);

  if (protocolReady) {
    // Shared by all commands
    // Read the random token from the command and increment the payload
    // pointer to the first parameter available in the payload
    uint32_t randomToken = TPROTO_GET_RANDOM_TOKEN(protocolSnapshot.payload);
    uint16_t *payloadPtr = ((uint16_t *)(protocolSnapshot).payload);
    uint16_t commandId = protocolSnapshot.command_id;
    DPRINTF(
        "Command ID: %d. Size: %d. Random token: 0x%08X, Checksum: 0x%04X, "
        "Overwrites: %lu\n",
        protocolSnapshot.command_id, protocolSnapshot.payload_size, randomToken,
        protocolSnapshot.final_checksum, (unsigned long)overwriteCountSnapshot);

#if defined(_DEBUG) && (_DEBUG != 0)
    // Jump the random token
    TPROTO_NEXT32_PAYLOAD_PTR(payloadPtr);

    // Read the payload parameters
    uint16_t payloadSizeTmp = 4;
    if ((protocolSnapshot.payload_size > payloadSizeTmp) &&
        (protocolSnapshot.payload_size <= TERM_PARAMETERS_MAX_SIZE)) {
      DPRINTF("Payload D3: 0x%04X\n", TPROTO_GET_PAYLOAD_PARAM32(payloadPtr));
      TPROTO_NEXT32_PAYLOAD_PTR(payloadPtr);
    }
    payloadSizeTmp += 4;
    if ((protocolSnapshot.payload_size > payloadSizeTmp) &&
        (protocolSnapshot.payload_size <= TERM_PARAMETERS_MAX_SIZE)) {
      DPRINTF("Payload D4: 0x%04X\n", TPROTO_GET_PAYLOAD_PARAM32(payloadPtr));
      TPROTO_NEXT32_PAYLOAD_PTR(payloadPtr);
    }
    payloadSizeTmp += 4;
    if ((protocolSnapshot.payload_size > payloadSizeTmp) &&
        (protocolSnapshot.payload_size <= TERM_PARAMETERS_MAX_SIZE)) {
      DPRINTF("Payload D5: 0x%04X\n", TPROTO_GET_PAYLOAD_PARAM32(payloadPtr));
      TPROTO_NEXT32_PAYLOAD_PTR(payloadPtr);
    }
    payloadSizeTmp += 4;
    if ((protocolSnapshot.payload_size > payloadSizeTmp) &&
        (protocolSnapshot.payload_size <= TERM_PARAMETERS_MAX_SIZE)) {
      DPRINTF("Payload D6: 0x%04X\n", TPROTO_GET_PAYLOAD_PARAM32(payloadPtr));
      TPROTO_NEXT32_PAYLOAD_PTR(payloadPtr);
    }
#endif

    // Handle the command
    switch (protocolSnapshot.command_id) {
      case APP_TERMINAL_KEYSTROKE: {
        uint16_t *payload = ((uint16_t *)(protocolSnapshot).payload);
        // Jump the random token
        TPROTO_NEXT32_PAYLOAD_PTR(payload);
        // Extract the 32 bit payload
        uint32_t payload32 = TPROTO_GET_PAYLOAD_PARAM32(payload);
        // Extract the ascii code from the payload lower 8 bits
        char keystroke = (char)(payload32 & TERM_KEYBOARD_KEY_MASK);
        // Get the shift key status from the higher byte of the payload
        uint8_t shiftKey =
            (payload32 & TERM_KEYBOARD_SHIFT_MASK) >> TERM_KEYBOARD_SHIFT_SHIFT;
        // Get the keyboard scan code from the bits 16 to 23 of the payload
        uint8_t scanCode =
            (payload32 & TERM_KEYBOARD_SCAN_MASK) >> TERM_KEYBOARD_SCAN_SHIFT;
        char normalizedKeystroke = termNormalizeKeystroke(keystroke, scanCode);
        if (keystroke >= TERM_KEYBOARD_KEY_START &&
            keystroke <= TERM_KEYBOARD_KEY_END) {
          // Print the keystroke and the shift key status
          DPRINTF("Keystroke: %c. Shift key: %d, Scan code: %d\n", keystroke,
                  shiftKey, scanCode);
        } else {
          // Print the keystroke and the shift key status
          DPRINTF("Keystroke: %d. Shift key: %d, Scan code: %d\n", keystroke,
                  shiftKey, scanCode);
        }
        if (normalizedKeystroke != 0) {
          termInputChar(normalizedKeystroke);
        }
        break;
      }
      default:
        // Unknown command
        DPRINTF("Unknown command\n");
        break;
    }
    if (memoryRandomTokenAddress != 0) {
      // Set the random token in the shared memory
      TPROTO_SET_RANDOM_TOKEN(memoryRandomTokenAddress, randomToken);

      // Init the random token seed in the shared memory for the next command
      uint32_t newRandomSeedToken =
          rand();  // Generate a new random 32-bit value
      TPROTO_SET_RANDOM_TOKEN(memoryRandomTokenSeedAddress, newRandomSeedToken);
    }
  }
}

void term_markMenuPromptCursor(void) {
  // Kept for API compatibility with callers that mark the prompt position.
  // Live row refresh is not used in the current build.
  (void)cursorX;
  (void)cursorY;
}
