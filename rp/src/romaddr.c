/**
 * File: romaddr.c
 * Author: Diego Parrilla Santamaria
 * Date: March 2026
 * Copyright: 2026 - GOODDATA LABS SL
 * Description: ROM3 bus capture using PIO + DMA ring buffer.
 */

#include "romaddr.h"

#include <string.h>

#include "debug.h"

#define ROMADDR_DMA_TRANSFER_COUNT 0xFFFFFFFFu
#define ROMADDR_PRINT_BATCH_WORDS 16u

static PIO romaddrPio = pio0;
static int romaddrSm = -1;
static int romaddrDmaChannel = -1;
static int romaddrProgramOffset = -1;
static bool romaddrInitialized = false;
static bool romaddrRunning = false;

static volatile uint16_t __attribute__((aligned(
    ROMADDR_RING_BUFFER_BYTES))) romaddrRingBuffer[ROMADDR_RING_BUFFER_SAMPLES];
static uint32_t romaddrConsumedSamples = 0u;
static uint32_t romaddrDmaInitialTransferCount = ROMADDR_DMA_TRANSFER_COUNT;
static uint32_t romaddrOverflowCount = 0u;

static inline uint32_t romaddrRingIndex(uint32_t sampleIndex) {
  return sampleIndex & ROMADDR_RING_BUFFER_MASK;
}

static void romaddrSetGpioInputSio(uint pin) {
  gpio_set_function(pin, GPIO_FUNC_SIO);
  gpio_set_dir(pin, GPIO_IN);
  gpio_disable_pulls(pin);
}

static void romaddrConfigurePinsForCapture(void) {
  for (int i = 0; i < READ_ADDR_PIN_COUNT; i++) {
    uint pin = (uint)(READ_ADDR_GPIO_BASE + i);
    pio_gpio_init(romaddrPio, pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
  }

  pio_gpio_init(romaddrPio, ROMADDR_ROM3_GPIO);
  gpio_set_dir(ROMADDR_ROM3_GPIO, GPIO_IN);
  gpio_set_pulls(ROMADDR_ROM3_GPIO, true, false);
  gpio_pull_up(ROMADDR_ROM3_GPIO);

  pio_gpio_init(romaddrPio, ROMADDR_ROM4_GPIO);
  gpio_set_dir(ROMADDR_ROM4_GPIO, GPIO_IN);
  gpio_set_pulls(ROMADDR_ROM4_GPIO, true, false);
  gpio_pull_up(ROMADDR_ROM4_GPIO);

  // READ/WRITE are side-set controlled (inverted signals).
  pio_gpio_init(romaddrPio, READ_SIGNAL_GPIO_BASE);
  gpio_set_dir(READ_SIGNAL_GPIO_BASE, GPIO_OUT);
  gpio_set_pulls(READ_SIGNAL_GPIO_BASE, true, false);
  gpio_put(READ_SIGNAL_GPIO_BASE, 1);

  pio_gpio_init(romaddrPio, WRITE_SIGNAL_GPIO_BASE);
  gpio_set_dir(WRITE_SIGNAL_GPIO_BASE, GPIO_OUT);
  gpio_set_pulls(WRITE_SIGNAL_GPIO_BASE, true, false);
  gpio_put(WRITE_SIGNAL_GPIO_BASE, 1);
}

static void romaddrRestorePins(void) {
  for (int i = 0; i < READ_ADDR_PIN_COUNT; i++) {
    romaddrSetGpioInputSio((uint)(READ_ADDR_GPIO_BASE + i));
  }
  romaddrSetGpioInputSio(ROMADDR_ROM3_GPIO);
  romaddrSetGpioInputSio(ROMADDR_ROM4_GPIO);
  romaddrSetGpioInputSio(READ_SIGNAL_GPIO_BASE);
  romaddrSetGpioInputSio(WRITE_SIGNAL_GPIO_BASE);
}

static void romaddrConfigureDma(bool start) {
  dma_channel_config cfg =
      dma_channel_get_default_config((uint)romaddrDmaChannel);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg,
                          pio_get_dreq(romaddrPio, (uint)romaddrSm, false));
  channel_config_set_ring(&cfg, true,
                          (uint)__builtin_ctz(ROMADDR_RING_BUFFER_BYTES));

  dma_channel_configure(
      (uint)romaddrDmaChannel, &cfg, (void *)romaddrRingBuffer,
      &romaddrPio->rxf[(uint)romaddrSm], ROMADDR_DMA_TRANSFER_COUNT, start);
}

static uint32_t __not_in_flash_func(romaddrProducedSamples)(void) {
  if (romaddrDmaChannel < 0) {
    return 0u;
  }
  uint32_t remaining = dma_hw->ch[(uint)romaddrDmaChannel].transfer_count;
  return romaddrDmaInitialTransferCount - remaining;
}

int romaddr_init(void) {
  if (romaddrInitialized) {
    return 0;
  }

  memset((void *)romaddrRingBuffer, 0, sizeof(romaddrRingBuffer));
  romaddrConsumedSamples = 0u;
  romaddrOverflowCount = 0u;

  int addedOffset = pio_add_program(romaddrPio, &romaddr_capture_program);
  if (addedOffset < 0) {
    DPRINTF("ROMADDR: unable to add PIO program (%d)\n", addedOffset);
    return -1;
  }
  romaddrProgramOffset = addedOffset;

  int sm = pio_claim_unused_sm(romaddrPio, false);
  if (sm < 0) {
    DPRINTF("ROMADDR: no free PIO state machine\n");
    pio_remove_program(romaddrPio, &romaddr_capture_program,
                       (uint)romaddrProgramOffset);
    romaddrProgramOffset = -1;
    return -1;
  }
  romaddrSm = sm;

  int dmaChannel = dma_claim_unused_channel(false);
  if (dmaChannel < 0) {
    DPRINTF("ROMADDR: no free DMA channel\n");
    pio_sm_unclaim(romaddrPio, (uint)romaddrSm);
    romaddrSm = -1;
    pio_remove_program(romaddrPio, &romaddr_capture_program,
                       (uint)romaddrProgramOffset);
    romaddrProgramOffset = -1;
    return -1;
  }
  romaddrDmaChannel = dmaChannel;

  romaddrConfigurePinsForCapture();
  DPRINTF("ROMADDR: ring buf @0x%08lx (bytes=%u, align_mask=0x%X)\n",
          (unsigned long)romaddrRingBuffer,
          (unsigned int)ROMADDR_RING_BUFFER_BYTES,
          (unsigned int)(ROMADDR_RING_BUFFER_BYTES - 1u));
  romaddr_capture_program_init(romaddrPio, (uint)romaddrSm,
                               (uint)romaddrProgramOffset, READ_ADDR_GPIO_BASE,
                               READ_ADDR_PIN_COUNT, READ_SIGNAL_GPIO_BASE,
                               SAMPLE_DIV_FREQ);

  pio_sm_clear_fifos(romaddrPio, (uint)romaddrSm);
  pio_sm_restart(romaddrPio, (uint)romaddrSm);
  pio_sm_set_enabled(romaddrPio, (uint)romaddrSm, false);

  romaddrConfigureDma(false);
  dma_channel_abort((uint)romaddrDmaChannel);

  romaddrInitialized = true;
  return 0;
}

void romaddr_start(void) {
  DPRINTF("ROMADDR: start requested (initialized=%d running=%d)\n",
          (int)romaddrInitialized, (int)romaddrRunning);
  if ((!romaddrInitialized) || romaddrRunning) {
    DPRINTF("ROMADDR: start ignored\n");
    return;
  }

  memset((void *)romaddrRingBuffer, 0, sizeof(romaddrRingBuffer));
  romaddrConsumedSamples = 0u;
  romaddrOverflowCount = 0u;
  romaddrDmaInitialTransferCount = ROMADDR_DMA_TRANSFER_COUNT;
  DPRINTF("ROMADDR: ring reset, dma_count=%lu\n",
          (unsigned long)romaddrDmaInitialTransferCount);

  pio_sm_set_enabled(romaddrPio, (uint)romaddrSm, false);
  pio_sm_clear_fifos(romaddrPio, (uint)romaddrSm);
  pio_sm_restart(romaddrPio, (uint)romaddrSm);
  DPRINTF("ROMADDR: SM %d reset\n", romaddrSm);

  dma_channel_abort((uint)romaddrDmaChannel);
  romaddrConfigureDma(true);
  DPRINTF("ROMADDR: DMA channel %d configured and started\n",
          romaddrDmaChannel);
  pio_sm_set_enabled(romaddrPio, (uint)romaddrSm, true);
  DPRINTF("ROMADDR: SM %d enabled\n", romaddrSm);

  romaddrRunning = true;
  DPRINTF("ROMADDR: start complete\n");
}

void romaddr_stop(void) {
  if ((!romaddrInitialized) || (!romaddrRunning)) {
    return;
  }

  dma_channel_abort((uint)romaddrDmaChannel);
  pio_sm_set_enabled(romaddrPio, (uint)romaddrSm, false);
  pio_sm_clear_fifos(romaddrPio, (uint)romaddrSm);
  pio_sm_restart(romaddrPio, (uint)romaddrSm);

  romaddrRunning = false;
}

void romaddr_deinit(void) {
  if (!romaddrInitialized) {
    return;
  }

  romaddr_stop();

  if (romaddrDmaChannel >= 0) {
    dma_channel_unclaim((uint)romaddrDmaChannel);
    romaddrDmaChannel = -1;
  }
  if (romaddrSm >= 0) {
    pio_sm_unclaim(romaddrPio, (uint)romaddrSm);
    romaddrSm = -1;
  }
  if (romaddrProgramOffset >= 0) {
    pio_remove_program(romaddrPio, &romaddr_capture_program,
                       (uint)romaddrProgramOffset);
    romaddrProgramOffset = -1;
  }

  romaddrRestorePins();
  romaddrInitialized = false;
}

bool romaddr_is_running(void) { return romaddrRunning; }

uint32_t __not_in_flash_func(romaddr_available)(void) {
  if ((!romaddrInitialized) || (!romaddrRunning)) {
    return 0u;
  }

  uint32_t produced = romaddrProducedSamples();
  if (produced < romaddrConsumedSamples) {
    romaddrConsumedSamples = produced;
    return 0u;
  }

  uint32_t available = produced - romaddrConsumedSamples;
  if (available > ROMADDR_RING_BUFFER_SAMPLES) {
    uint32_t dropped = available - ROMADDR_RING_BUFFER_SAMPLES;
    romaddrOverflowCount += dropped;
    romaddrConsumedSamples = produced - ROMADDR_RING_BUFFER_SAMPLES;
    available = ROMADDR_RING_BUFFER_SAMPLES;
  }

  return available;
}

uint32_t __not_in_flash_func(romaddr_read)(uint16_t *dst,
                                           uint32_t max_samples) {
  if ((dst == NULL) || (max_samples == 0u)) {
    return 0u;
  }

  uint32_t available = romaddr_available();
  if (available == 0u) {
    return 0u;
  }

  uint32_t toRead = (max_samples < available) ? max_samples : available;
  uint32_t consumed = romaddrConsumedSamples;

  for (uint32_t i = 0; i < toRead; i++) {
    dst[i] = romaddrRingBuffer[romaddrRingIndex(consumed + i)];
  }

  romaddrConsumedSamples = consumed + toRead;
  return toRead;
}

uint32_t __not_in_flash_func(romaddr_get_overflow_count)(void) {
  return romaddrOverflowCount;
}

void romaddr_consume_and_print_words(void) {
  uint16_t words[ROMADDR_PRINT_BATCH_WORDS];
  char line[160];
  uint32_t count = 0u;

  while ((count = romaddr_read(words, ROMADDR_PRINT_BATCH_WORDS)) > 0u) {
    size_t used = 0u;
    used += (size_t)snprintf(line + used, sizeof(line) - used, "ROM3:");
    for (uint32_t i = 0; (i < count) && (used < sizeof(line)); i++) {
      used +=
          (size_t)snprintf(line + used, sizeof(line) - used, " %04X", words[i]);
    }
    (void)snprintf(line + used, sizeof(line) - used, "\n");
    DPRINTF("%s", line);
  }
}
