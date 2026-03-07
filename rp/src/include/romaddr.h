/**
 * File: romaddr.h
 * Author: Diego Parrilla Santamaria
 * Date: March 2026
 * Copyright: 2026 - GOODDATA LABS SL
 * Description: ROM3 bus capture into DMA ring buffer.
 */

#ifndef ROMADDR_H
#define ROMADDR_H

#include <stdbool.h>
#include <stdint.h>

#include "constants.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "romaddr.pio.h"

#define ROMADDR_RING_BUFFER_SAMPLES 1024u
#define ROMADDR_RING_BUFFER_MASK (ROMADDR_RING_BUFFER_SAMPLES - 1u)
#define ROMADDR_RING_BUFFER_BYTES (ROMADDR_RING_BUFFER_SAMPLES * 2u)
#if (ROMADDR_RING_BUFFER_SAMPLES & ROMADDR_RING_BUFFER_MASK) != 0
#error "ROMADDR_RING_BUFFER_SAMPLES must be a power of two"
#endif
#if (ROMADDR_RING_BUFFER_BYTES & (ROMADDR_RING_BUFFER_BYTES - 1u)) != 0
#error "ROMADDR_RING_BUFFER_BYTES must be a power of two"
#endif

int romaddr_init(void);
void romaddr_start(void);
void romaddr_stop(void);
void romaddr_deinit(void);
bool romaddr_is_running(void);

uint32_t __not_in_flash_func(romaddr_available)(void);
uint32_t __not_in_flash_func(romaddr_read)(uint16_t *dst, uint32_t max_samples);
uint32_t __not_in_flash_func(romaddr_get_overflow_count)(void);
void romaddr_consume_and_print_words(void);

#endif  // ROMADDR_H
