/**
 * File: a2dp.h
 * Author: Diego Parrilla Santamaria
 * Date: March 2026
 * Copyright: 2026 - GOODDATA LABS SL
 * Description: Minimal A2DP source test helpers
 */

#ifndef A2DP_H
#define A2DP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  A2DP_SOURCE_DEMO_EXIT_NONE = 0,
  A2DP_SOURCE_DEMO_EXIT_BOOSTER,
  A2DP_SOURCE_DEMO_EXIT_DESKTOP,
} a2dp_source_demo_exit_action_t;

/**
 * @brief Launches a minimal A2DP source demo that streams a PCM sample.
 *
 * This call is blocking while BTstack run loop is active.
 */
a2dp_source_demo_exit_action_t a2dp_source_demo_launch(void);

/**
 * @brief Writes mono PCM16 samples into the A2DP streaming ring buffer.
 *
 * The ring buffer size is 4096 bytes (2048 mono samples). The return value
 * is the number of samples accepted.
 */
uint32_t a2dp_source_demo_pcm_write_mono(const int16_t *samples,
                                         uint32_t sampleCount);

#endif  // A2DP_H
