/**
 * File: a2dp.c
 * Author: Diego Parrilla Santamaria
 * Date: March 2026
 * Copyright: 2026 - GOODDATA LABS SL
 * Description: Minimal A2DP source demo (PCM sample playback)
 */

#include "a2dp.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "a2dp_pcm_sample.h"
#include "btstack.h"
#include "display.h"
#include "pico/cyw43_arch.h"
#include "romaddr.h"
#include "romemul.h"
#include "select.h"
#include "term.h"

#define A2DP_DEMO_INQUIRY_DURATION_1280MS 8
#define A2DP_DEMO_PREFERRED_SAMPLING_RATE A2DP_PCM_SAMPLE_RATE_HZ
#define A2DP_DEMO_AUDIO_TIMEOUT_MS 10
#define A2DP_DEMO_POST_CONNECT_ACTION_TIMEOUT_MS 5000u
#define A2DP_DEMO_NUM_CHANNELS 1
#define A2DP_DEMO_PCM_FRAME_MAX_SAMPLES 256
#define A2DP_DEMO_PCM_RING_BUFFER_SIZE_BYTES 4096u
#define A2DP_DEMO_PCM_SAMPLE_SIZE_BYTES 2u
#define A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES \
  (A2DP_DEMO_PCM_RING_BUFFER_SIZE_BYTES / A2DP_DEMO_PCM_SAMPLE_SIZE_BYTES)
#define A2DP_DEMO_PCM_RING_MASK (A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES - 1u)
#define A2DP_DEMO_PCM_RING_INDEX(cursor) \
  ((uint32_t)(cursor) & A2DP_DEMO_PCM_RING_MASK)
#if (A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES & A2DP_DEMO_PCM_RING_MASK) != 0
#error "A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES must be a power of two"
#endif
#define A2DP_DEMO_SBC_STORAGE_SIZE 1030

#define A2DP_DEMO_RETRY_COOLDOWN_MS 30000u
#define A2DP_DEMO_MAX_DISCOVERED_DEVICES 24
#define A2DP_DEMO_DEVICE_NAME_SIZE 64
#define A2DP_DEMO_SELECTION_INPUT_MAX_LEN 3
#define A2DP_DEMO_TERM_POLL_INTERVAL_MS 20
#define A2DP_DEMO_ROMADDR_GAIN_Q15 0 // No gain, no pain
#define A2DP_DEMO_ROMADDR_LPF_ALPHA_Q15 9000 

#if (A2DP_PCM_SAMPLE_CHANNELS != A2DP_DEMO_NUM_CHANNELS)
#error "A2DP PCM sample channels do not match A2DP_DEMO_NUM_CHANNELS"
#endif

#define A2DP_TERM_TRACE_ENABLED 0

#define A2DP_TRACE(fmt, ...)       \
  do {                             \
    if (A2DP_TERM_TRACE_ENABLED) { \
      TPRINTF(fmt, ##__VA_ARGS__); \
    }                              \
    DPRINTF(fmt, ##__VA_ARGS__);   \
  } while (0)

#define A2DP_TERM(fmt, ...)      \
  do {                           \
    TPRINTF(fmt, ##__VA_ARGS__); \
  } while (0)

typedef struct {
  bd_addr_t address;
  char name[A2DP_DEMO_DEVICE_NAME_SIZE];
  uint32_t cod;
  bool supports_sink;
} a2dp_demo_discovered_device_t;

typedef struct {
  bool scanning;
  bool scan_cancel_requested;
  bool waiting_for_selection;
  bool connect_request_pending;
  a2dp_demo_discovered_device_t
      discovered_devices[A2DP_DEMO_MAX_DISCOVERED_DEVICES];
  uint8_t discovered_device_count;
  uint8_t sink_device_indices[A2DP_DEMO_MAX_DISCOVERED_DEVICES];
  uint8_t sink_device_count;
  char selection_input[A2DP_DEMO_SELECTION_INPUT_MAX_LEN + 1u];
  uint8_t selection_input_len;
  bd_addr_t selected_sink_addr;
  char selected_sink_name[A2DP_DEMO_DEVICE_NAME_SIZE];
} a2dp_demo_scan_context_t;

typedef struct {
  uint16_t a2dp_cid;
  uint8_t local_seid;
  uint8_t remote_seid;
  bool stream_opened;
  bool streaming;
  bool waiting_for_post_connect_action;
  bool connecting;
  uint32_t post_connect_action_deadline_ms;
  bd_addr_t last_candidate_addr;
  bd_addr_t cooldown_addr;
  bool cooldown_active;
  uint32_t cooldown_until_ms;
  uint16_t max_media_payload_size;
  uint32_t rtp_timestamp;
  uint32_t samples_ready;
  uint32_t acc_num_missed_samples;
  uint32_t time_audio_data_sent;
  uint8_t sbc_storage[A2DP_DEMO_SBC_STORAGE_SIZE];
  uint16_t sbc_storage_count;
  bool sbc_ready_to_send;
  btstack_timer_source_t audio_timer;
} a2dp_demo_link_context_t;

typedef struct {
  a2dp_demo_scan_context_t scan;
  a2dp_demo_link_context_t link;
  bool term_poll_active;
  btstack_timer_source_t term_poll_timer;
} a2dp_demo_context_t;

typedef struct {
  int sampling_frequency;
  int block_length;
  int subbands;
  int max_bitpool_value;
  btstack_sbc_channel_mode_t channel_mode;
  btstack_sbc_allocation_method_t allocation_method;
} a2dp_demo_sbc_configuration_t;

static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t sdp_a2dp_source_service_buffer[150];
static uint8_t media_sbc_codec_configuration[4];
static uint8_t media_sbc_codec_capabilities[] = {
    (AVDTP_SBC_16000 << 4) | AVDTP_SBC_MONO, 0xFF, 2, 53};

static a2dp_demo_context_t a2dp_demo_context;
static a2dp_demo_sbc_configuration_t sbc_configuration;
static const btstack_sbc_encoder_t *sbc_encoder_instance;
static btstack_sbc_encoder_bluedroid_t sbc_encoder_state;

static int current_sample_rate = A2DP_DEMO_PREFERRED_SAMPLING_RATE;
static int16_t pcm_ring_buffer[A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES];
static volatile uint32_t pcm_ring_read_cursor = 0u;
static volatile uint32_t pcm_ring_write_cursor = 0u;
static uint32_t pcm_default_source_index = 0;
static bool pcm_default_source_enabled = true;
static bool a2dp_demo_initialized = false;
static bool a2dp_demo_cyw43_initialized = false;
static bool a2dp_demo_romaddr_capture_enabled = false;
static int32_t a2dp_demo_romaddr_lpf_state_q15 = 0;
static bool a2dp_demo_romaddr_lpf_state_valid = false;
static a2dp_source_demo_exit_action_t a2dp_demo_exit_action =
    A2DP_SOURCE_DEMO_EXIT_NONE;

static void a2dpDemoStartScanning(void);
static void a2dpDemoStartScanSession(void);
static void a2dpDemoLinkConnect(bd_addr_t address, const char *name);
static void a2dpDemoLinkHandlePendingScanSelection(void);
static bool a2dpDemoHandleTerminalKey(char chr);
static void a2dpDemoBootIntoRomaddrCapture(void);
static void a2dpDemoContinueToDesktop(void);
static void a2dpDemoPumpRomaddrIntoPcmRing(void);
static void a2dpDemoStopTermPollTimer(void);
static void a2dpDemoShutdownAndExit(a2dp_source_demo_exit_action_t action);
static void __not_in_flash_func(a2dpDemoApplyGainNoClipQ15)(
    int16_t *samples, uint32_t count, int32_t desiredGainQ15);
static void __not_in_flash_func(a2dpDemoApplyLowPassQ15)(int16_t *samples,
                                                         uint32_t count,
                                                         int32_t alphaQ15);

static inline uint32_t __not_in_flash_func(a2dpDemoPcmRingUsedSamples)(void) {
  return (uint32_t)(pcm_ring_write_cursor - pcm_ring_read_cursor);
}

static inline uint32_t __not_in_flash_func(a2dpDemoPcmRingFreeSamples)(void) {
  return (uint32_t)A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES -
         a2dpDemoPcmRingUsedSamples();
}

static void __not_in_flash_func(a2dpDemoPcmRingReset)(void) {
  pcm_ring_read_cursor = 0u;
  pcm_ring_write_cursor = 0u;
}

static void a2dpDemoDisableDefaultPcmSource(void) {
  pcm_default_source_enabled = false;
  pcm_default_source_index = 0u;
  a2dpDemoPcmRingReset();
}

static bool __not_in_flash_func(a2dpDemoPcmRingReadSample)(int16_t *sampleOut) {
  if (sampleOut == NULL) {
    return false;
  }

  uint32_t readCursor = pcm_ring_read_cursor;
  if (readCursor == pcm_ring_write_cursor) {
    return false;
  }

  *sampleOut = pcm_ring_buffer[A2DP_DEMO_PCM_RING_INDEX(readCursor)];
  pcm_ring_read_cursor = readCursor + 1u;
  return true;
}

uint32_t __not_in_flash_func(a2dp_source_demo_pcm_write_mono)(
    const int16_t *samples, uint32_t sampleCount) {
  if ((samples == NULL) || (sampleCount == 0u)) {
    return 0u;
  }

  uint32_t writeCursor = pcm_ring_write_cursor;
  uint32_t freeSamples = (uint32_t)A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES -
                         (uint32_t)(writeCursor - pcm_ring_read_cursor);
  if (freeSamples == 0u) {
    return 0u;
  }

  // Hot path: stream ISR writes one sample at a time.
  if (__builtin_expect(sampleCount == 1u, 1)) {
    pcm_ring_buffer[A2DP_DEMO_PCM_RING_INDEX(writeCursor)] = samples[0];
    pcm_ring_write_cursor = writeCursor + 1u;
    return 1u;
  }

  uint32_t samplesWritten = btstack_min(sampleCount, freeSamples);
  uint32_t firstChunk = btstack_min(
      samplesWritten, (uint32_t)A2DP_DEMO_PCM_RING_CAPACITY_SAMPLES -
                          A2DP_DEMO_PCM_RING_INDEX(writeCursor));
  uint32_t writeIndex = A2DP_DEMO_PCM_RING_INDEX(writeCursor);
  memcpy(&pcm_ring_buffer[writeIndex], samples, firstChunk * sizeof(int16_t));

  uint32_t remaining = samplesWritten - firstChunk;
  if (remaining > 0u) {
    memcpy(&pcm_ring_buffer[0], &samples[firstChunk],
           remaining * sizeof(int16_t));
  }

  pcm_ring_write_cursor = writeCursor + samplesWritten;
  return samplesWritten;
}

static void __not_in_flash_func(a2dpDemoPumpDefaultPcmSource)(void) {
  if (!pcm_default_source_enabled || (a2dp_pcm_sample_count == 0u)) {
    return;
  }

  while (true) {
    uint32_t freeSamples = a2dpDemoPcmRingFreeSamples();
    if (freeSamples == 0u) {
      break;
    }

    uint32_t remainingSamples =
        a2dp_pcm_sample_count - pcm_default_source_index;
    if (remainingSamples == 0u) {
      pcm_default_source_index = 0u;
      continue;
    }

    uint32_t chunkSamples = btstack_min(freeSamples, remainingSamples);
    uint32_t written = a2dp_source_demo_pcm_write_mono(
        &a2dp_pcm_sample_data[pcm_default_source_index], chunkSamples);
    if (written == 0u) {
      break;
    }

    pcm_default_source_index += written;
    if (pcm_default_source_index >= a2dp_pcm_sample_count) {
      pcm_default_source_index = 0u;
    }
  }
}

static void __not_in_flash_func(a2dpDemoApplyGainNoClipQ15)(
    int16_t *samples, uint32_t count, int32_t desiredGainQ15) {
  if ((samples == NULL) || (count == 0u) || (desiredGainQ15 <= 0)) {
    return;
  }

  // Clamp requested gain so boosted signal never clips.
  int32_t peakAbs = 0;
  for (uint32_t i = 0; i < count; i++) {
    int32_t v = samples[i];
    int32_t a = (v < 0) ? -v : v;
    if (a > peakAbs) {
      peakAbs = a;
    }
  }
  if (peakAbs == 0) {
    return;
  }

  int32_t maxNoClipGainQ15 = (int32_t)(((int64_t)INT16_MAX << 15) / peakAbs);
  int32_t gainQ15 = btstack_min(desiredGainQ15, maxNoClipGainQ15);

  for (uint32_t i = 0; i < count; i++) {
    int32_t scaled =
        (int32_t)((((int64_t)samples[i] * gainQ15) + (1 << 14)) >> 15);
    if (scaled > INT16_MAX) {
      scaled = INT16_MAX;
    } else if (scaled < INT16_MIN) {
      scaled = INT16_MIN;
    }
    samples[i] = (int16_t)scaled;
  }
}

static void __not_in_flash_func(a2dpDemoApplyLowPassQ15)(int16_t *samples,
                                                         uint32_t count,
                                                         int32_t alphaQ15) {
  if ((samples == NULL) || (count == 0u)) {
    return;
  }
  if (alphaQ15 <= 0) {
    return;
  }
  if (alphaQ15 > 32767) {
    alphaQ15 = 32767;
  }

  int32_t state = a2dp_demo_romaddr_lpf_state_q15;
  bool stateValid = a2dp_demo_romaddr_lpf_state_valid;

  for (uint32_t i = 0; i < count; i++) {
    int32_t xQ15 = ((int32_t)samples[i]) << 15;
    if (!stateValid) {
      state = xQ15;
      stateValid = true;
    } else {
      state +=
          (int32_t)((((int64_t)(xQ15 - state) * alphaQ15) + (1 << 14)) >> 15);
    }

    int32_t y = (state + (1 << 14)) >> 15;
    if (y > INT16_MAX) {
      y = INT16_MAX;
    } else if (y < INT16_MIN) {
      y = INT16_MIN;
    }
    samples[i] = (int16_t)y;
  }

  a2dp_demo_romaddr_lpf_state_q15 = state;
  a2dp_demo_romaddr_lpf_state_valid = stateValid;
}

static void a2dpDemoBootIntoRomaddrCapture(void) {
  a2dp_demo_romaddr_capture_enabled = false;
  a2dp_demo_romaddr_lpf_state_q15 = 0;
  a2dp_demo_romaddr_lpf_state_valid = false;
  deinit_romemul();
  sleep_ms(1000);
  if (romaddr_init() != 0) {
    DPRINTF("ROMADDR init failed.\n");
    return;
  }

  romaddr_start();
  a2dp_demo_romaddr_capture_enabled = true;
  DPRINTF("ROMADDR capture started (non-blocking).\n");
}

static void a2dpDemoContinueToDesktop(void) {
  a2dp_demo_context.link.waiting_for_post_connect_action = false;
  a2dp_demo_context.link.post_connect_action_deadline_ms = 0u;
  a2dpDemoDisableDefaultPcmSource();
  SEND_COMMAND_TO_DISPLAY(DISPLAY_COMMAND_CONTINUE);
  sleep_ms(3000);
  a2dpDemoBootIntoRomaddrCapture();
}

static void a2dpDemoPumpRomaddrIntoPcmRing(void) {
  if (!a2dp_demo_romaddr_capture_enabled) {
    return;
  }

  int16_t romaddrSamples[128];

  while (true) {
    uint32_t pcmFree = a2dpDemoPcmRingFreeSamples();
    if (pcmFree == 0u) {
      return;
    }

    uint32_t maxPull = btstack_min(
        pcmFree,
        (uint32_t)(sizeof(romaddrSamples) / sizeof(romaddrSamples[0])));
    uint32_t pulled = romaddr_read((uint16_t *)romaddrSamples, maxPull);
    if (pulled == 0u) {
      return;
    }

    // Transform 13-bit offset-binary words (LSB always zero) to signed PCM16.
    for (uint32_t i = 0; i < pulled; i++) {
      uint16_t raw = (uint16_t)romaddrSamples[i];
      int32_t centered13 = ((int32_t)(raw & 0x1FFEu) - 0x1000);
      romaddrSamples[i] = (int16_t)(centered13 << 3);
    }
    a2dpDemoApplyLowPassQ15(romaddrSamples, pulled,
                            A2DP_DEMO_ROMADDR_LPF_ALPHA_Q15);
    a2dpDemoApplyGainNoClipQ15(romaddrSamples, pulled,
                               A2DP_DEMO_ROMADDR_GAIN_Q15);

    (void)a2dp_source_demo_pcm_write_mono(romaddrSamples, pulled);
  }
}

static int a2dpDemoInitCyw43Chip(void) {
  if (a2dp_demo_cyw43_initialized) {
    return 0;
  }

  int res = cyw43_arch_init();
  if (res != 0) {
    return res;
  }

  a2dp_demo_cyw43_initialized = true;
  return 0;
}

static const char *a2dpDemoStatusToString(uint8_t status) {
  switch (status) {
    case ERROR_CODE_SUCCESS:
      return "SUCCESS";
    case ERROR_CODE_PAGE_TIMEOUT:
      return "PAGE_TIMEOUT";
    case ERROR_CODE_CONNECTION_TIMEOUT:
      return "CONNECTION_TIMEOUT";
    case ERROR_CODE_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS:
      return "CONNECTION_REJECTED_SECURITY";
    case ERROR_CODE_AUTHENTICATION_FAILURE:
      return "AUTHENTICATION_FAILURE";
    case ERROR_CODE_PIN_OR_KEY_MISSING:
      return "PIN_OR_KEY_MISSING";
    case L2CAP_CONNECTION_BASEBAND_DISCONNECT:
      return "L2CAP_BASEBAND_DISCONNECT";
    case SDP_QUERY_BUSY:
      return "SDP_QUERY_BUSY";
    case SDP_QUERY_INCOMPLETE:
      return "SDP_QUERY_INCOMPLETE";
    case SDP_SERVICE_NOT_FOUND:
      return "SDP_SERVICE_NOT_FOUND";
    default:
      return "UNKNOWN_STATUS";
  }
}

static void a2dpDemoTermPrintScanningStarted(void) {
  A2DP_TERM("Scanning started. Press C to stop.\n");
}

static void a2dpDemoTermPrintScanningStopped(void) {
  A2DP_TERM("Scanning stopped.\n");
}

static void a2dpDemoTermPrintDeviceFound(const bd_addr_t address,
                                         const char *name) {
  A2DP_TERM("Found: %s (%s)\n", name, bd_addr_to_str(address));
}

static void a2dpDemoTermPrintConnectionAttempt(const bd_addr_t address,
                                               const char *name) {
  A2DP_TERM("Connecting: %s (%s)\n", name, bd_addr_to_str(address));
}

static void a2dpDemoTermPrintConnectionFailed(const bd_addr_t address,
                                              uint8_t status) {
  A2DP_TERM("Connection failed: %s (0x%02x %s)\n", bd_addr_to_str(address),
            status, a2dpDemoStatusToString(status));
}

static void a2dpDemoTermPrintConnectionSuccess(const bd_addr_t address) {
  A2DP_TERM("Connected: %s\n", bd_addr_to_str(address));
}

static void a2dpDemoTermPrintPostConnectMenu(void) {
  A2DP_TERM("Press SPACE to exit to Desktop, or S to scan again\n");
}

static void a2dpDemoStopScanning(void) {
  if (!a2dp_demo_context.scan.scanning) {
    return;
  }

  a2dp_demo_context.scan.scanning = false;
  a2dpDemoTermPrintScanningStopped();
}

static void a2dpDemoAssumeConnected(void) {
  a2dpDemoStopScanning();
  a2dp_demo_context.scan.scan_cancel_requested = false;
  a2dp_demo_context.scan.waiting_for_selection = false;
  a2dp_demo_context.scan.connect_request_pending = false;
  a2dp_demo_context.link.waiting_for_post_connect_action = false;
  a2dp_demo_context.link.post_connect_action_deadline_ms = 0u;
  a2dp_demo_context.link.connecting = false;
  a2dp_demo_context.link.post_connect_action_deadline_ms = 0u;
  a2dp_demo_context.scan.selection_input_len = 0;
  a2dp_demo_context.scan.selection_input[0] = '\0';
  term_clearInputBuffer();
}

static bool a2dpDemoShouldBackoffOnStatus(uint8_t status) {
  switch (status) {
    case SDP_SERVICE_NOT_FOUND:
    case ERROR_CODE_PAGE_TIMEOUT:
    case ERROR_CODE_CONNECTION_TIMEOUT:
    case L2CAP_CONNECTION_BASEBAND_DISCONNECT:
      return true;
    default:
      return false;
  }
}

static bool a2dpDemoIsAddressInCooldown(const bd_addr_t address) {
  if (!a2dp_demo_context.link.cooldown_active) {
    return false;
  }

  uint32_t now = btstack_run_loop_get_time_ms();
  if ((int32_t)(now - a2dp_demo_context.link.cooldown_until_ms) >= 0) {
    a2dp_demo_context.link.cooldown_active = false;
    return false;
  }

  return bd_addr_cmp(address, a2dp_demo_context.link.cooldown_addr) == 0;
}

static void a2dpDemoSetAddressCooldown(const bd_addr_t address,
                                       uint8_t status) {
  bd_addr_copy(a2dp_demo_context.link.cooldown_addr, address);
  a2dp_demo_context.link.cooldown_until_ms =
      btstack_run_loop_get_time_ms() + A2DP_DEMO_RETRY_COOLDOWN_MS;
  a2dp_demo_context.link.cooldown_active = true;
  DPRINTF("A2DP: cooldown for %s for %lu ms due to 0x%02x (%s)\n",
          bd_addr_to_str(address), (unsigned long)A2DP_DEMO_RETRY_COOLDOWN_MS,
          status, a2dpDemoStatusToString(status));
}

static void a2dpDemoClearDiscoveredDevices(void) {
  memset(a2dp_demo_context.scan.discovered_devices, 0,
         sizeof(a2dp_demo_context.scan.discovered_devices));
  memset(a2dp_demo_context.scan.sink_device_indices, 0,
         sizeof(a2dp_demo_context.scan.sink_device_indices));
  a2dp_demo_context.scan.discovered_device_count = 0;
  a2dp_demo_context.scan.sink_device_count = 0;
}

static int a2dpDemoFindDiscoveredDeviceByAddress(const bd_addr_t address) {
  for (uint8_t i = 0; i < a2dp_demo_context.scan.discovered_device_count; i++) {
    if (bd_addr_cmp(address,
                    a2dp_demo_context.scan.discovered_devices[i].address) ==
        0) {
      return (int)i;
    }
  }

  return -1;
}

static bool a2dpDemoSinkListContains(uint8_t discoveredIndex) {
  for (uint8_t i = 0; i < a2dp_demo_context.scan.sink_device_count; i++) {
    if (a2dp_demo_context.scan.sink_device_indices[i] == discoveredIndex) {
      return true;
    }
  }

  return false;
}

static void a2dpDemoRememberSinkDevice(uint8_t discoveredIndex) {
  if ((a2dp_demo_context.scan.sink_device_count >=
       A2DP_DEMO_MAX_DISCOVERED_DEVICES) ||
      a2dpDemoSinkListContains(discoveredIndex)) {
    return;
  }

  a2dp_demo_context.scan
      .sink_device_indices[a2dp_demo_context.scan.sink_device_count++] =
      discoveredIndex;
}

static int a2dpDemoUpsertDiscoveredDevice(const bd_addr_t address,
                                          const char *name, uint32_t cod,
                                          bool supportsSink, bool *wasNewDevice,
                                          bool *sinkSupportChanged,
                                          bool *nameChanged) {
  if (wasNewDevice != NULL) {
    *wasNewDevice = false;
  }
  if (sinkSupportChanged != NULL) {
    *sinkSupportChanged = false;
  }
  if (nameChanged != NULL) {
    *nameChanged = false;
  }

  int index = a2dpDemoFindDiscoveredDeviceByAddress(address);
  if (index < 0) {
    if (a2dp_demo_context.scan.discovered_device_count >=
        A2DP_DEMO_MAX_DISCOVERED_DEVICES) {
      DPRINTF("A2DP: discovered device list full, dropping %s\n",
              bd_addr_to_str(address));
      return -1;
    }

    index = (int)a2dp_demo_context.scan.discovered_device_count++;
    a2dp_demo_discovered_device_t *entry =
        &a2dp_demo_context.scan.discovered_devices[index];
    memset(entry, 0, sizeof(*entry));
    bd_addr_copy(entry->address, address);
    entry->cod = cod;
    entry->supports_sink = supportsSink;
    snprintf(entry->name, sizeof(entry->name), "%s",
             (name != NULL) ? name : "<unknown>");
    if (supportsSink) {
      a2dpDemoRememberSinkDevice((uint8_t)index);
    }
    if (wasNewDevice != NULL) {
      *wasNewDevice = true;
    }
    if (sinkSupportChanged != NULL) {
      *sinkSupportChanged = supportsSink;
    }
    return index;
  }

  a2dp_demo_discovered_device_t *entry =
      &a2dp_demo_context.scan.discovered_devices[index];
  entry->cod = cod;
  if ((name != NULL) && (strcmp(name, "<unknown>") != 0)) {
    if (strcmp(entry->name, name) != 0) {
      snprintf(entry->name, sizeof(entry->name), "%s", name);
      if (nameChanged != NULL) {
        *nameChanged = true;
      }
    }
  }

  if (supportsSink && !entry->supports_sink) {
    entry->supports_sink = true;
    a2dpDemoRememberSinkDevice((uint8_t)index);
    if (sinkSupportChanged != NULL) {
      *sinkSupportChanged = true;
    }
  }

  return index;
}

static void a2dpDemoClearSelectionInput(void) {
  memset(a2dp_demo_context.scan.selection_input, 0,
         sizeof(a2dp_demo_context.scan.selection_input));
  a2dp_demo_context.scan.selection_input_len = 0;
}

static void a2dpDemoPrintSinkDeviceList(void) {
  a2dp_demo_context.scan.waiting_for_selection = true;
  a2dpDemoClearSelectionInput();
  term_clearInputBuffer();

  if (a2dp_demo_context.scan.sink_device_count == 0u) {
    A2DP_TERM("No sink devices found.\n");
    A2DP_TERM("Press C to restart scanning.\n");
    return;
  }

  A2DP_TERM("Select sink device:\n");
  for (uint8_t i = 0; i < a2dp_demo_context.scan.sink_device_count; i++) {
    uint8_t discoveredIndex = a2dp_demo_context.scan.sink_device_indices[i];
    a2dp_demo_discovered_device_t *entry =
        &a2dp_demo_context.scan.discovered_devices[discoveredIndex];
    A2DP_TERM("  %u) %s (%s)\n", (unsigned int)(i + 1u), entry->name,
              bd_addr_to_str(entry->address));
  }
  if (a2dp_demo_context.scan.sink_device_count <= 9u) {
    A2DP_TERM("Type number to connect.\n");
  } else {
    A2DP_TERM("Type number and press Enter.\n");
  }
}

static void a2dpDemoScanQueueConnectionBySelection(
    unsigned int selectionOneBased) {
  if ((selectionOneBased == 0u) ||
      (selectionOneBased > a2dp_demo_context.scan.sink_device_count)) {
    A2DP_TERM("Invalid selection. Choose 1-%u.\n",
              (unsigned int)a2dp_demo_context.scan.sink_device_count);
    return;
  }

  uint8_t sinkListIndex = (uint8_t)(selectionOneBased - 1u);
  uint8_t discoveredIndex =
      a2dp_demo_context.scan.sink_device_indices[sinkListIndex];
  a2dp_demo_discovered_device_t *entry =
      &a2dp_demo_context.scan.discovered_devices[discoveredIndex];
  if (a2dpDemoIsAddressInCooldown(entry->address)) {
    A2DP_TERM("Device %u is cooling down.\n", selectionOneBased);
    return;
  }

  a2dp_demo_context.scan.waiting_for_selection = false;
  a2dp_demo_context.scan.connect_request_pending = true;
  bd_addr_copy(a2dp_demo_context.scan.selected_sink_addr, entry->address);
  snprintf(a2dp_demo_context.scan.selected_sink_name,
           sizeof(a2dp_demo_context.scan.selected_sink_name), "%s",
           entry->name);
}

static void a2dpDemoLinkConnect(bd_addr_t address, const char *name) {
  if (a2dp_demo_context.link.connecting ||
      (a2dp_demo_context.link.a2dp_cid != 0u)) {
    DPRINTF(
        "A2DP: connect request ignored while busy (connecting=%d cid=0x%04x)\n",
        (int)a2dp_demo_context.link.connecting,
        a2dp_demo_context.link.a2dp_cid);
    return;
  }

  a2dp_demo_context.link.connecting = true;
  bd_addr_copy(a2dp_demo_context.link.last_candidate_addr, address);
  a2dpDemoTermPrintConnectionAttempt(address, name);

  uint8_t status =
      a2dp_source_establish_stream(address, &a2dp_demo_context.link.a2dp_cid);
  if (status != ERROR_CODE_SUCCESS) {
    a2dp_demo_context.link.connecting = false;
    a2dp_demo_context.link.a2dp_cid = 0;
    a2dpDemoTermPrintConnectionFailed(address, status);
    A2DP_TRACE("A2DP: connect request failed (0x%02x %s)\n", status,
               a2dpDemoStatusToString(status));
    if (a2dpDemoShouldBackoffOnStatus(status)) {
      a2dpDemoSetAddressCooldown(address, status);
    }
    a2dpDemoPrintSinkDeviceList();
  }
}

static void a2dpDemoLinkHandlePendingScanSelection(void) {
  if (!a2dp_demo_context.scan.connect_request_pending) {
    return;
  }
  if (a2dp_demo_context.link.connecting ||
      (a2dp_demo_context.link.a2dp_cid != 0u)) {
    return;
  }

  bd_addr_t address;
  char name[A2DP_DEMO_DEVICE_NAME_SIZE];
  bd_addr_copy(address, a2dp_demo_context.scan.selected_sink_addr);
  snprintf(name, sizeof(name), "%s", a2dp_demo_context.scan.selected_sink_name);
  a2dp_demo_context.scan.connect_request_pending = false;
  a2dpDemoLinkConnect(address, name);
}

static const char *a2dpDemoGetInquiryName(const uint8_t *packet,
                                          char *nameBuffer,
                                          size_t nameBufferSize) {
  if ((packet == NULL) || (nameBuffer == NULL) || (nameBufferSize < 2u) ||
      (!gap_event_inquiry_result_get_name_available(packet))) {
    return NULL;
  }

  uint8_t nameLen = gap_event_inquiry_result_get_name_len(packet);
  const uint8_t *name = gap_event_inquiry_result_get_name(packet);
  if ((name == NULL) || (nameLen == 0u)) {
    return NULL;
  }

  if (nameLen >= nameBufferSize) {
    nameLen = (uint8_t)(nameBufferSize - 1u);
  }

  memcpy(nameBuffer, name, nameLen);
  nameBuffer[nameLen] = '\0';
  return nameBuffer;
}

static bool a2dpDemoIsPotentialAudioSink(uint32_t cod) {
  // CoD can be missing/unknown on some devices during inquiry.
  if (cod == 0u) {
    return true;
  }

  // Bluetooth CoD: major device class is bits 12..8.
  const uint32_t majorDeviceClassMask = 0x001F00u;
  const uint32_t majorDeviceClassAudioVideo = 0x000400u;
  bool majorIsAudioVideo =
      (cod & majorDeviceClassMask) == majorDeviceClassAudioVideo;

  // Some devices do not report the expected major class but still report audio
  // service classes.
  const uint32_t audioServiceMask = 0x240000u;  // Audio + Rendering
  bool hasAudioService = (cod & audioServiceMask) != 0u;

  return majorIsAudioVideo || hasAudioService;
}

static void a2dpDemoConfigurePcmPlayback(int sampleRate) {
  if (sampleRate <= 0) {
    sampleRate = A2DP_DEMO_PREFERRED_SAMPLING_RATE;
  }

  current_sample_rate = sampleRate;
  pcm_default_source_index = 0u;
  pcm_default_source_enabled = true;
  a2dpDemoPcmRingReset();
  a2dpDemoPumpDefaultPcmSource();
}

static void a2dpDemoProduceAudio(int16_t *pcmBuffer, int numSamplesToWrite) {
  if ((pcmBuffer == NULL) || (numSamplesToWrite <= 0)) {
    return;
  }

  for (int i = 0; i < numSamplesToWrite; i++) {
    int16_t sample = 0;
    (void)a2dpDemoPcmRingReadSample(&sample);

    for (int channel = 0; channel < A2DP_DEMO_NUM_CHANNELS; channel++) {
      pcmBuffer[(i * A2DP_DEMO_NUM_CHANNELS) + channel] = sample;
    }
  }
}

static void a2dpDemoFillSbcAudioBuffer(a2dp_demo_link_context_t *context) {
  if ((context == NULL) || (sbc_encoder_instance == NULL)) {
    return;
  }

  unsigned int numAudioSamplesPerSbcFrame =
      sbc_encoder_instance->num_audio_frames(&sbc_encoder_state);
  if (numAudioSamplesPerSbcFrame == 0) {
    return;
  }

  if (context->max_media_payload_size <= 1u) {
    return;
  }
  uint16_t payloadBudget = (uint16_t)(context->max_media_payload_size - 1u);
  uint16_t sbcFrameSize =
      sbc_encoder_instance->sbc_buffer_length(&sbc_encoder_state);

  while (context->samples_ready >= numAudioSamplesPerSbcFrame) {
    if (context->sbc_storage_count >= payloadBudget) {
      break;
    }
    if ((sbcFrameSize != 0u) &&
        ((payloadBudget - context->sbc_storage_count) < sbcFrameSize)) {
      break;
    }
    if (numAudioSamplesPerSbcFrame > A2DP_DEMO_PCM_FRAME_MAX_SAMPLES) {
      break;
    }

    int16_t pcmFrame[A2DP_DEMO_PCM_FRAME_MAX_SAMPLES * A2DP_DEMO_NUM_CHANNELS];
    a2dpDemoProduceAudio(pcmFrame, (int)numAudioSamplesPerSbcFrame);

    sbc_encoder_instance->encode_signed_16(
        &sbc_encoder_state, pcmFrame,
        &context->sbc_storage[1 + context->sbc_storage_count]);

    sbcFrameSize = sbc_encoder_instance->sbc_buffer_length(&sbc_encoder_state);
    if (sbcFrameSize == 0u) {
      DPRINTF("A2DP: SBC encoder produced zero frame length after encode\n");
      break;
    }
    if ((context->sbc_storage_count + sbcFrameSize) > payloadBudget) {
      break;
    }

    context->sbc_storage_count += sbcFrameSize;
    context->samples_ready -= numAudioSamplesPerSbcFrame;
  }
}

static void a2dpDemoScheduleSendIfNeeded(a2dp_demo_link_context_t *context) {
  if ((context == NULL) || (!context->streaming) ||
      context->sbc_ready_to_send || (context->sbc_storage_count == 0)) {
    return;
  }

  if ((sbc_encoder_instance == NULL) ||
      (context->max_media_payload_size <= 1u)) {
    return;
  }

  uint16_t payloadBudget = (uint16_t)(context->max_media_payload_size - 1u);
  uint16_t sbcFrameSize =
      sbc_encoder_instance->sbc_buffer_length(&sbc_encoder_state);

  // Match BTstack demo behavior: request TX once no additional SBC frame fits.
  if ((sbcFrameSize == 0) ||
      ((context->sbc_storage_count + sbcFrameSize) <= payloadBudget)) {
    return;
  }

  context->sbc_ready_to_send = true;
  a2dp_source_stream_endpoint_request_can_send_now(context->a2dp_cid,
                                                   context->local_seid);
}

static void a2dpDemoSendMediaPacket(a2dp_demo_link_context_t *context) {
  if ((context == NULL) || (sbc_encoder_instance == NULL) ||
      (!context->sbc_ready_to_send) || (context->sbc_storage_count == 0)) {
    return;
  }

  int bytesInFrame =
      sbc_encoder_instance->sbc_buffer_length(&sbc_encoder_state);
  if (bytesInFrame <= 0) {
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = false;
    return;
  }

  uint8_t numSbcFrames = (uint8_t)(context->sbc_storage_count / bytesInFrame);
  if (numSbcFrames == 0) {
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = false;
    return;
  }

  context->sbc_storage[0] = numSbcFrames;
  uint8_t sendStatus = a2dp_source_stream_send_media_payload_rtp(
      context->a2dp_cid, context->local_seid, 0, context->rtp_timestamp,
      context->sbc_storage, context->sbc_storage_count + 1);
  if (sendStatus != ERROR_CODE_SUCCESS) {
    DPRINTF("A2DP: media send failed (0x%02x %s), frames=%u bytes=%u\n",
            sendStatus, a2dpDemoStatusToString(sendStatus), numSbcFrames,
            context->sbc_storage_count + 1);
  }

  context->rtp_timestamp +=
      numSbcFrames * sbc_encoder_instance->num_audio_frames(&sbc_encoder_state);
  context->sbc_storage_count = 0;
  context->sbc_ready_to_send = false;
}

static void a2dpDemoAudioTimeoutHandler(btstack_timer_source_t *timer) {
  a2dp_demo_link_context_t *context =
      (a2dp_demo_link_context_t *)btstack_run_loop_get_timer_context(timer);
  if ((context == NULL) || (!context->streaming)) {
    return;
  }

  btstack_run_loop_set_timer(&context->audio_timer, A2DP_DEMO_AUDIO_TIMEOUT_MS);
  btstack_run_loop_add_timer(&context->audio_timer);

  uint32_t now = btstack_run_loop_get_time_ms();
  uint32_t elapsedMs = A2DP_DEMO_AUDIO_TIMEOUT_MS;
  if (context->time_audio_data_sent > 0) {
    elapsedMs = now - context->time_audio_data_sent;
    if (elapsedMs == 0) {
      elapsedMs = A2DP_DEMO_AUDIO_TIMEOUT_MS;
    }
  }
  context->time_audio_data_sent = now;

  uint32_t numSamples = (elapsedMs * (uint32_t)current_sample_rate) / 1000u;
  context->acc_num_missed_samples +=
      (elapsedMs * (uint32_t)current_sample_rate) % 1000u;

  while (context->acc_num_missed_samples >= 1000u) {
    numSamples++;
    context->acc_num_missed_samples -= 1000u;
  }

  context->samples_ready += numSamples;
  a2dpDemoPumpDefaultPcmSource();
  a2dpDemoPumpRomaddrIntoPcmRing();

  if (context->sbc_ready_to_send) {
    return;
  }

  a2dpDemoFillSbcAudioBuffer(context);
  a2dpDemoScheduleSendIfNeeded(context);
}

static void a2dpDemoStartAudioTimer(a2dp_demo_link_context_t *context) {
  if ((context == NULL) || (sbc_encoder_instance == NULL)) {
    return;
  }

  context->max_media_payload_size = (uint16_t)btstack_min(
      a2dp_max_media_payload_size(context->a2dp_cid, context->local_seid),
      A2DP_DEMO_SBC_STORAGE_SIZE);
  context->sbc_storage_count = 0;
  context->sbc_ready_to_send = false;
  context->streaming = true;
  context->rtp_timestamp = 0;
  context->time_audio_data_sent = 0;
  context->acc_num_missed_samples = 0;
  context->samples_ready = 0;

  btstack_run_loop_remove_timer(&context->audio_timer);
  btstack_run_loop_set_timer_handler(&context->audio_timer,
                                     a2dpDemoAudioTimeoutHandler);
  btstack_run_loop_set_timer_context(&context->audio_timer, context);
  btstack_run_loop_set_timer(&context->audio_timer, A2DP_DEMO_AUDIO_TIMEOUT_MS);
  btstack_run_loop_add_timer(&context->audio_timer);

  DPRINTF("A2DP: media payload max=%u, sbc_frame=%u\n",
          context->max_media_payload_size,
          (unsigned int)sbc_encoder_instance->sbc_buffer_length(
              &sbc_encoder_state));
}

static void a2dpDemoStopAudioTimer(a2dp_demo_link_context_t *context) {
  if (context == NULL) {
    return;
  }

  context->streaming = false;
  context->time_audio_data_sent = 0;
  context->acc_num_missed_samples = 0;
  context->samples_ready = 0;
  context->sbc_storage_count = 0;
  context->sbc_ready_to_send = false;
  btstack_run_loop_remove_timer(&context->audio_timer);
}

static void a2dpDemoTermPollTimerHandler(btstack_timer_source_t *timer) {
  a2dp_demo_context_t *context =
      (a2dp_demo_context_t *)btstack_run_loop_get_timer_context(timer);
  if ((context == NULL) || (!context->term_poll_active)) {
    return;
  }

  select_checkPushReset();
  term_loop();
  if (context->link.waiting_for_post_connect_action &&
      (context->link.post_connect_action_deadline_ms != 0u)) {
    uint32_t now = btstack_run_loop_get_time_ms();
    if ((int32_t)(now - context->link.post_connect_action_deadline_ms) >= 0) {
      A2DP_TERM("No selection after 5 seconds. Exiting to Desktop.\n");
      a2dpDemoContinueToDesktop();
      return;
    }
  }
  a2dpDemoLinkHandlePendingScanSelection();
  a2dpDemoPumpRomaddrIntoPcmRing();
  btstack_run_loop_set_timer(&context->term_poll_timer,
                             A2DP_DEMO_TERM_POLL_INTERVAL_MS);
  btstack_run_loop_add_timer(&context->term_poll_timer);
}

static void a2dpDemoStartTermPollTimer(void) {
  if (a2dp_demo_context.term_poll_active) {
    return;
  }

  a2dp_demo_context.term_poll_active = true;
  btstack_run_loop_set_timer_handler(&a2dp_demo_context.term_poll_timer,
                                     a2dpDemoTermPollTimerHandler);
  btstack_run_loop_set_timer_context(&a2dp_demo_context.term_poll_timer,
                                     &a2dp_demo_context);
  btstack_run_loop_set_timer(&a2dp_demo_context.term_poll_timer,
                             A2DP_DEMO_TERM_POLL_INTERVAL_MS);
  btstack_run_loop_add_timer(&a2dp_demo_context.term_poll_timer);
}

static void a2dpDemoStopTermPollTimer(void) {
  if (!a2dp_demo_context.term_poll_active) {
    return;
  }

  a2dp_demo_context.term_poll_active = false;
  btstack_run_loop_remove_timer(&a2dp_demo_context.term_poll_timer);
}

static void a2dpDemoShutdownAndExit(a2dp_source_demo_exit_action_t action) {
  a2dp_demo_exit_action = action;
  a2dpDemoStopTermPollTimer();
  a2dpDemoStopAudioTimer(&a2dp_demo_context.link);

  if (a2dp_demo_context.scan.scanning) {
    a2dp_demo_context.scan.scan_cancel_requested = false;
    a2dpDemoStopScanning();
    gap_inquiry_stop();
  }

  a2dp_demo_context.scan.waiting_for_selection = false;
  a2dp_demo_context.scan.connect_request_pending = false;
  a2dp_demo_context.link.waiting_for_post_connect_action = false;
  a2dp_demo_context.link.connecting = false;
  a2dp_demo_context.link.streaming = false;
  a2dp_demo_context.link.stream_opened = false;

  if (a2dp_demo_context.link.a2dp_cid != 0u) {
    (void)a2dp_source_disconnect(a2dp_demo_context.link.a2dp_cid);
    a2dp_demo_context.link.a2dp_cid = 0u;
  }

  hci_power_control(HCI_POWER_OFF);
  btstack_run_loop_trigger_exit();
}

static void a2dpDemoStartScanning(void) {
  if (a2dp_demo_context.scan.scanning || a2dp_demo_context.link.connecting) {
    DPRINTF("A2DP: scan request ignored (scanning=%d connecting=%d)\n",
            (int)a2dp_demo_context.scan.scanning,
            (int)a2dp_demo_context.link.connecting);
    return;
  }

  a2dp_demo_context.scan.scanning = true;
  a2dp_demo_context.scan.scan_cancel_requested = false;
  a2dp_demo_context.scan.waiting_for_selection = false;
  a2dp_demo_context.scan.connect_request_pending = false;
  a2dp_demo_context.link.waiting_for_post_connect_action = false;
  a2dp_demo_context.link.post_connect_action_deadline_ms = 0u;
  a2dpDemoClearSelectionInput();
  term_clearInputBuffer();
  A2DP_TRACE("A2DP: scanning for Bluetooth speakers/headsets...\n");
  a2dpDemoTermPrintScanningStarted();
  gap_inquiry_start(A2DP_DEMO_INQUIRY_DURATION_1280MS);
}

static void a2dpDemoStartScanSession(void) {
  if (a2dp_demo_context.link.connecting ||
      (a2dp_demo_context.link.a2dp_cid != 0u)) {
    DPRINTF("A2DP: start scan session ignored (connecting=%d cid=0x%04x)\n",
            (int)a2dp_demo_context.link.connecting,
            a2dp_demo_context.link.a2dp_cid);
    return;
  }

  a2dpDemoClearDiscoveredDevices();
  a2dp_demo_context.scan.connect_request_pending = false;
  a2dpDemoStartScanning();
}

static void a2dpDemoRestartInquiryIfNeeded(void) {
  if (a2dp_demo_context.scan.scanning &&
      !a2dp_demo_context.scan.scan_cancel_requested &&
      !a2dp_demo_context.link.connecting &&
      (a2dp_demo_context.link.a2dp_cid == 0u)) {
    DPRINTF("A2DP: continuing scan cycle\n");
    gap_inquiry_start(A2DP_DEMO_INQUIRY_DURATION_1280MS);
  }
}

static bool a2dpDemoHandleTerminalKey(char chr) {
  if ((chr == 'x') || (chr == 'X')) {
    a2dpDemoShutdownAndExit(A2DP_SOURCE_DEMO_EXIT_BOOSTER);
    return true;
  }

  if ((chr == 'e') || (chr == 'E')) {
    a2dpDemoShutdownAndExit(A2DP_SOURCE_DEMO_EXIT_DESKTOP);
    return true;
  }

  if (a2dp_demo_context.scan.scanning) {
    if ((chr == 'c') || (chr == 'C')) {
      DPRINTF("A2DP: scan cancelled by C key\n");
      a2dp_demo_context.scan.scan_cancel_requested = true;
      a2dpDemoStopScanning();
      gap_inquiry_stop();
    }
    return true;
  }

  if (a2dp_demo_context.link.waiting_for_post_connect_action) {
    if (chr == ' ') {
      a2dpDemoContinueToDesktop();
      return true;
    }

    if ((chr == 's') || (chr == 'S')) {
      a2dp_demo_context.link.waiting_for_post_connect_action = false;
      a2dp_demo_context.link.post_connect_action_deadline_ms = 0u;
      if (a2dp_demo_context.link.a2dp_cid != 0u) {
        uint8_t status =
            a2dp_source_disconnect(a2dp_demo_context.link.a2dp_cid);
        if (status != ERROR_CODE_SUCCESS) {
          a2dp_demo_context.link.a2dp_cid = 0;
          a2dpDemoStartScanSession();
        }
      } else {
        a2dpDemoStartScanSession();
      }
      return true;
    }

    return true;
  }

  if (!a2dp_demo_context.scan.waiting_for_selection) {
    return a2dp_demo_context.scan.scan_cancel_requested ||
           a2dp_demo_context.link.connecting ||
           (a2dp_demo_context.link.a2dp_cid != 0u);
  }

  if ((chr == 'c') || (chr == 'C')) {
    A2DP_TERM("Restarting scan.\n");
    a2dpDemoStartScanSession();
    return true;
  }

  if (chr == '\b') {
    if (a2dp_demo_context.scan.selection_input_len > 0u) {
      a2dp_demo_context.scan.selection_input_len--;
      a2dp_demo_context.scan
          .selection_input[a2dp_demo_context.scan.selection_input_len] = '\0';
    }
    return true;
  }

  if ((chr >= '0') && (chr <= '9')) {
    if (a2dp_demo_context.scan.selection_input_len <
        A2DP_DEMO_SELECTION_INPUT_MAX_LEN) {
      a2dp_demo_context.scan
          .selection_input[a2dp_demo_context.scan.selection_input_len++] = chr;
      a2dp_demo_context.scan
          .selection_input[a2dp_demo_context.scan.selection_input_len] = '\0';
      if (a2dp_demo_context.scan.sink_device_count <= 9u) {
        unsigned int selected =
            (unsigned int)(a2dp_demo_context.scan.selection_input[0] - '0');
        a2dpDemoClearSelectionInput();
        term_clearInputBuffer();
        a2dpDemoScanQueueConnectionBySelection(selected);
      }
    }
    return true;
  }

  if ((chr == '\n') || (chr == '\r')) {
    if (a2dp_demo_context.scan.selection_input_len == 0u) {
      A2DP_TERM("Enter a device number first.\n");
      return true;
    }

    unsigned int selected = 0u;
    for (uint8_t i = 0; i < a2dp_demo_context.scan.selection_input_len; i++) {
      selected =
          (selected * 10u) +
          (unsigned int)(a2dp_demo_context.scan.selection_input[i] - '0');
    }
    a2dpDemoClearSelectionInput();
    term_clearInputBuffer();
    a2dpDemoScanQueueConnectionBySelection(selected);
    return true;
  }

  return true;
}

static void a2dpDemoHciPacketHandler(uint8_t packet_type, uint16_t channel,
                                     uint8_t *packet, uint16_t size) {
  UNUSED(channel);
  UNUSED(size);

  if (packet_type != HCI_EVENT_PACKET) {
    return;
  }

  bd_addr_t address;
  switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
      if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
        DPRINTF("A2DP: BTstack HCI state WORKING\n");
        a2dpDemoStartScanSession();
      }
      break;

    case HCI_EVENT_PIN_CODE_REQUEST:
      hci_event_pin_code_request_get_bd_addr(packet, address);
      DPRINTF("A2DP: PIN code requested by %s\n", bd_addr_to_str(address));
      gap_pin_code_response(address, "0000");
      break;

    case GAP_EVENT_INQUIRY_RESULT: {
      if (!a2dp_demo_context.scan.scanning) {
        DPRINTF("A2DP: ignoring inquiry result while not scanning\n");
        break;
      }

      gap_event_inquiry_result_get_bd_addr(packet, address);
      uint32_t cod = gap_event_inquiry_result_get_class_of_device(packet);
      char deviceName[A2DP_DEMO_DEVICE_NAME_SIZE];
      const char *name =
          a2dpDemoGetInquiryName(packet, deviceName, sizeof(deviceName));
      if (name == NULL) {
        name = "<unknown>";
      }

      bool isPotentialAudioSink = a2dpDemoIsPotentialAudioSink(cod);
      bool wasNewDevice = false;
      bool sinkSupportChanged = false;
      bool nameChanged = false;
      int discoveredIndex = a2dpDemoUpsertDiscoveredDevice(
          address, name, cod, isPotentialAudioSink, &wasNewDevice,
          &sinkSupportChanged, &nameChanged);
      DPRINTF("A2DP: inquiry result addr=%s cod=0x%06lx name=\"%s\"\n",
              bd_addr_to_str(address), (unsigned long)cod, name);
      if ((discoveredIndex >= 0) &&
          (wasNewDevice || sinkSupportChanged || nameChanged)) {
        a2dp_demo_discovered_device_t *entry =
            &a2dp_demo_context.scan.discovered_devices[discoveredIndex];
        if (entry->supports_sink && (strcmp(entry->name, "<unknown>") != 0)) {
          a2dpDemoTermPrintDeviceFound(entry->address, entry->name);
        }
      }
      break;
    }

    case GAP_EVENT_INQUIRY_COMPLETE:
      DPRINTF("A2DP: inquiry complete (connecting=%d cid=0x%04x)\n",
              (int)a2dp_demo_context.link.connecting,
              a2dp_demo_context.link.a2dp_cid);
      if (a2dp_demo_context.scan.scan_cancel_requested) {
        a2dp_demo_context.scan.scan_cancel_requested = false;
        a2dpDemoPrintSinkDeviceList();
      } else {
        a2dpDemoRestartInquiryIfNeeded();
      }
      break;

    default:
      break;
  }
}

static void a2dpDemoA2dpPacketHandler(uint8_t packet_type, uint16_t channel,
                                      uint8_t *packet, uint16_t size) {
  UNUSED(channel);
  UNUSED(size);

  if (packet_type != HCI_EVENT_PACKET) {
    return;
  }
  if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) {
    return;
  }

  uint8_t status;
  uint16_t cid;
  uint8_t localSeid;
  bd_addr_t address;

  switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
      DPRINTF("A2DP: subevent SIGNALING_CONNECTION_ESTABLISHED\n");
      a2dp_subevent_signaling_connection_established_get_bd_addr(packet,
                                                                 address);
      cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
      status =
          a2dp_subevent_signaling_connection_established_get_status(packet);
      a2dp_demo_context.link.connecting = false;
      if (status != ERROR_CODE_SUCCESS) {
        a2dp_demo_context.link.a2dp_cid = 0;
        a2dpDemoTermPrintConnectionFailed(address, status);
        A2DP_TRACE("A2DP: signaling connection failed (0x%02x %s)\n", status,
                   a2dpDemoStatusToString(status));
        if (a2dpDemoShouldBackoffOnStatus(status)) {
          a2dpDemoSetAddressCooldown(address, status);
        }
        a2dpDemoPrintSinkDeviceList();
        break;
      }
      a2dp_demo_context.link.a2dp_cid = cid;
      a2dpDemoAssumeConnected();
      a2dpDemoTermPrintConnectionSuccess(address);
      a2dp_demo_context.link.waiting_for_post_connect_action = true;
      a2dp_demo_context.link.post_connect_action_deadline_ms =
          btstack_run_loop_get_time_ms() +
          A2DP_DEMO_POST_CONNECT_ACTION_TIMEOUT_MS;
      a2dpDemoTermPrintPostConnectMenu();
      A2DP_TRACE("A2DP: signaling connected to %s (cid 0x%02x)\n",
                 bd_addr_to_str(address), cid);
      break;

    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
      DPRINTF("A2DP: subevent SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION\n");
      cid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_a2dp_cid(
          packet);
      if (cid != a2dp_demo_context.link.a2dp_cid) {
        return;
      }

      a2dp_demo_context.link.remote_seid =
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(
              packet);

      sbc_configuration.sampling_frequency =
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(
              packet);
      sbc_configuration.block_length =
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(
              packet);
      sbc_configuration.subbands =
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(
              packet);
      sbc_configuration.max_bitpool_value =
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(
              packet);

      avdtp_channel_mode_t channelMode = (avdtp_channel_mode_t)
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(
              packet);
      uint8_t allocationMethod =
          a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(
              packet);
      sbc_configuration.allocation_method =
          (btstack_sbc_allocation_method_t)(allocationMethod - 1);

      switch (channelMode) {
        case AVDTP_CHANNEL_MODE_JOINT_STEREO:
          sbc_configuration.channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO;
          break;
        case AVDTP_CHANNEL_MODE_STEREO:
          sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO;
          break;
        case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:
          sbc_configuration.channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL;
          break;
        case AVDTP_CHANNEL_MODE_MONO:
          sbc_configuration.channel_mode = SBC_CHANNEL_MODE_MONO;
          break;
        default:
          sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO;
          break;
      }

      sbc_encoder_instance =
          btstack_sbc_encoder_bluedroid_init_instance(&sbc_encoder_state);
      sbc_encoder_instance->configure(
          &sbc_encoder_state, SBC_MODE_STANDARD, sbc_configuration.block_length,
          sbc_configuration.subbands, sbc_configuration.allocation_method,
          sbc_configuration.sampling_frequency,
          sbc_configuration.max_bitpool_value, sbc_configuration.channel_mode);

      if (sbc_configuration.sampling_frequency != A2DP_PCM_SAMPLE_RATE_HZ) {
        A2DP_TRACE(
            "A2DP: sample rate mismatch stream=%dHz pcm=%uHz (pitch may "
            "change)\n",
            sbc_configuration.sampling_frequency, A2DP_PCM_SAMPLE_RATE_HZ);
      }
      a2dpDemoConfigurePcmPlayback(sbc_configuration.sampling_frequency);
      A2DP_TRACE("A2DP: SBC configured at %d Hz\n",
                 sbc_configuration.sampling_frequency);
      break;
    }

    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
      DPRINTF("A2DP: subevent STREAM_ESTABLISHED\n");
      status = a2dp_subevent_stream_established_get_status(packet);
      cid = a2dp_subevent_stream_established_get_a2dp_cid(packet);
      localSeid = a2dp_subevent_stream_established_get_local_seid(packet);
      if (status != ERROR_CODE_SUCCESS) {
        A2DP_TRACE("A2DP: stream establish failed (0x%02x %s)\n", status,
                   a2dpDemoStatusToString(status));
        if ((!a2dp_demo_context.scan.scanning) &&
            (!a2dp_demo_context.link.connecting) &&
            (a2dp_demo_context.link.a2dp_cid == 0u)) {
          a2dpDemoPrintSinkDeviceList();
        }
        break;
      }

      a2dp_demo_context.link.stream_opened = true;
      A2DP_TRACE("A2DP: stream established (cid 0x%02x, local seid 0x%02x)\n",
                 cid, localSeid);
      status = a2dp_source_start_stream(a2dp_demo_context.link.a2dp_cid,
                                        a2dp_demo_context.link.local_seid);
      if (status != ERROR_CODE_SUCCESS) {
        A2DP_TRACE("A2DP: start stream failed (0x%02x %s)\n", status,
                   a2dpDemoStatusToString(status));
      }
      break;

    case A2DP_SUBEVENT_STREAM_STARTED:
      DPRINTF("A2DP: subevent STREAM_STARTED\n");
      cid = a2dp_subevent_stream_started_get_a2dp_cid(packet);
      localSeid = a2dp_subevent_stream_started_get_local_seid(packet);
      a2dpDemoStartAudioTimer(&a2dp_demo_context.link);
      A2DP_TRACE("A2DP: streaming PCM sample (cid 0x%02x, seid 0x%02x)\n", cid,
                 localSeid);
      break;

    case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
      cid = a2dp_subevent_streaming_can_send_media_packet_now_get_a2dp_cid(
          packet);
      localSeid =
          a2dp_subevent_streaming_can_send_media_packet_now_get_local_seid(
              packet);
      if ((cid == a2dp_demo_context.link.a2dp_cid) &&
          (localSeid == a2dp_demo_context.link.local_seid)) {
        // DPRINTF("A2DP: can send now (cid 0x%02x, seid 0x%02x, bytes=%u)\n",
        // cid,
        //         localSeid, a2dp_demo_context.link.sbc_storage_count + 1u);
        a2dpDemoSendMediaPacket(&a2dp_demo_context.link);
        a2dpDemoFillSbcAudioBuffer(&a2dp_demo_context.link);
        a2dpDemoScheduleSendIfNeeded(&a2dp_demo_context.link);
      }
      break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:
      DPRINTF("A2DP: subevent STREAM_SUSPENDED\n");
      a2dpDemoStopAudioTimer(&a2dp_demo_context.link);
      A2DP_TRACE("A2DP: stream suspended\n");
      break;

    case A2DP_SUBEVENT_STREAM_RELEASED:
      DPRINTF("A2DP: subevent STREAM_RELEASED\n");
      a2dp_demo_context.link.stream_opened = false;
      a2dpDemoStopAudioTimer(&a2dp_demo_context.link);
      A2DP_TRACE("A2DP: stream released\n");
      break;

    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
      DPRINTF("A2DP: subevent SIGNALING_CONNECTION_RELEASED\n");
      cid = a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet);
      if (cid == a2dp_demo_context.link.a2dp_cid) {
        a2dp_demo_context.link.a2dp_cid = 0;
        a2dp_demo_context.link.stream_opened = false;
        a2dp_demo_context.link.waiting_for_post_connect_action = false;
        a2dp_demo_context.link.connecting = false;
        a2dpDemoStopAudioTimer(&a2dp_demo_context.link);
        A2DP_TRACE("A2DP: signaling released, scanning again\n");
        a2dpDemoStartScanSession();
      }
      break;

    default:
      break;
  }
}

static int a2dpDemoInit(void) {
  if (a2dp_demo_initialized) {
    return 0;
  }

  memset(&a2dp_demo_context, 0, sizeof(a2dp_demo_context));
  a2dpDemoConfigurePcmPlayback(A2DP_DEMO_PREFERRED_SAMPLING_RATE);

  hci_set_master_slave_policy(0);
  hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

  l2cap_init();

  a2dp_source_init();
  a2dp_source_register_packet_handler(&a2dpDemoA2dpPacketHandler);

  avdtp_stream_endpoint_t *localStreamEndpoint =
      a2dp_source_create_stream_endpoint(
          AVDTP_AUDIO, AVDTP_CODEC_SBC, media_sbc_codec_capabilities,
          sizeof(media_sbc_codec_capabilities), media_sbc_codec_configuration,
          sizeof(media_sbc_codec_configuration));
  if (localStreamEndpoint == NULL) {
    return -1;
  }

  avdtp_set_preferred_sampling_frequency(localStreamEndpoint,
                                         A2DP_DEMO_PREFERRED_SAMPLING_RATE);
  avdtp_set_preferred_channel_mode(localStreamEndpoint, AVDTP_SBC_MONO);
  a2dp_demo_context.link.local_seid = avdtp_local_seid(localStreamEndpoint);

  sdp_init();
  memset(sdp_a2dp_source_service_buffer, 0,
         sizeof(sdp_a2dp_source_service_buffer));
  a2dp_source_create_sdp_record(sdp_a2dp_source_service_buffer,
                                sdp_create_service_record_handle(),
                                AVDTP_SOURCE_FEATURE_MASK_PLAYER, NULL, NULL);
  if (de_get_len(sdp_a2dp_source_service_buffer) >
      sizeof(sdp_a2dp_source_service_buffer)) {
    return -1;
  }
  sdp_register_service(sdp_a2dp_source_service_buffer);

  gap_set_local_name("SidecarTridge BATMV16 00:00:00:00:00:00");
  gap_discoverable_control(1);
  gap_set_class_of_device(0x200408);

  hci_event_callback_registration.callback = &a2dpDemoHciPacketHandler;
  hci_add_event_handler(&hci_event_callback_registration);

  a2dp_demo_initialized = true;
  return 0;
}

a2dp_source_demo_exit_action_t a2dp_source_demo_launch(void) {
  a2dp_demo_exit_action = A2DP_SOURCE_DEMO_EXIT_NONE;

  int err = a2dpDemoInitCyw43Chip();
  if (err != 0) {
    A2DP_TRACE("A2DP: failed to initialize CYW43 chip (%d)\n", err);
    return A2DP_SOURCE_DEMO_EXIT_NONE;
  }

  if (a2dpDemoInit() != 0) {
    term_printString("A2DP: failed to initialize source demo.\n");
    return A2DP_SOURCE_DEMO_EXIT_NONE;
  }

  term_setRawKeyHandler(a2dpDemoHandleTerminalKey);
  term_clearInputBuffer();
  a2dpDemoStartTermPollTimer();
  hci_power_control(HCI_POWER_ON);
  btstack_run_loop_execute();
  a2dpDemoStopTermPollTimer();
  term_setRawKeyHandler(NULL);
  term_clearInputBuffer();
  if (a2dp_demo_cyw43_initialized) {
    cyw43_arch_deinit();
    a2dp_demo_cyw43_initialized = false;
  }
  a2dp_demo_initialized = false;
  return a2dp_demo_exit_action;
}
