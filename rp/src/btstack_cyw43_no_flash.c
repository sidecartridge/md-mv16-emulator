/**
 * File: btstack_cyw43_no_flash.c
 * Description: Override Pico SDK BTstack/CYW43 init to avoid flash-backed TLV.
 */

#include "btstack_memory.h"
#include "btstack_tlv.h"
#include "btstack_tlv_none.h"
#include "hci.h"

#if WANT_HCI_DUMP
#include "hci_dump.h"
#ifdef ENABLE_SEGGER_RTT
#include "hci_dump_segger_rtt_stdout.h"
#else
#include "hci_dump_embedded_stdout.h"
#endif
#endif

#include "pico/btstack_cyw43.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "pico/btstack_run_loop_async_context.h"

bool btstack_cyw43_init(async_context_t *context) {
  btstack_memory_init();
  btstack_run_loop_init(btstack_run_loop_async_context_get_instance(context));

#if WANT_HCI_DUMP
#ifdef ENABLE_SEGGER_RTT
  hci_dump_init(hci_dump_segger_rtt_stdout_get_instance());
#else
  hci_dump_init(hci_dump_embedded_stdout_get_instance());
#endif
#endif

  hci_init(hci_transport_cyw43_instance(), NULL);

  // Use no-op TLV backend so BTstack never touches flash NVM sectors.
  const btstack_tlv_t *btstack_tlv_impl = btstack_tlv_none_init_instance();
  btstack_tlv_set_instance(btstack_tlv_impl, NULL);

#ifdef ENABLE_CLASSIC
  // Disable persistent link key database; pairing keys are not stored.
  hci_set_link_key_db(NULL);
#endif

  return true;
}

void btstack_cyw43_deinit(__unused async_context_t *context) {
  hci_power_control(HCI_POWER_OFF);
  hci_close();
  btstack_run_loop_async_context_deinit();
  btstack_run_loop_deinit();
  btstack_memory_deinit();
}

