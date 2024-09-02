#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ble_ti.h"
#include "memory_map.h"
#include "optical.h"
#include "radio.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"
#include "tuning.h"

// If true, sweep through all fine codes.
#define BLE_TX_SWEEP_FINE true
//  #define BLE_TX_SWEEP_FINE false

// BLE TX period in milliseconds.
#define BLE_TX_PERIOD_MS 1000  // milliseconds

// BLE TX tuning code.
static tuning_code_t g_ble_tx_tuning_code = {
    .coarse = 19,
    .mid = 13,
    .fine = 0,
};

// BLE TX trigger.
static bool g_ble_tx_trigger = true;

// Transmit BLE packets.
static inline void ble_tx_trigger(void) {
#if BLE_TX_SWEEP_FINE
    for (uint8_t tx_fine_code = TUNING_MIN_CODE;
         tx_fine_code <= TUNING_MAX_CODE; ++tx_fine_code) {
        g_ble_tx_tuning_code.fine = tx_fine_code;
        tuning_tune_radio(&g_ble_tx_tuning_code);
        printf("Transmitting BLE packet on %u.%u.%u.\n",
               g_ble_tx_tuning_code.coarse, g_ble_tx_tuning_code.mid,
               g_ble_tx_tuning_code.fine);

        // Wait for the frequency to settle.
        for (uint32_t t = 0; t < 5000; ++t)
            ;

        ble_transmit();
    }
#else   // !BLE_TX_SWEEP_FINE
    tuning_tune_radio(&g_ble_tx_tuning_code);
    printf("Transmitting BLE packet on %u.%u.%u.\n",
           g_ble_tx_tuning_code.coarse, g_ble_tx_tuning_code.mid,
           g_ble_tx_tuning_code.fine);

    // Wait for frequency to settle.
    for (uint32_t t = 0; t < 5000; ++t)
        ;

    ble_transmit();
#endif  // BLE_TX_SWEEP_FINE
}

static void ble_tx_rftimer_callback(void) {
    // Trigger a BLE TX.
    g_ble_tx_trigger = true;
}

int main_ble(void) {
   initialize_mote();

   // Initialize BLE TX.
   printf("Initializing BLE TX.\n");
   ble_init();
   ble_init_tx();

   // Configure the RF timer.
   rftimer_set_callback_by_id(ble_tx_rftimer_callback, 7);
   rftimer_enable_interrupts();
   rftimer_enable_interrupts_by_id(7);

   analog_scan_chain_write();
   analog_scan_chain_load();

   crc_check();
   perform_calibration();

   // Generate a BLE packet.
   ble_generate_packet();

   while (true) {
       if (g_ble_tx_trigger) {
           printf("Triggering BLE TX.\n");
           ble_tx_trigger();
           g_ble_tx_trigger = false;
           delay_milliseconds_asynchronous(BLE_TX_PERIOD_MS, 7);
       }
   }
}
