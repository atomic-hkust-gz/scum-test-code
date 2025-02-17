#ifndef __BLE_TI_H
#define __BLE_TI_H

#include <stdbool.h>
#include <stdint.h>

#include "tuning.h"

// BLE preamble.
#define BLE_PREAMBLE 0x55
// BLE access address. The access address is split over multiple bytes to avoid
// big-/little-endianness issue.
#define BLE_ACCESS_ADDRESS1 0x6B
#define BLE_ACCESS_ADDRESS2 0x7D
#define BLE_ACCESS_ADDRESS3 0x91
#define BLE_ACCESS_ADDRESS4 0x71

// BLE protocol data unit.
#define BLE_PDU_HEADER1 0x40
// The PDU is 37 bytes long (6 bytes advertiser address + 31 bytes data).
#define BLE_PDU_HEADER2 0xA4

// Short name.
#define BLE_SHORT_NAME_LENGTH 5
// The name is 6 bytes long (1 byte GAP code + 5 bytes data).
#define BLE_SHORT_NAME_HEADER 0x60
#define BLE_SHORT_NAME_GAP_CODE 0x10

// Appearance
#define BLE_GAP_APPEARANCE_LENGTH 2
#define BLE_GAP_APPEARANCE_HEADER \
    0xC0  // 1byte GAP code + 2 byte data and inverted
#define BLE_GAP_APPEARANCE_GAP_CODE 0x98  //(0x19 but inverted)

// Tuning code.
#define BLE_TUNING_CODE_LENGTH 2
// The tuning code is 3 bytes long (1 byte GAP code + 2 bytes data).
#define BLE_TUNING_CODE_HEADER 0xC0
// Customn GAP code for the tuning code (0xC0 LSB first).
#define BLE_TUNING_CODE_GAP_CODE 0x03

// Counters (2M and 32kHz).
#define BLE_COUNTERS_LENGTH 8
// The counters are 9 bytes long (1 byte GAP code + 4 bytes 2M counter + 4 bytes
// 32k counter).
#define BLE_COUNTERS_HEADER 0x90
// Custom GAP code for counters (0xC2 LSB first).
#define BLE_COUNTERS_GAP_CODE 0x43

// Temperature.
#define BLE_TEMPERATURE_LENGTH 2
// The temperature is 3 bytes long (1 byte GAP code + 2 bytes data).
#define BLE_TEMPERATURE_HEADER 0xC0
// Custom GAP code for temperature (0xC1 LSB first).
#define BLE_TEMPERATURE_GAP_CODE 0x83

// Custom data.
#define BLE_CUSTOM_DATA_LENGTH 4
// The custom data is 5 bytes long (1 byte GAP code + 4 bytes data).
#define BLE_CUSTOM_DATA_HEADER 0xA0
// Custom GAP code for custom data (0xC3 LSB first).
#define BLE_CUSTOM_DATA_GAP_CODE 0xC3

// location data
#define BLE_LOCATION_DATA_LENGTH 8
#define BLE_LOCATION_DATA_HEADER 0x90
// custom GAP code for location data (0xC4 LSB first).
#define BLE_LOCATION_DATA_GAP_CODE 0x23

// Tuning code but mid/fine only.
#define BLE_TUNING_CODE_MID_FINE_LENGTH 2
// The tuning code is 3 bytes long (1 byte GAP code + 2 bytes data).
#define BLE_TUNING_CODE_MID_FINE_HEADER 0xC0
// Customn GAP code for the tuning code (0xC5 LSB first).
#define BLE_TUNING_CODE_MID_FINE_GAP_CODE 0xA3

// BLE advertiser address.
#define BLE_ADVERTISER_ADDRESS_LENGTH 6
// BLE protocol data unit length (2 bytes PDU header + 37 bytes PDU).
#define BLE_PDU_LENGTH 39
// BLE CRC length.
#define BLE_CRC_LENGTH 3

// Maximum BLE packet length.
#define BLE_MAX_PACKET_LENGTH 64

typedef struct {
    uint8_t packet[BLE_MAX_PACKET_LENGTH];
    uint8_t advertiser_address[BLE_ADVERTISER_ADDRESS_LENGTH];
    uint8_t channel;

    // BLE packet contents enable.
    // The total data length cannot exceed 31 bytes.
    bool name_tx_en;
    bool tuning_code_tx_en;
    bool counters_tx_en;
    bool temperature_tx_en;
    bool data_tx_en;
    bool appearance_en;

    bool location_en;
    bool tuning_code_mid_fine_tx_en;

    // BLE packet data.
    char name[BLE_SHORT_NAME_LENGTH];
    tuning_code_t tuning_code;
    uint8_t appearance[BLE_GAP_APPEARANCE_LENGTH];
    uint32_t count_2M;
    uint32_t count_32k;
    double temperature;
    uint8_t data[BLE_CUSTOM_DATA_LENGTH];
    uint32_t location_x;
    uint32_t location_y;
} ble_vars_t;

// Initialize BLE.
void ble_init(void);

// Initialize BLE for TX.
void ble_init_tx(void);

// Initialize BLE for RX. The IF clock frequency must be set after calling this
// function.
void ble_init_rx(void);

// Generate a BLE packet. This function should be called after setting the
// packet data and before transmitting the packet.
void ble_generate_packet(void);

// Generate a BLE packet and add location information. This function should be
// called after setting the packet data and before transmitting the packet.
void ble_generate_location_packet(void);

// Generate a BLE test packet.
void ble_generate_test_packet(void);

// Set the advertiser address.
void ble_set_advertiser_address(const uint8_t* advertiser_address);

// Set the BLE channel.
void ble_set_channel(uint8_t channel);

// Enable transmitting the short name.
void ble_set_name_tx_en(bool enable);

// Set the short name.
void ble_set_name(const char* name);

// Enable transmitting the tuning code.
void ble_set_tuning_code_tx_en(bool enable);

// Set the tuning code.
void ble_set_tuning_code(const tuning_code_t* tuning_code);

// Enable transmitting the counters.
void ble_set_counters_tx_en(bool enable);

// Set the 2M count value.
void ble_set_count_2M(uint32_t count_2M);

// Set the 32k count value.
void ble_set_count_32k(uint32_t count_32k);

// Enable transmitting the temperature.
void ble_set_temperature_tx_en(bool enable);

// Se the temperature.
void ble_set_temperature(double temperature);

// Enable transmitting the custom data.
void ble_set_data_tx_en(bool enable);

// Set the custom data.
void ble_set_data(const uint8_t* data);

// Transmit the BLE packet.
void ble_transmit(void);

#endif
