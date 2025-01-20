#include "ble_ti.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "memory_map.h"
#include "radio.h"
#include "scm3c_hw_interface.h"
#include "tuning.h"

ble_vars_t ble_vars;

static inline void ble_load_tx_arb_fifo(void) {
    // Initialize variables.
    int i;  // used to loop through the bytes
    int j;  // used to loop through the 8 bits of each individual byte

    uint8_t current_byte;  // temporary variable used to get through each byte
                           // at a time
    uint8_t
        current_bit;  // temporary variable to put the current bit into the GPIO
    uint32_t fifo_ctrl_reg =
        0x00000000;  // local storage for analog CFG register

    // Prepare FIFO for loading.
    fifo_ctrl_reg |= 0x00000001;  // raise LSB - data in valid
    fifo_ctrl_reg &= 0xFFFFFFFB;  // lower 3rd bit - data out ready
    fifo_ctrl_reg &=
        0xFFFFFFDF;  // lower clock select bit to clock in from Cortex

    ANALOG_CFG_REG__11 = fifo_ctrl_reg;  // load in the configuration settings

    // Load packet into FIFO.
    for (i = 0; i < 64; ++i) {
        current_byte =
            ble_vars.packet[i];  // put a byte into the temporary storage

        for (j = 7; j >= 0; --j) {
            // current_bit = ((current_byte << (j - 1)) & 0x80) >> 7;
            current_bit = (current_byte >> j) & 0x1;

            if (current_bit == 0) {
                fifo_ctrl_reg &= 0xFFFFFFFD;
                ANALOG_CFG_REG__11 = fifo_ctrl_reg;
            }
            if (current_bit == 1) {
                fifo_ctrl_reg |= 0x00000002;
                ANALOG_CFG_REG__11 = fifo_ctrl_reg;
            }

            fifo_ctrl_reg |= 0x00000008;
            ANALOG_CFG_REG__11 = fifo_ctrl_reg;
            fifo_ctrl_reg &= 0xFFFFFFF7;
            ANALOG_CFG_REG__11 = fifo_ctrl_reg;  // toggle the clock!
        }
    }
}

static inline void ble_transmit_tx_arb_fifo(void) {
    uint32_t fifo_ctrl_reg =
        0x00000000;  // local storage for analog CFG register

    // Release data from FIFO
    fifo_ctrl_reg |= 0x00000010;  // enable div-by-2
    fifo_ctrl_reg &=
        0xFFFFFFFE;  // lower data in valid (set FIFO to output mode)
    fifo_ctrl_reg |=
        0x00000004;  // raise data out ready (set FIFO to output mode)
    fifo_ctrl_reg |= 0x00000020;  // set choose clk to 1

    ANALOG_CFG_REG__11 = fifo_ctrl_reg;
}

void ble_init(void) {
    // Clear variables.
    // memset(&ble_vars, 0, sizeof(ble_vars));

    // Set default advertiser address.
    ble_vars.advertiser_address[0] = 0x00;
    ble_vars.advertiser_address[1] = 0x02;
    ble_vars.advertiser_address[2] = 0x72;
    ble_vars.advertiser_address[3] = 0x32;
    ble_vars.advertiser_address[4] = 0x80;
    ble_vars.advertiser_address[5] = 0xC6;

    // Set default channel.
    ble_vars.channel = 37;

    // Set default name.
    ble_vars.name_tx_en = true;
    ble_vars.name[0] = 'S';
    ble_vars.name[1] = 'C';
    ble_vars.name[2] = 'U';
    ble_vars.name[3] = 'M';
    ble_vars.name[4] = '3';

    // Set appearance to Mouse
    ble_vars.appearance_en = true;
    ble_vars.appearance[0] = 0xc2;
    ble_vars.appearance[1] = 0x03;

    // ble_vars.tuning_code_tx_en = true;
    // ble_vars.counters_tx_en = true;
    // ble_vars.temperature_tx_en = true;
    ble_vars.location_en = true;
    // ble_vars.data_tx_en = true;
}

void ble_generate_packet(void) {
    uint8_t i = 0;
    uint8_t j = 0;

    int k;
    uint8_t common;
    uint8_t crc3 = 0xAA;
    uint8_t crc2 = 0xAA;
    uint8_t crc1 = 0xAA;

    uint8_t pdu_crc[BLE_PDU_LENGTH + BLE_CRC_LENGTH];

    uint8_t lfsr = (ble_vars.channel & 0x3F) | (1 << 6);  // [1 channel[6]]

    memset(ble_vars.packet, 0, 64 * sizeof(uint8_t));
    memset(pdu_crc, 0, (BLE_PDU_LENGTH + BLE_CRC_LENGTH) * sizeof(uint8_t));

    ble_vars.packet[i++] = BLE_PREAMBLE;

    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS1;
    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS2;
    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS3;
    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS4;

    pdu_crc[j++] = BLE_PDU_HEADER1;
    pdu_crc[j++] = BLE_PDU_HEADER2;

    for (k = BLE_ADVERTISER_ADDRESS_LENGTH - 1; k >= 0; --k) {
        pdu_crc[j++] = flipChar(ble_vars.advertiser_address[k]);
    }

    if (ble_vars.name_tx_en) {
        pdu_crc[j++] = BLE_SHORT_NAME_HEADER;
        pdu_crc[j++] = BLE_SHORT_NAME_GAP_CODE;

        for (k = 0; k < BLE_SHORT_NAME_LENGTH; ++k) {
            pdu_crc[j++] = flipChar(ble_vars.name[k]);
        }
    }

    if (ble_vars.tuning_code_tx_en) {
        pdu_crc[j++] = BLE_TUNING_CODE_HEADER;
        pdu_crc[j++] = BLE_TUNING_CODE_GAP_CODE;

        const uint16_t tuning_code = (ble_vars.tuning_code.coarse << 10) |
                                     (ble_vars.tuning_code.mid << 5) |
                                     ble_vars.tuning_code.fine;
        pdu_crc[j++] =
            flipChar((tuning_code >> 8) & 0xFF);      // LC freq codes MSB
        pdu_crc[j++] = flipChar(tuning_code & 0xFF);  // LC freq codes LSB
    }

    if (ble_vars.counters_tx_en) {
        pdu_crc[j++] = BLE_COUNTERS_HEADER;
        pdu_crc[j++] = BLE_COUNTERS_GAP_CODE;

        pdu_crc[j++] =
            flipChar((ble_vars.count_2M >> 24) & 0xFF);  // count_2M MSB
        pdu_crc[j++] = flipChar((ble_vars.count_2M >> 16) & 0xFF);
        pdu_crc[j++] = flipChar((ble_vars.count_2M >> 8) & 0xFF);
        pdu_crc[j++] = flipChar(ble_vars.count_2M & 0xFF);  // count_2M LSB

        pdu_crc[j++] =
            flipChar((ble_vars.count_32k >> 24) & 0xFF);  // count_32k MSB
        pdu_crc[j++] = flipChar((ble_vars.count_32k >> 16) & 0xFF);
        pdu_crc[j++] = flipChar((ble_vars.count_32k >> 8) & 0xFF);
        pdu_crc[j++] = flipChar(ble_vars.count_32k & 0xFF);  // count_32k LSB
    }

    if (ble_vars.temperature_tx_en) {
        double temperature_kelvin =
            ble_vars.temperature + 273.15;  // Temperature in Kelvin
        int temperature_payload =
            100 * temperature_kelvin + 1;  // Floating point error

        pdu_crc[j++] = BLE_TEMPERATURE_HEADER;
        pdu_crc[j++] = BLE_TEMPERATURE_GAP_CODE;

        pdu_crc[j++] =
            flipChar((temperature_payload >> 8) & 0xFF);      // Temperature MSB
        pdu_crc[j++] = flipChar(temperature_payload & 0xFF);  // Temperature LSB
    }

    if (ble_vars.data_tx_en) {
        pdu_crc[j++] = BLE_CUSTOM_DATA_HEADER;
        pdu_crc[j++] = BLE_CUSTOM_DATA_GAP_CODE;

        for (k = 0; k < BLE_CUSTOM_DATA_LENGTH; ++k) {
            pdu_crc[j++] = flipChar(ble_vars.data[k]);
        }
    }

    if (ble_vars.appearance_en) {
        pdu_crc[j++] = BLE_GAP_APPEARANCE_HEADER;
        pdu_crc[j++] = BLE_GAP_APPEARANCE_GAP_CODE;

        for (k = 0; k < BLE_GAP_APPEARANCE_LENGTH; ++k) {
            pdu_crc[j++] = flipChar(ble_vars.appearance[k]);
        }
    }

    for (j = 0; j < BLE_PDU_LENGTH; ++j) {
        for (k = 7; k >= 0; --k) {
            common = (crc1 & 1) ^ ((pdu_crc[j] & (1 << k)) >> k);
            crc1 = (crc1 >> 1) | ((crc2 & 1) << 7);
            crc2 = ((crc2 >> 1) | ((crc3 & 1) << 7)) ^ (common << 6) ^
                   (common << 5);
            crc3 = ((crc3 >> 1) | (common << 7)) ^ (common << 6) ^
                   (common << 4) ^ (common << 3) ^ (common << 1);
        }
    }

    crc1 = flipChar(crc1);
    crc2 = flipChar(crc2);
    crc3 = flipChar(crc3);

    pdu_crc[j++] = crc1;
    pdu_crc[j++] = crc2;
    pdu_crc[j++] = crc3;

    for (j = 0; j < BLE_PDU_LENGTH + BLE_CRC_LENGTH; ++j) {
        for (k = 7; k >= 0; --k) {
            pdu_crc[j] = (pdu_crc[j] & ~(1 << k)) |
                         ((pdu_crc[j] & (1 << k)) ^ ((lfsr & 1) << k));
            lfsr = ((lfsr >> 1) | ((lfsr & 1) << 6)) ^ ((lfsr & 1) << 2);
        }
    }

    memcpy(&ble_vars.packet[i], pdu_crc, BLE_PDU_LENGTH + BLE_CRC_LENGTH);

    printf("pkt : ");
    for (j = 0; j < BLE_PDU_LENGTH + BLE_CRC_LENGTH + 5; j++) {
        printf("%x ", flipChar(ble_vars.packet[j]));
    }
    printf("\r\n");
}

void ble_generate_location_packet(void) {
    uint8_t i = 0;
    uint8_t j = 0;

    int k;
    uint8_t common;
    uint8_t crc3 = 0xAA;
    uint8_t crc2 = 0xAA;
    uint8_t crc1 = 0xAA;

    uint8_t pdu_crc[BLE_PDU_LENGTH + BLE_CRC_LENGTH];

    uint8_t lfsr = (ble_vars.channel & 0x3F) | (1 << 6);  // [1 channel[6]]

    memset(ble_vars.packet, 0, 64 * sizeof(uint8_t));
    memset(pdu_crc, 0, (BLE_PDU_LENGTH + BLE_CRC_LENGTH) * sizeof(uint8_t));

    ble_vars.packet[i++] = BLE_PREAMBLE;

    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS1;
    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS2;
    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS3;
    ble_vars.packet[i++] = BLE_ACCESS_ADDRESS4;

    pdu_crc[j++] = BLE_PDU_HEADER1;
    pdu_crc[j++] = BLE_PDU_HEADER2;

    for (k = BLE_ADVERTISER_ADDRESS_LENGTH - 1; k >= 0; --k) {
        pdu_crc[j++] = flipChar(ble_vars.advertiser_address[k]);
    }

    if (ble_vars.name_tx_en) {
        pdu_crc[j++] = BLE_SHORT_NAME_HEADER;
        pdu_crc[j++] = BLE_SHORT_NAME_GAP_CODE;

        for (k = 0; k < BLE_SHORT_NAME_LENGTH; ++k) {
            pdu_crc[j++] = flipChar(ble_vars.name[k]);
        }
    }

    if (ble_vars.tuning_code_tx_en) {
        pdu_crc[j++] = BLE_TUNING_CODE_HEADER;
        pdu_crc[j++] = BLE_TUNING_CODE_GAP_CODE;

        const uint16_t tuning_code = (ble_vars.tuning_code.coarse << 10) |
                                     (ble_vars.tuning_code.mid << 5) |
                                     ble_vars.tuning_code.fine;
        pdu_crc[j++] =
            flipChar((tuning_code >> 8) & 0xFF);      // LC freq codes MSB
        pdu_crc[j++] = flipChar(tuning_code & 0xFF);  // LC freq codes LSB
    }
    // counter has 2 32bit value, I use this so donot need generate new gap code
    // or header
    if (ble_vars.location_en) {
        pdu_crc[j++] = BLE_LOCATION_DATA_HEADER;
        pdu_crc[j++] = BLE_LOCATION_DATA_GAP_CODE;

        pdu_crc[j++] =
            flipChar((ble_vars.location_x >> 24) & 0xFF);  // location_x MSB
        pdu_crc[j++] = flipChar((ble_vars.location_x >> 16) & 0xFF);
        pdu_crc[j++] = flipChar((ble_vars.location_x >> 8) & 0xFF);
        pdu_crc[j++] = flipChar(ble_vars.location_x & 0xFF);  // location_x LSB

        pdu_crc[j++] =
            flipChar((ble_vars.location_y >> 24) & 0xFF);  // location_y MSB
        pdu_crc[j++] = flipChar((ble_vars.location_y >> 16) & 0xFF);
        pdu_crc[j++] = flipChar((ble_vars.location_y >> 8) & 0xFF);
        pdu_crc[j++] = flipChar(ble_vars.location_y & 0xFF);  // location_y LSB
    }

    if (ble_vars.temperature_tx_en) {
        double temperature_kelvin =
            ble_vars.temperature + 273.15;  // Temperature in Kelvin
        int temperature_payload =
            100 * temperature_kelvin + 1;  // Floating point error

        pdu_crc[j++] = BLE_TEMPERATURE_HEADER;
        pdu_crc[j++] = BLE_TEMPERATURE_GAP_CODE;

        pdu_crc[j++] =
            flipChar((temperature_payload >> 8) & 0xFF);      // Temperature MSB
        pdu_crc[j++] = flipChar(temperature_payload & 0xFF);  // Temperature LSB
    }

    if (ble_vars.data_tx_en) {
        pdu_crc[j++] = BLE_CUSTOM_DATA_HEADER;
        pdu_crc[j++] = BLE_CUSTOM_DATA_GAP_CODE;

        for (k = 0; k < BLE_CUSTOM_DATA_LENGTH; ++k) {
            pdu_crc[j++] = flipChar(ble_vars.data[k]);
        }
    }

    if (ble_vars.appearance_en) {
        pdu_crc[j++] = BLE_GAP_APPEARANCE_HEADER;
        pdu_crc[j++] = BLE_GAP_APPEARANCE_GAP_CODE;

        for (k = 0; k < BLE_GAP_APPEARANCE_LENGTH; ++k) {
            pdu_crc[j++] = flipChar(ble_vars.appearance[k]);
        }
    }

    for (j = 0; j < BLE_PDU_LENGTH; ++j) {
        for (k = 7; k >= 0; --k) {
            common = (crc1 & 1) ^ ((pdu_crc[j] & (1 << k)) >> k);
            crc1 = (crc1 >> 1) | ((crc2 & 1) << 7);
            crc2 = ((crc2 >> 1) | ((crc3 & 1) << 7)) ^ (common << 6) ^
                   (common << 5);
            crc3 = ((crc3 >> 1) | (common << 7)) ^ (common << 6) ^
                   (common << 4) ^ (common << 3) ^ (common << 1);
        }
    }

    crc1 = flipChar(crc1);
    crc2 = flipChar(crc2);
    crc3 = flipChar(crc3);

    pdu_crc[j++] = crc1;
    pdu_crc[j++] = crc2;
    pdu_crc[j++] = crc3;

    for (j = 0; j < BLE_PDU_LENGTH + BLE_CRC_LENGTH; ++j) {
        for (k = 7; k >= 0; --k) {
            pdu_crc[j] = (pdu_crc[j] & ~(1 << k)) |
                         ((pdu_crc[j] & (1 << k)) ^ ((lfsr & 1) << k));
            lfsr = ((lfsr >> 1) | ((lfsr & 1) << 6)) ^ ((lfsr & 1) << 2);
        }
    }

    memcpy(&ble_vars.packet[i], pdu_crc, BLE_PDU_LENGTH + BLE_CRC_LENGTH);

    printf("pkt : ");
    for (j = 0; j < BLE_PDU_LENGTH + BLE_CRC_LENGTH + 5; j++) {
        printf("%x ", flipChar(ble_vars.packet[j]));
    }
    printf("\r\n");
}

void ble_gen_test_packet(void) {
    uint8_t i = 0;

    memset(ble_vars.packet, 0, sizeof(ble_vars.packet) * sizeof(uint8_t));
    ble_vars.packet[i++] = 0x1D;
    ble_vars.packet[i++] = 0x55;
    ble_vars.packet[i++] = 0xAD;
    ble_vars.packet[i++] = 0xF6;
    ble_vars.packet[i++] = 0x45;
    ble_vars.packet[i++] = 0xC7;
    ble_vars.packet[i++] = 0xC5;
    ble_vars.packet[i++] = 0x0E;
    ble_vars.packet[i++] = 0x26;
    ble_vars.packet[i++] = 0x13;
    ble_vars.packet[i++] = 0xC2;
    ble_vars.packet[i++] = 0xAC;
    ble_vars.packet[i++] = 0x98;
    ble_vars.packet[i++] = 0x37;
    ble_vars.packet[i++] = 0xB8;
    ble_vars.packet[i++] = 0x30;
    ble_vars.packet[i++] = 0xA1;
    ble_vars.packet[i++] = 0xC9;
    ble_vars.packet[i++] = 0xE4;
    ble_vars.packet[i++] = 0x93;
    ble_vars.packet[i++] = 0x75;
    ble_vars.packet[i++] = 0xB7;
    ble_vars.packet[i++] = 0x41;
    ble_vars.packet[i++] = 0x6C;
    ble_vars.packet[i++] = 0xD1;
    ble_vars.packet[i++] = 0x2D;
    ble_vars.packet[i++] = 0xB8;
}

void ble_set_advertiser_address(const uint8_t* advertiser_address) {
    memcpy(ble_vars.advertiser_address, advertiser_address,
           BLE_ADVERTISER_ADDRESS_LENGTH);
}

void ble_set_channel(const uint8_t channel) { ble_vars.channel = channel; }

void ble_set_name_tx_en(const bool enable) { ble_vars.name_tx_en = enable; }

void ble_set_name(const char* name) {
    memcpy(ble_vars.name, name, BLE_SHORT_NAME_LENGTH);
}

void ble_set_tuning_code_tx_en(const bool enable) {
    ble_vars.tuning_code_tx_en = enable;
}

void ble_set_tuning_code(const tuning_code_t* tuning_code) {
    ble_vars.tuning_code = *tuning_code;
}

void ble_set_counters_tx_en(const bool enable) {
    ble_vars.counters_tx_en = enable;
}

void ble_set_count_2M(const uint32_t count_2M) { ble_vars.count_2M = count_2M; }

void ble_set_count_32k(const uint32_t count_32k) {
    ble_vars.count_32k = count_32k;
}

void ble_set_temperature_tx_en(const bool enable) {
    ble_vars.temperature_tx_en = enable;
}

void ble_set_temperature(const double temperature) {
    ble_vars.temperature = temperature;
}

void ble_set_data_tx_en(const bool enable) { ble_vars.data_tx_en = enable; }

void ble_set_data(const uint8_t* data) {
    memcpy(ble_vars.data, data, BLE_CUSTOM_DATA_LENGTH);
}

void ble_init_tx(void) {
    // Set up BLE modulation source.
    // ----
    // mod_logic<3:0> = ASC<996:999>
    // The two LSBs change the mux from cortex mod (00) source to pad (11).
    // They can also be used to set the modulation to 0 (10) or 1 (01).
    // The other bits are used for inverting the modulation bitstream and can be
    // cleared. The goal is to remove the 15.4 DAC from the modulation path.
    /*
    clear_asc_bit(996);
    set_asc_bit(997);
    clear_asc_bit(998);
    clear_asc_bit(999);
    */
    set_asc_bit(996);
    clear_asc_bit(997);
    clear_asc_bit(998);
    clear_asc_bit(999);

    clear_asc_bit(1000);
    clear_asc_bit(1001);
    clear_asc_bit(1002);
    clear_asc_bit(1003);

    // Make sure the BLE modulation mux is set.
    // Bit 1013 sets the BLE mod dac to cortex control.
    // The BLE tone spacing cannot be set, it is a fixed DAC.
    set_asc_bit(1013);
    // ----

    // Set the bypass scan bits of the BLE FIFO and GFSK modules.
    set_asc_bit(1041);
    set_asc_bit(1042);

    // Ensure cortex control of LO.
    clear_asc_bit(964);

    // Ensure cortex control of divider.
    clear_asc_bit(1081);

    // Need to set analog_cfg<183> to 0 to select BLE for chips out.
    ANALOG_CFG_REG__11 = 0x0000;

    // Set current in LC tank.
    set_LC_current(127);

    // Set LDO voltages for PA and LO.
    set_PA_supply(127);
    set_LO_supply(63, 0);
}

void ble_init_rx(void) {
    radio_init_rx_ZCC();

    // Set clock mux to internal RC oscillator.
    clear_asc_bit(424);
    set_asc_bit(425);

    // Need to turn on ADC clock generation to get /4 output for clock
    // calibration.
    set_asc_bit(422);

    // Set gain for I and Q.
    set_IF_gain_ASC(63, 63);

    // Set gm for stg3 ADC drivers.
    set_IF_stg3gm_ASC(7, 7);  // (I, Q)

    // Set comparator trims.
    // set_IF_comparator_trim_I(0, 7); // (p,n)
    // set_IF_comparator_trim_Q(15, 0); // (p,n)

    // Set clock divider value for zcc.
    // The IF clock divided by this value must equal 1 MHz for BLE 1M PHY.
    set_IF_ZCC_clkdiv(76);

    // Disable feedback.
    clear_asc_bit(123);

    // Mixer and polyphase control settings can be driven from either ASC or
    // memory mapped I/O. Mixers and polyphase should both be enabled for RX and
    // both disabled for TX.
    // --
    // For polyphase (1=enabled),
    //  mux select signal ASC<746>=0 gives control to ASC<971>.
    //  mux select signal ASC<746>=1 gives control to analog_cfg<256> (bit 0 of
    //  ANALOG_CFG_REG__16).
    // --
    // For mixers (0=enabled), both I and Q should be enabled for matched filter
    // mode.
    //  mux select signals ASC<744>=0 and ASC<745>=0 give control to ASC<298>
    //  and ASC<307>. mux select signals ASC<744>=1 and ASC<745>=1 give control
    //  to analog_cfg<257> analog_cfg<258> (bits 1 and 2 in ANALOG_CFG_REG__16).

    // Set mixer and polyphase control signals to memory mapped I/O.
    set_asc_bit(744);
    set_asc_bit(745);
    set_asc_bit(746);

    // Enable both polyphase and mixers via memory mapped IO (...001 = 0x1).
    // To disable both you would invert these values (...110 = 0x6).
    ANALOG_CFG_REG__16 = 0x1;
}

void ble_transmit(void) {
    ble_load_tx_arb_fifo();

    // Turn on LO, PA, and DIV.
    radio_txEnable();

    // Wait for LDOs to turn on.
    for (uint32_t t = 0; t < 50; ++t) {
    }

    // Send the packet.
    ble_transmit_tx_arb_fifo();

    // Wait for transmission to finish.
    // Don't know if there is some way to know when this has finished or just
    // busy wait (?). This was determined empirically using trial and error.
    for (uint32_t t = 0; t < 1000; ++t) {
    }

    // radio_rfOff();
}

//=========================== private =========================================
