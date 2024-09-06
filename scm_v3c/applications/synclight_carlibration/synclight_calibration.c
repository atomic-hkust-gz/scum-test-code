#include <string.h>

#include "ble_ti.h"
#include "calibrate_interrupt.h"
#include "gpio.h"
#include "lighthouse_position.h"
#include "memory_map.h"
#include "optical.h"
#include "radio.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"
#include "scum_defs.h"
#include "tuning.h"

//=========================== defines =========================================

#define CRC_VALUE (*((unsigned int*)0x0000FFFC))
#define CODE_LENGTH (*((unsigned int*)0x0000FFF8))

#define CALIBRATE_SYNCLIGHT_INPUT 8  // receive sync light on this pin
#define CALIBRATE_OUTPUT \
    10  // toggle this pin to show scum received a sync light
#define OPTICAL_DATA_RAW_PIN \
    ((0x0008 & GPIO_REG__INPUT) >> 3)  // optical receiver digital data pin
// indicate the type of light
#define type_sync 0
#define type_sweep 1
#define type_skip_sync 2

// initialized value for frequency configuration(from scm3c_hw_interface.c)
#define INIT_HF_CLOCK_FINE 17
#define INIT_HF_CLOCK_COARSE 3
#define INIT_RC2M_COARSE 21
#define INIT_RC2M_FINE 15
#define INIT_RC2M_SUPERFINE 15
#define INIT_IF_CLK_TARGET 1600000
#define INIT_IF_COARSE 22
#define INIT_IF_FINE 18

//=========================== variables ===========================

typedef struct {
    uint8_t count;
} app_vars_t;

app_vars_t app_vars;

extern int8_t need_optical;

enum State {
    // this state, reserved for debugging
    DEFAULT,
    // receive light and calculate localization
    COLLECTING,
    // use sync light to calibrate clock
    CALIBRATING,
    // generate BLE packet which carried location information
    INTEGRATING,
    // transmate BLE packet
    SENDING
};
enum State scum_state;
// a state indicator to memorize current state
uint8_t counter_state = 0;

int t;
// Variables for lighthouse RX, store OPTICAL_DATA_RAW pin state
unsigned short current_gpio = 0, last_gpio = 0, state = 0, nextstate = 0,
               pulse_type = 0;
unsigned int timestamp_rise, timestamp_fall, pulse_width;
// variables from lighthouse tracking repo
uint32_t t_0_start = 0x00;
uint32_t t_0_end = 0x00;
uint32_t t_opt_pulse = 0x00;
uint32_t t_opt_pulse_us = 0x00;
uint32_t t_1_start = 0x00;
uint32_t t_d_start = 0x00;
uint8_t flag_start = 0;
// 0:sync;1:sweep;2:skip_sync
uint8_t flag_light_type = 0;

// after a sweep, wo is the first which is the station A
uint8_t flag_station = 0;
// 0：NULL,1:A,2:B
uint8_t flag_A_station = 0;
uint32_t loca_duration = 0;
uint8_t loca_x = 0;
uint32_t A_X = 0;
uint32_t A_Y = 0;
uint32_t B_X = 0;
uint32_t B_Y = 0;

// store synclight then call synclight calibration
uint8_t count_sync_light = 0;
// whether enable the synclight calibration
uint8_t need_sync_calibration = 0;
// record 10 synclight duration
uint32_t servel_synclights_start = 0;
// uint32_t servel_synclights_end = 0;
uint32_t servel_synclights_duration = 0;
// total calibration times
uint32_t count_calibration = 0;

// Read HF_CLK counter,from optical_sfd_isr() at optical.c
#define HF_CLK_RDATA_LSB *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x100000)
#define HF_CLK_RDATA_MSB *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x140000)
#define HF_CLK_COUNT (HF_CLK_RDATA_LSB + (HF_CLK_RDATA_MSB << 16))

// ble variables from ble_tx_ti.c
/************************************************************************ */
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
        for (uint32_t t = 0; t < 5000; ++t);

        ble_transmit();
    }
#else   // !BLE_TX_SWEEP_FINE
    tuning_tune_radio(&g_ble_tx_tuning_code);
    printf("Transmitting BLE packet on %u.%u.%u.\n",
           g_ble_tx_tuning_code.coarse, g_ble_tx_tuning_code.mid,
           g_ble_tx_tuning_code.fine);

    // Wait for frequency to settle.
    for (uint32_t t = 0; t < 5000; ++t);

    ble_transmit();
#endif  // BLE_TX_SWEEP_FINE
}

static void ble_tx_rftimer_callback(void) {
    // Trigger a BLE TX.
    g_ble_tx_trigger = true;
}
/************************************************************************ */
//=========================== prototypes ======================================
// gpio pins, RFtimer, HCLK... config for lighthouse localization
void config_lighthouse_mote(void) {
    scm3c_hw_interface_init();
    optical_init();
    radio_init();
    rftimer_init();
    // a copy of optical_init
    sync_light_calibrate_init();

    //  I think RF timer needs to be reset before use, but not essential.
    // RF Timer rolls over at this value and starts a new cycle
    RFTIMER_REG__MAX_COUNT = 0xFFFFFFFF;
    // Enable RF Timer
    RFTIMER_REG__CONTROL = 0x7;

    // Init LDO control
    init_ldo_control();

    // Select banks for GPIO inputs
    GPI_control(0, 0, 0, 0);
    // Select banks for GPIO outputs, now IO 8,10 is used for test(XX6X)
    GPO_control(0, 0, 6, 0);
    // Set all GPIOs as outputs
    GPI_enables(0x000F);  // 0008=io3?
    GPO_enables(0xFFFF);

    // Set HCLK source as HF_CLOCK
    set_asc_bit(1147);

    // Set initial coarse/fine on HF_CLOCK
    // coarse 0:4 = 860 861 875b 876b 877b
    // fine 0:4 870 871 872 873 874b
    set_sys_clk_secondary_freq(INIT_HF_CLOCK_COARSE, INIT_HF_CLOCK_FINE);

    // Set RFTimer source as HF_CLOCK
    set_asc_bit(1151);

    // Disable LF_CLOCK
    set_asc_bit(553);

    // HF_CLOCK will be trimmed to 20MHz, so set RFTimer div value to 2 to get
    // 10MHz (inverted, so 0000 0010-->1111 1101)
    // infact, the max freq is 10M, 20M need to raise the supply voltage for
    // VDDD. once change HCLK, UART baudrate will change too.
    set_asc_bit(49);
    set_asc_bit(48);
    set_asc_bit(47);
    set_asc_bit(46);
    set_asc_bit(45);
    set_asc_bit(44);
    clear_asc_bit(43);
    set_asc_bit(42);

    // try to use divider on HFCLK
    // Set HCLK divider to 2(0000 0010->0000 0110,only third low bit need
    // invert) infact, the max freq is 10M, 20M need to raise the supply voltage
    // for VDDD. HCLK is for cortex core, if you do not set it to 10M, it stay
    // at 5M by default. if HCLK is 5M, RFtimer can not faster than 5M. I do not
    // know the reason.
    clear_asc_bit(57);
    clear_asc_bit(56);
    clear_asc_bit(55);
    clear_asc_bit(54);
    clear_asc_bit(53);
    set_asc_bit(52);  // inverted
    set_asc_bit(51);
    clear_asc_bit(50);

    // Set RF Timer divider to pass through so that RF Timer is 20 MHz,(not
    // faster than 5M if didnt set HCLK)
    //  passthrough means ignore the divider.
    // set_asc_bit(36);

    // need LC, IF, 2M to calibrate.
    // Set 2M RC as source for chip CLK
    set_asc_bit(1156);

    // Enable 32k for cal
    set_asc_bit(623);

    // Enable passthrough on chip CLK divider
    set_asc_bit(41);

    // Init counter setup - set all to analog_cfg control
    // scm3c_hw_interface_vars.ASC[0] is leftmost
    // scm3c_hw_interface_vars.ASC[0] |= 0x6F800000;
    for (t = 2; t < 9; t++) set_asc_bit(t);

    // Init RX
    radio_init_rx_MF();

    // Init TX
    radio_init_tx();

    // Set initial IF ADC clock frequency
    set_IF_clock_frequency(INIT_IF_COARSE, INIT_IF_FINE, 0);  // cr

    // Set initial TX clock frequency
    set_2M_RC_frequency(31, 31, INIT_RC2M_COARSE, INIT_RC2M_FINE,
                        INIT_RC2M_SUPERFINE);  // cr

    // Turn on RC 2M for cal
    set_asc_bit(1114);  // cr

    // Set initial LO frequency
    LC_monotonic(DEFUALT_INIT_LC_CODE);

    // Init divider settings
    radio_init_divider(2000);

    analog_scan_chain_write();
    analog_scan_chain_load();
}

void config_ble_tx_mote(void) {
    // Set HCLK source as HF_CLOCK
    set_asc_bit(1147);

    // Set initial coarse/fine on HF_CLOCK
    // coarse 0:4 = 860 861 875b 876b 877b
    // fine 0:4 870 871 872 873 874b
    set_sys_clk_secondary_freq(INIT_HF_CLOCK_COARSE, INIT_HF_CLOCK_FINE);

    // Set RFTimer source as HF_CLOCK
    set_asc_bit(1151);

    // Disable LF_CLOCK
    set_asc_bit(553);

    // HF_CLOCK will be trimmed to 20MHz, so set RFTimer div value to 40 to get
    // 500kHz (inverted, so 1101 0111)
    set_asc_bit(49);
    set_asc_bit(48);
    clear_asc_bit(47);
    set_asc_bit(46);
    clear_asc_bit(45);
    set_asc_bit(44);
    set_asc_bit(43);
    set_asc_bit(42);

    // Set 2M RC as source for chip CLK
    set_asc_bit(1156);

    // Enable 32k for cal
    set_asc_bit(623);

    // Enable passthrough on chip CLK divider
    set_asc_bit(41);

    // Init counter setup - set all to analog_cfg control
    // scm3c_hw_interface_vars.ASC[0] is leftmost
    // scm3c_hw_interface_vars.ASC[0] |= 0x6F800000;
    for (t = 2; t < 9; t++) set_asc_bit(t);

    // Init RX
    radio_init_rx_MF();

    // Init TX
    radio_init_tx();

    // Set initial IF ADC clock frequency
    set_IF_clock_frequency(INIT_IF_COARSE, INIT_IF_FINE, 0);

    // Set initial TX clock frequency
    set_2M_RC_frequency(31, 31, INIT_RC2M_COARSE, INIT_RC2M_FINE,
                        INIT_RC2M_SUPERFINE);

    // Turn on RC 2M for cal
    set_asc_bit(1114);

    // Set initial LO frequency
    LC_monotonic(DEFUALT_INIT_LC_CODE);

    // Init divider settings
    radio_init_divider(2000);

    // Program analog scan chain
    analog_scan_chain_write();
    analog_scan_chain_load();
    //--------------------------------------------------------
};
// a method used to replace old xy distinguish method from kilberg code.
#define WIDTH_BIAS (0 - 0)
void distinguish_xy(uint32_t light_duration) {
    // Identify what kind of pulse this was

    // if (light_duration < 585 + WIDTH_BIAS && light_duration > 100 +
    // WIDTH_BIAS)
    //     loca_x = LASER;  // Laser sweep (THIS NEEDS TUNING)
    if (light_duration < 675 + WIDTH_BIAS &&
        light_duration > 500 + WIDTH_BIAS) {
        loca_x = 1;  // Azimuth sync, data=0, skip = 0
        count_sync_light += 1;
    } else if (light_duration >= 675 + WIDTH_BIAS &&
               light_duration < 781 + WIDTH_BIAS) {
        loca_x = 0;  // Elevation sync, data=0, skip = 0
        count_sync_light += 1;
    } else if (light_duration >= 781 + WIDTH_BIAS &&
               light_duration < 885 + WIDTH_BIAS) {
        loca_x = 1;  // Azimuth sync, data=1, skip = 0
        count_sync_light += 1;
    } else if (light_duration >= 885 + WIDTH_BIAS &&
               light_duration < 989 + WIDTH_BIAS) {
        loca_x = 0;  // Elevation sync, data=1, skip = 0
        count_sync_light += 1;
    } else if (light_duration >= 989 + WIDTH_BIAS &&
               light_duration < 1083 + WIDTH_BIAS)
        loca_x = 1;  // Azimuth sync, data=0, skip = 1
    else if (light_duration >= 1083 + WIDTH_BIAS &&
             light_duration < 1200 + WIDTH_BIAS)
        loca_x = 0;  // elevation sync, data=0, skip = 1
    else if (light_duration >= 1200 + WIDTH_BIAS &&
             light_duration < 1300 + WIDTH_BIAS)
        loca_x = 1;  // Azimuth sync, data=1, skip = 1
    else if (light_duration >= 1300 + WIDTH_BIAS &&
             light_duration < 1400 + WIDTH_BIAS)
        loca_x = 0;  // Elevation sync, data=1, skip = 1
}
// some debug vars, canbe deleted later
// divide timer then multiply it to make codecompatible
int para_temp = 1;
int tmp_count_sweep = 0;
int tmp_count_sync = 0;
int tmp_count_skip = 0;
uint32_t tmp_sync_width = 0;

// after get 10 sync lights, call this function to calibrate clocks
void perform_synclight_calibration(void) { sync_light_calibrate_isr(); }

void decode_lighthouse(void) {
    // This is the main function of lighthouse protocol decoding
    // lighthouse code start
    last_gpio = current_gpio;
    current_gpio = OPTICAL_DATA_RAW_PIN;

    // Detect rising edge
    if (last_gpio == 0 && current_gpio == 1) {
        // Reset RF Timer count register at rising edge of first sync pulse

        // Save when this event happened
        timestamp_rise = RFTIMER_REG__COUNTER;
        //      only for debugging
        gpio_10_toggle();
        switch (flag_start) {
            case 0:
                t_0_start = timestamp_rise;

                flag_start = 1;
                break;
            case 1:
                break;
            default:
                break;
        }

    }

    // Detect falling edge
    else if (last_gpio == 1 && current_gpio == 0) {
        // Save when this event happened
        timestamp_fall = RFTIMER_REG__COUNTER;

        // Calculate how wide this pulse was
        pulse_width = timestamp_fall - timestamp_rise;
        gpio_10_toggle();
        //      gpio_8_toggle();

        // Need to determine what kind of pulse this was
        // Laser sweep pulses will have widths of only a few us
        // Sync pulses have a width corresponding to
        // 62.5 us - azimuth   - data=0 (625 ticks of 10MHz clock)
        // 72.9 us - elevation - data=0 (729 ticks)
        // 83.3 us - azimuth   - data=1 (833 ticks)
        // 93.8 us - elevation - data=0 (938 ticks)
        // A second lighthouse can be distinguished by differences in these
        // pulse widths
        //            update_state(classify_pulse(timestamp_rise,
        //            timestamp_fall),
        //                         timestamp_rise);

        switch (flag_start) {
            case 0:
                break;
            case 1:
                t_0_end = timestamp_fall;
                flag_start = 0;
                t_opt_pulse = t_0_end - t_0_start;
                // Dividing the two signals by 50us: 0.000,050/(1/10M) = 500
                // = 0x320,99us(990ticks) for skip/sync
                (t_opt_pulse <
                 500)  // actual boundary condition maybe a little different
                    ? (flag_light_type = type_sweep)
                    : ((t_opt_pulse < 990)
                           ? (flag_light_type = type_sync)
                           : (flag_light_type = type_skip_sync));
                t_opt_pulse_us = t_opt_pulse / 10;
                switch (flag_light_type) {
                    // More than 50us then it must be sync, then the next
                    // falling edge interrupt will need to calculate
                    // position
                    case type_sync:
                        // If sync, distance measurement starts from this
                        // time.
                        t_d_start = t_0_start;
                        // It is only based on sweep that you can determine
                        // whether you are currently in A or B.
                        if (flag_station >= 1) {
                            //  Where an even ten digit number is the
                            //  X-axis, an odd number is the Y-axis.
                            // offcourse ,but scum not very accurate, I should
                            // replace it with another medthod to distinguish XY
                            // ((t_opt_pulse_us / 10) % 2 == 0) ? (loca_x = 1)
                            //                                  : (loca_x = 0);

                            // update loca_x
                            distinguish_xy(t_opt_pulse);
                        }
                        // Determine whether this is an A or B base station
                        // based on the value of flag_station
                        switch (flag_station) {
                            case 0:
                                break;
                            case 1:
                                flag_A_station = 1;
                                break;
                            case 2:
                                flag_A_station = 2;
                                break;
                            default:
                                break;
                        }
                        // after a sync, the count_sync_light will increase 1,
                        // so when the count_sync_light = 1, it means this is
                        // the first sync light. record start time.
                        if (count_sync_light == 1) {
                            servel_synclights_start = t_0_start;
                        }
                        // when the count turn to 10, it is a sync calibration
                        // period.
                        if ((count_sync_light == 10) &&
                            (need_sync_calibration == 1)) {
                            count_sync_light = 0;
                            servel_synclights_duration =
                                t_0_start - servel_synclights_start;
                            gpio_8_toggle();  // debug,remove later
                            perform_synclight_calibration();
                            count_calibration += 1;
                        }
                        break;
                    case type_sweep:
                        // gpio_8_toggle();  // debug,remove later
                        //  0 ：NULL,1: next is A,2:next is B
                        flag_station = 1;

                        loca_duration = t_0_start - t_d_start;
                        switch (flag_A_station) {
                            case 0:
                                break;
                            // A
                            case 1:
                                if (loca_x == 1) {
                                    A_X = loca_duration;
                                } else {
                                    A_Y = loca_duration;
                                }
                                flag_A_station = 0;
                                break;
                            // B
                            case 2:
                                if (loca_x == 1) {
                                    B_X = loca_duration;
                                } else {
                                    B_Y = loca_duration;
                                }
                                flag_A_station = 0;
                                break;
                            default:
                                break;
                        }

                        break;
                    case type_skip_sync:
                        if (flag_station >= 1) {
                            flag_station++;
                            // Exceeding 2 means that a sweep was not seen,
                            // which often happens.
                            if (flag_station >= 3) {
                                flag_station--;
                            }
                        }
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }
}

static void update_state(enum State current_state) {
    if (current_state == DEFAULT) {
        scum_state = SENDING;
    }
};
//=========================== main ============================================

int main(void) {
    uint32_t i;
    uint32_t hf_rdata_lsb, hf_rdata_msb, hf_count_HFclock;  // test HF_CLK
    uint32_t counter_state;
    // how many times to transmite ble packets in single SENDING state
    uint8_t counter_ble_tx;

    memset(&app_vars, 0, sizeof(app_vars_t));

    printf("Initializing...");
    initialize_mote();
    crc_check();
    perform_calibration();
    printf("~~~~my code start~~~~~%d\n", app_vars.count);

    config_lighthouse_mote();
    radio_rxEnable();  // openLC,IF?

    // clean optical and ex3 interrupt, then re-open ext_3 interrupt
    need_optical = 0;
    // enbale perform calibration in decode_lighthouse() by set to 1
    need_sync_calibration = 0;

    // disable all interrupts
    ICER = 0xFFFF;

    //  test rftimer
    // ISER = 0x0080;

    // test HF_CLK
    // Read HF_CLOCK counter
    hf_rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x100000);
    hf_rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x140000);
    hf_count_HFclock = hf_rdata_lsb + (hf_rdata_msb << 16);

    printf("~~~~start to say HELLO?~~~~~%d\n", app_vars.count);

    // delay_milliseconds_asynchronous(1000, 7);//test RFtimer freq,if rftimer
    // is 500KHz, interrupt per 1s(1000ms)
    //  here is the timecounter to control print velocity
    i = 0;
    scum_state = DEFAULT;
    while (1) {
        update_state(scum_state);
        switch (scum_state) {
            case COLLECTING:
                printf("State: Locating SCUM.\n");
                // in this state, we only need location information, so disable
                // calibration part.
                need_sync_calibration = 0;

                config_lighthouse_mote();
                // disable all interrupts. rftimer interrupt is used in ble
                // transmitting
                ICER = 0xFFFF;
                decode_lighthouse();
                //      wait some time then print to uart
                i++;
                // i=10,000~=100ms(85ms), a brief printf timer.
                if (i == 11760) {
                    i = 0;
                    // gpio_8_toggle();
                    // sync_light_calibrate_isr();
                    // printf("13 syc: %u, total num: %u\n",
                    // servel_synclights_duration,
                    //        count_calibration);
                    // printf("syc: %u\n", tmp_sync_width);

                    //            printf("opt_pulse: %u, interval: %u\n",
                    //            t_opt_pulse,loca_duration);

                    // printf("hfclk: %u\n", hf_count_HFclock);

                    // printf("A_X: %u, A_Y: %u, B_X: %u, B_Y: %u\n", A_X, A_Y,
                    // B_X, B_Y);
                }

                //        printf("Hello World! %d\n", app_vars.count);
                // app_vars.count += 1;

                // for (i = 0; i < 1000000; i++);

                break;

            case CALIBRATING:
                printf("State: Calibrating.\n");
                // to get sync light for calibration must need decode
                // lighthouse, but can we make this function to two separate
                // parts? Is that essential?
                need_sync_calibration = 1;
                decode_lighthouse();
                break;
            case INTEGRATING:
                printf("State: Integrating ble packet.\n");
                // a small gap used to generate the ble packet. for this
                // fuction, I think it does not need to be an individual state,
                // but in this way will be clearly
                need_sync_calibration = 0;
                ble_generate_location_packet();
                break;
            case SENDING:
                printf("State: BLE transimitting.\n");
                // disable synclight calibration
                need_sync_calibration = 0;

                config_ble_tx_mote();
                // disable all interrupts. Is this step useful or essential?
                ICER = 0xFFFF;
                // set this value to control how many times to transmitting
                counter_ble_tx = 5;
                // copy from ble_tx(titan version) ble_init();
                ble_init_tx();
                analog_scan_chain_write();
                analog_scan_chain_load();

                // Generate a BLE packet.
                ble_generate_packet();

                // Configure the RF timer.
                rftimer_set_callback_by_id(ble_tx_rftimer_callback, 7);
                rftimer_enable_interrupts();
                rftimer_enable_interrupts_by_id(7);
                // transmitting...
                while (true) {  // counter_ble_tx
                    if (g_ble_tx_trigger) {
                        printf("Triggering BLE TX.\n");
                        ble_tx_trigger();
                        g_ble_tx_trigger = false;
                        delay_milliseconds_asynchronous(BLE_TX_PERIOD_MS, 7);
                    }
                    counter_ble_tx--;
                }
                break;
            case DEFAULT:
                printf("State: Scum is running.\n");
                for (i = 0; i < 1000000; i++);
                break;

            default:
                printf("Unknown state\n");
                break;
        }
    }
}

//=========================== public ==========================================

//=========================== private =========================================
