#include <stdint.h>
#include <stdio.h>
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

extern optical_vars_t optical_vars;
extern synclight_calibrate_vars_t synclight_cal_vars;
extern enum State_INTERRUPT_IO8 gpio_ext8_state;

enum State {
    // this state, reserved for debugging
    DEFAULT,
    // whether localization or calibration, we need to turn on 10M rftimer, use
    // optical receiver etc. it has two sub state, COLLECTING and CALIBRATING
    OPTICAL_WORKING,
    // receive light and calculate localization, sub state of OPTICAL_WORKING
    COLLECTING,
    // use sync light to calibrate clock, sub state of OPTICAL_WORKING
    OPTICAL_CALIBRATING,
    // use perform calibration to calibrate clock
    RAW_CALIBRATING,
    // generate BLE packet which carried location information
    INTEGRATING,
    // transmate BLE packet
    SENDING
};
enum State scum_state;
typedef struct {
    // store synclight then call synclight calibration
    uint8_t count_sync_light;
    // 添加一个标志位表示当前是否在有效的计数周期内
    bool in_valid_counting_period;
    // whether enable the synclight calibration
    uint8_t need_sync_calibration;
    // record 10 synclight duration
    uint32_t several_synclights_start;
    // uint32_t servel_synclights_end = 0;
    uint32_t several_synclights_duration;
    // total calibration times
    uint32_t count_calibration;

    // a counter to record how many lighthouse decoding process passed.
    uint32_t counter_localization;
    // to set a entire lighthouse localization period duration.
    uint32_t counter_lighthouse_state_period;
    // to be a individual timer for recording time.
    uint32_t counter_global_timer;
} sync_light_calibration_t;

sync_light_calibration_t sync_cal = {.count_sync_light = 0,
                                     .in_valid_counting_period = false,
                                     .need_sync_calibration = 0,
                                     .several_synclights_start = 0,
                                     .several_synclights_duration = 0,
                                     .count_calibration = 0,
                                     .counter_localization = 0,
                                     .counter_lighthouse_state_period = 20000,
                                     .counter_global_timer = 0};
// indicate the type of light
enum Lighthouse_light_type {
    // duration longer than 50ms
    sync_light,
    // duration shorter than 30ms(usually 15ms)
    sweep_light,
    // duration longer than 100ms
    sync_skip_light

};
typedef struct {
    // Variables for lighthouse RX, store OPTICAL_DATA_RAW pin state
    unsigned short current_gpio, last_gpio, state, nextstate, pulse_type;
    unsigned int timestamp_rise, timestamp_fall, pulse_width;
    // variables from lighthouse tracking repo
    uint32_t t_0_start;
    uint32_t t_0_end;
    uint32_t t_opt_pulse;
    uint32_t t_opt_pulse_us;
    uint32_t t_1_start;
    uint32_t t_d_start;
    uint8_t flag_start;
    // sync;sweep;skip_sync
    enum Lighthouse_light_type flag_light_type;
    // after a sweep, wo is the first which is the station A
    uint8_t flag_station;
    // 0：NULL,1:A,2:B
    uint8_t flag_A_station;
    uint32_t loca_duration;
    uint8_t loca_x;
    // angle x output from lighthouse station A
    uint32_t A_X;
    // angle y output from lighthouse station A
    uint32_t A_Y;
    // angle x output from lighthouse station B
    uint32_t B_X;
    // angle y output from lighthouse station B
    uint32_t B_Y;
} ligththouse_protocal_t;

ligththouse_protocal_t lighthouse_ptc = {.current_gpio = 0,
                                         .last_gpio = 0,
                                         .state = 0,
                                         .nextstate = 0,
                                         .pulse_type = 0,
                                         .t_0_start = 0x00,
                                         .t_0_end = 0x00,
                                         .t_opt_pulse = 0x00,
                                         .t_opt_pulse_us = 0x00,
                                         .t_1_start = 0x00,
                                         .t_d_start = 0x00,
                                         .flag_start = 0,
                                         .flag_light_type = sync_light,
                                         .flag_station = 0,
                                         .flag_A_station = 0,
                                         .loca_duration = 0,
                                         .loca_x = 0,
                                         .A_X = 0,
                                         .A_Y = 0,
                                         .B_X = 0,
                                         .B_Y = 0};

// LC calibration need read LC Div Counter at the start of a sync light
// first the register value will be record in rdata_lsb/msb, but we will only
// know the light is a sync light after the light ends. Thus, we will have valid
// values only if current light is a sync light while number=expect number.
typedef struct {
    uint32_t rdata_lsb, rdata_msb;
    uint32_t count_LC, count_32k, count_2M, count_HFclock, count_IF;
    uint32_t first_sync_LC_start, last_sync_LC_start;
    // when we decide discard the LC value, set flag_reset_counter to 1, when we
    // want to save, set flag_save_counter_value to 1
    uint8_t flag_reset_counter, flag_save_counter_value;
} sync_light_cal_Registers_t;

sync_light_cal_Registers_t sync_cal_registers = {
    .rdata_lsb = 0,
    .rdata_msb = 0,
    .count_LC = 0,
    .count_2M = 0,
    .count_32k = 0,
    .count_HFclock = 0,
    .count_IF = 0,
    .first_sync_LC_start = 0,
    .last_sync_LC_start = 0,
    .flag_reset_counter = 0,
    .flag_save_counter_value = 0,
};
// ble variables from ble_tx_ti.c
/************************************************************************ */
// If true, sweep through all fine codes.
#define BLE_TX_SWEEP_FINE true
//  #define BLE_TX_SWEEP_FINE false
#define BLE_CALIBRATE_LC true
//  LC cal gives an optimal value, but we can decide whether use this.
#define BLE_USE_OPTIMAL_LC true
// BLE TX period in milliseconds.
#define BLE_TX_PERIOD_MS 1000  // milliseconds

// BLE TX tuning code.
static tuning_code_t g_ble_tx_tuning_code = {
    .coarse = 19,
    .mid = 20,
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
    uint8_t t;
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
    LC_monotonic(DEFAULT_INIT_LC_CODE);

    // Init divider settings
    radio_init_divider(2000);

    analog_scan_chain_write();
    analog_scan_chain_load();
}

void config_ble_tx_mote(void) {
    uint8_t t;

    scm3c_hw_interface_init();
    optical_init();
    // a copy of optical_init
    sync_light_calibrate_init();
    radio_init();
    rftimer_init();
    ble_init();

    //--------------------------------------------------------
    // SCM3C Analog Scan Chain Initialization
    //--------------------------------------------------------
    // Init LDO control
    init_ldo_control();

    // Set LDO reference voltages
    // set_VDDD_LDO_voltage(0);
    // set_AUX_LDO_voltage(0);
    // set_ALWAYSON_LDO_voltage(0);

    // Select banks for GPIO inputs
    GPI_control(0, 0, 1, 0);  // 1 in 3rd arg connects GPI8 to EXT_INTERRUPT<1>
                              // needed for 3WB cal

    // Select banks for GPIO outputs
    GPO_control(6, 6, 0,
                6);  // 0 in 3rd arg connects clk_3wb to GPO8 for 3WB cal

    // Set all GPIOs as outputs
    // Set GPI enables
    // Hex entry 2: 0x1 = 1 = 0b0001 = GPI 8 on for 3WB cal clk interrupt
    GPI_enables(0x0100);

    // Set HCLK source as HF_CLOCK
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
    LC_monotonic(DEFAULT_INIT_LC_CODE);

    // Init divider settings
    radio_init_divider(2000);

    // Program analog scan chain
    analog_scan_chain_write();
    analog_scan_chain_load();
    //--------------------------------------------------------
};
// a method used to replace old xy distinguish method from kilberg code.
#define WIDTH_BIAS (-100)
void distinguish_xy(uint32_t light_duration) {
    // Identify what kind of pulse this was

    // if (light_duration < 585 + WIDTH_BIAS && light_duration > 100 +
    // WIDTH_BIAS)
    //     loca_x = LASER;  // Laser sweep (THIS NEEDS TUNING)
    if (light_duration < 675 + WIDTH_BIAS &&
        light_duration > 500 + WIDTH_BIAS) {
        lighthouse_ptc.loca_x = 1;  // Azimuth sync, data=0, skip = 0
        // sync_cal.count_sync_light += 1;
    } else if (light_duration >= 675 + WIDTH_BIAS &&
               light_duration < 781 + WIDTH_BIAS) {
        lighthouse_ptc.loca_x = 0;  // Elevation sync, data=0, skip = 0
        // sync_cal.count_sync_light += 1;
    } else if (light_duration >= 781 + WIDTH_BIAS &&
               light_duration < 885 + WIDTH_BIAS) {
        lighthouse_ptc.loca_x = 1;  // Azimuth sync, data=1, skip = 0
        // sync_cal.count_sync_light += 1;
    } else if (light_duration >= 885 + WIDTH_BIAS &&
               light_duration < 989 + WIDTH_BIAS) {
        lighthouse_ptc.loca_x = 0;  // Elevation sync, data=1, skip = 0
        // sync_cal.count_sync_light += 1;
    } else if (light_duration >= 989 + WIDTH_BIAS &&
               light_duration < 1083 + WIDTH_BIAS)
        lighthouse_ptc.loca_x = 1;  // Azimuth sync, data=0, skip = 1
    else if (light_duration >= 1083 + WIDTH_BIAS &&
             light_duration < 1200 + WIDTH_BIAS)
        lighthouse_ptc.loca_x = 0;  // elevation sync, data=0, skip = 1
    else if (light_duration >= 1200 + WIDTH_BIAS &&
             light_duration < 1300 + WIDTH_BIAS)
        lighthouse_ptc.loca_x = 1;  // Azimuth sync, data=1, skip = 1
    else if (light_duration >= 1300 + WIDTH_BIAS &&
             light_duration < 1400 + WIDTH_BIAS)
        lighthouse_ptc.loca_x = 0;  // Elevation sync, data=1, skip = 1
}
// some debug vars, canbe deleted later
// divide timer then multiply it to make codecompatible
// int para_temp = 1;
// int tmp_count_sweep = 0;
// int tmp_count_sync = 0;
// int tmp_count_skip = 0;
// uint32_t tmp_sync_width = 0;

void decode_lighthouse(void) {
    // This is the main function of lighthouse protocol decoding
    // lighthouse code start
    lighthouse_ptc.last_gpio = lighthouse_ptc.current_gpio;
    lighthouse_ptc.current_gpio = OPTICAL_DATA_RAW_PIN;

    // Detect rising edge
    if (lighthouse_ptc.last_gpio == 0 && lighthouse_ptc.current_gpio == 1) {
        // Reset RF Timer count register at rising edge of first sync pulse

        // Save when this event happened
        lighthouse_ptc.timestamp_rise = RFTIMER_REG__COUNTER;

        // read counters, but first we have to know whether reset the counters
        // to zero or not by sync_cal_registers.flag_reset_counter.now we want
        // to use sync_cal.clockSyncPulseCount replaced
        if (sync_cal.in_valid_counting_period &&
            sync_cal.count_sync_light == 0) {
            // Reset all counters
            ANALOG_CFG_REG__0 = 0x0000;
            // Enable all counters
            ANALOG_CFG_REG__0 = 0x3FFF;
            gpio_8_toggle();  // debug,remove later
        } else {
            // Enable all counters
            ANALOG_CFG_REG__0 = 0x3FFF;
        }
        // Disable all counters
        // ANALOG_CFG_REG__0 = 0x007F;

        // Read LC_div counter (via counter4),this is the accurate moment of a
        // light start.
        sync_cal_registers.rdata_lsb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x280000);
        sync_cal_registers.rdata_msb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x2C0000);
        sync_cal_registers.count_LC =
            sync_cal_registers.rdata_lsb + (sync_cal_registers.rdata_msb << 16);

        // Read HF_CLOCK counter
        sync_cal_registers.rdata_lsb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x100000);
        sync_cal_registers.rdata_msb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x140000);
        sync_cal_registers.count_HFclock =
            sync_cal_registers.rdata_lsb + (sync_cal_registers.rdata_msb << 16);

        // Read 2M counter
        sync_cal_registers.rdata_lsb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x180000);
        sync_cal_registers.rdata_msb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x1C0000);
        sync_cal_registers.count_2M =
            sync_cal_registers.rdata_lsb + (sync_cal_registers.rdata_msb << 16);

        // Read 32k counter
        sync_cal_registers.rdata_lsb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x000000);
        sync_cal_registers.rdata_msb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x040000);
        sync_cal_registers.count_32k =
            sync_cal_registers.rdata_lsb + (sync_cal_registers.rdata_msb << 16);

        // Read IF ADC_CLK counter
        sync_cal_registers.rdata_lsb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x300000);
        sync_cal_registers.rdata_msb =
            *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x340000);
        sync_cal_registers.count_IF =
            sync_cal_registers.rdata_lsb + (sync_cal_registers.rdata_msb << 16);

        // only for debugging,should toggle before uart send , uart latency is
        // large
        gpio_10_toggle();
        // printf("LC div: %u\n", sync_cal_registers.count_LC);
        // printf("2m: %u, lc: %u, 32k: %u, Hf: %u\r\n",
        //        sync_cal_registers.count_2M, sync_cal_registers.count_LC,
        //        sync_cal_registers.count_32k,
        //        sync_cal_registers.count_HFclock);
        // set the CFG_REG to config counters
        // // Reset all counters
        // ANALOG_CFG_REG__0 = 0x0000;
        // // Enable all counters
        // ANALOG_CFG_REG__0 = 0x3FFF;

        // //      only for debugging
        // gpio_10_toggle();
        switch (lighthouse_ptc.flag_start) {
            case 0:
                lighthouse_ptc.t_0_start = lighthouse_ptc.timestamp_rise;

                lighthouse_ptc.flag_start = 1;
                break;
            case 1:
                break;
            default:
                break;
        }

    }

    // Detect falling edge
    else if (lighthouse_ptc.last_gpio == 1 &&
             lighthouse_ptc.current_gpio == 0) {
        // Save when this event happened
        lighthouse_ptc.timestamp_fall = RFTIMER_REG__COUNTER;

        // Calculate how wide this pulse was
        lighthouse_ptc.pulse_width =
            lighthouse_ptc.timestamp_fall - lighthouse_ptc.timestamp_rise;
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

        switch (lighthouse_ptc.flag_start) {
            case 0:
                break;
            case 1:
                lighthouse_ptc.t_0_end = lighthouse_ptc.timestamp_fall;
                lighthouse_ptc.flag_start = 0;
                lighthouse_ptc.t_opt_pulse =
                    lighthouse_ptc.t_0_end - lighthouse_ptc.t_0_start;
                // Dividing the two signals by 50us: 0.000,050/(1/10M) = 500
                // = 0x320,99us(990ticks) for skip/sync
                // usually soft read time is 10+us shorter than hw read time
                // so I set the boundary condition to 320ticks(). Ofcourse,
                // it is not accurate, we need to adjust it  300-500ticks
                // to get the best result. Remember to change it when temperature
                // changes.
                (lighthouse_ptc.t_opt_pulse <
                 320)  // actual boundary condition maybe a little different
                    ? (lighthouse_ptc.flag_light_type = sweep_light)
                    : ((lighthouse_ptc.t_opt_pulse < 990)
                           ? (lighthouse_ptc.flag_light_type = sync_light)
                           : (lighthouse_ptc.flag_light_type =
                                  sync_skip_light));
                lighthouse_ptc.t_opt_pulse_us = lighthouse_ptc.t_opt_pulse / 10;
                switch (lighthouse_ptc.flag_light_type) {
                    // More than 50us then it must be sync, then the next
                    // falling edge interrupt will need to calculate
                    // position
                    case sync_light:
                        // If sync, distance measurement starts from this
                        // time.
                        lighthouse_ptc.t_d_start = lighthouse_ptc.t_0_start;
                        // It is only based on sweep that you can determine
                        // whether you are currently in A or B.
                        if (lighthouse_ptc.flag_station >= 1) {
                            //  Where an even ten digit number is the
                            //  X-axis, an odd number is the Y-axis.
                            // offcourse ,but scum not very accurate, I should
                            // replace it with another medthod to distinguish XY
                            // ((t_opt_pulse_us / 10) % 2 == 0) ? (loca_x = 1)
                            //                                  : (loca_x = 0);

                            // update loca_x
                            distinguish_xy(lighthouse_ptc.t_opt_pulse);
                        }
                        // Determine whether this is an A or B base station
                        // based on the value of flag_station
                        switch (lighthouse_ptc.flag_station) {
                            case 0:
                                break;
                            case 1:
                                lighthouse_ptc.flag_A_station = 1;
                                break;
                            case 2:
                                lighthouse_ptc.flag_A_station = 2;
                                break;
                            default:
                                break;
                        }
                        // after a sync, the count_sync_light will increase 1,
                        // so when the count_sync_light = 1, it means this is
                        // the first sync light. record start time.
                        // if (sync_cal.count_sync_light == 1) {
                        //     sync_cal.several_synclights_start =
                        //         lighthouse_ptc.t_0_start;
                        //     // also need record first synclight LC counter
                        //     value sync_cal_registers.first_sync_LC_start =
                        //         sync_cal_registers.count_LC;
                        //     // start record 10 sync, donot reset the counter
                        //     sync_cal_registers.flag_reset_counter = 0;
                        // }

                        // 如果不在有效计数周期内且检测到同步光，开始新的计数周期

                        if (sync_cal.in_valid_counting_period) {
                            sync_cal.count_sync_light += 1;

                            // when the count turn to 12, it is a sync
                            // calibration period.
                            if ((sync_cal.count_sync_light == 13) &&
                                (sync_cal.need_sync_calibration == 1)) {
                                // 结束当前计数周期
                                sync_cal.in_valid_counting_period = false;
                                // once the count_sync_light == 0,will reset
                                // clock counters in rising edge
                                sync_cal.count_sync_light = 0;

                                // gpio_8_toggle();  // debug,remove later

                                // record the number of sync light calibration
                                sync_cal.count_calibration += 1;

                                if (!synclight_cal_vars
                                         .optical_LC_cal_finished) {
                                    sync_light_calibrate_all_clocks(
                                        sync_cal_registers.count_HFclock,
                                        sync_cal_registers.count_2M,
                                        sync_cal_registers.count_IF,
                                        sync_cal_registers.count_LC);
                                } else {
                                    printf(
                                        "2m: %u, lc: %u, 32k: %u, Hf: %u\r\n",
                                        sync_cal_registers.count_2M,
                                        sync_cal_registers.count_LC,
                                        sync_cal_registers.count_32k,
                                        sync_cal_registers.count_HFclock);
                                }
                            }
                        } else {
                            sync_cal.in_valid_counting_period = true;
                            sync_cal.count_sync_light = 0;  // 确保从0开始计数
                            // 下一个上升沿会重置计数器
                        }
                        break;
                    case sweep_light:
                        // gpio_8_toggle();  // debug,remove later
                        //  0 ：NULL,1: next is A,2:next is B
                        lighthouse_ptc.flag_station = 1;

                        lighthouse_ptc.loca_duration =
                            lighthouse_ptc.t_0_start - lighthouse_ptc.t_d_start;
                        switch (lighthouse_ptc.flag_A_station) {
                            case 0:
                                break;
                            // A
                            case 1:
                                if (lighthouse_ptc.loca_x == 1) {
                                    lighthouse_ptc.A_X =
                                        lighthouse_ptc.loca_duration;
                                } else {
                                    lighthouse_ptc.A_Y =
                                        lighthouse_ptc.loca_duration;
                                }
                                lighthouse_ptc.flag_A_station = 0;
                                break;
                            // B
                            case 2:
                                if (lighthouse_ptc.loca_x == 1) {
                                    lighthouse_ptc.B_X =
                                        lighthouse_ptc.loca_duration;
                                } else {
                                    lighthouse_ptc.B_Y =
                                        lighthouse_ptc.loca_duration;
                                }
                                lighthouse_ptc.flag_A_station = 0;
                                break;
                            default:
                                break;
                        }

                        break;
                    case sync_skip_light:
                        if (lighthouse_ptc.flag_station >= 1) {
                            lighthouse_ptc.flag_station++;
                            // Exceeding 2 means that a sweep was not seen,
                            // which often happens.
                            if (lighthouse_ptc.flag_station >= 3) {
                                lighthouse_ptc.flag_station--;
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
    // if (current_state == DEFAULT) {
    //     scum_state = SENDING;
    // }
    scum_state = OPTICAL_CALIBRATING;
}

// a counter to record how many lighthouse decoding process passed.
// uint8_t sync_cal_counter_localization;
// uint8_t sync_cal_lighthouse_state_period = 20000;

static inline void state_optical_collecting(void) {
    printf("State: Locating SCUM.\n");
    // in this state, we only need location information, so disable
    // calibration part.
    sync_cal.need_sync_calibration = 0;

    config_lighthouse_mote();
    // disable all interrupts. rftimer interrupt is used in ble
    // transmitting
    ICER = 0xFFFF;
    // close to 2s for changing  state between localization and
    // calibration
    sync_cal.counter_localization = sync_cal.counter_lighthouse_state_period;
    // printf counter
    sync_cal.counter_global_timer = 0;

    while (sync_cal.counter_localization) {
        decode_lighthouse();
        // same function as 2d_stable branch
        sync_cal.counter_global_timer++;
        if (sync_cal.counter_global_timer == 10000) {
            sync_cal.counter_global_timer = 0;
            gpio_8_toggle();
            // sync_light_calibrate_isr();
            printf("A_X: %u, A_Y: %u, B_X: %u, B_Y: %u\n", lighthouse_ptc.A_X,
                   lighthouse_ptc.A_Y, lighthouse_ptc.B_X, lighthouse_ptc.B_Y);
        }
        sync_cal.counter_localization--;
        // when comes to the period transfer statement, we use a
        // counter to change state
        if (sync_cal.counter_localization == 0) {
            switch (sync_cal.need_sync_calibration) {
                case 0:
                    // sync_cal.need_sync_calibration = 1;
                    sync_cal.counter_localization =
                        sync_cal.counter_lighthouse_state_period + 30000;
                    break;
                case 1:
                    // sync_cal.need_sync_calibration = 0;
                    sync_cal.counter_localization =
                        sync_cal.counter_lighthouse_state_period;
                    break;
                default:
                    break;
            }
        }
    }
    //      wait some time then print to uart
    sync_cal.counter_global_timer++;
    // i=10,000~=100ms(85ms), a brief printf timer.
    if (sync_cal.counter_global_timer == 11760) {
        sync_cal.counter_global_timer = 0;
        // gpio_8_toggle();
        // sync_light_calibrate_isr();
        // printf("13 syc: %u, total num: %u\n",
        // servel_synclights_duration,
        //        count_calibration);
        // printf("syc: %u\n", tmp_sync_width);

        //            printf("opt_pulse: %u, interval: %u\n",
        //            t_opt_pulse,loca_duration);

        // printf("hfclk: %u\n", hf_count_HFclock);

        // printf("A_X: %u, A_Y: %u, B_X: %u, B_Y: %u\n", A_X, A_Y, B_X, B_Y);
    }

    //        printf("Hello World! %d\n", app_vars.count);
    // app_vars.count += 1;
}

static inline void synclight_cal_enable_LC_calibration(void) {
    synclight_cal_vars.cal_LC_coarse = LC_CAL_COARSE_MIN;
    synclight_cal_vars.cal_LC_mid = LC_CAL_MID_MIN;
    synclight_cal_vars.cal_LC_fine = LC_CAL_FINE_MIN;
    synclight_cal_vars.optical_LC_cal_enable = true;
    synclight_cal_vars.optical_LC_cal_finished = false;
    printf(
        "Debug: LC cal enabled, coarse=%d, mid=%d, fine=%d, enable=%d, "
        "finished=%d\r\n",
        synclight_cal_vars.cal_LC_coarse, synclight_cal_vars.cal_LC_mid,
        synclight_cal_vars.cal_LC_fine,
        synclight_cal_vars.optical_LC_cal_enable,
        synclight_cal_vars.optical_LC_cal_finished);
    LC_FREQCHANGE(synclight_cal_vars.cal_LC_coarse,
                  synclight_cal_vars.cal_LC_mid,
                  synclight_cal_vars.cal_LC_fine);
}

static inline void state_opt_calibrating(void) {
    // now I use optical cal for debugging
    // gpio_ext8_state = OPTICAL_ISR;
    gpio_ext8_state = SYNC_LIGHT_ISR;
    // optical_enableLCCalibration();

    config_lighthouse_mote();
    // use inline function, this time should work. remember this should after
    // config_lighthouse_mote.
    synclight_cal_enable_LC_calibration();

    // For TX, LC target freq = 2.402G - 0.25M = 2.40175 GHz.Then divide by
    // 960 to get the target LC value.But here we use 250370 for sync light
    // calibration.
    synclight_cal_setLCTarget(250370);

    // this line only used for enable LC
    radio_txEnable();
    //  control delay time.
    int8_t ticks = 1000;
    while (1) {
        // a little delay for debug, can be removed.
        while (ticks) {
            ticks--;
        };
        // keep read lighthouse to get light.
        decode_lighthouse();
    }
}

// I guess it does not need init each sending state
bool sync_cal_ble_init_enable = true;
// how many times to transmite ble packets in single SENDING state
uint8_t counter_ble_tx;

static inline void state_sending(void) {
    printf("State: BLE transimitting.\n");
    // enable/disable synclight calibration
    sync_cal.need_sync_calibration = 1;
    // now I use optical cal for debugging
    // gpio_ext8_state = OPTICAL_ISR;
    gpio_ext8_state = SYNC_LIGHT_ISR;

    // set this value to control how many times to transmitting
    counter_ble_tx = 10;

    // if (sync_cal_ble_init_enable == true) {
    //     // initialize_mote();
    //     config_ble_tx_mote();

    //     // Initialize BLE TX.
    //     printf("Initializing BLE TX.\n");
    //     ble_init();
    //     ble_init_tx();

    //     // Configure the RF timer.
    //     rftimer_set_callback_by_id(ble_tx_rftimer_callback, 7);
    //     rftimer_enable_interrupts();
    //     rftimer_enable_interrupts_by_id(7);

    //     analog_scan_chain_write();
    //     analog_scan_chain_load();

    //     crc_check();
    //     // perform_calibration();
    // }

#if BLE_CALIBRATE_LC
    // optical_vars.optical_cal_finished = false;
    // optical_enableLCCalibration();

    // does not worked,why?
    synclight_cal_enableLCCalibration();
    // this executed in radio_txEnable(), from perform_calibration(), I open it
    // here.but I believe this included in 0x78 since 0110 and 0111
    // ANALOG_CFG_REG__10 = 0x68;
    // Turn on LO, DIV, PA, and IF
    // move it below
    // ANALOG_CFG_REG__10 = 0x78;

    // Turn off polyphase and disable mixer
    ANALOG_CFG_REG__16 = 0x6;

    // For TX, LC target freq = 2.402G - 0.25M = 2.40175 GHz.
    // optical_setLCTarget(250182);
    synclight_cal_setLCTarget(250182);
#endif
    printf("Config lighthouse mode\r\n");
    config_lighthouse_mote();
    // Turn on LO, DIV, PA, and IF
    ANALOG_CFG_REG__10 = 0x78;
    analog_scan_chain_write();
    analog_scan_chain_load();
    // Do not use perfom_calibration() here, it has radio_rxEnable() function,
    // which will affect accuracy.

    //  Enable optical SFD interrupt for optical calibration
    optical_enable();

    printf("Start synclight calibrating\r\n");
    ICER = 0xFFFF;
    // Wait for optical cal to finish
    // while (!optical_getCalibrationFinished());

    // use inline function, this time should work.
    synclight_cal_enable_LC_calibration();

    printf("LC_CAL_ON GOING. enbable:%u,fininshed:%u \r\n",
           synclight_cal_vars.optical_LC_cal_enable,
           synclight_cal_vars.optical_LC_cal_finished);
    while (!synclight_cal_getCalibrationFinished()) {
        decode_lighthouse();
    };

    printf("Cal complete\r\n");
    if (sync_cal_ble_init_enable == true) {
        // initialize_mote();
        config_ble_tx_mote();

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
        // perform_calibration();
    }

    // Disable static divider to save power
    divProgram(480, 0, 0);

    // Configure coarse, mid, and fine codes for TX.
#if BLE_CALIBRATE_LC

    g_ble_tx_tuning_code.coarse = synclight_cal_getLCCoarse();
    g_ble_tx_tuning_code.mid = synclight_cal_getLCMid();
    g_ble_tx_tuning_code.fine = synclight_cal_getLCFine();
#else
    // CHANGE THESE VALUES AFTER LC CALIBRATION.
    app_vars.tx_coarse = 23;
    app_vars.tx_mid = 11;
    app_vars.tx_fine = 23;
#endif

    // Generate a BLE packet.
    ble_generate_packet();

    while (true) {  // counter_ble_tx
        if (g_ble_tx_trigger) {
            printf("Triggering BLE TX.\n");
            ble_tx_trigger();
            g_ble_tx_trigger = false;
            delay_milliseconds_asynchronous(BLE_TX_PERIOD_MS, 7);
        }
        // counter_ble_tx--;
    }
}
//=========================== main ============================================

int main(void) {
    memset(&app_vars, 0, sizeof(app_vars_t));

    printf("Initializing...");

    initialize_mote();
    // config_ble_tx_mote();

    crc_check();
    perform_calibration();
    printf("~~~~my code start~~~~~%d\n", app_vars.count);

    // config_lighthouse_mote();

    // the scum tx and rx are different hardware, why need open this? I forgot.
    // we only need tx part.
    // radio_rxEnable();  // openLC,IF?

    // clean optical and ex3 interrupt, then re-open ext_3 interrupt
    gpio_ext8_state = SYNC_LIGHT_ISR;
    // enbale perform calibration in decode_lighthouse() by set to 1
    sync_cal.need_sync_calibration = 0;
    printf("sync_cal.need_sync_calibration:%d\n ",
           sync_cal.need_sync_calibration);

    // disable all interrupts
    ICER = 0xFFFF;

    //  test rftimer
    // ISER = 0x0080;

    printf("~~~~start to say HELLO?~~~~~%d\n", app_vars.count);

    // delay_milliseconds_asynchronous(1000, 7);//test RFtimer freq,if rftimer
    // is 500KHz, interrupt per 1s(1000ms)
    //  here is the timecounter to control print velocity
    sync_cal.counter_global_timer = 0;
    sync_cal.counter_localization = sync_cal.counter_lighthouse_state_period;
    scum_state = DEFAULT;
    while (1) {
        update_state(scum_state);
        switch (scum_state) {
            case COLLECTING:
                printf("State: Lighthouse Locating.\n");
                // disable synclight calibration
                sync_cal.need_sync_calibration = 0;
                state_optical_collecting();
                break;

            case OPTICAL_CALIBRATING:
                // Disable all counters
                ANALOG_CFG_REG__0 = 0x007F;
                // Reset all counters
                ANALOG_CFG_REG__0 = 0x0000;

                // Enable all counters
                ANALOG_CFG_REG__0 = 0x3FFF;
                printf("State: OPTICAL Calibrating.\n");
                // to get sync light for calibration must need decode
                // lighthouse, but can we make this function to two separate
                // parts? Is that essential?
                sync_cal.need_sync_calibration = 1;
                state_opt_calibrating();

                break;

            case RAW_CALIBRATING:
                printf("State: Raw Calibrating.\n");
                // Of course there will be some time I want to use old
                // calibration, not sync light calibration
                perform_calibration();
                break;

            case INTEGRATING:
                printf("State: Integrating ble packet.\n");
                // a small gap used to generate the ble packet. for this
                // fuction, I think it does not need to be an individual state,
                // but in this way will be clearly
                sync_cal.need_sync_calibration = 0;
                ble_init();
                ble_generate_location_packet();
                break;
            case SENDING:
                printf("State: BLE transimitting.\n");
                state_sending();
                break;
            case DEFAULT:
                printf("State: Scum is running.\n");
                for (sync_cal.counter_global_timer = 0;
                     sync_cal.counter_global_timer < 1000000;
                     sync_cal.counter_global_timer++);
                break;

            default:
                printf("Unknown state\n");
                break;
        }
    }
}

//=========================== public ==========================================

//=========================== private =========================================
