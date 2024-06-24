#include <string.h>

#include "calibrate_interrupt.h"
#include "gpio.h"
#include "lighthouse_position.h"
#include "memory_map.h"
#include "optical.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"

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

//=========================== variables ===========================

typedef struct {
    uint8_t count;
} app_vars_t;

app_vars_t app_vars;

extern int8_t need_optical;

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

// Read HF_CLK counter,from optical_sfd_isr() at optical.c
#define HF_CLK_RDATA_LSB *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x100000)
#define HF_CLK_RDATA_MSB *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x140000)
#define HF_CLK_COUNT (HF_CLK_RDATA_LSB + (HF_CLK_RDATA_MSB << 16))

//=========================== prototypes ======================================
void config_lighthouse_mote(void) {
    //  I think RF timer needs to be reset before use, but not essential.
    // RF Timer rolls over at this value and starts a new cycle
    RFTIMER_REG__MAX_COUNT = 0xFFFFFFFF;
    // Enable RF Timer
    RFTIMER_REG__CONTROL = 0x7;

    // Select banks for GPIO inputs
    GPI_control(0, 0, 0, 0);
    // Select banks for GPIO outputs, now IO 8,10 is used for test(XX6X)
    GPO_control(0, 0, 6, 0);
    // Set all GPIOs as outputs
    GPI_enables(0x000F);  // 0008=io3?
    GPO_enables(0xFFFF);

    // Set HCLK source as HF_CLOCK
    set_asc_bit(1147);

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

    analog_scan_chain_write();
    analog_scan_chain_load();
}
// a method used to replace old xy distinguish method from kilberg code.
#define WIDTH_BIAS (0 - 0)
void distinguish_xy(uint32_t light_duration) {
    // Identify what kind of pulse this was

    // if (light_duration < 585 + WIDTH_BIAS && light_duration > 100 +
    // WIDTH_BIAS)
    //     loca_x = LASER;  // Laser sweep (THIS NEEDS TUNING)
    if (light_duration < 675 + WIDTH_BIAS && light_duration > 500 + WIDTH_BIAS)
        loca_x = 1;  // Azimuth sync, data=0, skip = 0
    else if (light_duration >= 675 + WIDTH_BIAS &&
             light_duration < 781 + WIDTH_BIAS)
        loca_x = 0;  // Elevation sync, data=0, skip = 0
    else if (light_duration >= 781 + WIDTH_BIAS &&
             light_duration < 885 + WIDTH_BIAS)
        loca_x = 1;  // Azimuth sync, data=1, skip = 0
    else if (light_duration >= 885 + WIDTH_BIAS &&
             light_duration < 989 + WIDTH_BIAS)
        loca_x = 0;  // Elevation sync, data=1, skip = 0
    else if (light_duration >= 989 + WIDTH_BIAS &&
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
                        break;
                    case type_sweep:
                        gpio_8_toggle();  // debug,remove later
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
//=========================== main ============================================

int main(void) {
    uint32_t i;
    uint32_t hf_rdata_lsb, hf_rdata_msb, hf_count_HFclock;  // test HF_CLK

    memset(&app_vars, 0, sizeof(app_vars_t));

    printf("Initializing...");
    initialize_mote();
    crc_check();
    perform_calibration();
    printf("~~~~my code start~~~~~%d\n", app_vars.count);

    config_lighthouse_mote();

    // clean optical and ex3 interrupt, then re-open ext_3 interrupt
    need_optical = 0;

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
    while (1) {
        decode_lighthouse();
        //      wait some time then print to uart
        i++;
        if (i == 100000) {
            i = 0;
            // printf("syc: %u\n", tmp_sync_width);

            //            printf("opt_pulse: %u, interval: %u\n",
            //            t_opt_pulse,loca_duration);

            printf("hfclk: %u\n", hf_count_HFclock);

            // printf("A_X: %u, A_Y: %u, B_X: %u, B_Y: %u\n", A_X, A_Y, B_X,
            // B_Y);
        }

        //        printf("Hello World! %d\n", app_vars.count);
        //        app_vars.count += 1;

        // for (i = 0; i < 1000000; i++);
    }
}

//=========================== public ==========================================

//=========================== private =========================================
