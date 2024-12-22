#include "calibrate_interrupt.h"

#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "memory_map.h"
#include "optical.h"
#include "scm3c_hw_interface.h"
#include "scum_defs.h"

//=========================== defines =========================================

// #define LC_CAL_COARSE_MIN 19
// #define LC_CAL_COARSE_MAX 25
// #define LC_CAL_MID_MIN 0
// #define LC_CAL_MID_MAX 31
// #define LC_CAL_FINE_MIN 15
// #define LC_CAL_FINE_MAX 15
// #define MIN_LC_DIFF 100

//=========================== variables =======================================

// typedef struct {
//     uint8_t optical_cal_iteration;
//     uint8_t optical_cal_finished;

//     uint32_t num_32k_ticks_in_100ms;
//     uint32_t num_2MRC_ticks_in_100ms;
//     uint32_t num_IFclk_ticks_in_100ms;
//     uint32_t num_LC_ch11_ticks_in_100ms;
//     uint32_t num_HFclock_ticks_in_100ms;

//     // reference to calibrate
//     uint32_t LC_target;
//     uint32_t LC_code;
// } synclight_calibrate_vars_t;

synclight_calibrate_vars_t synclight_cal_vars;

extern int8_t need_optical;
//=========================== prototypes ======================================
void calibration_isr() { printf("External Interrupt GPIO9 triggered\r\n"); }

//=========================== main ============================================

//=========================== public ==========================================
void gpio_ext_3_interrupt_enable(void) { ISER |= 0x0002; }

void gpio_ext_3_interrupt_disable(void) { ICER |= 0x0002; }

void gpio_ext_9_interrupt_enable(void) { ISER |= 0x2000; }

void gpio_ext_9_interrupt_disable(void) { ICER |= 0x2000; }

void gpio_ext_10_interrupt_enable(void) { ISER |= 0x4000; }

void gpio_ext_10_interrupt_disable(void) { ICER |= 0x4000; }

void sync_light_calibrate_init(void) {
    memset(&synclight_cal_vars, 0, sizeof(synclight_calibrate_vars_t));

    // Target radio LO freq = 2.4025G
    // Divide ratio is currently 480*2
    // Calibration counts for 100ms
    synclight_cal_vars.LC_target = REFERENCE_LC_TARGET;
    synclight_cal_vars.LC_code = DEFAULT_INIT_LC_CODE;

    synclight_cal_vars.LC_coarse = DEFAULT_INIT_LC_COARSE;
    synclight_cal_vars.LC_mid = DEFAULT_INIT_LC_MID;
    synclight_cal_vars.LC_fine = DEFAULT_INIT_LC_FINE;
}

void synclight_cal_enableLCCalibration(void) {
    synclight_cal_vars.optical_LC_cal_enable = true;
    synclight_cal_vars.optical_LC_cal_finished = false;

    synclight_cal_vars.cal_LC_coarse = LC_CAL_COARSE_MIN;
    synclight_cal_vars.cal_LC_mid = LC_CAL_MID_MIN;
    synclight_cal_vars.cal_LC_fine = LC_CAL_FINE_MIN;
    synclight_cal_vars.cal_LC_diff = 0xFFFFFFFF;

    LC_FREQCHANGE(synclight_cal_vars.cal_LC_coarse,
                  synclight_cal_vars.cal_LC_mid,
                  synclight_cal_vars.cal_LC_fine);
}

bool synclight_cal_getCalibrationFinished(void) {
    return synclight_cal_vars.optical_cal_finished;
}

void synclight_cal_setLCTarget(uint32_t LC_target) {
    synclight_cal_vars.LC_target = LC_target;
}

uint8_t synclight_cal_getLCCoarse(void) {
    return synclight_cal_vars.LC_coarse & 0x1F;
}

uint8_t synclight_cal_getLCMid(void) {
    return synclight_cal_vars.LC_mid & 0x1F;
}

uint8_t synclight_cal_getLCFine(void) {
    return synclight_cal_vars.LC_fine & 0x1F;
}

void synclight_cal_enable(void) {
    ISER = 0x1800;  // 1 is for enabling GPIO8 ext interrupt (3WB cal) and 8 is
                    // for enabling optical interrupt
}

// after get 10 sync lights, call this function to calibrate clocks.
// this function is not similar to old perform_calibration(),but
// BLE_CALIBRATE_LC
void perform_synclight_calibration(void) {
    // For the LO, calibration for RX channel 11, so turn on AUX, IF, and LO
    // LDOs by calling radio rxEnable
    // radio_rxEnable();

    // Enable optical SFD interrupt for optical calibration:ISER = 0x1800
    // optical_enable();

    // Wait for optical cal to finish
    // while (synclight_cal_getCalibrationFinished() == 0);

    // Disable the radio now that it is calibrated
    // radio_rfOff();

    sync_light_calibrate_isr();
}

void sync_light_calibrate_isr(void) {
    //	gpio_10_toggle();

    // This interrupt goes off when the optical register holds the value {221,
    // 176,
    // 231, 47} This interrupt can also be used to synchronize to the start of
    // an optical data transfer Need to make sure a new bit has been clocked in
    // prior to returning from this ISR, or else it will immediately execute
    // again
    // 1.1V/VDDD tap fix
    // helps reorder assembly code
    // Not completely sure why this works
    //    uint32_t dummy = 0;

    //    int32_t t;
    uint32_t rdata_lsb, rdata_msb;
    int32_t count_LC;
    uint32_t count_32k, count_2M, count_HFclock, count_IF;
    // a new LC_diff to replace the struct variable one
    // (synclight_cal_vars.cal_LC_diff).
    int32_t real_LC_diff;
    uint32_t HF_CLOCK_fine;
    uint32_t HF_CLOCK_coarse;
    uint32_t RC2M_coarse;
    uint32_t RC2M_fine;
    uint32_t RC2M_superfine;
    //    uint32_t IF_clk_target;
    uint32_t IF_coarse;
    uint32_t IF_fine;

    int32_t tmp_countLC, tmp_LC_target;

    HF_CLOCK_fine = scm3c_hw_interface_get_HF_CLOCK_fine();
    HF_CLOCK_coarse = scm3c_hw_interface_get_HF_CLOCK_coarse();
    RC2M_coarse = scm3c_hw_interface_get_RC2M_coarse();
    RC2M_fine = scm3c_hw_interface_get_RC2M_fine();
    RC2M_superfine = scm3c_hw_interface_get_RC2M_superfine();
    //    IF_clk_target = scm3c_hw_interface_get_IF_clk_target();
    IF_coarse = scm3c_hw_interface_get_IF_coarse();
    IF_fine = scm3c_hw_interface_get_IF_fine();

    // Disable all counters
    ANALOG_CFG_REG__0 = 0x007F;

    // Keep track of how many calibration iterations have been completed
    synclight_cal_vars.optical_cal_iteration++;

    // Read 32k counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x000000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x040000);
    count_32k = rdata_lsb + (rdata_msb << 16);

    // Read HF_CLOCK counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x100000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x140000);
    count_HFclock = rdata_lsb + (rdata_msb << 16);

    // Read 2M counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x180000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x1C0000);
    count_2M = rdata_lsb + (rdata_msb << 16);

    // Read LC_div counter (via counter4)
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x280000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x2C0000);
    count_LC = rdata_lsb + (rdata_msb << 16);

    // Read IF ADC_CLK counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x300000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x340000);
    count_IF = rdata_lsb + (rdata_msb << 16);

    // Reset all counters
    ANALOG_CFG_REG__0 = 0x0000;

    // Enable all counters
    ANALOG_CFG_REG__0 = 0x3FFF;

    // Don't make updates on the first two executions of this ISR
    if (synclight_cal_vars.optical_cal_iteration > 2) {
        // Do correction on HF CLOCK
        // Fine DAC step size is about 6000 counts
        if (count_HFclock < 1997000) {  // 1997000 original value
            if (HF_CLOCK_fine == 0) {
                HF_CLOCK_coarse--;
                HF_CLOCK_fine = 10;
                synclight_cal_vars.optical_cal_iteration = 3;
            } else {
                HF_CLOCK_fine--;
            }
        }
        if (count_HFclock > 2003000) {  // new value I picked was
                                        // 2010000, originally 2003000
            if (HF_CLOCK_fine == 31) {
                HF_CLOCK_coarse++;
                HF_CLOCK_fine = 20;
                synclight_cal_vars.optical_cal_iteration = 3;
            } else {
                HF_CLOCK_fine++;
            }
        }

        set_sys_clk_secondary_freq(HF_CLOCK_coarse, HF_CLOCK_fine);
        scm3c_hw_interface_set_HF_CLOCK_coarse(HF_CLOCK_coarse);
        scm3c_hw_interface_set_HF_CLOCK_fine(HF_CLOCK_fine);

        // Do correction on LC
        // debugging, why diff cannot be calculated?
        tmp_countLC = count_LC;
        tmp_LC_target = synclight_cal_vars.LC_target;
        real_LC_diff = (tmp_countLC > tmp_LC_target)
                           ? (tmp_countLC - tmp_LC_target)
                           : (tmp_LC_target - tmp_countLC);

        //    disable it to reduce time cost
        // printf("condition in %u, %u, diff:%u\r\n",
        //        synclight_cal_vars.optical_LC_cal_enable,
        //        synclight_cal_vars.optical_LC_cal_finished, real_LC_diff);

        if (synclight_cal_vars.optical_LC_cal_enable &&
            (!synclight_cal_vars.optical_LC_cal_finished)) {
            // printf("condition in %u,
            // %u",nclight_cal_vars.optical_LC_cal_enable,synclight_cal_vars.optical_LC_cal_finished);
            // This if function just calculate the cal_LC_diff, then give
            // coarse/mid/fine, since I am already get the real_LC_diff, I just
            // give new parameters(coarse/mid/fine)
            // if ((count_LC <= synclight_cal_vars.LC_target) &&
            //     (synclight_cal_vars.LC_target - count_LC <
            //      synclight_cal_vars.cal_LC_diff)) {
            //     printf("condition 1\r\n");
            //     synclight_cal_vars.cal_LC_diff =
            //         synclight_cal_vars.LC_target - count_LC;
            //     synclight_cal_vars.LC_coarse =
            //     synclight_cal_vars.cal_LC_coarse; synclight_cal_vars.LC_mid =
            //     synclight_cal_vars.cal_LC_mid; synclight_cal_vars.LC_fine =
            //     synclight_cal_vars.cal_LC_fine;
            // } else if ((count_LC > synclight_cal_vars.LC_target) &&
            //            (count_LC - synclight_cal_vars.LC_target <
            //             synclight_cal_vars.cal_LC_diff)) {
            //     printf("condition 2\r\n");
            //     synclight_cal_vars.cal_LC_diff =
            //         count_LC - synclight_cal_vars.LC_target;
            //     synclight_cal_vars.LC_coarse =
            //     synclight_cal_vars.cal_LC_coarse; synclight_cal_vars.LC_mid =
            //     synclight_cal_vars.cal_LC_mid; synclight_cal_vars.LC_fine =
            //     synclight_cal_vars.cal_LC_fine;
            // }
            synclight_cal_vars.LC_coarse = synclight_cal_vars.cal_LC_coarse;
            synclight_cal_vars.LC_mid = synclight_cal_vars.cal_LC_mid;
            synclight_cal_vars.LC_fine = synclight_cal_vars.cal_LC_fine;

            printf("count_LC: %u, LC_target: %u, LC_diff: %u\r\n", count_LC,
                   synclight_cal_vars.LC_target, real_LC_diff);

            // By moving this print, time cost 133-125ms, but I need this
            // info...so reduce synclight count to 7
            printf("coarse: %u, mid: %u, fine: %u\n",
                   synclight_cal_vars.LC_coarse, synclight_cal_vars.LC_mid,
                   synclight_cal_vars.LC_fine);
            // why the stop codition is not related to LC_diff? I find
            // that the mid is correct enought when LC_diff is smaller
            // than 100.
            if (real_LC_diff < MIN_LC_DIFF) {
                synclight_cal_vars.optical_LC_cal_finished = true;
            } else {
                ++synclight_cal_vars.cal_LC_fine;
                if (synclight_cal_vars.cal_LC_fine > LC_CAL_FINE_MAX) {
                    synclight_cal_vars.cal_LC_fine = LC_CAL_FINE_MIN;
                    ++synclight_cal_vars.cal_LC_mid;
                    if (synclight_cal_vars.cal_LC_mid > LC_CAL_MID_MAX) {
                        synclight_cal_vars.cal_LC_mid = LC_CAL_MID_MIN;
                        ++synclight_cal_vars.cal_LC_coarse;
                        // why the stop codition is not related to
                        // LC_diff?
                        if ((synclight_cal_vars.cal_LC_coarse >
                             LC_CAL_COARSE_MAX) ||
                            (real_LC_diff < 100)) {
                            synclight_cal_vars.optical_LC_cal_finished = true;
                            printf("coarse: %u, mid: %u, fine: %u\n",
                                   synclight_cal_vars.LC_coarse,
                                   synclight_cal_vars.LC_mid,
                                   synclight_cal_vars.LC_fine);
                        }
                    }
                }
            }

            if (!synclight_cal_vars.optical_LC_cal_finished) {
                LC_FREQCHANGE(synclight_cal_vars.cal_LC_coarse,
                              synclight_cal_vars.cal_LC_mid,
                              synclight_cal_vars.cal_LC_fine);
            } else {
                LC_FREQCHANGE(synclight_cal_vars.LC_coarse,
                              synclight_cal_vars.LC_mid,
                              synclight_cal_vars.LC_fine);
            }
        }

        // Do correction on 2M RC
        // Coarse step ~1100 counts, fine ~150 counts, superfine ~25
        // Too fast
        if (count_2M > (200600)) {
            RC2M_coarse += 1;
        } else {
            if (count_2M > (200080)) {
                RC2M_fine += 1;
            } else {
                if (count_2M > (200015)) {
                    RC2M_superfine += 1;
                }
            }
        }

        // Too slow
        if (count_2M < (199400)) {
            RC2M_coarse -= 1;
        } else {
            if (count_2M < (199920)) {
                RC2M_fine -= 1;
            } else {
                if (count_2M < (199985)) {
                    RC2M_superfine -= 1;
                }
            }
        }

        set_2M_RC_frequency(31, 31, RC2M_coarse, RC2M_fine, RC2M_superfine);
        scm3c_hw_interface_set_RC2M_coarse(RC2M_coarse);
        scm3c_hw_interface_set_RC2M_fine(RC2M_fine);
        scm3c_hw_interface_set_RC2M_superfine(RC2M_superfine);

        // Do correction on IF RC clock
        // Fine DAC step size is ~2800 counts
        if (count_IF > (1600000 + 1400)) {
            IF_fine += 1;
        }
        if (count_IF < (1600000 - 1400)) {
            IF_fine -= 1;
        }

        set_IF_clock_frequency(IF_coarse, IF_fine, 0);
        scm3c_hw_interface_set_IF_coarse(IF_coarse);
        scm3c_hw_interface_set_IF_fine(IF_fine);

        analog_scan_chain_write();
        analog_scan_chain_load();
    }

    // Debugging output
    // 1.1V/VDDD tap fix
    // The print is now broken down into 3 statements instead of one big
    // print statement
    // doing this prevent a long string of loads back to back
    printf("HF=%d-%d   2M=%d-%d", count_HFclock, HF_CLOCK_fine, count_2M,
           RC2M_coarse);
    printf(",%d,%d   LC=%d-%d   ", RC2M_fine, RC2M_superfine, count_LC,
           synclight_cal_vars.LC_code);
    printf("IF=%d-%d\r\n", count_IF, IF_fine);

    if (synclight_cal_vars.optical_cal_iteration >= 25 &&
        (!synclight_cal_vars.optical_LC_cal_enable ||
         synclight_cal_vars.optical_LC_cal_finished)) {
        // Disable this ISR
        ICER = 0x1800;
        synclight_cal_vars.optical_cal_iteration = 0;
        synclight_cal_vars.optical_cal_finished = 1;

        // Store the last count values
        synclight_cal_vars.num_32k_ticks_in_100ms = count_32k;
        synclight_cal_vars.num_2MRC_ticks_in_100ms = count_2M;
        synclight_cal_vars.num_IFclk_ticks_in_100ms = count_IF;
        synclight_cal_vars.num_LC_ch11_ticks_in_100ms = count_LC;
        synclight_cal_vars.num_HFclock_ticks_in_100ms = count_HFclock;

        // Debug prints
        // printf("LC_code=%d\r\n", synclight_cal_vars.LC_code);
        // printf("IF_fine=%d\r\n", IF_fine);

        // This was an earlier attempt to build out a complete table of
        // LC_code for TX/RX on each channel It doesn't really work well
        // yet so leave it commented printf("Building channel
        // table...");

        // radio_build_channel_table(LC_code);

        // printf("done\r\n");

        // radio_disable_all();

        // Halt all counters
        ANALOG_CFG_REG__0 = 0x0000;
    }
}

void sync_light_calibrate_all_clocks(uint32_t count_HFclock, uint32_t count_2M,
                                     uint32_t count_IF, uint32_t count_LC) {
    //	gpio_10_toggle();

    // This interrupt goes off when the optical register holds the value {221,
    // 176,
    // 231, 47} This interrupt can also be used to synchronize to the start of
    // an optical data transfer Need to make sure a new bit has been clocked in
    // prior to returning from this ISR, or else it will immediately execute
    // again
    // 1.1V/VDDD tap fix
    // helps reorder assembly code
    // Not completely sure why this works

    uint32_t count_32k;

    int32_t real_LC_diff;
    uint32_t HF_CLOCK_fine;
    uint32_t HF_CLOCK_coarse;
    uint32_t RC2M_coarse;
    uint32_t RC2M_fine;
    uint32_t RC2M_superfine;
    //    uint32_t IF_clk_target;
    uint32_t IF_coarse;
    uint32_t IF_fine;

    int32_t tmp_countLC, tmp_LC_target;

    HF_CLOCK_fine = scm3c_hw_interface_get_HF_CLOCK_fine();
    HF_CLOCK_coarse = scm3c_hw_interface_get_HF_CLOCK_coarse();
    RC2M_coarse = scm3c_hw_interface_get_RC2M_coarse();
    RC2M_fine = scm3c_hw_interface_get_RC2M_fine();
    RC2M_superfine = scm3c_hw_interface_get_RC2M_superfine();
    //    IF_clk_target = scm3c_hw_interface_get_IF_clk_target();
    IF_coarse = scm3c_hw_interface_get_IF_coarse();
    IF_fine = scm3c_hw_interface_get_IF_fine();

    // Keep track of how many calibration iterations have been completed
    synclight_cal_vars.optical_cal_iteration++;

    printf("run in sync cal\r\n");

    // Don't make updates on the first two executions of this ISR
    if (synclight_cal_vars.optical_cal_iteration > 2) {
        // Do correction on HF CLOCK
        // Fine DAC step size is about 6000 counts
        if (count_HFclock < 1997000) {  // 1997000 original value
            if (HF_CLOCK_fine == 0) {
                HF_CLOCK_coarse--;
                HF_CLOCK_fine = 10;
                synclight_cal_vars.optical_cal_iteration = 3;
            } else {
                HF_CLOCK_fine--;
            }
        }
        if (count_HFclock > 2003000) {  // new value I picked was
                                        // 2010000, originally 2003000
            if (HF_CLOCK_fine == 31) {
                HF_CLOCK_coarse++;
                HF_CLOCK_fine = 20;
                synclight_cal_vars.optical_cal_iteration = 3;
            } else {
                HF_CLOCK_fine++;
            }
        }

        set_sys_clk_secondary_freq(HF_CLOCK_coarse, HF_CLOCK_fine);
        scm3c_hw_interface_set_HF_CLOCK_coarse(HF_CLOCK_coarse);
        scm3c_hw_interface_set_HF_CLOCK_fine(HF_CLOCK_fine);

        // Do correction on LC
        // debugging, why diff cannot be calculated?
        tmp_countLC = count_LC;
        tmp_LC_target = synclight_cal_vars.LC_target;
        real_LC_diff = (tmp_countLC > tmp_LC_target)
                           ? (tmp_countLC - tmp_LC_target)
                           : (tmp_LC_target - tmp_countLC);

        synclight_cal_vars.optical_LC_cal_enable = 1;  // just test

        //    disable it to reduce time cost
        printf("condition in %u, %u, diff:%u\r\n",
               synclight_cal_vars.optical_LC_cal_enable,
               synclight_cal_vars.optical_LC_cal_finished, real_LC_diff);

        if (synclight_cal_vars.optical_LC_cal_enable &&
            (!synclight_cal_vars.optical_LC_cal_finished)) {

            synclight_cal_vars.LC_coarse = synclight_cal_vars.cal_LC_coarse;
            synclight_cal_vars.LC_mid = synclight_cal_vars.cal_LC_mid;
            synclight_cal_vars.LC_fine = synclight_cal_vars.cal_LC_fine;

            printf("count_LC: %u, LC_target: %u, LC_diff: %u\r\n", count_LC,
                   synclight_cal_vars.LC_target, real_LC_diff);

            // By moving this print, time cost 133-125ms, but I need this
            // info...so reduce synclight count to 7
            printf("coarse: %u, mid: %u, fine: %u\n",
                   synclight_cal_vars.LC_coarse, synclight_cal_vars.LC_mid,
                   synclight_cal_vars.LC_fine);
            // why the stop codition is not related to LC_diff? I find
            // that the mid is correct enought when LC_diff is smaller
            // than 100.
            if (real_LC_diff < MIN_LC_DIFF) {
                synclight_cal_vars.optical_LC_cal_finished = true;
            } else {
                ++synclight_cal_vars.cal_LC_fine;
                if (synclight_cal_vars.cal_LC_fine > LC_CAL_FINE_MAX) {
                    synclight_cal_vars.cal_LC_fine = LC_CAL_FINE_MIN;
                    ++synclight_cal_vars.cal_LC_mid;
                    if (synclight_cal_vars.cal_LC_mid > LC_CAL_MID_MAX) {
                        synclight_cal_vars.cal_LC_mid = LC_CAL_MID_MIN;
                        ++synclight_cal_vars.cal_LC_coarse;
                        // why the stop codition is not related to
                        // LC_diff?
                        if ((synclight_cal_vars.cal_LC_coarse >
                             LC_CAL_COARSE_MAX) ||
                            (real_LC_diff < 100)) {
                            synclight_cal_vars.optical_LC_cal_finished = true;
                            printf("coarse: %u, mid: %u, fine: %u\n",
                                   synclight_cal_vars.LC_coarse,
                                   synclight_cal_vars.LC_mid,
                                   synclight_cal_vars.LC_fine);
                        }
                    }
                }
            }

            if (!synclight_cal_vars.optical_LC_cal_finished) {
                LC_FREQCHANGE(synclight_cal_vars.cal_LC_coarse,
                              synclight_cal_vars.cal_LC_mid,
                              synclight_cal_vars.cal_LC_fine);
            } else {
                LC_FREQCHANGE(synclight_cal_vars.LC_coarse,
                              synclight_cal_vars.LC_mid,
                              synclight_cal_vars.LC_fine);
            }
        }

        // Do correction on 2M RC
        // Coarse step ~1100 counts, fine ~150 counts, superfine ~25
        // Too fast
        if (count_2M > (200600)) {
            RC2M_coarse += 1;
        } else {
            if (count_2M > (200080)) {
                RC2M_fine += 1;
            } else {
                if (count_2M > (200015)) {
                    RC2M_superfine += 1;
                }
            }
        }

        // Too slow
        if (count_2M < (199400)) {
            RC2M_coarse -= 1;
        } else {
            if (count_2M < (199920)) {
                RC2M_fine -= 1;
            } else {
                if (count_2M < (199985)) {
                    RC2M_superfine -= 1;
                }
            }
        }

        set_2M_RC_frequency(31, 31, RC2M_coarse, RC2M_fine, RC2M_superfine);
        scm3c_hw_interface_set_RC2M_coarse(RC2M_coarse);
        scm3c_hw_interface_set_RC2M_fine(RC2M_fine);
        scm3c_hw_interface_set_RC2M_superfine(RC2M_superfine);

        // Do correction on IF RC clock
        // Fine DAC step size is ~2800 counts
        if (count_IF > (1600000 + 1400)) {
            IF_fine += 1;
        }
        if (count_IF < (1600000 - 1400)) {
            IF_fine -= 1;
        }

        set_IF_clock_frequency(IF_coarse, IF_fine, 0);
        scm3c_hw_interface_set_IF_coarse(IF_coarse);
        scm3c_hw_interface_set_IF_fine(IF_fine);

        analog_scan_chain_write();
        analog_scan_chain_load();
    }

    // Debugging output
    // 1.1V/VDDD tap fix
    // The print is now broken down into 3 statements instead of one big
    // print statement
    // doing this prevent a long string of loads back to back
    printf("HF=%d-%d   2M=%d-%d", count_HFclock, HF_CLOCK_fine, count_2M,
           RC2M_coarse);
    printf(",%d,%d   LC=%d-%d   ", RC2M_fine, RC2M_superfine, count_LC,
           synclight_cal_vars.LC_code);
    printf("IF=%d-%d\r\n", count_IF, IF_fine);

    if (synclight_cal_vars.optical_cal_iteration >= 25 &&
        (!synclight_cal_vars.optical_LC_cal_enable ||
         synclight_cal_vars.optical_LC_cal_finished)) {
        // Disable this ISR
        // ICER = 0x1800;
        synclight_cal_vars.optical_cal_iteration = 0;
        synclight_cal_vars.optical_cal_finished = 1;

        // Store the last count values
        synclight_cal_vars.num_32k_ticks_in_100ms = count_32k;
        synclight_cal_vars.num_2MRC_ticks_in_100ms = count_2M;
        synclight_cal_vars.num_IFclk_ticks_in_100ms = count_IF;
        synclight_cal_vars.num_LC_ch11_ticks_in_100ms = count_LC;
        synclight_cal_vars.num_HFclock_ticks_in_100ms = count_HFclock;

        // Debug prints
        // printf("LC_code=%d\r\n", synclight_cal_vars.LC_code);
        // printf("IF_fine=%d\r\n", IF_fine);

        // This was an earlier attempt to build out a complete table of
        // LC_code for TX/RX on each channel It doesn't really work well
        // yet so leave it commented printf("Building channel
        // table...");

        // radio_build_channel_table(LC_code);

        // printf("done\r\n");

        // radio_disable_all();

        // Halt all counters
        // ANALOG_CFG_REG__0 = 0x0000;
    }
}
// this function use to test call calibration frenquency, so just some print.
// use this in decode_lighthouse #600
void sync_light_calibrate_isr_placeholder(void) {
    //    int32_t t;
    uint32_t rdata_lsb, rdata_msb;
    int32_t count_LC;
    uint32_t count_32k, count_2M, count_HFclock, count_IF;
    // a new LC_diff to replace the struct variable one
    // (synclight_cal_vars.cal_LC_diff).
    int32_t real_LC_diff;
    uint32_t HF_CLOCK_fine;
    uint32_t HF_CLOCK_coarse;
    uint32_t RC2M_coarse;
    uint32_t RC2M_fine;
    uint32_t RC2M_superfine;
    //    uint32_t IF_clk_target;
    uint32_t IF_coarse;
    uint32_t IF_fine;

    int32_t tmp_countLC, tmp_LC_target;

    HF_CLOCK_fine = scm3c_hw_interface_get_HF_CLOCK_fine();
    HF_CLOCK_coarse = scm3c_hw_interface_get_HF_CLOCK_coarse();
    RC2M_coarse = scm3c_hw_interface_get_RC2M_coarse();
    RC2M_fine = scm3c_hw_interface_get_RC2M_fine();
    RC2M_superfine = scm3c_hw_interface_get_RC2M_superfine();
    //    IF_clk_target = scm3c_hw_interface_get_IF_clk_target();
    IF_coarse = scm3c_hw_interface_get_IF_coarse();
    IF_fine = scm3c_hw_interface_get_IF_fine();

    // Disable all counters
    ANALOG_CFG_REG__0 = 0x007F;

    // Keep track of how many calibration iterations have been completed
    synclight_cal_vars.optical_cal_iteration++;

    // Read 32k counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x000000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x040000);
    count_32k = rdata_lsb + (rdata_msb << 16);

    // Read HF_CLOCK counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x100000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x140000);
    count_HFclock = rdata_lsb + (rdata_msb << 16);

    // Read 2M counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x180000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x1C0000);
    count_2M = rdata_lsb + (rdata_msb << 16);

    // Read LC_div counter (via counter4)
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x280000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x2C0000);
    count_LC = rdata_lsb + (rdata_msb << 16);

    // Read IF ADC_CLK counter
    rdata_lsb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x300000);
    rdata_msb = *(unsigned int*)(APB_ANALOG_CFG_BASE + 0x340000);
    count_IF = rdata_lsb + (rdata_msb << 16);

    // Reset all counters
    ANALOG_CFG_REG__0 = 0x0000;

    // Enable all counters
    ANALOG_CFG_REG__0 = 0x3FFF;
    synclight_cal_vars.LC_coarse = synclight_cal_vars.cal_LC_coarse;
    synclight_cal_vars.LC_mid = synclight_cal_vars.cal_LC_mid;
    synclight_cal_vars.LC_fine = synclight_cal_vars.cal_LC_fine;

    printf("count_LC: %u, LC_target: %u, LC_diff: %u\r\n", count_LC,
           synclight_cal_vars.LC_target, real_LC_diff);

    // By moving this print, time cost 133-125ms, but I need this
    // info...so reduce synclight count to 7
    printf("coarse: %u, mid: %u, fine: %u\n", synclight_cal_vars.LC_coarse,
           synclight_cal_vars.LC_mid, synclight_cal_vars.LC_fine);
    // why the stop codition is not related to LC_diff? I find
    // that the mid is correct enought when LC_diff is smaller
    // than 100.

    printf("coarse: %u, mid: %u, fine: %u\n", synclight_cal_vars.LC_coarse,
           synclight_cal_vars.LC_mid, synclight_cal_vars.LC_fine);

    printf("HF=%d-%d   2M=%d-%d", count_HFclock, HF_CLOCK_fine, count_2M,
           RC2M_coarse);
    printf(",%d,%d   LC=%d-%d   ", RC2M_fine, RC2M_superfine, count_LC,
           synclight_cal_vars.LC_code);
    printf("IF=%d-%d\r\n", count_IF, IF_fine);
}
//=========================== private =========================================
