#ifndef __OPTICAL_H
#define __OPTICAL_H

#include <stdbool.h>
#include <stdint.h>

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================
typedef struct {
    uint8_t optical_cal_iteration;
    bool optical_cal_finished;

    bool optical_LC_cal_enable;
    bool optical_LC_cal_finished;
    uint8_t cal_LC_coarse;
    uint8_t cal_LC_mid;
    uint8_t cal_LC_fine;
    uint32_t cal_LC_diff;

    uint32_t num_32k_ticks_in_100ms;
    uint32_t num_2MRC_ticks_in_100ms;
    uint32_t num_IFclk_ticks_in_100ms;
    uint32_t num_LC_ch11_ticks_in_100ms;
    uint32_t num_HFclock_ticks_in_100ms;

    // reference to calibrate
    uint32_t LC_target;
    uint32_t LC_code;
    uint8_t LC_coarse;
    uint8_t LC_mid;
    uint8_t LC_fine;
} optical_vars_t;
//=========================== prototypes ======================================

//==== admin
void optical_init(void);
void optical_enableLCCalibration(void);
bool optical_getCalibrationFinished(void);
void optical_setLCTarget(uint32_t LC_target);
uint8_t optical_getLCCoarse(void);
uint8_t optical_getLCMid(void);
uint8_t optical_getLCFine(void);
void optical_enable(void);
void perform_calibration(void);
void optical_sfd_isr(void);

#endif
