#ifndef __CALIBRATE_INTERRUPT
#define __CALIBRATE_INTERRUPT

#include <string.h>

#include "memory_map.h"
#include "optical.h"
#include "scm3c_hw_interface.h"

//=========================== defines =========================================
#define LC_CAL_COARSE_MIN 19
#define LC_CAL_COARSE_MAX 25
#define LC_CAL_MID_MIN 0
#define LC_CAL_MID_MAX 31
#define LC_CAL_FINE_MIN 15
#define LC_CAL_FINE_MAX 15
#define MIN_LC_DIFF 160  // can be 100 for strict equal, can be adjusted
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

    int32_t last_LC_diff;  // record the last LC diff value
    bool midChange;        //     record the last mid change
    bool coarseChange;     //     record the last coarse change

    // reference to calibrate
    int32_t LC_target;
    uint32_t LC_code;
    uint8_t LC_coarse;
    uint8_t LC_mid;
    uint8_t LC_fine;
    // the optimal LC parameters
    uint8_t LC_coarse_opt;
    uint8_t LC_mid_opt;
    uint8_t LC_fine_opt;

    uint8_t HF_coarse_opt;
    uint8_t HF_fine_opt;
    uint8_t RC2M_coarse_opt;
    uint8_t RC2M_fine_opt;
    uint8_t RC2M_superfine_opt;
    uint8_t IF_coarse_opt;
    uint8_t IF_fine_opt;

} synclight_calibrate_vars_t;

typedef struct {
    uint32_t lighthouse_clock[ASC_LEN];
    uint32_t ble_clock[ASC_LEN];

} asc_state_t;

//=========================== prototypes ======================================

//=========================== main ============================================

//=========================== public ==========================================
void sync_light_calibrate_init(void);
void synclight_cal_enableLCCalibration(void);
bool synclight_cal_getCalibrationFinished(void);
void synclight_cal_setLCTarget(uint32_t LC_target);
uint8_t synclight_cal_getLCCoarse(void);
uint8_t synclight_cal_getLCMid(void);
uint8_t synclight_cal_getLCFine(void);
void synclight_cal_enable(void);
void perform_synclight_calibration(void);
// void optical_sfd_isr(void);

void calibration_isr(void);
void gpio_ext_3_interrupt_enable(void);
void gpio_ext_3_interrupt_disable(void);
void gpio_ext_9_interrupt_enable(void);
void gpio_ext_9_interrupt_disable(void);
void gpio_ext_10_interrupt_enable(void);
void gpio_ext_10_interrupt_disable(void);
void sync_light_calibrate_init(void);
void reload_sync_light_calibrate_init(void);
void sync_light_calibrate_isr(void);
void sync_light_calibrate_all_clocks(uint32_t count_HFclock, uint32_t count_2M,
                                     uint32_t count_IF, uint32_t count_LC);
void sync_light_calibrate_isr_placeholder(void);
void sync_light_calibrate_set_optimal_clocks(void);
void save_ASC_state(uint32_t* asc_state);
void restore_ASC_state(uint32_t* asc_state);
//=========================== private =========================================

#endif
