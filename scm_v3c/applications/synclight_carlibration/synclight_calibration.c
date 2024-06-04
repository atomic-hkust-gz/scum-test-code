#include <string.h>

#include "calibrate_interrupt.h"
#include "gpio.h"
#include "lighthouse_position.h"
#include "memory_map.h"
#include "optical.h"
#include "scm3c_hw_interface.h"

//=========================== defines =========================================

#define CRC_VALUE (*((unsigned int*)0x0000FFFC))
#define CODE_LENGTH (*((unsigned int*)0x0000FFF8))

#define CALIBRATE_SYNCLIGHT_INPUT 8  // receive sync light on this pin
#define CALIBRATE_OUTPUT \
    10  // toggle this pin to show scum received a sync light
//#define OPTICAL_DATA_RAW_PIN \
//    ((0x0008 & GPIO_REG__INPUT) >> 3)// optical receiver digital data pin

//// indicate the type of light
// #define type_sync 0
// #define type_sweep 1
// #define type_skip_sync 2
//=========================== variables =======================================

typedef struct {
    uint8_t count;
} app_vars_t;

app_vars_t app_vars;

extern int8_t need_optical;

int t;

//=========================== prototypes ======================================
void config_lighthouse_mote(void) {
    //  I think RF timer needs to be reset before use, but not essential.
    // RF Timer rolls over at this value and starts a new cycle
    RFTIMER_REG__MAX_COUNT = 0xFFFFFFFF;
    // Enable RF Timer
    RFTIMER_REG__CONTROL = 0x7;

    // Select banks for GPIO inputs
    GPI_control(0, 0, 0, 0);
    // Select banks for GPIO outputs
    GPO_control(0, 0, 0, 0);
    // Set all GPIOs as outputs
    GPI_enables(0x000F);  // 0008=io3?
    GPO_enables(0xFFFF);

    analog_scan_chain_write();
    analog_scan_chain_load();
}
//=========================== main ============================================

int main(void) {
    uint32_t i;

    memset(&app_vars, 0, sizeof(app_vars_t));

    printf("Initializing...");
    // config_lighthouse_mote();
    initialize_mote();
    crc_check();
    perform_calibration();
    printf("~~~~my code start~~~~~%d\n", app_vars.count);

    config_lighthouse_mote();

    // clean optical and ex3 interrupt, then re-open ext_3 interrupt
    need_optical = 0;

    // from lighthouse..c

    printf("~~~~start to say HELLO?~~~~~%d\n", app_vars.count);
    while (1) {
        printf("Hello World! %d\n", app_vars.count);
        app_vars.count += 1;

        for (i = 0; i < 1000000; i++);
    }
}

//=========================== public ==========================================

//=========================== private =========================================