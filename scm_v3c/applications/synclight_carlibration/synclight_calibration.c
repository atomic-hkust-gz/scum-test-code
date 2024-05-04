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

//=========================== main ============================================

int main(void) {
    uint32_t i;

    memset(&app_vars, 0, sizeof(app_vars_t));

    printf("Initializing...");

    initialize_mote();
    crc_check();
    perform_calibration();
    printf("~~~~my code start~~~~~%d\n", app_vars.count);
    //		gpio_init();

    // gpio9 = 0x0200
    GPO_control(6, 6, 6, 6);  // bank6 is the GPIO port out
    analog_scan_chain_write();
    analog_scan_chain_load();
    GPO_enables(0x0000);
    GPO_enables(1 << CALIBRATE_OUTPUT);  // IO 10,0x0400

    //	GPI_enables(0x0008);//IO 3
    GPI_enables(1 << CALIBRATE_SYNCLIGHT_INPUT);  // IO 8
    //	GPI_enables(0x0200);//IO 9
    analog_scan_chain_write();
    analog_scan_chain_load();

    // gpio_3_set();
    printf("~~~~enable INTs~~~~~%d\n", app_vars.count);
    gpio_ext_3_interrupt_enable();
    // clean optical and ex3 interrupt, then re-open ext_3 interrupt
    need_optical = 0;
    //		enable extern interrupts and first clean EXT_8 and optical
    // interrupt
    //    ICER = 0x1800;
    //		set the ISER to 0x1000,only gpio
    //    ISER = 0x1000;
    //		set the ISER to 0x0800,only optical
    //    ISER = 0x0800;
    //    if polling GPIO 3 ,we don't need any INT
    //    ICER = 0xFFFF;
    //    GPO_control(0,6,6,6);// bank6 is the GPIO port out
    //    GPI_control(0,6,6,6);// bank6 is the GPIO port out
    GPO_control(0, 6, 0, 0);  // bank6 is the GPIO port out
    GPI_control(0, 0, 1, 0);  // bank6 is the GPIO port out
    analog_scan_chain_write();
    analog_scan_chain_load();
    //    GPI_enables(0x0008);//IO 3
    GPI_enables(0xFF08);  // IO 0-3
    GPO_enables(0xD0FF);  // IO 0-3
    analog_scan_chain_write();
    analog_scan_chain_load();

//    ICER = 0xFFFF;
    //	gpio_3_set();
    //	gpio_8_set();
    //	gpio_9_set();
    //	gpio_10_set();
    //    gpio_10_clr();
    //    gpio_8_clr();

    // from lighthouse..c

//    // Set HCLK source as HF_CLOCK
//    set_asc_bit(1147);

//    // Set initial coarse/fine on HF_CLOCK

//    set_sys_clk_secondary_freq(3, 17);

//    // Set RFTimer source as HF_CLOCK
//    set_asc_bit(1151);

//    // Disable LF_CLOCK
//    set_asc_bit(553);

    analog_scan_chain_write();
    analog_scan_chain_load();

    printf("~~~~start to say HELLO?~~~~~%d\n", app_vars.count);
    while (1) {
        printf("Hello World! %d\n", app_vars.count);
        app_vars.count += 1;

        for (i = 0; i < 1000000; i++)
            ;
    }
}

//=========================== public ==========================================

//=========================== private =========================================
