#ifndef __CALIBRATE_INTERRUPT
#define __CALIBRATE_INTERRUPT


#include <string.h>

#include "memory_map.h"
#include "optical.h"
#include "scm3c_hw_interface.h"

//=========================== defines =========================================


//=========================== variables =======================================


//=========================== prototypes ======================================

//=========================== main ============================================



//=========================== public ==========================================

void calibration_isr();
void gpio_ext_3_interrupt_enable(void);
void gpio_ext_3_interrupt_disable(void);
void gpio_ext_9_interrupt_enable(void);
void gpio_ext_9_interrupt_disable(void);
void gpio_ext_10_interrupt_enable(void);
void gpio_ext_10_interrupt_disable(void);
void sync_light_calibrate_isr(void);

//=========================== private =========================================

#endif 