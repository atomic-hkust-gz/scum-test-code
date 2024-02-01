#include <string.h>

#include "calibrate_interrupt.h"
#include "memory_map.h"
//#include "optical.h"
#include "scm3c_hw_interface.h"
#include "gpio.h"

//=========================== defines =========================================


//=========================== variables =======================================

//=========================== prototypes ======================================
void calibration_isr() {
    printf("External Interrupt GPIO9 triggered\r\n");
}

//=========================== main ============================================



//=========================== public ==========================================
void gpio_ext_3_interrupt_enable(void){
		ISER |= 0x0002; 
}

void gpio_ext_3_interrupt_disable(void){
		ICER |= 0x0002; 
}


void gpio_ext_9_interrupt_enable(void){
		ISER |= 0x2000; 
}

void gpio_ext_9_interrupt_disable(void){
		ICER |= 0x2000; 
}

void gpio_ext_10_interrupt_enable(void){
		ISER |= 0x4000; 
}

void gpio_ext_10_interrupt_disable(void){
		ICER |= 0x4000; 
}

void sync_light_calibrate_isr(void){
	gpio_10_toggle();
}
//=========================== private =========================================

