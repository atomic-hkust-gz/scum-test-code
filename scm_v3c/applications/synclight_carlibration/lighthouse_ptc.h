#ifndef __LIGHTHOUSE_PTC_H
#define __LIGHTHOUSE_PTC_H

#include <stdint.h>

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

#endif
