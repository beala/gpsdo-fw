#ifndef _HWPPS_H_
#define _HWPPS_H_

// The PPS output, on PA11 (TIM1 channel 4): each rising edge is made by the timer's compare at an exact count of
// the 70 MHz clock (the OCXO's), so it carries no software timing. TIM1 also captures the GPS module's pulse
// (channel 1), so the output is compared with it on the same count and steered onto it with a slow phase loop
// (hwpps_core.c): the GPS pulse's jitter averaged out, no steps once locked, the OCXO carrying it through GPS
// outages. Runs from boot (free until the GPS pulse is in). A 100 ms pulse, 3.3 V. (It replaces PB1's output,
// which was set by software in TIM2's interrupt.) Settings: the PPS menu's Sync (steer or run free), Delay and
// Threshold (how long and how far off before it jumps to the GPS pulse), Force Sync.

#include <stdbool.h>
#include <stdint.h>

void hwpps_start(void);                      // after TIM1 is running (frequency_start)
void hwpps_overflow(void);                   // TIM1 update interrupt
void hwpps_gps(uint16_t capture);           // TIM1 channel 1 capture (the GPS pulse)
void hwpps_force(void);                      // jump to the GPS pulse at the next one (the menu's Force sync)
void hwpps_cc4(void);                        // TIM1 channel 4 compare interrupt

// for the menu
bool     hwpps_locked(void);
int32_t  hwpps_err_ns(void);                 // GPS pulse minus output edge, averaged over about a minute
int32_t  hwpps_err_now_ns(void);             // the same, the latest second
uint32_t hwpps_jumps(void);                  // jumps to the GPS pulse (the first lock counts)
uint32_t hwpps_late(void);                   // edges missed (the timer armed too late: should stay 0)
uint32_t hwpps_holdover_s(void);             // seconds since the last GPS pulse

#endif
