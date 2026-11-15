#ifndef _HWPPS_CORE_H_
#define _HWPPS_CORE_H_

// Hardware-timed PPS output: the steering logic, with no hardware in it (so it can be tested on a computer).
//
// Times are TIM1 cycles (70 MHz, from the OCXO) on a 64-bit count. Each output edge is placed at an exact count by
// the timer hardware; this decides where the next one goes. The GPS module's pulse (captured on the same timer) is
// compared with the output edge each second, and the output is steered towards it with a slow phase loop: the GPS
// pulse's jitter (20-50 ns on an ATGM336H) is averaged out, the OCXO carries the time in between, and there are no
// steps once locked. Without GPS the output carries on at the last rate it learned (holdover).

#include <stdbool.h>
#include <stdint.h>

#define HWPPS_F 70000000LL        // cycles in a second at the nominal clock

// Far off (more than coarse cycles for coarse_s seconds running): jump straight to the GPS pulse, as at start.
// Defaults: 700 cycles = 10 us, far beyond the GPS pulse's jitter, for 10 s (the menu's Threshold and Delay)
#define HWPPS_COARSE   700
#define HWPPS_COARSE_S 10

#define HWPPS_RATE_S   64      // seconds a rate span covers
#define HWPPS_RATE_MIN 16      // the first lock waits for this many seconds of GPS pulses

typedef enum { HWPPS_FREE = 0, HWPPS_LOCKED = 1 } hwpps_state;

typedef struct {
    // the loop
    double   integ;        // the output's rate correction, cycles per second (the OCXO's offset from 70 MHz)
    double   frac;         // the part of a cycle not yet placed (each edge falls on a whole cycle)
    double   kp, ki;       // loop gains (from the time constant)
    // settings (the menu's): steer off = free running at the last rate learned, no jumps
    bool     steer;
    int64_t  coarse;       // cycles
    uint32_t coarse_s;
    // the GPS pulse
    int64_t  err_sum;      // GPS pulse minus its nearest output edge, each pulse since the last edge, summed
    uint32_t err_n;
    uint64_t gps_prev;
    bool     gps_prev_ok;
    // the GPS pulses' rate on our clock (cycles a second minus HWPPS_F): the rate the output starts from at a
    // jump, from the span of up to HWPPS_RATE_S pulses (first to last: the jitter of two pulses over the span)
    uint64_t rate_g0;      // the first pulse of the span
    uint32_t rate_n;       // seconds in it
    double   rate;         // the last whole span's rate
    bool     rate_ok;      // a span of HWPPS_RATE_MIN seconds or more is in
    // edges
    uint64_t last_edge;    // made
    uint64_t next_edge;    // scheduled
    bool     have_edge;
    uint32_t far;          // seconds running the output has been more than coarse off
    bool     force;        // jump at the next edge
    // what it reports
    hwpps_state state;
    int32_t  err;          // the latest GPS pulse minus the output edge, cycles
    double   err_avg;      // the same, averaged (about a minute), cycles
    uint32_t jumps;        // times it jumped to the GPS pulse (the first lock counts)
    uint32_t holdover_s;   // seconds since the last GPS pulse used
} hwpps_core_t;

// tau_s: the phase loop's time constant in seconds (100: the GPS pulse's jitter averaged over a few minutes)
void hwpps_core_init(hwpps_core_t* s, double tau_s);

// the GPS module's pulse was captured at g. It's measured at once against the nearest output edge, the last one
// made or the next one scheduled (holding it for the next edge instead lost the ones that came just after an edge
// whenever the next came just before one: the average leaned early by about 12 ns in the simulation)
void hwpps_core_gps(hwpps_core_t* s, uint64_t g);

// ask for a jump to the GPS pulse at the next edge (the menu's Force sync)
void hwpps_core_force(hwpps_core_t* s);

// the first edge, before any is made (free running, any phase)
void hwpps_core_start(hwpps_core_t* s, uint64_t first);

// the output edge has just been made at e (the one scheduled): returns when the next one goes
uint64_t hwpps_core_edge(hwpps_core_t* s, uint64_t e);

// Arming the timer for the next edge, decided at a wake-up at time now (the hardware layer wakes up once per timer
// period). HWPPS_WAIT: not yet; HWPPS_ARM: set the compare for the edge now (it falls within this or the next
// timer period, far enough ahead); HWPPS_LATE: too close or past (the edge is skipped: the caller moves on a second)
typedef enum { HWPPS_WAIT, HWPPS_ARM, HWPPS_LATE } hwpps_arm_t;
#define HWPPS_ARM_MAX  49152      // arm when the edge is at most this far ahead (the 16-bit compare matches once)
#define HWPPS_ARM_MIN  256        // and at least this far (time to write the compare register)
hwpps_arm_t hwpps_core_arm(uint64_t now, uint64_t edge);

#endif
