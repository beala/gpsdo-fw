#include "hwpps_core.h"
#include <math.h>
#include <stdlib.h>

void hwpps_core_init(hwpps_core_t* s, double tau_s)
{
    *s = (hwpps_core_t){0};
    // a second-order loop, critically damped: natural frequency 1/tau
    double wn = 1.0 / tau_s;
    s->kp = 2.0 * wn;
    s->ki = wn * wn;
    s->steer = true;
    s->coarse = HWPPS_COARSE;
    s->coarse_s = HWPPS_COARSE_S;
}

void hwpps_core_gps(hwpps_core_t* s, uint64_t g)
{
    if (s->gps_prev_ok) {
        int64_t d = (int64_t)(g - s->gps_prev) - HWPPS_F;
        if (d > -70000 && d < 70000) {   // within 1000 ppm: one second on (a missed pulse or a glitch otherwise)
            if (s->rate_n == 0) s->rate_g0 = s->gps_prev;
            s->rate_n++;
            double r = ((double)(int64_t)(g - s->rate_g0) - (double)s->rate_n * HWPPS_F) / s->rate_n;
            if (s->rate_n >= HWPPS_RATE_MIN) { s->rate = r; s->rate_ok = true; }
            if (s->rate_n >= HWPPS_RATE_S) s->rate_n = 0;   // a new span from this pulse
        } else {
            s->rate_n = 0;
        }
    }
    s->gps_prev = g;
    s->gps_prev_ok = true;
    int64_t a = (int64_t)(g - s->next_edge);
    int64_t b = s->have_edge ? (int64_t)(g - s->last_edge) : INT64_MAX;
    int64_t e = llabs(a) < llabs(b) ? a : b;
    if (llabs(e) < HWPPS_F / 2) {
        s->err_sum += e;
        s->err_n++;
    }
}

void hwpps_core_start(hwpps_core_t* s, uint64_t first) { s->next_edge = first; }

void hwpps_core_force(hwpps_core_t* s) { s->force = true; }

uint64_t hwpps_core_edge(hwpps_core_t* s, uint64_t e)
{
    s->last_edge = e;
    s->have_edge = true;

    double u = s->integ;                // cycles to add to this coming second
    int64_t err = 0;
    bool fresh = s->err_n > 0;
    if (fresh) {
        err = s->err_sum / (int64_t)s->err_n;   // nearly always one pulse
        s->err_sum = 0;
        s->err_n = 0;
    }

    if (fresh) {
        s->holdover_s = 0;
        s->err = (int32_t)err;
        s->far = llabs(err) > s->coarse ? s->far + 1 : 0;
        bool locked = s->state == HWPPS_LOCKED;
        if (!s->steer) {
            s->force = false;                   // free running: measured, not steered
            s->state = HWPPS_FREE;
            s->err_avg += ((double)err - s->err_avg) / 64.0;
        } else if (locked ? (s->force || s->far >= s->coarse_s) : s->rate_ok) {
            // jump: the next edge lands where the GPS pulse says, at the rate the GPS pulses show
            s->jumps++;
            s->force = false;
            s->far = 0;
            if (s->rate_ok) s->integ = s->rate;
            s->frac = 0;
            s->err_avg = 0;
            s->state = HWPPS_LOCKED;
            u = s->integ + (double)err;
        } else if (locked) {
            // steer: proportional and integral on the phase
            s->integ += s->ki * (double)err;
            if (s->integ > 70000) s->integ = 70000;
            if (s->integ < -70000) s->integ = -70000;
            u = s->integ + s->kp * (double)err;
            s->err_avg += ((double)err - s->err_avg) / 64.0;
        }
    } else {
        s->holdover_s++;
    }

    // whole cycles only: what's left over goes into the next second
    s->frac += u;
    double whole = floor(s->frac + 0.5);
    s->frac -= whole;
    s->next_edge = e + (uint64_t)(HWPPS_F + (int64_t)whole);
    return s->next_edge;
}

hwpps_arm_t hwpps_core_arm(uint64_t now, uint64_t edge)
{
    int64_t d = (int64_t)(edge - now);
    if (d > HWPPS_ARM_MAX) return HWPPS_WAIT;
    if (d >= HWPPS_ARM_MIN) return HWPPS_ARM;
    return HWPPS_LATE;
}
