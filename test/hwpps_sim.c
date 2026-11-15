// The hardware PPS's steering and arming, simulated on a computer: an OCXO with an offset, a slow wander and a
// step; GPS pulses with an ATGM336H's jitter; a 10-minute GPS outage; the timer's 16-bit compare armed from
// wake-ups that run late by random amounts. Reports how far each output edge is from true seconds.
// Build and run: cc -O2 -I../src -o /tmp/hwpps_sim hwpps_sim.c ../src/hwpps_core.c -lm && /tmp/hwpps_sim
#include "hwpps_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static double randn(void)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

// the OCXO: cycles counted by true time t (s); its rate offset y (fraction) changes once a second
static double y = 50e-9, cyc = 1234567.0;   // starts 50 ppb off, at an arbitrary count
static double cyc_at(double c0, double dt) { return c0 + HWPPS_F * (1 + y) * dt; }

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: " __VA_ARGS__); printf("\n"); fails++; } } while (0)

// arming: for any latency, the first compare match after arming is the edge itself, or the edge is called late
static void test_arm(void)
{
    long armed = 0, late = 0;
    for (int i = 0; i < 2000000; i++) {
        uint64_t edge = ((uint64_t)rand() << 20) ^ rand();
        // wake-ups: each time the low 16 bits reach (edge - 32768), plus an interrupt delay
        uint64_t w = edge - 32768 - 65536 * (uint64_t)(rand() % 3);
        uint32_t lat = rand() % 100 < 98 ? rand() % 3000 : rand() % 40000;
        uint64_t now = w + lat;
        hwpps_arm_t a = hwpps_core_arm(now, edge);
        int64_t d = (int64_t)(edge - now);
        if (a == HWPPS_ARM) {
            armed++;
            // the hardware: the first count at or after now (+ a few cycles to write) whose low 16 bits match
            uint64_t t = now + 8;
            uint64_t m = (t & ~0xFFFFULL) | (edge & 0xFFFF);
            if (m < t) m += 65536;
            CHECK(m == edge, "armed at d=%lld: matched %llu, edge %llu", (long long)d, (unsigned long long)m,
                  (unsigned long long)edge);
        } else if (a == HWPPS_WAIT) {
            CHECK(d > HWPPS_ARM_MAX, "waited at d=%lld", (long long)d);
        } else {
            late++;
            CHECK(d < HWPPS_ARM_MIN, "late at d=%lld", (long long)d);
        }
    }
    printf("arming: %ld armed, %ld late (delays over 32 k cycles), every armed edge matched exactly\n", armed, late);
}

int main(void)
{
    srand(42);
    test_arm();

    hwpps_core_t s;
    hwpps_core_init(&s, 100.0);
    const double jit_ns = 30.0;            // the ATGM336H's pulse jitter, rms
    const int N = 4 * 3600;
    // the output starts free-running at an arbitrary phase
    uint64_t next = (uint64_t)cyc + 12345678;
    hwpps_core_start(&s, next);
    // statistics by period: settled with GPS (from 15 min, leaving out the 10 min after the step and the outage
    // with 10 min after it), after the step, in the outage and the 10 min after
    double q1 = 0, q2 = 0, qmax = 0; int qn = 0;
    double smax = 0, hmax = 0;
    int lock_k = 0;
    int out_s = 1800, out_e = 2400;        // GPS outage
    FILE* csv = fopen("/tmp/hwpps_sim.csv", "w");
    for (int k = 1; k <= N; k++) {
        // a second passes: the OCXO wanders (random walk 2e-12/s), steps 5 ppb at 1 h
        double c1 = cyc_at(cyc, 1.0);
        y += 2e-12 * randn();
        if (k == 3600) y += 5e-9;
        // the GPS pulse at true second k: its count, with jitter, as the timer captures it (whole cycles)
        double gc = c1 + jit_ns * 1e-9 * HWPPS_F * randn();
        // edges that fall within this second, in order with the GPS capture
        bool gps_done = false;
        while ((double)next < c1 + HWPPS_F * 0.5) {
            if (!gps_done && gc < (double)next && !(k > out_s && k <= out_e)) {
                hwpps_core_gps(&s, (uint64_t)floor(gc));
                gps_done = true;
            }
            uint64_t e = next;
            // the edge's true time against true seconds: (e - count at second k) / rate
            double err_ns = ((double)e - c1) / (HWPPS_F * (1 + y)) * 1e9;
            next = hwpps_core_edge(&s, e);
            if (!lock_k && s.state == HWPPS_LOCKED) lock_k = k;
            if (k > out_s && k <= out_e + 600) { if (fabs(err_ns) > hmax) hmax = fabs(err_ns); }
            else if (k > 3600 && k <= 4200) { if (fabs(err_ns) > smax) smax = fabs(err_ns); }
            else if (k > 900) { q1 += err_ns; q2 += err_ns * err_ns; qn++; if (fabs(err_ns) > qmax) qmax = fabs(err_ns); }
            if (csv) fprintf(csv, "%d,%.2f,%d,%.1f\n", k, err_ns, s.err, s.integ);
        }
        if (!gps_done && !(k > out_s && k <= out_e)) hwpps_core_gps(&s, (uint64_t)floor(gc));
        cyc = c1;
    }
    if (csv) fclose(csv);
    printf("locked %d s after start, jumps %u; output edge against true seconds (GPS pulses %g ns rms):\n", lock_k, s.jumps, jit_ns);
    printf("  settled: mean %+.1f ns, %.1f ns rms, worst %.1f ns (%d edges)\n", q1 / qn, sqrt(q2 / qn - (q1 / qn) * (q1 / qn)), qmax, qn);
    printf("  10 min after a 5 ppb OCXO frequency step: worst %.1f ns\n", smax);
    printf("  10 min without GPS and 10 min after: worst %.1f ns\n", hmax);
    CHECK(s.jumps == 1, "jumped %u times (once expected: the first lock)", s.jumps);
    CHECK(sqrt(q2 / qn) < 12, "settled: too far from true seconds");
    printf(fails ? "FAILED (%d)\n" : "all good\n", fails);
    return fails != 0;
}
