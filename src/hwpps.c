#include "hwpps.h"
#include "hwpps_core.h"
#include "main.h"
#include "int.h"
#include "stm32f1xx_hal.h"

// The output edge: TIM1 channel 4's compare register holds the low 16 bits of a time. It matches once every
// 65536 cycles (0.94 ms), so it doubles as a wake-up: while waiting it holds (edge - 32768), and each wake-up
// checks whether the edge is now close enough to arm (hwpps_core_arm). Armed, the channel is in "active on
// match" mode with the compare at the edge: the pin goes high on that exact cycle. 100 ms later a wake-up forces
// it low. All of this runs in TIM1's interrupts, which share one priority with the update interrupt (they never
// interrupt each other).

#define WAKE_AHEAD 32768u
#define PULSE      (HWPPS_F / 10)

typedef enum { OUT_LOW, OUT_ARMED, OUT_HIGH } out_state;

static hwpps_core_t      core;
static volatile uint32_t ovf;          // TIM1 updates (the count's upper bits)
static out_state         st = OUT_LOW;
static uint64_t          edge;         // the next edge
static uint64_t          last;         // the last edge made
static volatile uint32_t late;
static bool              started;
static volatile bool     force_req;

// the 64-bit count, from a TIM1 interrupt (or with interrupts off): an update not yet counted shows as UIF set
// with the count already past the wrap
static uint64_t now(void)
{
    uint32_t o = ovf;
    uint16_t c = TIM1->CNT;
    if ((TIM1->SR & TIM_SR_UIF) && c < 0x8000)
        o++;
    return ((uint64_t)o << 16) | c;
}

static void oc4_mode(uint32_t m) { TIM1->CCMR2 = (TIM1->CCMR2 & ~TIM_CCMR2_OC4M) | m; }
#define OC4_ACTIVE_ON_MATCH TIM_CCMR2_OC4M_0
#define OC4_FORCE_LOW       TIM_CCMR2_OC4M_2

// the PPS menu's settings: Sync (steer), Threshold (ns), Delay (s)
static void settings(void)
{
    core.steer = pps_sync_on;
    core.coarse = (int64_t)pps_sync_threshold * 7 / 100;
    core.coarse_s = pps_sync_delay;
}

static void wake_for(uint64_t e) { TIM1->CCR4 = (uint16_t)(e - WAKE_AHEAD); }

void hwpps_start(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = GPIO_PIN_11;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    hwpps_core_init(&core, 100.0);
    __disable_irq();
    // channel 4: output (CC4S 00), no preload (a new compare value counts at once), low
    TIM1->CCMR2 &= ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M);
    oc4_mode(OC4_FORCE_LOW);
    TIM1->CCER = (TIM1->CCER & ~TIM_CCER_CC4P) | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    edge = now() + HWPPS_F / 2;           // free running until the GPS pulse is in
    hwpps_core_start(&core, edge);
    wake_for(edge);
    st = OUT_LOW;
    started = true;
    TIM1->SR = ~TIM_SR_CC4IF;
    TIM1->DIER |= TIM_DIER_CC4IE;
    __enable_irq();
}

void hwpps_overflow(void) { ovf++; }

void hwpps_force(void) { force_req = true; }

void hwpps_gps(uint16_t capture)
{
    if (!started) return;
    uint64_t n = now();
    // the capture's full count: it was taken less than a timer period ago (the interrupt runs well within that)
    uint64_t g = n - (uint16_t)((uint16_t)n - capture);
    if (force_req) { force_req = false; hwpps_core_force(&core); }
    hwpps_core_gps(&core, g);
}

void hwpps_cc4(void)
{
    if (!started) return;
    uint64_t n = now();
    switch (st) {
        case OUT_ARMED:
            if ((int64_t)(n - edge) < 0) return;   // not the edge (can't happen: armed within one period)
            last = edge;
            settings();
            edge = hwpps_core_edge(&core, last);
            wake_for(edge);                        // the pin stays high ("active on match" again changes nothing)
            st = OUT_HIGH;
            break;
        case OUT_HIGH:
            if (n - last >= PULSE) {
                oc4_mode(OC4_FORCE_LOW);
                st = OUT_LOW;
            }
            break;
        case OUT_LOW:
            switch (hwpps_core_arm(n, edge)) {
                case HWPPS_ARM:
                    TIM1->CCR4 = (uint16_t)edge;
                    oc4_mode(OC4_ACTIVE_ON_MATCH);
                    st = OUT_ARMED;
                    break;
                case HWPPS_LATE:                   // an interrupt held this one off too long: skip a second
                    late++;
                    last = edge;
                    settings();
                    edge = hwpps_core_edge(&core, last);
                    wake_for(edge);
                    break;
                default:
                    break;
            }
            break;
    }
}

bool     hwpps_locked(void) { return core.state == HWPPS_LOCKED; }
int32_t  hwpps_err_ns(void) { return (int32_t)(core.err_avg * 1000.0 / 70.0); }
int32_t  hwpps_err_now_ns(void) { return (int32_t)((int64_t)core.err * 100 / 7); }
uint32_t hwpps_jumps(void) { return core.jumps; }
uint32_t hwpps_late(void) { return late; }
uint32_t hwpps_holdover_s(void) { return core.holdover_s; }
