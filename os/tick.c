/* Universal tick driver for ARM CPUs */

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>

#include <os/os.h>
#include <os/tick.h>


#define SYSCLK_PERIOD_MS  (1000 / (TICK_HZ))


static volatile uint32_t  uptime_ms = 0;
static          unsigned  cycle_per_us = 0;


void sys_tick_handler(void) {
  uptime_ms += SYSCLK_PERIOD_MS;
}

void tick_setup(void) {
  unsigned period = (rcc_ahb_frequency) * (SYSCLK_PERIOD_MS) / 1000 - 1;

  systick_interrupt_disable();

  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_clear();

  nvic_set_priority(NVIC_SYSTICK_IRQ, 80);
  systick_set_reload(period);
  systick_interrupt_enable();
  systick_counter_enable();

  cycle_per_us = rcc_ahb_frequency / 1000000;
  sys_tick_handler(); /* FIXME: workaround for missing ticks due to DVFS */
}

/* NOTE: Warps every 49 days... */
volatile uint32_t tick_get_uptime(void) {
  return uptime_ms;
}

void tick_sleep_until(uint32_t target_ms) {
  while (uptime_ms < target_ms) {
    asm("wfi");
  }
}

void tick_sleep(uint32_t ms) {
  uint32_t t_entry = uptime_ms;
  uint32_t target = t_entry + ms;

  tick_sleep_until(target);
}

__attribute__((optimize("unroll-loops")))
void tick_delay_us(uint32_t us) {
  uint32_t i, j;

  for (i = 0; i < us; i ++) {
    for (j = 0; j < cycle_per_us; j ++) {
      asm("nop");
    }
  }
}
