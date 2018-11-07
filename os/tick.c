/* Universal tick driver for ARM CPUs */

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/rcc.h>

#include <os/os.h>
#include <os/tick.h>


#define TICK_HZ           1000 /* Use 1000 to keep it simple and the same as reference code. */

#define SYSCLK_PERIOD_MS  (1000 / (TICK_HZ))
#define SYSCLK_PERIOD     ((rcc_ahb_frequency) * (SYSCLK_PERIOD_MS) / 1000 - 1)


static volatile uint32_t  uptime_ms = 0;


void sys_tick_handler(void) {
  uptime_ms += SYSCLK_PERIOD_MS;
}

void tick_setup(void) {
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_clear();

  nvic_set_priority(NVIC_SYSTICK_IRQ, 80);
  systick_set_reload(SYSCLK_PERIOD);
  systick_interrupt_enable();
  systick_counter_enable();
}

/* NOTE: Warps every 49 days... */
uint32_t tick_get_uptime(void) {
  return uptime_ms;
}

void tick_sleep(uint32_t ms) {
  uint32_t t_entry = uptime_ms;
  uint32_t target = t_entry + ms;

  while (uptime_ms < target) {
    asm("wfi");
  }
}
