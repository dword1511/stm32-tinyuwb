#include <stdbool.h>

#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

#include <os/os.h>


void os_init(void) {
  /* Configure GPIO for LP */
  rcc_periph_clock_enable(RCC_GPIOA);
  //gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL & (~(GPIO13 | GPIO14))); /* Keep SWD active during development */
  /* Not powering it down since we need RST, SS, etc. */
  //
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  rcc_periph_clock_disable(RCC_GPIOB);
  //
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  rcc_periph_clock_disable(RCC_GPIOC);

  /* MCU is on HSI after reset */
  RCC_ICSCR = (RCC_ICSCR & (~(RCC_ICSCR_MSIRANGE_MASK << RCC_ICSCR_MSIRANGE_SHIFT))) | (RCC_ICSCR_MSIRANGE_4MHZ << RCC_ICSCR_MSIRANGE_SHIFT);

  rcc_periph_clock_enable(RCC_PWR);
  pwr_set_vos_scale(PWR_SCALE3);
  pwr_voltage_regulator_low_power_in_stop();
  PWR_CR |= PWR_CR_LPSDSR;
  //PWR_CR |= PWR_CR_LPRUN; /* NOTE: LPRUN is for kHz only */
  flash_prefetch_enable();
  flash_set_ws(FLASH_ACR_LATENCY_0WS);
  rcc_periph_clock_disable(RCC_PWR);

  rcc_set_hpre(RCC_CFGR_HPRE_NODIV); /* Actually very marginal, cannot afford to be any slower */
  rcc_set_ppre1(RCC_CFGR_PPRE1_NODIV); /* We are not using anything on it, the bridge should be off */
  rcc_set_ppre2(RCC_CFGR_PPRE2_NODIV);

  rcc_ahb_frequency   = 4194000;
  rcc_apb1_frequency  = 4194000;
  rcc_apb2_frequency  = 4194000;

  tick_setup();
}

void os_halt(void) {
  rcc_periph_clock_enable(RCC_PWR);
  pwr_disable_power_voltage_detect();
  pwr_disable_wakeup_pin();

  /* According to ST App note... */
  SCB_SCR |= SCB_SCR_SLEEPDEEP;
  /* Only sets PDDS... Well, others are universal to CM3 anyway. */
  pwr_set_standby_mode();
  pwr_clear_wakeup_flag();
  /* Enter standby */
  asm("wfi");
}

void os_panic(void) {
  while (true) {
    asm("nop");
  }
}

void nmi_handler(void)
__attribute__ ((alias ("os_panic")));

void hard_fault_handler(void)
__attribute__ ((alias ("os_panic")));
