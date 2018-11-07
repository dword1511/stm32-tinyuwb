#include <stdbool.h>

#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/gpio.h>

#include <os/os.h>


void os_init(void) {
  /* To run SPI1 @ 16MHz we can only run at maximum speed */

  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  /* Not powering it down since we need RST, SS, etc. */

  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  rcc_periph_clock_disable(RCC_GPIOB);

  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  rcc_periph_clock_disable(RCC_GPIOC);

  tick_setup();
}

void os_halt(void) {
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
