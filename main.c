#include <string.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>

#include <os/os.h>

#include <decartls/instance.h>

#include <deca_ctl.h>


#define DECA_RSTN_PORT  GPIOA
#define DECA_RSTN_PIN   GPIO0
#define DECA_NODE_MODE  0 /* Lower power than mode 1 according to datasheet */
#define DECA_NODE_ADDR  0x00

/* TODO: see whether we can squeeze in STM32L011F4 (16K/2K) */

const instanceConfig_t iconfig = {
#if DECA_NODE_MODE == 0
  /* TREK1000 mode 1 */
  .channelNumber  = 2,
  .preambleCode   = 4,
  .pulseRepFreq   = DWT_PRF_16M,
  .dataRate       = DWT_BR_110K,
  .preambleLen    = DWT_PLEN_1024,
  .pacSize        = DWT_PAC32,
  .nsSFD          = 1,
  .sfdTO          = 1025 + 64 - 32,
#else
  /* TREK1000 mode 2 */
  .channelNumber  = 2,
  .preambleCode   = 4,
  .pulseRepFreq   = DWT_PRF_16M,
  .dataRate       = DWT_BR_6M8,
  .preambleLen    = DWT_PLEN_128,
  .pacSize        = DWT_PAC8,
  .nsSFD          = 0,
  .sfdTO          = 129 + 8 - 8,
#endif
};

const sfConfig_t sconfig = {
#if DECA_NODE_MODE == 0
  /* TREK1000 mode 1 */
  .slotDuration_ms        = 28,
  .numSlots               = 10,
  .sfPeriod_ms            = 10 * 28, /* Localization rate */
  .tagPeriod_ms           = 10 * 28,
  .pollTxToFinalTxDly_us  = 20000
#else
  /* TREK1000 mode 2 */
  .slotDuration_ms        = 10,
  .numSlots               = 10,
  .sfPeriod_ms            = 10 * 10, /* Localization rate */
  .tagPeriod_ms           = 10 * 10,
  .pollTxToFinalTxDly_us  = 2500,
#endif
};


/* Reset is global, which can be a problem... */
/* e.g. "When in DEEPSLEEP mode, the DW1000 drives RSTn to ground" */
static void hw_reset(void) {
  decaIrqStatus_t s;

  s = decamutexon(); /* Avoid glitch from cause IRQs */
  gpio_clear(DECA_RSTN_PORT, DECA_RSTN_PIN);
  tick_sleep(1); /* NOTE: our reset trace is super long, but 1 millisecond should do it */
  gpio_set(DECA_RSTN_PORT, DECA_RSTN_PIN);
  tick_sleep(2);
  decamutexoff(s);
}

/* Wake-up is triggered when nSS driven low, but oscillator needs 5ms to stabilize. */
static void wakeup_chip(void) {
  gpio_clear(GPIOA, GPIO4);
  tick_sleep(1);
  gpio_set(GPIOA, GPIO4);
  tick_sleep(7);
}

static void multispi_config_spi(uint32_t spi, uint32_t clkdiv) {
  /* TODO: FIFO and interrupts */
  spi_init_master(spi, clkdiv, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_software_slave_management(spi);
  spi_set_nss_high(spi);
  spi_disable_crc(spi);
  spi_set_full_duplex_mode(spi);
  spi_set_unidirectional_mode(spi);
  spi_set_dff_8bit(spi);
  spi_enable(spi);
}

static void multispi_setup(void) {
  rcc_periph_clock_enable(RCC_SPI1);

  gpio_set_af(GPIOA, GPIO_AF0, GPIO5 | GPIO6 | GPIO7);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO5 | GPIO6 | GPIO7);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);

  /* SPI1 on APB2 (16 MHz), DW1000 mode 0, 2 MHz (< 3 MHz before PLL lock, < 20 MHz after PLL lock) */
  multispi_config_spi(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8);
}

static void multispi_speedup(void) {
  rcc_periph_reset_pulse(RST_SPI1);

  /* SPI1 on APB2 (16 MHz), DW1000 mode 0, 8 MHz (< 3 MHz before PLL lock, < 20 MHz after PLL lock) */
  multispi_config_spi(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2);
}


static void wait_pgood(void) {

}


int main(void) {
  int ret;
  decaIrqStatus_t s;

  os_init();

  /* This is important for at least init */
  s = decamutexon();

  gpio_set(DECA_RSTN_PORT, DECA_RSTN_PIN);
  gpio_set_output_options(DECA_RSTN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, DECA_RSTN_PIN); /* NOTE: cannot be pulled-up according to DS */
  gpio_mode_setup(DECA_RSTN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DECA_RSTN_PIN);

  deca_ctl_init_entry();
  multispi_setup();

  wakeup_chip();
  if (DWT_DEVICE_ID != instance_readdeviceid()) {
    os_panic();
  }
  dwt_softreset(); /* Clear sleep bit if any */

  hw_reset();

  ret = instance_init(TAG);
  if (ret != 0) {
    os_panic();
  }
  instance_set_16bit_address(DECA_NODE_ADDR);
  instance_config(&iconfig, &sconfig);

  multispi_speedup();

  decamutexoff(s);

  while (true) {
    tag_run();
    asm("wfi");
  }

  return 0;
}
