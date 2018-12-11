#include <string.h>

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>

#include <os/os.h>

#include <decadriver/deca_device_api.h>

#include <decartls/instance.h>


/* NOTE: we only use GPIOA */
#define DECA_IRQ_PIN    GPIO3
#define DECA_IRQ_EXTI   EXTI3
#define DECA_IRQ        NVIC_EXTI2_3_IRQ
#define DECA_IRQ_ISR    exti2_3_isr

#define DECA_RST_PIN    GPIO2

#define LTC_PG_PIN      GPIO0

#define DECA_NODE_MODE  1 /* Mode 0 has lower power than mode 1 according to datasheet, but mode 1 consumes much less energy */
#define DECA_NODE_ADDR  0x00

/* NOTE: although BSS is only around 1KB, the application needs quite some stack for packet handling. Minimum requirement would be a STM32L031 */


static const instanceConfig_t iconfig = {
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

static const sfConfig_t sconfig = {
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
  .pollTxToFinalTxDly_us  = 2700, /* Was 2500, slacked a bit (should not exceed 5000). Debug with 'b instance_tag.c:244' */
#endif
};

static dwt_local_data_t dw1000local;
static instance_data_t  instance;


/* Wake-up is triggered when NSS driven low (here by enabling SPI), but oscillator needs 5ms to stabilize. */
static void __attribute__((unused)) wakeup_chip(void) {
  spi_enable(SPI1);
  tick_sleep(1);
  spi_disable(SPI1);
  tick_sleep(7);
}

/* DW1000 mode 0, < 3 MHz before PLL lock, < 20 MHz after PLL lock */
/* Always run SPI as fast as we can, tune system clock instead to conserve power */
/* TODO: FIFO and interrupts (but libopencm3 do not sleep during SPI operation anyways) */
static void spi_setup(unsigned clk_div) {
  rcc_periph_clock_enable(RCC_SPI1);
  spi_reset(SPI1);
  spi_init_master(SPI1, clk_div, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_disable_software_slave_management(SPI1);
  spi_enable_ss_output(SPI1);
  spi_disable_crc(SPI1);
  spi_set_full_duplex_mode(SPI1);
  spi_set_unidirectional_mode(SPI1);
  spi_set_dff_8bit(SPI1);
}

static void wait_pgood(void) {
  rcc_periph_clock_enable(RCC_GPIOA);
#if 1
  while (!gpio_get(GPIOA, LTC_PG_PIN)) {
    asm("wfi");
  }
#else
  /* For debugging start-up process power consumption */
  while (tick_get_uptime() < 2000);
#endif
  rcc_periph_clock_disable(RCC_GPIOA);
}


int main(void) {
  os_init(); /* NOTE: system clock is now 4MHz */

  rcc_periph_clock_enable(RCC_GPIOA);

  /* NOTE: DW1000 still consumes 4mA even when RST driven down. Needs to enter deep sleep ASAP by finishing instance_init(). */

  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, DECA_IRQ_PIN); /* TODO: use external pull-down with higher resistance? may not help much */
  nvic_set_priority(DECA_IRQ, 60);
  nvic_enable_irq(DECA_IRQ);
  exti_select_source(DECA_IRQ_EXTI, GPIOA);
  exti_set_trigger(DECA_IRQ_EXTI, EXTI_TRIGGER_RISING);
  exti_enable_request(DECA_IRQ_EXTI);

  gpio_set_af(GPIOA, GPIO_AF0, GPIO4 | GPIO5 | GPIO6 | GPIO7);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO4 | GPIO5 | GPIO6 | GPIO7);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5 | GPIO6 | GPIO7);

  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, LTC_PG_PIN);
  //rcc_periph_clock_disable(RCC_GPIOA); /* wait_pgood() does this for us */

  dwt_setdevicedataptr(&dw1000local);
  spi_setup(SPI_CR1_BAUDRATE_FPCLK_DIV_2);
  /*
  wakeup_chip();
  if (DWT_DEVICE_ID != instance_readdeviceid()) {
    os_panic();
  }
  */
  dwt_softreset(); /* Clear sleep bit if any, as good as hardware reset in most cases */
  if (instance_init() != 0) {
    os_panic();
  }
  instance_set_16bit_address(DECA_NODE_ADDR);
  instance_config(&iconfig, &sconfig);
#if DEEP_SLEEP == 1
  tag_run(); /* Run once to complete tag INIT and enter deep sleep */
#endif

  rcc_periph_clock_disable(RCC_SPI1); /* Shouldn't have communication for a while */

  wait_pgood();
  os_dvfs_hsi16();

  while (true) {
    wait_pgood();

    rcc_periph_clock_enable(RCC_SPI1);
    exti_enable_request(DECA_IRQ_EXTI);
    do {
      tag_run();
    } while (instance.testAppState != TA_SLEEP_DONE);
    exti_disable_request(DECA_IRQ_EXTI); /* Make sure nothing interrupts sleep :) */
    rcc_periph_clock_disable(RCC_SPI1);

    os_pm_sleep_until(instance.instanceWakeTime_ms + instance.tagPeriod_ms - 1);
  }

  return 0;
}

/* Implementations of low-level deca_device_api */

static void spi_rx(uint32_t spi, uint8_t *rxbuf, uint32_t len) {
  uint32_t i;

  for (i = 0; i < len; i ++) {
    rxbuf[i] = spi_xfer(spi, 0x00); /* Need to feed to get */
  }
}

static void spi_tx(uint32_t spi, const uint8_t *txbuf, uint32_t len) {
  uint32_t i;

  for (i = 0; i < len; i ++) {
    spi_xfer(spi, txbuf[i]); /* One in, one out. Read it is critical, even if it is garbage. */
  }
  return;
}

/* A bit different from spi_clean_disable() */
static void spi_disable_after_io(uint32_t spi) {
  while (!(SPI_SR(spi) & SPI_SR_TXE));
  while (SPI_SR(spi) & SPI_SR_BSY);
  SPI_CR1(spi) &= ~SPI_CR1_SPE;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer) {
  spi_enable(SPI1);
  spi_tx(SPI1, headerBuffer, headerLength);
  spi_tx(SPI1, bodyBuffer, bodylength);
  spi_disable_after_io(SPI1);

  return DWT_SUCCESS;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
  spi_enable(SPI1);
  spi_tx(SPI1, headerBuffer, headerLength);
  spi_rx(SPI1, readBuffer, readlength);
  spi_disable_after_io(SPI1);

  return DWT_SUCCESS;
}

/* Does not seem to be useful */
decaIrqStatus_t decamutexon(void) {
  //exti_disable_request(DECA_IRQ_EXTI);
  return 0;
}

void decamutexoff(__attribute__((unused)) decaIrqStatus_t s) {
  //exti_enable_request(DECA_IRQ_EXTI);
}

void deca_sleep(unsigned int time_ms) {
  tick_sleep(time_ms);
}

/* Implementations of deca_rtls_port */

void port_wakeup_dw1000_fast(void) {
  spi_enable(SPI1);
  tick_sleep(1);
  spi_disable(SPI1);
  tick_delay_us(35);
}

instance_data_t* instance_get_local_structure_ptr(__attribute__((unused)) unsigned int x) {
  return &instance;
}


/* IRQ handler */
void DECA_IRQ_ISR(void) {
  exti_reset_request(DECA_IRQ_EXTI);
  dwt_isr();
}
