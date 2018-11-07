#include <string.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>

#include <os/os.h>

#include <decartls/instance.h>

#include <deca_ctl.h>


#define DECA_RSTN_PORT  GPIOA
#define DECA_RSTN_PIN   GPIO0
#define DECA_NODE_MODE  0
#define DECA_NODE_ADDR  0x00


/* In instance_modes.c, was in dw_main.c */
extern instanceConfig_t chConfig[4];
extern sfConfig_t sfConfig[4];


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

static void check_chip(void) {
  uint32_t dev_id;

  dev_id = dwt_readdevid();
  if (DWT_DEVICE_ID != dev_id) {
    if ((dev_id == 0) || (dev_id == 0xffffffff)) {
      os_panic();
    }
  }
}

/* Handling each chip */

/* Wake-up is triggered when nSS driven low, but oscillator needs 5ms to stabilize. */
static void wakeup_one_chip(void) {
  gpio_clear(GPIOA, GPIO4);
  tick_sleep(1);
  gpio_set(GPIOA, GPIO4);
  tick_sleep(7);
}

static void probe_one_chip(void) {
  uint32_t dev_id;

  wakeup_one_chip();
  dev_id = instance_readdeviceid();
  if (DWT_DEVICE_ID != dev_id) {
    os_panic();
  }
  dwt_softreset(); /* Clear sleep bit if any */
}

static void init_one_chip(void) {
  int ret;

  ret = instance_init(TAG);
  if (ret != 0) {
    os_panic();
  }

  instance_set_16bit_address(DECA_NODE_ADDR);
  instance_config(&chConfig[DECA_NODE_MODE], &sfConfig[DECA_NODE_MODE]);
}


/* Main loop from dw_main.c */
static void range_one_chip(void) {
  instance_data_t *inst = instance_get_local_structure_ptr(0);
  int monitor_local = inst->monitor;
  int txdiff = tick_get_uptime() - (inst->timeofTx);

  tag_run();

  /* Original comments:
   * if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
   * if anchor just go into RX and wait for next message from tags/anchors
   * if tag handle as a timeout
   * NOTE: in this system tick is 10 ms rather than 1 ms, thus detection of such event would be delayed.
   */
  /* TODO: this check does not work at all --- have to debug instance codes manually */
  if ((monitor_local == 1) && (txdiff > (inst->slotDuration_ms))) {
    inst->wait4ack = 0;
    tag_process_rx_timeout(inst);
    inst->monitor = 0;
  }

  instance_newrange();
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
  decaIrqStatus_t s;

  os_init();

  /* This is important for at least init */
  s = decamutexon();

  gpio_set(DECA_RSTN_PORT, DECA_RSTN_PIN);
  gpio_set_output_options(DECA_RSTN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, DECA_RSTN_PIN); /* NOTE: cannot be pulled-up according to DS */
  gpio_mode_setup(DECA_RSTN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DECA_RSTN_PIN);

  deca_ctl_init_entry();
  multispi_setup();
  probe_one_chip();
  hw_reset();
  init_one_chip();

  multispi_speedup();

  decamutexoff(s);

  while (true) {
    range_one_chip();
  }

  return 0;
}
