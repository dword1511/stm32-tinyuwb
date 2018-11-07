#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include <libopencmsis/core_cm3.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>

#include <os/os.h>

#include <decartls/instance.h>
#include <decadriver/deca_device_api.h>

#include <deca_ctl.h>


static bool             irq_enabled = true;
static instance_data_t  instance;
static dwt_local_data_t dw1000local;


void deca_ctl_init_entry(void) {
  memset(&instance, 0, sizeof(instance));
  memset(&dw1000local, 0, sizeof(dw1000local));

  gpio_set(GPIOA, GPIO4); /* Do not forget to disable those chips all before they get fighting on the bus... */
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO4); /* Using the same speed as data lines to ensure timing */
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);

  /* PA1 as IRQ pin */
  gpio_clear(GPIOA, GPIO1);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO1);
  nvic_set_priority(NVIC_EXTI0_1_IRQ, 60);
  nvic_enable_irq(NVIC_EXTI0_1_IRQ);
  exti_select_source(EXTI1, GPIOA);
  exti_set_trigger(EXTI1, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI1);

  dwt_setdevicedataptr(&dw1000local);
}

static inline void multispi_io(uint32_t spi, const uint8_t *txbuf, uint8_t *rxbuf, uint32_t len) {
  /* RX only */
  if ((txbuf == NULL) && (rxbuf != NULL)) {
    uint32_t i;
    for (i = 0; i < len; i ++) {
      rxbuf[i] = spi_xfer(spi, 0x00); /* Need to feed to get */
    }
    return;
  }

  /* TX only */
  if ((txbuf != NULL) && (rxbuf == NULL)) {
    uint32_t i;
    for (i = 0; i < len; i ++) {
      spi_xfer(spi, txbuf[i]); /* One in, one out. Read it is critical, even if it is garbage. */
    }
    return;
  }

  /* Full-duplex */
  if ((txbuf != NULL) && (rxbuf != NULL)) {
    uint32_t i;
    for (i = 0; i < len; i ++) {
      rxbuf[i] = spi_xfer(spi, txbuf[i]);
    }
    return;
  }

  /* Error */
  os_panic();
}

/* Implementations of low-level deca_device_api */
/* __builtin_return_address() only accepts 0 on ARM, which is not useful here */

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer) {
  gpio_clear(GPIOA, GPIO4);
  multispi_io(SPI1, headerBuffer, NULL, headerLength);
  multispi_io(SPI1, bodyBuffer, NULL, bodylength);
  gpio_set(GPIOA, GPIO4);

  return DWT_SUCCESS;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
  gpio_clear(GPIOA, GPIO4);
  /* NOTE: DW1000 is half-duplex */
  multispi_io(SPI1, headerBuffer, NULL, headerLength);
  multispi_io(SPI1, NULL, readBuffer, readlength);
  gpio_set(GPIOA, GPIO4);

  return DWT_SUCCESS;
}

/* Track these closely, we should not need these if the deca system is implemented correctly. These are sources of bugs and errors. */
/* NOTE: these can be called from interrupt context as well */
decaIrqStatus_t decamutexon(void) {
  decaIrqStatus_t old_state = irq_enabled;
  irq_enabled = false;
  return old_state;
}

void decamutexoff(__attribute__((unused)) decaIrqStatus_t s) {
  irq_enabled = (bool)s;
}

void deca_sleep(unsigned int time_ms) {
  tick_sleep(time_ms);
}

instance_data_t* instance_get_local_structure_ptr(__attribute__((unused)) unsigned int x) {
  return &instance;
}

/* IRQ handler */
void exti0_1_isr(void) {
  exti_reset_request(EXTI1);
  dwt_isr();
}
