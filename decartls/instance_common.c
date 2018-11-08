#include <string.h>
#include <math.h> /* Big no no unless you are using Cortex-M4F */

#include <os/os.h>

#include <decadriver/deca_device_api.h>

#include <decartls/instance.h>

/* instance_calib.c */
extern const uint16 rfDelays[2];
extern const uint16 rfDelaysTREK[2];
extern const tx_struct txSpectrumConfig[8];
/* deca_params_init.c */
extern uint8 dwnsSFDlen[];
extern uint8 chan_idx[];


static uint64 instance_convert_usec_to_devtimeu(float microsecu) {
  /* NOTE: float to uint64_t calls __aeabi_f2ulz(), which then calls __aeabi_dsub()/__aeabi_d2uiz(). */
  /* To avoid double-precision float points, we need to eliminate this stuff */
  uint32_t hi, lo;
  microsecu = (microsecu / DWT_TIME_UNITS) / 1e6f;
  hi = microsecu / ((1 << 31) * 2.0f);
  lo = microsecu;
  return ((uint64_t)hi) << 32 | lo;
}

static float calc_length_data(float msgdatalen) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  int x = ceil(msgdatalen * 8 / 330.0f);

  msgdatalen = msgdatalen * 8 + x * 48;

  /* assume PHR length is 172308ns for 110k and 21539ns for 850k/6.81M */
  if (inst->configData.dataRate == DWT_BR_110K) {
    msgdatalen *= 8205.13f;
    msgdatalen += 172308;

  } else if (inst->configData.dataRate == DWT_BR_850K) {
    msgdatalen *= 1025.64f;
    msgdatalen += 21539;
  } else {
    msgdatalen *= 128.21f;
    msgdatalen += 21539;
  }

  return msgdatalen;
}

static void instance_set_replydelay(int delayus) {
  instance_data_t *inst = instance_get_local_structure_ptr(0);

  int margin = 2000;
  int respframe = 0;
  int respframe_sy = 0;
  int pollframe_sy = 0;
  float msgdatalen_resp = 0;
  float msgdatalen_poll = 0;
  float preamblelen = 0;
  int sfdlen = 0;

  msgdatalen_resp = calc_length_data(ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
  msgdatalen_poll = calc_length_data(TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);

  sfdlen = dwnsSFDlen[inst->configData.dataRate];

  /* TODO: make it a look-up table rather than this... */
  switch (inst->configData.txPreambLength) {
    case DWT_PLEN_4096:
      preamblelen = 4096.0f;
      break;
    case DWT_PLEN_2048:
      preamblelen = 2048.0f;
      break;
    case DWT_PLEN_1536:
      preamblelen = 1536.0f;
      break;
    case DWT_PLEN_1024:
      preamblelen = 1024.0f;
      break;
    case DWT_PLEN_512:
      preamblelen = 512.0f;
      break;
    case DWT_PLEN_256:
      preamblelen = 256.0f;
      break;
    case DWT_PLEN_128:
      preamblelen = 128.0f;
      break;
    case DWT_PLEN_64:
      preamblelen = 64.0f;
      break;
  }

  if (inst->configData.prf == DWT_PRF_16M) {
    preamblelen = (sfdlen + preamblelen) * 0.99359f;
  } else {
    preamblelen = (sfdlen + preamblelen) * 1.01763f;
  }

  respframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_resp + margin) / 1000.0)) / 1.0256));
  pollframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_poll + margin) / 1000.0)) / 1.0256));

  inst->pollTx2FinalTxDelay = instance_convert_usec_to_devtimeu(delayus);

  respframe = (int)(preamblelen + (msgdatalen_resp / 1000.0));
  if (inst->configData.dataRate == DWT_BR_110K) {
    inst->preambleDuration32h = (uint32) (((uint64) instance_convert_usec_to_devtimeu (preamblelen)) >> 8) + DW_RX_ON_DELAY;
  } else {
    inst->preambleDuration32h = (uint32) (((uint64) instance_convert_usec_to_devtimeu (preamblelen)) >> 8) + DW_RX_ON_DELAY;
  }

  inst->tagRespRxDelay_sy = RX_RESPONSE_TURNAROUND + respframe_sy - pollframe_sy;
  inst->fixedReplyDelayAnc32h = ((uint64)instance_convert_usec_to_devtimeu (respframe + RX_RESPONSE_TURNAROUND) >> 8);
  inst->fwto4RespFrame_sy = respframe_sy;
}

int instance_init(void) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  inst->testAppState  = TA_INIT;
  inst->instToSleep   = FALSE;

  if (DWT_SUCCESS != dwt_initialise(DWT_LOADUCODE)) {
    return -1;
  }

  /* Enable TXLED and RXLED. NOTE: DW1000 will blink LEDs once during power-on */
#if ENABLE_LEDS
  dwt_setleds(3);
#endif

  inst->frameSN = 0;
  inst->wait4ack = 0;
  inst->instanceTimerEn = 0;

  instance_clearevents();

  memset(inst->eui64, 0, ADDR_BYTE_SIZE_L);
  inst->panID = 0xdeca;
  inst->tagSleepCorrection_ms = 0;

  dwt_setdblrxbuffmode(0);

  /* if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
   * other errors which need to be checked (as they disable receiver) are
   */
  //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);
  dwt_setcallbacks(tx_conf_cb, rx_ok_cb_tag, rx_err_cb_tag, rx_err_cb_tag);

  inst->remainingRespToRx = -1;

  dwt_setlnapamode(1, 1);

  inst->delayedTRXTime32h = 0;

  return 0;
}

/* OTP memory addresses for TREK calibration data */
#define TXCFG_ADDRESS  (0x10)
#define ANTDLY_ADDRESS (0x1c)
#define TREK_ANTDLY_1  (0x0d)
#define TREK_ANTDLY_2  (0x0e)
#define TREK_ANTDLY_3  (0x0f)
#define TREK_ANTDLY_4  (0x1d)

void instance_config(const instanceConfig_t *config, const sfConfig_t *sfConfig) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  uint32 power = 0;
  uint8 otprev;

  inst->configData.chan = config->channelNumber;
  inst->configData.rxCode =  config->preambleCode;
  inst->configData.txCode = config->preambleCode;
  inst->configData.prf = config->pulseRepFreq;
  inst->configData.dataRate = config->dataRate;
  inst->configData.txPreambLength = config->preambleLen;
  inst->configData.rxPAC = config->pacSize;
  inst->configData.nsSFD = config->nsSFD;
  inst->configData.phrMode = DWT_PHRMODE_STD;
  inst->configData.sfdTO = config->sfdTO;

  dwt_configure(&inst->configData);

  /* NOTE: main.c ensures this function is only called when SPI is slow */
  dwt_otpread(TXCFG_ADDRESS+(config->pulseRepFreq - DWT_PRF_16M) + (chan_idx[inst->configData.chan] * 2), &power, 1);
  if ((power == 0x0) || (power == 0xffffffff)) {
    /* No cal found, load defaults */
    power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
  }

  inst->configTX.power = power;
  inst->configTX.PGdly = txSpectrumConfig[config->channelNumber].pgDelay;
  dwt_configuretxrf(&inst->configTX);

  otprev = dwt_otprevision();

  if ((2 == otprev) || (3 == otprev)) {
    /* board is calibrated with TREK1000 with antenna delays set for each use case) */
    uint8 mode = 0; /* TAG */
    uint32 dly = 0;

    switch(inst->configData.chan) {
      case 2: {
        if (inst->configData.dataRate == DWT_BR_6M8) {
          dwt_otpread(TREK_ANTDLY_1, &dly, 1);
        } else if (inst->configData.dataRate == DWT_BR_110K) {
          dwt_otpread(TREK_ANTDLY_2, &dly, 1);
        }
        break;
      }

      case 5: {
        if (inst->configData.dataRate == DWT_BR_6M8) {
          dwt_otpread(TREK_ANTDLY_3, &dly, 1);
        } else if (inst->configData.dataRate == DWT_BR_110K) {
          dwt_otpread(TREK_ANTDLY_4, &dly, 1);
        }
        break;
      }

      default: {
        dly = 0;
        break;
      }
    }

    if ((dly == 0) || (dly == 0xffffffff)) {
      uint8 chanindex = (inst->configData.chan == 5) ? 1 : 0;
      inst->txAntennaDelay = rfDelaysTREK[chanindex];
    } else {
      inst->txAntennaDelay = (dly >> (16 * (mode & 0x1))) & 0xFFFF;
    }
  } else {
    uint32 antennaDly;

    dwt_otpread(ANTDLY_ADDRESS, &antennaDly, 1);
    if ((antennaDly == 0) || (antennaDly == 0xffffffff)) {
      inst->txAntennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
    } else {
      inst->txAntennaDelay = ((antennaDly >> (16 * (inst->configData.prf-DWT_PRF_16M))) & 0xFFFF) >> 1;
    }
  }

  dwt_setrxantennadelay(inst->txAntennaDelay);
  dwt_settxantennadelay(inst->txAntennaDelay);

  if (config->preambleLen == DWT_PLEN_64) {
    dwt_loadopsettabfromotp(0);
  }

  inst->tagPeriod_ms = sfConfig->tagPeriod_ms;
  inst->tagSleepRnd_ms = sfConfig->slotDuration_ms;
  instance_set_replydelay(sfConfig->pollTxToFinalTxDly_us);
}

void instance_set_16bit_address(uint16 address) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  inst->instanceAddress16 = address;
}

void instance_config_frameheader_16bit(instance_data_t *inst) {
  /* set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6) */
  inst->msg_f.frameCtrl[0] = 0x1 /* frame type 0x1 == data */ | 0x40 /* PID comp */;
  inst->msg_f.frameCtrl[1] = 0x8 /* dest extended address (16bits) */ | 0x80 /* src extended address (16bits) */;

  inst->msg_f.panID[0] = (inst->panID) & 0xff;
  inst->msg_f.panID[1] = inst->panID >> 8;

  inst->msg_f.seqNum = 0;
}

int instance_send_delayed_frame(instance_data_t *inst, int delayedTx) {
  int result = 0;

  dwt_writetxfctrl(inst->psduLength, 0, 1);
  if (delayedTx == DWT_START_TX_DELAYED) {
    dwt_setdelayedtrxtime(inst->delayedTRXTime32h);
  }

  if (dwt_starttx(delayedTx | inst->wait4ack)) {
    /* delayed start was too late */
    result = 1;
  }
  return result;
}

void tx_conf_cb(const dwt_cb_data_t *txd) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};
  event_data_t dw_event;

  dwt_readtxtimestamp(txTimeStamp);
  instance_seteventtime(&dw_event, txTimeStamp);

  dw_event.rxLength = inst->psduLength;
  dw_event.type =  0;
  dw_event.typePend =  0;

  memcpy((uint8 *)&dw_event.msgu.frame[0], (uint8 *)&inst->msg_f, inst->psduLength);

  instance_putevent(dw_event, DWT_SIG_TX_DONE);
}

void instance_seteventtime(event_data_t *dw_event, uint8* timeStamp) {
  dw_event->timeStamp32l =  (uint32)timeStamp[0] + ((uint32)timeStamp[1] << 8) + ((uint32)timeStamp[2] << 16) + ((uint32)timeStamp[3] << 24);
  dw_event->timeStamp = timeStamp[4];
  dw_event->timeStamp <<= 32;
  dw_event->timeStamp += dw_event->timeStamp32l;
  dw_event->timeStamp32h = ((uint32)timeStamp[4] << 24) + (dw_event->timeStamp32l >> 8);
}

int instance_peekevent(void) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  return inst->dwevent[inst->dweventIdxOut].type; //return the type of event that is in front of the queue
}

void instance_putevent(event_data_t newevent, uint8 etype) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  inst->dwevent[inst->dweventIdxIn] = newevent;
  inst->dwevent[inst->dweventIdxIn].type = etype;
  inst->dweventIdxIn ++;

  if (MAX_EVENT_NUMBER == inst->dweventIdxIn) {
    inst->dweventIdxIn = 0;
  }
}

event_data_t* instance_getevent(int x) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  int indexOut = inst->dweventIdxOut;

  if (inst->dwevent[indexOut].type == 0) {
    /* exit with "no event" */
    inst->dw_event_g.type = 0;
    return &(inst->dw_event_g);
  }

  memcpy(&(inst->dw_event_g), &(inst->dwevent[indexOut]), sizeof(inst->dw_event_g));
  memcpy(&(inst->dw_event_g.msgu), &inst->dwevent[indexOut].msgu, sizeof(inst->dwevent[indexOut].msgu));

  /* clear the event */
  inst->dwevent[indexOut].type = 0;

  inst->dweventIdxOut++;
  if (MAX_EVENT_NUMBER == inst->dweventIdxOut) {
    inst->dweventIdxOut = 0;
  }

  return &(inst->dw_event_g);
}

void instance_clearevents(void) {
  int i;
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  for(i = 0; i < MAX_EVENT_NUMBER; i++) {
    memset(&inst->dwevent[i], 0, sizeof(event_data_t));
  }

  inst->dweventIdxIn = 0;
  inst->dweventIdxOut = 0;
}

void instance_config_txpower(uint32 txpower) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  inst->txPower = txpower;
  inst->txPowerChanged = 1;
}

void instance_set_txpower(void) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  if (inst->txPowerChanged == 1) {
    dwt_write32bitreg(0x1e, inst->txPower);
    inst->txPowerChanged = 0;
  }
}
