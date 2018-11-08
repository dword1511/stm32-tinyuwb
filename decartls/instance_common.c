#include <string.h>
#include <math.h> /* Big no no unless you are using Cortex-M4F */

#include <os/os.h>

#include <decadriver/deca_device_api.h>

#include <decartls/instance.h>

#include <deca_ctl.h>

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

  int x = (int) ceil(msgdatalen * 8 / 330.0f);

  msgdatalen = msgdatalen * 8 + x * 48;

  //assume PHR length is 172308ns for 110k and 21539ns for 850k/6.81M
  if(inst->configData.dataRate == DWT_BR_110K)
  {
    msgdatalen *= 8205.13f;
    msgdatalen += 172308; // PHR length in nanoseconds

  }
  else if(inst->configData.dataRate == DWT_BR_850K)
  {
    msgdatalen *= 1025.64f;
    msgdatalen += 21539; // PHR length in nanoseconds
  }
  else
  {
    msgdatalen *= 128.21f;
    msgdatalen += 21539; // PHR length in nanoseconds
  }

  return msgdatalen;
}

static void instance_set_replydelay(int delayus) {
  instance_data_t *inst = instance_get_local_structure_ptr(0);

  int margin = 3000; //2000 symbols
  int respframe = 0;
  int respframe_sy = 0;
  int pollframe_sy = 0;

  //configure the rx delay receive delay time, it is dependent on the message length
  float msgdatalen_resp = 0;
  float msgdatalen_poll = 0;
  float preamblelen = 0;
  int sfdlen = 0;

  //Set the RX timeouts based on the longest expected message - the Final message
  //Poll = 13, Response = 20, Final = 44 bytes
  //msgdatalen = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
  msgdatalen_resp = calc_length_data(ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
  msgdatalen_poll = calc_length_data(TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);

  //SFD length is 64 for 110k (always)
  //SFD length is 8 for 6.81M, and 16 for 850k, but can vary between 8 and 16 bytes
  sfdlen = dwnsSFDlen[inst->configData.dataRate];

  /* TODO: make it a look-up table rather than this... */
  switch (inst->configData.txPreambLength)
  {
  case DWT_PLEN_4096 :
    preamblelen = 4096.0f;
    break;
  case DWT_PLEN_2048 :
    preamblelen = 2048.0f;
    break;
  case DWT_PLEN_1536 :
    preamblelen = 1536.0f;
    break;
  case DWT_PLEN_1024 :
    preamblelen = 1024.0f;
    break;
  case DWT_PLEN_512  :
    preamblelen = 512.0f;
    break;
  case DWT_PLEN_256  :
    preamblelen = 256.0f;
    break;
  case DWT_PLEN_128  :
    preamblelen = 128.0f;
    break;
  case DWT_PLEN_64   :
    preamblelen = 64.0f;
    break;
  }

  if(inst->configData.prf == DWT_PRF_16M) {
    preamblelen = (sfdlen + preamblelen) * 0.99359f;
  } else {
    preamblelen = (sfdlen + preamblelen) * 1.01763f;
  }

  respframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_resp + margin)/1000.0))/ 1.0256));
  pollframe_sy = (DW_RX_ON_DELAY + (int)((preamblelen + ((msgdatalen_poll + margin)/1000.0))/ 1.0256));

  //tag to anchor ranging consists of poll, 4xresponse and final
  //pollTx2FinalTxDelay delay is the time from start of sending of the poll to the start of sending of the final message
  //this is configured by the user with pollTxToFinalTxDly in sfConfig_t
  inst->pollTx2FinalTxDelay = instance_convert_usec_to_devtimeu(delayus);

  //this is the delay the anchors 1, 2, etc.. will send the response back at...
  //anchor 2 will have the delay set to 2 * fixedReplyDelayAnc
  //andhor 3 will have the delay set to 3 * fixedReplyDelayAnc and so on...
  //this delay depends on how quickly the tag can receive and process the message from previous anchor
  //(and also the frame length of course)
  respframe = (int)(preamblelen + (msgdatalen_resp/1000.0)); //length of response frame (micro seconds)
  if(inst->configData.dataRate == DWT_BR_110K) {
    inst->preambleDuration32h = (uint32) (((uint64) instance_convert_usec_to_devtimeu (preamblelen)) >> 8) + DW_RX_ON_DELAY; //preamble duration + 16 us for RX on
  } else {
    inst->preambleDuration32h = (uint32) (((uint64) instance_convert_usec_to_devtimeu (preamblelen)) >> 8) + DW_RX_ON_DELAY; //preamble duration + 16 us for RX on
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
  dwt_setleds(3);

  inst->frameSN = 0;
  inst->wait4ack = 0;
  inst->instanceTimerEn = 0;

  instance_clearevents();

  memset(inst->eui64, 0, ADDR_BYTE_SIZE_L);
  inst->panID = 0xdeca ;
  inst->tagSleepCorrection_ms = 0;

  dwt_setdblrxbuffmode(0); //disable double RX buffer

  // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
  // other errors which need to be checked (as they disable receiver) are
  //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);
  dwt_setcallbacks(tx_conf_cb, rx_ok_cb_tag, rx_err_cb_tag, rx_err_cb_tag);

  inst->monitor = 0;
  inst->remainingRespToRx = -1; //initialise
  inst->rxResps = 0;

  dwt_setlnapamode(1, 1); //enable TX, RX state on GPIOs 6 and 5

  inst->delayedTRXTime32h = 0;

  return 0 ;
}

//OTP memory addresses for TREK calibration data
#define TXCFG_ADDRESS  (0x10)
#define ANTDLY_ADDRESS (0x1C)
#define TREK_ANTDLY_1  (0xD)
#define TREK_ANTDLY_2  (0xE)
#define TREK_ANTDLY_3  (0xF)
#define TREK_ANTDLY_4  (0x1D)

void instance_config(const instanceConfig_t *config, const sfConfig_t *sfConfig) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  uint32 power = 0;
  uint8 otprev ;

  inst->configData.chan = config->channelNumber ;
  inst->configData.rxCode =  config->preambleCode ;
  inst->configData.txCode = config->preambleCode ;
  inst->configData.prf = config->pulseRepFreq ;
  inst->configData.dataRate = config->dataRate ;
  inst->configData.txPreambLength = config->preambleLen ;
  inst->configData.rxPAC = config->pacSize ;
  inst->configData.nsSFD = config->nsSFD ;
  inst->configData.phrMode = DWT_PHRMODE_STD ;
  inst->configData.sfdTO = config->sfdTO;

  //the DW1000 will automatically use gating gain for frames < 1ms duration (i.e. 6.81Mbps data rate)
  //smartPowerEn should be set based on the frame length, but we can also use dtaa rate.
  if(inst->configData.dataRate == DWT_BR_6M8)
  {
    inst->smartPowerEn = 1;
  }
  else
  {
    inst->smartPowerEn = 0;
  }

  //configure the channel parameters
  dwt_configure(&inst->configData) ;

  /* NOTE: main.c ensures this function is only called when SPI is slow */
  //load TX values from OTP
  dwt_otpread(TXCFG_ADDRESS+(config->pulseRepFreq - DWT_PRF_16M) + (chan_idx[inst->configData.chan] * 2), &power, 1);
  //check if there are calibrated TX power value in the DW1000 OTP
  if((power == 0x0) || (power == 0xFFFFFFFF)) {
    /* No cal found, load defaults */
    power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
  }

  //Configure TX power and PG delay
  inst->configTX.power = power;
  inst->configTX.PGdly = txSpectrumConfig[config->channelNumber].pgDelay ;
  //configure the tx spectrum parameters (power and PG delay)
  dwt_configuretxrf(&inst->configTX);

  otprev = dwt_otprevision() ;  // this revision tells us how OTP is programmed.

  if ((2 == otprev) || (3 == otprev))  // board is calibrated with TREK1000 with antenna delays set for each use case)
  {
    uint8 mode = 0; /* TAG */
    uint8 chanindex = 0;
    uint32 dly = 0;

    //read 32-bit antenna delay value from OTP, high 16 bits is value for Anchor mode, low 16-bits for Tag mode
    switch(inst->configData.chan)
    {
    case 2:
      if(inst->configData.dataRate == DWT_BR_6M8)
        dwt_otpread(TREK_ANTDLY_1, &dly, 1);
      else if(inst->configData.dataRate == DWT_BR_110K)
        dwt_otpread(TREK_ANTDLY_2, &dly, 1);
      break;
    case 5:
      if(inst->configData.dataRate == DWT_BR_6M8)
        dwt_otpread(TREK_ANTDLY_3, &dly, 1);
      else if(inst->configData.dataRate == DWT_BR_110K)
        dwt_otpread(TREK_ANTDLY_4, &dly, 1);
      break;
    default:
      dly = 0;
      break;
    }


    // if nothing was actually programmed then set a reasonable value anyway
    if ((dly == 0) || (dly == 0xffffffff))
    {
      if(inst->configData.chan == 5)
      {
        chanindex = 1;
      }

      inst->txAntennaDelay = rfDelaysTREK[chanindex];
    }
    else
    {
      inst->txAntennaDelay = (dly >> (16*(mode & 0x1))) & 0xFFFF;
    }

  }
  else // assume it is older EVK1000 programming.
  {
    uint32 antennaDly;
    //read the antenna delay that was programmed in the OTP calibration area
    dwt_otpread(ANTDLY_ADDRESS, &antennaDly, 1) ;

    // if nothing was actually programmed then set a reasonable value anyway
    if ((antennaDly == 0) || (antennaDly == 0xffffffff)) {
      inst->txAntennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
    }
    else
    {
      // 32-bit antenna delay value read from OTP, high 16 bits is value for 64 MHz PRF, low 16-bits for 16 MHz PRF
      inst->txAntennaDelay = ((antennaDly >> (16*(inst->configData.prf-DWT_PRF_16M))) & 0xFFFF) >> 1;
    }
  }

  dwt_setrxantennadelay(inst->txAntennaDelay);
  dwt_settxantennadelay(inst->txAntennaDelay);

  inst->rxAntennaDelay = inst->txAntennaDelay;

  if(config->preambleLen == DWT_PLEN_64) //if preamble length is 64
  {
    dwt_loadopsettabfromotp(0);
  }


  inst->tagPeriod_ms = sfConfig->tagPeriod_ms; //set the Tag sleep time
  inst->sframePeriod_ms = sfConfig->sfPeriod_ms;
  inst->slotDuration_ms = sfConfig->slotDuration_ms;
  inst->tagSleepRnd_ms = sfConfig->slotDuration_ms;
  //set the default response delays
  instance_set_replydelay(sfConfig->pollTxToFinalTxDly_us);
}


/* @fn 	  instanceSet16BitAddress
 * @brief set the 16-bit MAC address
 *
 */
void instance_set_16bit_address(uint16 address)
{
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  inst->instanceAddress16 = address ;     // copy configurations
}

/**
 * @brief this function configures the Frame Control and PAN ID bits
 */
void instance_config_frameheader_16bit(instance_data_t *inst)
{
  //set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
  inst->msg_f.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;

  //source/dest addressing modes and frame version
  inst->msg_f.frameCtrl[1] = 0x8 /*dest extended address (16bits)*/ | 0x80 /*src extended address (16bits)*/;

  inst->msg_f.panID[0] = (inst->panID) & 0xff;
  inst->msg_f.panID[1] = inst->panID >> 8;

  inst->msg_f.seqNum = 0;
}

/**
 * @brief this function writes DW TX Frame Control, Delay TX Time and Starts Transmission
 */
int instance_send_delayed_frame(instance_data_t *inst, int delayedTx)
{
  int result = 0;

  dwt_writetxfctrl(inst->psduLength, 0, 1);
  if(delayedTx == DWT_START_TX_DELAYED)
  {
    dwt_setdelayedtrxtime(inst->delayedTRXTime32h) ; //should be high 32-bits of delayed TX TS
  }

  //begin delayed TX of frame
  if (dwt_starttx(delayedTx | inst->wait4ack))  // delayed start was too late
  {
    result = 1; //late/error
    //inst->lateTX++;
  }
  else
  {
    inst->timeofTx = portGetTickCnt();
    inst->monitor = 1;
  }
  return result;                        // state changes
}

void tx_conf_cb(const dwt_cb_data_t *txd) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};
  event_data_t dw_event;

  dwt_readtxtimestamp(txTimeStamp) ;
  instance_seteventtime(&dw_event, txTimeStamp);

  dw_event.rxLength = inst->psduLength;
  dw_event.type =  0;
  dw_event.typePend =  0;

  memcpy((uint8 *)&dw_event.msgu.frame[0], (uint8 *)&inst->msg_f, inst->psduLength);

  instance_putevent(dw_event, DWT_SIG_TX_DONE);

  inst->monitor = 0;
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
  return inst->dwevent[inst->dweventPeek].type; //return the type of event that is in front of the queue
}

void instance_putevent(event_data_t newevent, uint8 etype) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  inst->dwevent[inst->dweventIdxIn] = newevent;
  inst->dwevent[inst->dweventIdxIn].type = etype;
  inst->dweventIdxIn++;

  if(MAX_EVENT_NUMBER == inst->dweventIdxIn) {
    inst->dweventIdxIn = 0;
  }
}

event_data_t* instance_getevent(int x) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  int indexOut = inst->dweventIdxOut;
  if(inst->dwevent[indexOut].type == 0) //exit with "no event"
  {
    inst->dw_event_g.type = 0;
    return &(inst->dw_event_g);
  }

  //copy the event
  inst->dw_event_g.typePend = inst->dwevent[indexOut].typePend;
  inst->dw_event_g.rxLength = inst->dwevent[indexOut].rxLength;
  inst->dw_event_g.timeStamp = inst->dwevent[indexOut].timeStamp;
  inst->dw_event_g.timeStamp32l = inst->dwevent[indexOut].timeStamp32l;
  inst->dw_event_g.timeStamp32h = inst->dwevent[indexOut].timeStamp32h;

  memcpy(&(inst->dw_event_g.msgu), &inst->dwevent[indexOut].msgu, sizeof(inst->dwevent[indexOut].msgu));

  inst->dw_event_g.type = inst->dwevent[indexOut].type ;

  inst->dwevent[indexOut].type = 0; //clear the event

  inst->dweventIdxOut++;
  if(MAX_EVENT_NUMBER == inst->dweventIdxOut) //wrap the counter
  {
    inst->dweventIdxOut = 0;
  }
  inst->dweventPeek = inst->dweventIdxOut; //set the new peek value

  return &(inst->dw_event_g);
}

void instance_clearevents(void)
{
  int i = 0;
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  for(i=0; i<MAX_EVENT_NUMBER; i++)
  {
    memset(&inst->dwevent[i], 0, sizeof(event_data_t));
  }

  inst->dweventIdxIn = 0;
  inst->dweventIdxOut = 0;
  inst->dweventPeek = 0;

}


void instance_config_txpower(uint32 txpower)
{
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  inst->txPower = txpower ;

  inst->txPowerChanged = 1;

}

void instance_set_txpower(void)
{
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  if(inst->txPowerChanged == 1)
  {
    //Configure TX power
    dwt_write32bitreg(0x1E, inst->txPower);

    inst->txPowerChanged = 0;
  }
}



