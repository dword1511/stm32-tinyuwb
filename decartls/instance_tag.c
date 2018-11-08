#include <string.h>

#include <os/os.h>

#include <decadriver/deca_device_api.h>
#include <decadriver/deca_regs.h>

#include <decartls/instance.h>

#include <deca_ctl.h>

// NOTE: the maximum RX timeout is ~ 65ms

#define ADDR16_TO_ANCHOR_N(x) (x & (~0x80))


static void tag_enable_rx(uint32 dlyTime) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  dwt_setdelayedtrxtime(dlyTime - inst->preambleDuration32h) ;
  if (dwt_rxenable(DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR)) {
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy * 2);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    dwt_setpreambledetecttimeout(PTO_PACS);
    dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy);
  }
}

static void tag_process_rx_timeout(instance_data_t *inst) {
  if (inst->rxResponseMask == 0) {
    inst->instToSleep   = TRUE;
    inst->testAppState  = TA_TXE_WAIT ;
    inst->nextState     = TA_TXPOLL_WAIT_SEND ;
  } else if (inst->previousState == TA_TXFINAL_WAIT_SEND) {
    dwt_forcetrxoff(); /* this will clear all events */
    inst->instToSleep   = TRUE;
    inst->testAppState  = TA_TXE_WAIT;
    inst->nextState     = TA_TXPOLL_WAIT_SEND;
  } else {
    inst->testAppState  = TA_TXE_WAIT;
    inst->nextState     = TA_TXFINAL_WAIT_SEND;
  }
}

static uint8 tag_rx_reenable(uint16 sourceAddress, uint8 error) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  uint8 anc = ADDR16_TO_ANCHOR_N(sourceAddress);

  if (anc != NUM_EXPECTED_RESPONSES) {
    /* Not last anchor */
    if (inst->remainingRespToRx > 0) {
      if (error == 0) {
        inst->remainingRespToRx = NUM_EXPECTED_RESPONSES - anc;
      }
      tag_enable_rx(inst->tagPollTxTime32h + (MAX_ANCHOR_LIST_SIZE - inst->remainingRespToRx + 1) * (inst->fixedReplyDelayAnc32h));

      return DWT_SIG_RX_PENDING;
    } else {
      return DWT_SIG_DW_IDLE;
    }
  } else {
    return DWT_SIG_DW_IDLE;
  }
}

static void tag_handle_error_unknownframe(event_data_t dw_event) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  inst->remainingRespToRx--;
  dw_event.typePend = tag_rx_reenable(0, 1);
  dw_event.type = 0;
  dw_event.rxLength = 0;

  instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
}

void rx_err_cb_tag(const dwt_cb_data_t *rxd) {
  event_data_t dw_event;
  tag_handle_error_unknownframe(dw_event);
}

void rx_ok_cb_tag(const dwt_cb_data_t *rxd) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};
  uint8 rxd_event = SIG_RX_UNKNOWN;
  uint8 fcode_index  = 0;
  uint8 srcAddr_index = 0;
  event_data_t dw_event;

  dw_event.rxLength = rxd->datalength;

  if(rxd->fctrl[0] == 0x41) {
    if((rxd->fctrl[1] & 0xcc) == 0x88) {
      fcode_index = FRAME_CRTL_AND_ADDRESS_S;
      srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
      rxd_event = DWT_SIG_RX_OKAY;
    }
  }

  dwt_readrxtimestamp(rxTimeStamp) ;
  dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);
  instance_seteventtime(&dw_event, rxTimeStamp);

  dw_event.type = 0;
  dw_event.typePend = DWT_SIG_DW_IDLE;

  if (rxd_event == DWT_SIG_RX_OKAY) {
    if (dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_ANCH_RESP) {
      uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index + 1]) << 8) + dw_event.msgu.frame[srcAddr_index];
      uint8 index = RRXT0 + 5 * ADDR16_TO_ANCHOR_N(sourceAddress);

      inst->remainingRespToRx--;
      dw_event.typePend = tag_rx_reenable(sourceAddress, 0);
      inst->rxResponseMask |= (0x1 << ADDR16_TO_ANCHOR_N(sourceAddress));
      memcpy(&(inst->msg_f.messageData[index]), rxTimeStamp, 5);
      instance_putevent(dw_event, rxd_event);
      return;
    }
  }

  tag_handle_error_unknownframe(dw_event);
}

static int tag_do_ta_init(instance_data_t *inst) {
  uint16 sleep_mode = 0;

  dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;

  inst->eui64[0] += inst->instanceAddress16;
  dwt_seteui(inst->eui64);
  dwt_setpanid(inst->panID);
  memcpy(inst->eui64, &inst->instanceAddress16, ADDR_BYTE_SIZE_S);
  dwt_setaddress16(inst->instanceAddress16);

  inst->nextState = TA_TXPOLL_WAIT_SEND;
  inst->testAppState = TA_TXE_WAIT;
  inst->instToSleep = TRUE ;
  inst->tagSleepTime_ms = inst->tagPeriod_ms ;
  inst->rangeNum = 0;
  inst->tagSleepCorrection_ms = 0;

  sleep_mode = (DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV);

  if(inst->configData.txPreambLength == DWT_PLEN_64) {
    sleep_mode |= DWT_LOADOPSET;
  }

#if (DEEP_SLEEP == 1)
  dwt_configuresleep(sleep_mode, DWT_WAKE_WK | DWT_WAKE_CS | DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#endif
  instance_config_frameheader_16bit(inst);
  inst->instanceWakeTime_ms = portGetTickCnt();

  return INST_NOT_DONE_YET;
}

static int tag_do_ta_sleep_done(instance_data_t *inst) {
  event_data_t* dw_event = instance_getevent(10); //clear the event from the queue

  if (dw_event->type != DWT_SIG_RX_TIMEOUT) {
    /* Wait for sleep timeout */
    return INST_DONE_WAIT_FOR_NEXT_EVENT;
  }

  inst->instToSleep = FALSE ;
  inst->testAppState = inst->nextState;
  inst->nextState = 0; //clear
  inst->instanceWakeTime_ms = portGetTickCnt();
#if (DEEP_SLEEP == 1)
  port_wakeup_dw1000_fast();
  dwt_setleds(1);
  /* MP bug - TX antenna delay needs reprogramming as it is not preserved (only RX) */
  dwt_settxantennadelay(inst->txAntennaDelay);
  /* set EUI (not preserved) */
  dwt_seteui(inst->eui64);
#endif

  instance_set_txpower();

  return INST_NOT_DONE_YET;
}

static int tag_do_ta_txe_wait(instance_data_t *inst) {
  /* if we are scheduled to go to sleep before next transmission then sleep first. */
  if((inst->nextState == TA_TXPOLL_WAIT_SEND) && (inst->instToSleep)) {
    inst->rangeNum ++;
    inst->testAppState = TA_SLEEP_DONE;
#if (DEEP_SLEEP == 1)
    dwt_entersleep();
#endif
    if(inst->rxResponseMask != 0) {
      inst->rxResponseMask = 0;
    }
    return INST_DONE_WAIT_FOR_NEXT_EVENT_TO;
  } else {
    /* Proceed */
    inst->testAppState = inst->nextState;
    inst->nextState = 0;
    return INST_NOT_DONE_YET;
  }
}

static int tag_do_ta_txpoll_wait_send(instance_data_t *inst) {
  inst->msg_f.messageData[POLL_RNUM] = inst->rangeNum;
  inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_TAG_POLL;
  inst->psduLength = (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
  inst->msg_f.seqNum = inst->frameSN ++;
  inst->msg_f.sourceAddr[0] = inst->instanceAddress16 & 0xff;
  inst->msg_f.sourceAddr[1] = (inst->instanceAddress16 >> 8) & 0xff;
  inst->msg_f.destAddr[0] = 0xff;
  inst->msg_f.destAddr[1] = 0xff;
  dwt_writetxdata(inst->psduLength, (uint8 *)&inst->msg_f, 0);

  dwt_setrxaftertxdelay((uint32)inst->tagRespRxDelay_sy);

  inst->remainingRespToRx = MAX_ANCHOR_LIST_SIZE;
  dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy);
  dwt_setpreambledetecttimeout(PTO_PACS);

  inst->rxResponseMask = 0;
  inst->wait4ack = DWT_RESPONSE_EXPECTED;

  dwt_writetxfctrl(inst->psduLength, 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  inst->testAppState  = TA_TX_WAIT_CONF;
  inst->previousState = TA_TXPOLL_WAIT_SEND;

  return INST_DONE_WAIT_FOR_NEXT_EVENT;
}

static int tag_do_ta_txfinal_wait_send(instance_data_t *inst) {
  inst->msg_f.messageData[POLL_RNUM] = inst->rangeNum;
  inst->msg_f.messageData[VRESP] = inst->rxResponseMask;
  inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_TAG_FINAL;
  inst->psduLength = (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
  inst->msg_f.seqNum = inst->frameSN ++;
  dwt_writetxdata(inst->psduLength, (uint8 *) &inst->msg_f, 0);

  inst->wait4ack = 0;

  if (instance_send_delayed_frame(inst, DWT_START_TX_DELAYED)) {
    /* go to TA_TXE_WAIT first to check if it's sleep time */
    inst->testAppState = TA_TXE_WAIT;
    inst->nextState = TA_TXPOLL_WAIT_SEND;
    inst->instToSleep = TRUE;
    return INST_NOT_DONE_YET;
  } else {
    inst->testAppState = TA_TX_WAIT_CONF;
    inst->previousState = TA_TXFINAL_WAIT_SEND;
    inst->instToSleep = TRUE ;
    return INST_DONE_WAIT_FOR_NEXT_EVENT;
  }
}

static int tag_app_run(instance_data_t *inst) {
  int instDone = INST_NOT_DONE_YET;
  int message = instance_peekevent(); //get any of the received events from ISR

  switch (inst->testAppState) {
    case TA_INIT: {
      instDone = tag_do_ta_init(inst);
      break;
    }

    case TA_SLEEP_DONE: {
      instDone = tag_do_ta_sleep_done(inst);
      break;
    }

    case TA_TXE_WAIT: {
      instDone = tag_do_ta_txe_wait(inst);
      break;
    }

    case TA_TXPOLL_WAIT_SEND: {
      instDone = tag_do_ta_txpoll_wait_send(inst);
      break;
    }

    case TA_TXFINAL_WAIT_SEND: {
      instDone = tag_do_ta_txfinal_wait_send(inst);
      break;
    }

    case TA_TX_WAIT_CONF: {
        event_data_t* dw_event = instance_getevent(11); //get and clear this event

                if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
                {
            instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    break;
                }

                instDone = INST_NOT_DONE_YET;

                if(inst->previousState == TA_TXFINAL_WAIT_SEND)
                {
                     inst->testAppState = TA_TXE_WAIT ;
                     inst->nextState = TA_TXPOLL_WAIT_SEND ;
                    break;
                }
                else
                {
          inst->txu.txTimeStamp = dw_event->timeStamp;
          inst->tagPollTxTime32h = dw_event->timeStamp32h;

          if(inst->previousState == TA_TXPOLL_WAIT_SEND)
          {
                    uint64 tagCalculatedFinalTxTime ;
                    // Embed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime
                    tagCalculatedFinalTxTime =  (inst->txu.txTimeStamp + inst->pollTx2FinalTxDelay) & MASK_TXDTS;

                    inst->delayedTRXTime32h = tagCalculatedFinalTxTime >> 8; //high 32-bits
                    // Calculate Time Final message will be sent and write this field of Final message
                    // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
                    // zeroing its low 9 bits, and then having the TX antenna delay added
                    // getting antenna delay from the device and add it to the Calculated TX Time
                    tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txAntennaDelay;
                    tagCalculatedFinalTxTime &= MASK_40BIT;

                    // Write Calculated TX time field of Final message
            memcpy(&(inst->msg_f.messageData[FTXT]), (uint8 *)&tagCalculatedFinalTxTime, 5);
                    // Write Poll TX time field of Final message
            memcpy(&(inst->msg_f.messageData[PTXT]), (uint8 *)&inst->txu.tagPollTxTime, 5);

          }

                    inst->testAppState = TA_RX_WAIT_DATA ;                      // After sending, tag expects response/report, anchor waits to receive a final/new poll

                    message = 0;
                    //fall into the next case (turn on the RX)
                }

            }

            //break ; // end case TA_TX_WAIT_CONF

        case TA_RX_WAIT_DATA:

            switch (message)
            {

        //if we have received a DWT_SIG_RX_OKAY event - this means that the message is IEEE data type - need to check frame control to know which addressing mode is used
                case DWT_SIG_RX_OKAY:
                {
          event_data_t* dw_event = instance_getevent(15); //get and clear this event
          uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};
          uint8  dstAddr[8] = {0,0,0,0,0,0,0,0};
                    int fcode = 0;
          uint8 *messageData;

          memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
          memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ss.destAddr[0]), ADDR_BYTE_SIZE_S);
          fcode = dw_event->msgu.rxmsg_ss.messageData[FCODE];
          messageData = &dw_event->msgu.rxmsg_ss.messageData[0];

          //process ranging messages
          switch(fcode)
          {
            case RTLS_DEMO_MSG_ANCH_RESP:
            {
              if(GATEWAY_ANCHOR_ADDR == (srcAddr[0] | ((uint32)(srcAddr[1] << 8)))) //if response from gateway then use the correction factor
              {
                // int sleepCorrection = (int16) (((uint16) messageData[RES_TAG_SLP1] << 8) + messageData[RES_TAG_SLP0]);
                // casting received bytes to int because this is a signed correction -0.5 periods to +1.5 periods
                inst->tagSleepCorrection_ms = (int16) (((uint16) messageData[RES_TAG_SLP1] << 8) + messageData[RES_TAG_SLP0]);
                inst->tagSleepRnd_ms = 0; // once we have initial response from Anchor #0 the slot correction acts and we don't need this anymore
              }

              if(dw_event->typePend == DWT_SIG_RX_PENDING)
              {
                // stay in TA_RX_WAIT_DATA - receiver is already enabled, waiting for next response.
              }
              //DW1000 idle - send the final
              else //if(dw_event->type_pend == DWT_SIG_DW_IDLE)
              {
                  inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final
              }
            }
            break; //RTLS_DEMO_MSG_ANCH_RESP

            default:
            {
              tag_process_rx_timeout(inst); //if unknown message process as timeout
            }
            break;
          } //end switch (fcode)

                }
        break ; //end of DWT_SIG_RX_OKAY

          case RTLS_DEMO_MSG_RNG_INIT :
            //LOG("RTLS_DEMO_MSG_RNG_INIT");
                {
                  event_data_t* dw_event = instance_getevent(16); //get and clear this event
                  uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};

                  uint8* messageData = &dw_event->msgu.rxmsg_ls.messageData[0];
                  memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ls.sourceAddr[0]), ADDR_BYTE_SIZE_S);

          if(GATEWAY_ANCHOR_ADDR == (srcAddr[0] | ((uint32)(srcAddr[1] << 8)))) //if response from gateway then use the correction factor
          {
            // casting received bytes to int because this is a signed correction -0.5 periods to +1.5 periods
            inst->tagSleepCorrection_ms = (int16) (((uint16) messageData[RES_TAG_SLP1] << 8) + messageData[RES_TAG_SLP0]);
            inst->tagSleepRnd_ms = 0; // once we have initial response from Anchor #0 the slot correction acts and we don't need this anymore
          }

          //get short address from anchor
          inst->instanceAddress16 = (int16) (((uint16) messageData[RES_TAG_ADD1] << 8) + messageData[RES_TAG_ADD0]);

          //set source address
          dwt_setaddress16(inst->instanceAddress16);

          inst->nextState = TA_TXPOLL_WAIT_SEND;
          inst->testAppState = TA_TXE_WAIT;
          inst->instToSleep = TRUE ;

          inst->tagSleepTime_ms = inst->tagPeriod_ms ;

          break; //RTLS_DEMO_MSG_RNG_INIT
                }

                case DWT_SIG_RX_TIMEOUT:
                  {
                    event_data_t* dw_event = instance_getevent(17); //get and clear this event

            //printf("PD_DATA_TIMEOUT %d\n", inst->previousState) ;

                    //Anchor can time out and then need to send response - so will be in TX pending
                    if(dw_event->typePend == DWT_SIG_TX_PENDING)
                    {
                      inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                      inst->previousState = TA_TXRESPONSE_SENT_TORX ;    //wait for TX confirmation of sent response
                    }
                    else if(dw_event->typePend == DWT_SIG_DW_IDLE) //if timed out and back in receive then don't process as timeout
            {
                      tag_process_rx_timeout(inst);
            }
                    //else if RX_PENDING then wait for next RX event...
            message = 0; //clear the message as we have processed the event
                  }
                break ;

                default :
                {
                    if(message) // == DWT_SIG_TX_DONE)
                    {
                      instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    }

                  if(instDone == INST_NOT_DONE_YET) instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                }
                break;

            }
            break; // end case TA_RX_WAIT_DATA
            default:
            break;
    } // end switch on testAppState

    return instDone;
}

int tag_run(void) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  int done = INST_NOT_DONE_YET;

  while(done == INST_NOT_DONE_YET) {
    done = tag_app_run(inst);
  }

  if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) {
    int32 nextPeriod ;

    // next period will be a positive number because correction is -0.5 to +1.5 periods, (and tagSleepTime_ms is the period)
    nextPeriod = inst->tagSleepRnd_ms + inst->tagSleepTime_ms + inst->tagSleepCorrection_ms;

    inst->nextWakeUpTime_ms = (uint32)nextPeriod;
    inst->tagSleepCorrection_ms = 0;
    inst->instanceTimerEn = 1;
  }

  //check if timer has expired
  if(inst->instanceTimerEn == 1) {
    if((portGetTickCnt() - inst->instanceWakeTime_ms) > inst->nextWakeUpTime_ms) {
      event_data_t dw_event;
      inst->instanceTimerEn = 0;
      dw_event.rxLength = 0;
      dw_event.type = 0;
      instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
    }
  }

  return 0;
}
