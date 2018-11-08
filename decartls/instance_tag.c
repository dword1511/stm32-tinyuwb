#include <string.h>

#include <os/os.h>

#include <decadriver/deca_device_api.h>
#include <decadriver/deca_regs.h>

#include <decartls/instance.h>

#include <deca_ctl.h>

// NOTE: the maximum RX timeout is ~ 65ms


static void tag_enable_rx(uint32 dlyTime) {
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  //subtract preamble duration (because when instructing delayed TX the time is the time of SFD,
  //however when doing delayed RX the time is RX on time)
  dwt_setdelayedtrxtime(dlyTime - inst->preambleDuration32h) ;
  if (dwt_rxenable(DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR)) {
    dwt_setpreambledetecttimeout(0); //clear preamble timeout as RX is turned on early/late
    dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy * 2); //reconfigure the timeout before enable
    //longer timeout as we cannot do delayed receive... so receiver needs to stay on for longer
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    dwt_setpreambledetecttimeout(PTO_PACS); //configure preamble timeout
    dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy); //restore the timeout for next RX enable
  }
}

static void tag_process_rx_timeout(instance_data_t *inst) {
  if(inst->rxResponseMask == 0) //if any response have been received send a Final else go to SLEEP
  {
      inst->instToSleep = TRUE ; //set sleep to TRUE so that tag will go to DEEP SLEEP before next ranging attempt
    inst->testAppState = TA_TXE_WAIT ;
    inst->nextState = TA_TXPOLL_WAIT_SEND ;
  }
  else if (inst->previousState == TA_TXFINAL_WAIT_SEND) //got here from main (error sending final - handle as timeout)
  {
    dwt_forcetrxoff();  //this will clear all events
    inst->instToSleep = TRUE ;
    // initiate the re-transmission of the poll that was not responded to
    inst->testAppState = TA_TXE_WAIT ;
    inst->nextState = TA_TXPOLL_WAIT_SEND ;
  }
  else //send the final
  {
    // initiate the transmission of the final
    inst->testAppState = TA_TXE_WAIT ;
    inst->nextState = TA_TXFINAL_WAIT_SEND ;
  }
}

static uint8 tag_rx_reenable(uint16 sourceAddress, uint8 error) {
  uint8 type_pend = DWT_SIG_DW_IDLE;
  uint8 anc = sourceAddress & 0x3;
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  if (anc == 3) {
    /* Last anchor */
    type_pend = DWT_SIG_DW_IDLE;
  } else {
    if(inst->remainingRespToRx > 0) //can get here as result of error frame so need to check
      {
        //can't use anc address as this is an error frame, so just re-enable TO based on remainingRespToRx count
        if(error == 0)
        {
          switch (anc)
          {
            case 0:
              inst->remainingRespToRx = 3; //expecting 3 more responses
              break;
            case 1:
              inst->remainingRespToRx = 2; //expecting 2 more responses
              break;
            case 2:
              inst->remainingRespToRx = 1; //expecting 1 more response
              break;
          }
        }
        //Poll sent at tagPollTxTime_32bit
        //1st response is delayTime + fixedReplyDelayAnc32h - preambleDuration_32MSBs
        //2nd is delayTime + fixedReplyDelayAnc32h - preambleDuration_32MSBs + fixedReplyDelayAnc32h
        tag_enable_rx(inst->tagPollTxTime32h +
            (MAX_ANCHOR_LIST_SIZE-inst->remainingRespToRx+1)*(inst->fixedReplyDelayAnc32h));

        type_pend = DWT_SIG_RX_PENDING ;
      }
      else //finished waiting for responses - no responses left to receive... send a final
      {
        type_pend = DWT_SIG_DW_IDLE; //report timeout - send the final if due to be sent
      }
  }

  return type_pend;
}

/**
 * @brief this function handles frame error event, it will either signal TO or re-enable the receiver
 */
static void tag_handle_error_unknownframe(event_data_t dw_event)
{
  instance_data_t* inst = instance_get_local_structure_ptr(0);

  if(inst->twrMode != GREETER)
  {
    //re-enable the receiver (after error frames as we are not using auto re-enable
    //for ranging application rx error frame is same as TO - as we are not going to get the expected frame
    inst->remainingRespToRx--; //got something (need to reduce timeout (for remaining responses))

    dw_event.typePend = tag_rx_reenable(0, 1); //check if receiver will be re-enabled or it's time to send the final
  }
  else
  {
    dw_event.typePend = DWT_SIG_DW_IDLE; //in GREETER mode only waiting for 1 frame
  }

  dw_event.type = 0;
  //dw_event.typeSave = 0x40 | DWT_SIG_RX_TIMEOUT;
  dw_event.rxLength = 0;

  instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
}

/**
 * @brief this is the receive timeout event callback handler
 */
void rx_to_cb_tag(const dwt_cb_data_t *rxd)
{
  event_data_t dw_event;

  //microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
    tag_handle_error_unknownframe(dw_event);
}

/**
 * @brief this is the receive error event callback handler
 */
void rx_err_cb_tag(const dwt_cb_data_t *rxd)
{
  event_data_t dw_event;

  //microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
    tag_handle_error_unknownframe(dw_event);
}

/**
 * @brief this is the receive event callback handler, the received event is processed and the instance either
 * responds by sending a response frame or re-enables the receiver to await the next frame
 * once the immediate action is taken care of the event is queued up for application to process
 */
void rx_ok_cb_tag(const dwt_cb_data_t *rxd)
{
  instance_data_t* inst = instance_get_local_structure_ptr(0);
  uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};

    uint8 rxd_event = 0;
  uint8 fcode_index  = 0;
  uint8 srcAddr_index = 0;
  event_data_t dw_event;

  //microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();

    //if we got a frame with a good CRC - RX OK
    {
     dw_event.rxLength = rxd->datalength;

    //need to process the frame control bytes to figure out what type of frame we have received
    if(rxd->fctrl[0] == 0x41)
    {
      if((rxd->fctrl[1] & 0xCC) == 0x88) //short address
      {
        fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
        srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
        rxd_event = DWT_SIG_RX_OKAY;
      }
      else
      {
        rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
      }
    }
    else
    {
      rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
    }

        //read RX timestamp
        dwt_readrxtimestamp(rxTimeStamp) ;
        dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
        instance_seteventtime(&dw_event, rxTimeStamp);

        dw_event.type = 0; //type will be added as part of adding to event queue
    //dw_event.typeSave = rxd_event;
    dw_event.typePend = DWT_SIG_DW_IDLE;

    if(rxd_event == DWT_SIG_RX_OKAY) //Process good/known frame types
    {
      uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index+1]) << 8) + dw_event.msgu.frame[srcAddr_index];

      //if tag got a good frame - this is probably a response, but could also be some other non-ranging frame
      //(although due to frame filtering this is limited as non-addressed frames are filtered out)

      //check if this is a TWR message (and also which one)
      switch(dw_event.msgu.frame[fcode_index])
      {
        //we got a response from a "responder" (anchor)
        case RTLS_DEMO_MSG_ANCH_RESP:
        {
          if(inst->twrMode == INITIATOR)
          {
            //if tag is involved in the ranging exchange expecting responses
            uint8 index ;
            inst->remainingRespToRx--; //got 1 more response or other RX frame - need to reduce timeout (for next response)
            dw_event.typePend = tag_rx_reenable(sourceAddress, 0); //remainingRespToRx decremented above...
            index = RRXT0 + 5*(sourceAddress & 0x3);

            inst->rxResponseMask |= (0x1 << (sourceAddress & 0x3)); //add anchor ID to the mask
            // Write Response RX time field of Final message
            memcpy(&(inst->msg_f.messageData[index]), rxTimeStamp, 5);
            break;
          }
        }
        case RTLS_DEMO_MSG_ANCH_POLL:
        case RTLS_DEMO_MSG_TAG_POLL:
        case RTLS_DEMO_MSG_TAG_FINAL:
        case RTLS_DEMO_MSG_ANCH_FINAL:
        case RTLS_DEMO_MSG_ANCH_RESP2:
        default:
        //tag should ignore any other frames - only receive responses
        {
          tag_handle_error_unknownframe(dw_event);
          //inst->rxMsgCount++;
          return;
        }
      }
            instance_putevent(dw_event, rxd_event);

      //inst->rxMsgCount++;
    }
    else //if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx (got unknown frame type)
    {
      tag_handle_error_unknownframe(dw_event);
    }
  }
}


static int tag_app_run(instance_data_t *inst)
{
  int instDone = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR

    //LOG("Processing tag state %u", inst->testAppState);
    switch (inst->testAppState)
    {
        case TA_INIT :
            //LOG("TA_INIT");
            // printf("TA_INIT") ;
            switch (inst->mode)
            {
                case TAG:
                {
                  uint16 sleep_mode = 0;

                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;

                    inst->eui64[0] += inst->instanceAddress16; //so switch 5,6,7 can be used to emulate more tags
                    dwt_seteui(inst->eui64);

                    dwt_setpanid(inst->panID);

                    //dwt_setpanid(inst->panID);
                    memcpy(inst->eui64, &inst->instanceAddress16, ADDR_BYTE_SIZE_S);
                    //set source address
                    inst->newRangeTagAddress = inst->instanceAddress16 ;
                    dwt_setaddress16(inst->instanceAddress16);

                    //Start off by Sleeping 1st -> set instToSleep to TRUE
                    inst->nextState = TA_TXPOLL_WAIT_SEND;
                    inst->testAppState = TA_TXE_WAIT;
                    inst->instToSleep = TRUE ;
                    inst->tagSleepTime_ms = inst->tagPeriod_ms ;
                    inst->rangeNum = 0;
                    inst->tagSleepCorrection_ms = 0;

                    sleep_mode = (DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);

          if(inst->configData.txPreambLength == DWT_PLEN_64)  //if using 64 length preamble then use the corresponding OPSet
            sleep_mode |= DWT_LOADOPSET;

#if (DEEP_SLEEP == 1)
            dwt_configuresleep(sleep_mode, DWT_WAKE_WK | DWT_WAKE_CS | DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#endif
            instance_config_frameheader_16bit(inst);
            inst->instanceWakeTime_ms = portGetTickCnt();
                }
                break;
                default:
                break;
            }
            break; // end case TA_INIT

        case TA_SLEEP_DONE :
        {
            //LOG("TA_SLEEP_DONE");
          event_data_t* dw_event = instance_getevent(10); //clear the event from the queue
      // waiting for timout from application to wakeup IC
      if (dw_event->type != DWT_SIG_RX_TIMEOUT)
      {
                //LOG("TA_SLEEP_DONE, dw_event->type %u != sleep not done yet", dw_event->type);
        // if no pause and no wake-up timeout continue waiting for the sleep to be done.
                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //wait here for sleep timeout
                break;
            }

            instDone = INST_NOT_DONE_YET;
            inst->instToSleep = FALSE ;
            //LOG("TA_SLEEP_DONE, state %u -> %u", inst->testAppState, inst->nextState);
            inst->testAppState = inst->nextState;
            inst->nextState = 0; //clear
      inst->instanceWakeTime_ms = portGetTickCnt(); // Record the time count when we wake-up
#if (DEEP_SLEEP == 1)
            {
                port_wakeup_dw1000_fast();
                //this is platform dependent - only program if DW EVK/EVB
                dwt_setleds(1);
                //MP bug - TX antenna delay needs reprogramming as it is not preserved (only RX)
                dwt_settxantennadelay(inst->txAntennaDelay) ;
                //set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
                dwt_seteui(inst->eui64);
            }
#else
            //Sleep(3); //to approximate match the time spent in the #if above // TODO: Why?
            tick_sleep(3);
#endif

            instance_set_txpower(); //configure TX power if it has changed
       }
            break;

        case TA_TXE_WAIT : //either go to sleep or proceed to TX a message
            //LOG("TA_TXE_WAIT");
            //if we are scheduled to go to sleep before next transmission then sleep first.
          if((inst->nextState == TA_TXPOLL_WAIT_SEND) && (inst->instToSleep)  //go to sleep before sending the next poll/ starting new ranging exchange
                    )
            {
              inst->rangeNum++; //increment the range number before going to sleep
                //the app should put chip into low power state and wake up after tagSleepTime_ms time...
                //the app could go to *_IDLE state and wait for uP to wake it up...
                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //don't sleep here but kick off the Sleep timer countdown
                inst->testAppState = TA_SLEEP_DONE;

                {

#if (DEEP_SLEEP == 1)
                  //put device into low power mode
          dwt_entersleep(); //go to sleep
#endif
          if(inst->rxResponseMask != 0)
          {
            //DW1000 gone to sleep - report the received range
            //inst->newRange = instance_calc_ranges(&inst->tofArray[0], MAX_ANCHOR_LIST_SIZE, TOF_REPORT_T2A, &inst->rxResponseMask);
            inst->rxResponseMaskReport = inst->rxResponseMask;
            inst->rxResponseMask = 0;
            //inst->newRangeTime = portGetTickCnt() ;
          }
                }

            }
            else //proceed to configuration and transmission of a frame
            {
                inst->testAppState = inst->nextState;
                inst->nextState = 0; //clear
            }
            break ; // end case TA_TXE_WAIT
        case TA_TXBLINK_WAIT_SEND :
            //LOG("TA_TXBLINK_WAIT_SEND");
            {
        int flength = (BLINK_FRAME_CRTL_AND_ADDRESS + FRAME_CRC);

                //blink frames with IEEE EUI-64 tag ID
                inst->blinkmsg.frameCtrl = 0xC5 ;
                inst->blinkmsg.seqNum = inst->frameSN++;

        dwt_writetxdata(flength, (uint8 *)  (&inst->blinkmsg), 0) ;  // write the frame data
        dwt_writetxfctrl(flength, 0, 1);

        inst->twrMode = GREETER;
        //using wait for response to do delayed receive
        inst->wait4ack = DWT_RESPONSE_EXPECTED;
        inst->rxResponseMask = 0;

        dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy*2);  //units are symbols (x2 as ranging init > response)
        //set the delayed rx on time (the ranging init will be sent after this delay)
        dwt_setrxaftertxdelay((uint32)inst->tagRespRxDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

        dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack); //always using immediate TX and enable delayed RX

        inst->instToSleep = 1; //go to Sleep after this blink
                inst->testAppState = TA_RX_WAIT_DATA ; // to to RX, expecting ranging init response
                inst->previousState = TA_TXBLINK_WAIT_SEND ;
                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

            }
            break ; // end case TA_TXBLINK_WAIT_SEND

        case TA_TXPOLL_WAIT_SEND :
            //LOG("TA_TXPOLL_WAIT_SEND");
            {
                inst->msg_f.messageData[POLL_RNUM] = inst->rangeNum; //copy new range number
              inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_TAG_POLL; //message function code (specifies if message is a poll, response or other...)
                inst->psduLength = (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
                inst->msg_f.seqNum = inst->frameSN++; //copy sequence number and then increment
                inst->msg_f.sourceAddr[0] = inst->instanceAddress16 & 0xff; //inst->eui64[0]; //copy the address
                inst->msg_f.sourceAddr[1] = (inst->instanceAddress16>>8) & 0xff; //inst->eui64[1]; //copy the address
              inst->msg_f.destAddr[0] = 0xff;  //set the destination address (broadcast == 0xffff)
              inst->msg_f.destAddr[1] = 0xff;  //set the destination address (broadcast == 0xffff)
                dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;  // write the frame data

        //set the delayed rx on time (the response message will be sent after this delay (from A0))
        dwt_setrxaftertxdelay((uint32)inst->tagRespRxDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

        inst->remainingRespToRx = MAX_ANCHOR_LIST_SIZE; //expecting 4 responses
        dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy);  //configure the RX FWTO
        dwt_setpreambledetecttimeout(PTO_PACS); //configure preamble timeout

        inst->rxResponseMask = 0;  //reset/clear the mask of received responses when tx poll

        inst->wait4ack = DWT_RESPONSE_EXPECTED; //response is expected - automatically enable the receiver

        dwt_writetxfctrl(inst->psduLength, 0, 1); //write frame control

        inst->twrMode = INITIATOR;

        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); //transmit the frame

                inst->testAppState = TA_TX_WAIT_CONF ;  // wait confirmation
                inst->previousState = TA_TXPOLL_WAIT_SEND ;
                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)

            }
            break;

        case TA_TXFINAL_WAIT_SEND :
            //LOG("TA_TXFINAL_WAIT_SEND");
            {
              //the final has the same range number as the poll (part of the same ranging exchange)
                inst->msg_f.messageData[POLL_RNUM] = inst->rangeNum;
                //the mask is sent so the anchors know whether the response RX time is valid
        inst->msg_f.messageData[VRESP] = inst->rxResponseMask;
              inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_TAG_FINAL; //message function code (specifies if message is a poll, response or other...)
                inst->psduLength = (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
                inst->msg_f.seqNum = inst->frameSN++;
        dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;  // write the frame data

        inst->wait4ack = 0; //clear the flag not using wait for response as this message ends the ranging exchange

        if(instance_send_delayed_frame(inst, DWT_START_TX_DELAYED))
                {
                    // initiate the re-transmission
          inst->testAppState = TA_TXE_WAIT ; //go to TA_TXE_WAIT first to check if it's sleep time
          inst->nextState = TA_TXPOLL_WAIT_SEND ;
          inst->instToSleep = TRUE ;
                    break; //exit this switch case...
                }
                else
                {
                    inst->testAppState = TA_TX_WAIT_CONF;   // wait confirmation
                }

        inst->previousState = TA_TXFINAL_WAIT_SEND;
        inst->instToSleep = TRUE ;
              instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)
            }
            break;


        case TA_TX_WAIT_CONF :
            //LOG("TA_TX_WAIT_CONF");
            {
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
                case DWT_SIG_RX_OKAY :
            //LOG("DWT_SIG_RX_OKAY");
                {
          event_data_t* dw_event = instance_getevent(15); //get and clear this event
          uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};
          uint8  dstAddr[8] = {0,0,0,0,0,0,0,0};
                    int fcode = 0;
          uint8 tof_idx  = 0;
          uint8 *messageData;

          memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
          memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ss.destAddr[0]), ADDR_BYTE_SIZE_S);
          fcode = dw_event->msgu.rxmsg_ss.messageData[FCODE];
          messageData = &dw_event->msgu.rxmsg_ss.messageData[0];

          tof_idx = srcAddr[0] & 0x3 ;
          //process ranging messages
          switch(fcode)
          {
            case RTLS_DEMO_MSG_ANCH_RESP:
            {
              uint8 currentRangeNum = (messageData[TOFRN] + 1); //current = previous + 1

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

                {
                  inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final
                }

              }

              if(currentRangeNum == inst->rangeNum) //these are the previous ranges...
              {
                //copy the ToF and put into array (array holds last 4 ToFs)
                memcpy(&inst->tofArray[tof_idx], &(messageData[TOFR]), 4);

                //check if the ToF is valid, this makes sure we only report valid ToFs
                //e.g. consider the case of reception of response from anchor a1 (we are anchor a2)
                //if a1 got a Poll with previous Range number but got no Final, then the response will have
                //the correct range number but the range will be INVALID_TOF
                if(inst->tofArray[tof_idx] != INVALID_TOF)
                {
                  inst->rxResponseMask |= (0x1 << tof_idx);
                }

              }
              else
              {
                if(inst->tofArray[tof_idx] != INVALID_TOF)
                {
                  inst->tofArray[tof_idx] = INVALID_TOF;
                }
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
          inst->newRangeTagAddress = inst->instanceAddress16 ;
          dwt_setaddress16(inst->instanceAddress16);

          inst->nextState = TA_TXPOLL_WAIT_SEND;
          inst->testAppState = TA_TXE_WAIT;
          inst->instToSleep = TRUE ;

          inst->tagSleepTime_ms = inst->tagPeriod_ms ;

          //inst->twrMode = INITIATOR;

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
            //LOG("default!");
                {
                    if(message) // == DWT_SIG_TX_DONE)
                    {
                      instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    }

                  if(instDone == INST_NOT_DONE_YET) instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                }
                break;

            }
            break ; // end case TA_RX_WAIT_DATA
            default:
                //printf("\nERROR - invalid state %d - what is going on??\n", inst->testAppState) ;
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

    inst->nextWakeUpTime_ms = (uint32) nextPeriod ; //set timeout time, CAST the positive period to UINT for correct wrapping.
    inst->tagSleepCorrection_ms = 0; //clear the correction
    inst->instanceTimerEn = 1; //start timer
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
