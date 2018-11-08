#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#include <stdint.h>

#include <decadriver/deca_device_api.h>
#include <decadriver/deca_types.h>

/* You have stdint.h, why do this? */
typedef uint64_t uint64;
typedef int64_t int64;
/* You have stdbool.h, why do this? */
#define FALSE 0
#define TRUE  1

/******************************************************************************************************************
********************* NOTES on TREK compile/build time features/options ***********************************************************
*******************************************************************************************************************/
#define DEEP_SLEEP (0) //To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode while it is
//waiting for start of next ranging exchange


/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define SPEED_OF_LIGHT    (299702547.0)   // in m/s in air
#define MASK_40BIT      (0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS      (0x00FFFFFFFE00)  // The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

//! callback events
#define DWT_SIG_RX_NOERR      0
#define DWT_SIG_TX_DONE       1     // Frame has been sent
#define DWT_SIG_RX_OKAY       2     // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR      3     // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT      4     // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE      6     // ACK frame has been sent (as a result of auto-ACK)
#define DWT_SIG_RX_BLINK      7    // Received ISO EUI 64 blink message
#define DWT_SIG_RX_PHR_ERROR    8     // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS     9     // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT     10    // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT     11    // Got preamble detection timeout (no preamble detected)

#define DWT_SIG_TX_PENDING      12    // TX is pending
#define DWT_SIG_TX_ERROR      13    // TX failed
#define DWT_SIG_RX_PENDING      14    // RX has been re-enabled
#define DWT_SIG_DW_IDLE       15    // DW radio is in IDLE (no TX or RX pending)

#define SIG_RX_UNKNOWN        99    // Received an unknown frame

//DecaRTLS frame function codes
#define RTLS_DEMO_MSG_RNG_INIT        (0x71)      // Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL        (0x81)      // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP       (0x70)      // Anchor response to poll
#define RTLS_DEMO_MSG_ANCH_POLL        (0x7A)      // Anchor to anchor poll message
#define RTLS_DEMO_MSG_ANCH_RESP2      (0x7B)      // Anchor response to poll from anchor
#define RTLS_DEMO_MSG_ANCH_FINAL      (0x7C)      // Anchor final massage back to Anchor
#define RTLS_DEMO_MSG_TAG_FINAL       (0x82)      // Tag final massage back to Anchor


//lengths including the Decaranging Message Function Code byte
//absolute length = 17 +
#define RANGINGINIT_MSG_LEN          5        // FunctionCode(1), Sleep Correction Time (2), Tag Address (2)

//absolute length = 11 +
#define TAG_POLL_MSG_LEN          2        // FunctionCode(1), Range Num (1)
#define ANCH_RESPONSE_MSG_LEN         8         // FunctionCode(1), Sleep Correction Time (2), Measured_TOF_Time(4), Range Num (1) (previous)
#define TAG_FINAL_MSG_LEN           33        // FunctionCode(1), Range Num (1), Poll_TxTime(5),
                              // Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Final_TxTime(5), Valid Response Mask (1)
#define ANCH_POLL_MSG_LEN_S          2        // FunctionCode(1), Range Num (1),
#define ANCH_POLL_MSG_LEN           4        // FunctionCode(1), Range Num (1), Next Anchor (2)
#define ANCH_FINAL_MSG_LEN          33        // FunctionCode(1), Range Num (1), Poll_TxTime(5),
                              // Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Final_TxTime(5), Valid Response Mask (1)
#define MAX_MAC_MSG_DATA_LEN        (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE     127

#define ADDR_BYTE_SIZE_L      (8)
#define ADDR_BYTE_SIZE_S      (2)

#define FRAME_CONTROL_BYTES     2
#define FRAME_SEQ_NUM_BYTES     1
#define FRAME_PANID         2
#define FRAME_CRC          2
#define FRAME_SOURCE_ADDRESS_S    (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S      (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L    (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L      (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP          (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L  (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S  (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS  (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for one 16-bit address and one 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL   (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS   (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS   (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING  MAX_USER_PAYLOAD_STRING_LL

#define BLINK_FRAME_CONTROL_BYTES     (1)
#define BLINK_FRAME_SEQ_NUM_BYTES     (1)
#define BLINK_FRAME_CRC          (FRAME_CRC)
#define BLINK_FRAME_SOURCE_ADDRESS    (ADDR_BYTE_SIZE_L)
#define BLINK_FRAME_CTRLP        (BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS  (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes
#define BLINK_FRAME_LEN_BYTES       (BLINK_FRAME_CRTL_AND_ADDRESS + BLINK_FRAME_CRC)


#if (DISCOVERY == 0)
#define MAX_TAG_LIST_SIZE        (8)
#else
#define MAX_TAG_LIST_SIZE        (100)
#endif


#define MAX_ANCHOR_LIST_SIZE      (4) //this is limited to 4 in this application
#define NUM_EXPECTED_RESPONSES      (3) //e.g. MAX_ANCHOR_LIST_SIZE - 1

#define GATEWAY_ANCHOR_ADDR        (0x8000)
#define A1_ANCHOR_ADDR          (0x8001)
#define A2_ANCHOR_ADDR          (0x8002)
#define A3_ANCHOR_ADDR          (0x8003)

#define WAIT4TAGFINAL          2
#define WAIT4ANCFINAL          1

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define INST_DONE_WAIT_FOR_NEXT_EVENT     1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO  2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                        //which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET           0   //this signifies that the instance is still processing the current event

//application data message byte offsets
#define FCODE                 0         // Function code is 1st byte of messageData
#define PTXT                2        // Poll TX time
#define RRXT0                 7        // A0 Response RX time
#define RRXT1                 12        // A1 Response RX time
#define RRXT2                 17        // A2 Response RX time
#define RRXT3                 22        // A3 Response RX time
#define FTXT                27        // Final TX time
#define VRESP                 32        // Mask of valid response times (e.g. if bit 1 = A0's response time is valid)
#define RES_TAG_SLP0            1         // Response tag sleep correction LSB
#define RES_TAG_SLP1            2         // Response tag sleep correction MSB
#define TOFR                3        // ToF (n-1) 4 bytes
#define TOFRN                7        // range number 1 byte
#define POLL_RNUM               1         // Poll message range number
#define POLL_NANC              2        // Address of next anchor to send a poll message (e.g. A1)
#define RES_TAG_ADD0            3         // Tag's short address (slot num) LSB
#define RES_TAG_ADD1            4         // Tag's short address (slot num) MSB


#define BLINK_PERIOD    (2000) //ms (Blink at 2Hz initially)

#define DW_RX_ON_DELAY    (16) //us - the DW RX has 16 us RX on delay before it will receive any data

//this it the delay used for configuring the receiver on delay (wait for response delay)
//NOTE: this RX_RESPONSE_TURNAROUND is dependent on the microprocessor and code optimisations
/* TODO: optimize this... value would be different when we have logging. Unit should be in usec. Originally 300. With logging it gets huge... */
#define RX_RESPONSE_TURNAROUND (300) //this takes into account any turnaround/processing time (reporting received poll and sending the response)

#define PTO_PACS        (3)   //tag will use PTO to reduce power consumption (if no response coming stop RX) // TODO: should we get rid of this?

//Tag will range to 3 or 4 anchors
//Each ranging exchange will consist of minimum of 3 messages (Poll, Response, Final)
//and a maximum of 6 messages (Poll, Response x 4, Final)
//Thus the ranging exchange will take either 28 ms for 110 kbps and 5 ms for 6.81 Mbps.
//NOTE: the above times are for 110k rate with 64 symb non-standard SFD and 1024 preamble length



typedef enum instanceModes{TAG, ANCHOR, ANCHOR_RNG, NUM_MODES} INST_MODE;
typedef enum instanceTWRModes{INITIATOR, RESPONDER_A, RESPONDER_B, RESPONDER_T, LISTENER, GREETER, ATWR_MODES} ATWR_MODE;


#define TOF_REPORT_NUL 0
#define TOF_REPORT_T2A 1
#define TOF_REPORT_A2A 2

#define INVALID_TOF (0xABCDFFFF)

typedef enum inst_states
{
  TA_INIT, //0

  TA_TXE_WAIT,        //1 - state in which the instance will enter sleep (if ranging finished) or proceed to transmit a message
  TA_TXBLINK_WAIT_SEND,     //2 - configuration and sending of Blink message
  TA_TXPOLL_WAIT_SEND,    //2 - configuration and sending of Poll message
  TA_TXFINAL_WAIT_SEND,     //3 - configuration and sending of Final message
  TA_TXRESPONSE_WAIT_SEND,  //4 - a place holder - response is sent from call back
  TA_TX_WAIT_CONF,      //5 - confirmation of TX done message

  TA_RXE_WAIT,        //6
  TA_RX_WAIT_DATA,      //7

  TA_SLEEP_DONE,         //8
  TA_TXRESPONSE_SENT_POLLRX,   //9
  TA_TXRESPONSE_SENT_RESPRX,   //10
  TA_TXRESPONSE_SENT_TORX,   //11
  TA_TXRESPONSE_SENT_APOLLRX,  //12
  TA_TXRESPONSE_SENT_ARESPRX   //13

} INST_STATES;


typedef struct {
  uint8 frameCtrl[2];
  uint8 seqNum;
  uint8 panID[2];
  uint8 destAddr[ADDR_BYTE_SIZE_L];
  uint8 sourceAddr[ADDR_BYTE_SIZE_S];
  uint8 messageData[MAX_USER_PAYLOAD_STRING_LS];
  uint8 fcs[2];
} srd_msg_dlss;

typedef struct {
  uint8 frameCtrl[2];
  uint8 seqNum;
  uint8 panID[2];
  uint8 destAddr[ADDR_BYTE_SIZE_S];
  uint8 sourceAddr[ADDR_BYTE_SIZE_S];
  uint8 messageData[MAX_USER_PAYLOAD_STRING_SS];
  uint8 fcs[2];
} srd_msg_dsss;

typedef struct {
  uint8 channelNumber;
  uint8 preambleCode;
  uint8 pulseRepFreq;
  uint8 dataRate;
  uint8 preambleLen;
  uint8 pacSize;
  uint8 nsSFD;
  uint16 sfdTO;
} instanceConfig_t ;

typedef struct {
  uint16 slotDuration_ms;
  uint16 numSlots;
  uint16 sfPeriod_ms;
  uint16 tagPeriod_ms;
  uint16 pollTxToFinalTxDly_us;
} sfConfig_t;

//size of the event queue, in this application there should be at most 2 unprocessed events,
//i.e. if there is a transmission with wait for response then the TX callback followed by RX callback could be executed
//in turn and the event queued up before the instance processed the TX event.
#define MAX_EVENT_NUMBER (4)

typedef struct {
  uint8  type; // if 0 there is no event in the queue
  uint8  typePend; // DW is not in IDLE (TX/RX pending)
  uint16 rxLength;

  uint64 timeStamp; // 40 bit DW1000 time

  uint32 timeStamp32l; // low 32 bits of the 40 bit DW1000 time
  uint32 timeStamp32h; // high 32 bits of the 40 bit DW1000 time

  union {
  uint8   frame[STANDARD_FRAME_SIZE];
  srd_msg_dlss rxmsg_ls;
  srd_msg_dsss rxmsg_ss; //16 bit addresses
  } msgu;
} event_data_t ;

// TX power and PG delay configuration structure
typedef struct {
  uint8 pgDelay;

  //TX POWER
  //31:24   BOOST_0.125ms_PWR
  //23:16   BOOST_0.25ms_PWR-TX_SHR_PWR
  //15:8    BOOST_0.5ms_PWR-TX_PHR_PWR
  //7:0     DEFAULT_PWR-TX_DATA_PWR
  uint32 txPwr[2];
} tx_struct;

typedef struct
{
  INST_MODE mode;        //instance mode (tag or anchor)
  ATWR_MODE twrMode;
  INST_STATES testAppState ;      //state machine - current state
  INST_STATES nextState ;        //state machine - next state
  INST_STATES previousState ;      //state machine - previous state

  //configuration structures
  dwt_config_t  configData ;  //DW1000 channel configuration
  dwt_txconfig_t  configTX ;    //DW1000 TX power configuration
  uint16      txAntennaDelay ; //DW1000 TX antenna delay
  uint16      rxAntennaDelay ; //DW1000 RX antenna delay
  uint32       txPower ;     //DW1000 TX power
  uint8 txPowerChanged ;      //power has been changed - update the register on next TWR exchange

  uint16 instanceAddress16; //contains tag/anchor 16 bit address

  //timeouts and delays
  int32 tagPeriod_ms; // in ms, tag ranging + sleeping period
  int32 tagSleepTime_ms; //in milliseconds - defines the nominal Tag sleep time period
  int32 tagSleepRnd_ms; //add an extra slot duration to sleep time to avoid collision before getting synced by anchor 0

  //this is the delay used for the delayed transmit
  uint64 pollTx2FinalTxDelay ; //this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit)
  uint64 pollTx2FinalTxDelayAnc ; //this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit) for Anchor to Anchor ranging
  uint32 fixedReplyDelayAnc32h ; //this is a delay used for calculating delayed TX/delayed RX on time (units: 32bit of 40bit DW time)
  uint32 preambleDuration32h ; //preamble duration in device time (32 MSBs of the 40 bit time)
  uint32 tagRespRxDelay_sy ; //TX to RX delay time when tag is awaiting response message an another anchor

  int fwto4RespFrame_sy ; //this is a frame wait timeout used when awaiting reception of Response frames (used by both tag/anchor)
  int fwto4FinalFrame_sy ; //this is a frame wait timeout used when awaiting reception of Final frames
  uint32 delayedTRXTime32h;    // time at which to do delayed TX or delayed RX (note TX time is time of SFD, RX time is RX on time)

  //message structure used for holding the data of the frame to transmit before it is written to the DW1000
  srd_msg_dsss msg_f ; // ranging message frame with 16-bit addresses

  //Tag function address/message configuration
  uint8   shortAdd_idx ;        // device's 16-bit address low byte (used as index into arrays [0 - 3])
  uint8   eui64[8];        // device's EUI 64-bit address
  uint16  psduLength ;      // used for storing the TX frame length
  uint8   frameSN;        // modulo 256 frame sequence number - it is incremented for each new frame transmission
  uint16  panID ;          // panid used in the frames

  //64 bit timestamps
  //union of TX timestamps
  union {
    uint64 txTimeStamp;       // last tx timestamp
    uint64 tagPollTxTime;       // tag's poll tx timestamp
  } txu;
  uint32 tagPollTxTime32h;

  //application control parameters
  uint8  wait4ack;        // if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
  uint8   wait4final;

  uint8   instToSleep;      // if set the instance will go to sleep before sending the blink/poll message
  uint8  instanceTimerEn;    // enable/start a timer
  uint32  instanceWakeTime_ms;  // micro time at which the tag was waken up
  uint32  nextWakeUpTime_ms;    // micro time at which to wake up tag

  /* NOTE: this dude needs expansion if we change MAX_ANCHOR_LIST_SIZE */
  uint8   rxResponseMask;      // bit mask - bit 0 = received response from anchor ID = 0, bit 1 from anchor ID = 1 etc...
  uint8  rangeNum;        // incremented for each sequence of ranges (each slot)

  int8  rxResps;        // how many responses were received to a poll (in current ranging exchange)
  int8  remainingRespToRx ;    // how many responses remain to be received (in current ranging exchange)

  uint16  sframePeriod_ms;    // superframe period in ms
  uint16  slotDuration_ms;    // slot duration in ms
  int32   tagSleepCorrection_ms;  // tag's sleep correction to keep it in it's assigned slot

  //event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
  event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
  uint8 dweventIdxOut;
  uint8 dweventIdxIn;
  uint8 dweventPeek;
  uint8 monitor;
  uint32 timeofTx;

  uint8 smartPowerEn;

  event_data_t dw_event_g; /* Was in instance_common.c used by instance_getevent(). */
} instance_data_t ;

//-------------------------------------------------------------------------------------------------------------
//
//  Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------

// Call init, then call config, then call run.
// initialise the instance (application) structures and DW1000 device
int instance_init(int role);
// configure the instance and DW1000 device
void instance_config(const instanceConfig_t *config, const sfConfig_t *sfconfig) ;

// configure the MAC address
void instance_set_16bit_address(uint16 address) ;
void instance_config_frameheader_16bit(instance_data_t *inst);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int tag_run(void) ;

// configure TX/RX callback functions that are called from DW1000 ISR
void rx_ok_cb_tag(const dwt_cb_data_t *cb_data);
void rx_err_cb_tag(const dwt_cb_data_t *cb_data);
void tx_conf_cb(const dwt_cb_data_t *cb_data);

#define instance_readdeviceid() dwt_readdevid()

int instance_send_delayed_frame(instance_data_t *inst, int delayedTx);

void instance_seteventtime(event_data_t *dw_event, uint8* timeStamp);
int instance_peekevent(void);
void instance_saveevent(event_data_t newevent, uint8 etype);
event_data_t instance_getsavedevent(void);
void instance_putevent(event_data_t newevent, uint8 etype);
event_data_t* instance_getevent(int x);
void instance_clearevents(void);

// configure the TX power
void instance_config_txpower(uint32 txpower);
void instance_set_txpower(void);

instance_data_t* instance_get_local_structure_ptr(unsigned int x);

#endif
