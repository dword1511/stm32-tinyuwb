/* Detached from dw_main.c. TODO: fix naming */
#include <decartls/instance.h>

//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
const instanceConfig_t chConfig[4] ={
                    //mode 1 - S1: 2 off, 3 off
                    {
                        .channelNumber = 2,             // channel
                        .preambleCode = 4,              // preambleCode
                        .pulseRepFreq = DWT_PRF_16M,    // prf
                        .dataRate = DWT_BR_110K,        // datarate
                        .preambleLen = DWT_PLEN_1024,   // preambleLength
                        .pacSize = DWT_PAC32,           // pacSize
                        .nsSFD = 1,                     // non-standard SFD
                        .sfdTO = (1025 + 64 - 32)       // SFD timeout
                    },
                    //mode 2 - S1: 2 on, 3 off
                    {
						.channelNumber = 2,            // channel
						.preambleCode = 4,             // preambleCode
						.pulseRepFreq = DWT_PRF_16M,   // prf
						.dataRate = DWT_BR_6M8,        // datarate
						.preambleLen = DWT_PLEN_128,   // preambleLength
						.pacSize = DWT_PAC8,           // pacSize
						.nsSFD = 0,                    // non-standard SFD
						.sfdTO = (129 + 8 - 8)        // SFD timeout
                    },
                    //mode 3 - S1: 2 off, 3 on
                    {
						.channelNumber = 5,             // channel
						.preambleCode = 3,              // preambleCode
						.pulseRepFreq = DWT_PRF_16M,    // prf
						.dataRate = DWT_BR_110K,        // datarate
						.preambleLen = DWT_PLEN_1024,   // preambleLength
						.pacSize = DWT_PAC32,           // pacSize
						.nsSFD = 1,                     // non-standard SFD
						.sfdTO = (1025 + 64 - 32)       // SFD timeout
                    },
                    //mode 4 - S1: 2 on, 3 on
                    {
						.channelNumber = 5,            // channel
						.preambleCode = 3,             // preambleCode
						.pulseRepFreq = DWT_PRF_16M,   // prf
						.dataRate = DWT_BR_6M8,        // datarate
						.preambleLen = DWT_PLEN_128,   // preambleLength
						.pacSize = DWT_PAC8,           // pacSize
						.nsSFD = 0,                    // non-standard SFD
						.sfdTO = (129 + 8 - 8)        // SFD timeout
                    }
};
//Slot and Superframe Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
const sfConfig_t sfConfig[4] ={
                     //mode 1 - S1: 2 off, 3 off
					{
						.slotDuration_ms = (28), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*28),  //in ms => 280ms frame means 3.57 Hz location rate
						.tagPeriod_ms = (10*28), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (20000) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
					},
                    //mode 2 - S1: 2 on, 3 off
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*10),  //in ms => 100 ms frame means 10 Hz location rate
						.tagPeriod_ms = (10*10), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)

                    },

                    //mode 3 - S1: 2 off, 3 on
                   {
						.slotDuration_ms = (28), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*28),  //in ms => 280ms frame means 3.57 Hz location rate
						.tagPeriod_ms = (10*28), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (20000) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
					},
                    //mode 4 - S1: 2 on, 3 on
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*10),  //in ms => 100 ms frame means 10 Hz location rate
						.tagPeriod_ms = (10*10), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
                    }
};

