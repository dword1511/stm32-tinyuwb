#include <decadriver/deca_device_api.h>

#include <decartls/instance.h>


// The table below specifies the default TX spectrum configuration parameters... this has been tuned for DW EVK hardware units
// the table is set for smart power - see below in the instance_config function how this is used when not using smart power
const tx_struct txSpectrumConfig[8] = {
  //Channel 0 ----- this is just a place holder so the next array element is channel 1
  {
      0x0,   //0
      {
          0x0, //0
          0x0 //0
      }
  },
  //Channel 1
  {
      0xc9,   //PG_DELAY
      {
          0x15355575, //16M prf power
          0x07274767 //64M prf power
      }

  },
  //Channel 2
  {
      0xc2,   //PG_DELAY
      {
          0x15355575, //16M prf power
          0x07274767 //64M prf power
      }
  },
  //Channel 3
  {
      0xc5,   //PG_DELAY
      {
          0x0f2f4f6f, //16M prf power
          0x2b4b6b8b //64M prf power
      }
  },
  //Channel 4
  {
      0x95,   //PG_DELAY
      {
          0x1f1f3f5f, //16M prf power
          0x3a5a7a9a //64M prf power
      }
  },
  //Channel 5
  {
      0xc0,   //PG_DELAY
      {
          0x0E082848, //16M prf power
          0x25456585 //64M prf power
      }
  },
  //Channel 6 ----- this is just a place holder so the next array element is channel 7
  {
      0x0,   //0
      {
          0x0, //0
          0x0 //0
      }
  },
  //Channel 7
  {
      0x93,   //PG_DELAY
      {
          0x32527292, //16M prf power
          0x5171B1d1 //64M prf power
      }
  }
};

/* TODO: These won't be useful for us as they are board-dependent. DWM1000 has those in OTP, and will be accounted by end-to-end calibration anyways. */

//these are default antenna delays for EVB1000, these can be used if there is no calibration data in the DW1000,
//or instead of the calibration data
const uint16 rfDelays[2] = {
  (uint16) ((513.9067f / 2.0) * 1e-9 / DWT_TIME_UNITS),//PRF 16
  (uint16) ((514.462f  / 2.0) * 1e-9 / DWT_TIME_UNITS)
};

//these are default TREK Tag/Anchor antenna delays
const uint16 rfDelaysTREK[2] = {
  (uint16) ((514.83f / 2.0) * 1e-9 / DWT_TIME_UNITS),//channel 2
  (uint16) ((514.65f / 2.0) * 1e-9 / DWT_TIME_UNITS) //channel 5
};
