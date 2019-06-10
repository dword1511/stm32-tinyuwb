# Tiny, Ultra-low-power UWB Localization Tag

This is a UWB localization tag based on Decawave `DWM1000` module and STMicro `STM32L031F6P6` ultra-low-power MCU.
The average power consumption is less than 5 mW in 6.8 Mbps mode (10 Hz update rate).

The tag has a built-in `LTC3588-1` energy harvester. WARNING: do not use `LTC3588-2`, their functionalities are very different!
The harvester can be left unpopulated and bypassed by connecting +3V3 directly to an external power source,
and then disabling the power-good logic in the source code.

## General Build Instructions

You will need the following packages (Debian/Ubuntu):

* build-essential
* gcc-arm-none-eabi
* git
* stlink-tools
* gdb-multiarch

You will have to flash and debug via the SWD test-points. Fine air-wire soldering skill required.

This project uses `libopencm3`. It will be automatically checked out during the building process.
To build the firmware, run `make`. To flash it, connect the board to a ST-Link, and then run `make flash`.
Alternatively, you can use `openocd` and any SWD debugger you fancy.

## Publication

This project was part of the following work. Please kindly consider citing the paper as shown:

```
@inproceedings{Capttery,
 author = {Zhang, Chi and Kumar, Sidharth and Bharadia, Dinesh},
 title = {{Capttery: Scalable Battery-like Room-level Wireless Power}},
 booktitle = {Proc. of ACM MobiSys},
 year = {2019},
}
```
