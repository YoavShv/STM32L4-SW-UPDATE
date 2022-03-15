# STM32 SW UPDATE
This project demonstrates how to update SW FIRMWARE on the STM32 MCU.  
Here I use the evaluation board NUCLEO-L412RB-P with the MCU STM32L412RBxP.  
https://www.st.com/en/evaluation-tools/nucleo-l412rb-p.html

## Table of Contents
- [Repository organization](#Repository-organization)
- [Tools](#Tools)
- [Debug](#Debug)

## Repository organization
### lib
The `lib` folder contains the source files which implement the SW update mechanism for 
ST MCU.  

The next 2 folders contain 2 projects with simple Makefile mechanism.   
Use `make` and `make clear` commands to re/compile the projects.

### sw-update
The `sw_update` folder contains an example project showing how to use the sw_update source files.  
It is divided into several folders:  
The `AC_CMSIS` folder contains the CMSIS (Cortex Microcontroller Software Interface Standard) drivers from ST.  
The `AC_debug` folder contains files for supporting printf() and LED operations.  
The `AC_src` folder contains the project's main functions.  
The `AC_src/AC_new_sw_example.h` file contains the bin content of the **blink_led** project,
used as a "SW update".  
The project takes the new bin, writes it to start address 0x800F000 (flash base address @ page 30) and jumps to the new code.  

### blink-led
The `blink_led` folder contains an example project of a 10 times blinking led project 
that is used as a SW update project.  
Its bin sits in the flash, starting from address 0x800F000 (flash base address @ page 30).
To support this, the STM32L422RBTx_FLASH.ld file is rewritten with:  
```
MEMORY
{
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 40K
FLASH (rx)      : ORIGIN = 0x800F000, LENGTH = 64K
}
```  
User can run BLINK-LED project as a standalone SW, but keep in mind to flash to address 0x0800F000 with: `st-flash write BLINK-LED.bin`

## Tools
### stlink
***installation:***  
Linux users should install the stlink utilities for getting board info, flashing and more:  
1. install the necessary libraries and build tools
```
sudo apt-get install git make cmake libusb-1.0-0-dev
sudo apt-get install gcc build-essential
```
2. copy binaries & udev rules to their place
```
sudo cp installations/stlink/stlink-out/st-* /usr/local/bin
sudo cp installations/stlink/stlink-out/*.so* /lib32
sudo cp installations/stlink/stlink-out/49-stlinkv* /etc/udev/rules.d/
```
3. check everything is OK by connecting the ST board NUCLEO-L412RB-P to the USB port and run `st-info --descr`. You should get output of: `L41x`  

4. For more info, visit  
https://freeelectron.ro/installing-st-link-v2-to-flash-stm32-targets-on-linux/

***how to use:***  
To flash a bin file to the board do the following:  
1. Erase flash: `st-flash erase`
2. Press the board RESET button
3. Burn image: `st-flash write build/SW-UPDATE.bin 0x08000000` 

To read the flash content do: `st-flash read myFlash  0x08000000 0x1000`
It will read into ***myFlash*** bin file from 0x08000000 address, 4096 bytes.

To view them use: `cat myFlash |hd` or `xxd myFlash`

For more info, visit https://github.com/stlink-org/stlink

## Debug
### Printf() 
The files AC_debug.c and AC_debug.h are added to use printf() in the code.
In order to see the printf output follow these steps:
1. Connect the USART-USB converter Rx  pin to PC10 (CN5 pin 1).
2. Connect the USART-USB converter Tx  pin to PC11 (CN5 pin 2).
3. Connect the USART-USB converter GND pin to board GND (CN5 pin 20).
4. Connect USART-USB converter to your PC and open the corresponding terminal with:  
`picocom -b 115200 /dev/ttyUSB0`


