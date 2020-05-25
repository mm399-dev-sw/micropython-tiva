# MicroPython Port for Tiva Launchpad

![Micropython](https://github.com/micropython/micropython/blob/8402c26cfa98b4689f5ac4673952a654cfe5b678/logo/logo.jpg)
![Tiva Launchpad](https://www.ti.com/diagrams/med_ek-tm4c123gxl_tivalp_angle_new.jpg)

## Build

1. Install Toolchain:
  * On Debian: `sudo apt install gcc-arm-none-eabi`
  * On Ubuntu: `sudo apt install gcc-arm-embedded`
  * Other: `gcc-arm-none-eabi` is available on most other distros
  * On Windows you can install WSL with Debian or Ubuntu for easy setup
2. Download & install TivaWare:
   [Here](http://software-dl.ti.com/tiva-c/SW-TM4C/latest/index_FDS.html) 
   - Workaround if no account:
   Find Filename for TivaWare and append to "http://software-dl.ti.com/tiva-c/SWTM4C/latest/exports/"
3. Clone git repo:
   ```bash
   cd <your target directory>
   git clone https://github.com/rk-exxec/micropython.git && git checkout tiva_from_stable
   cd micropython/ports/tm4c123
   ```
4. Adjust makefile and set the path to your TivaWare
5. Build:
   ```bash
   make
   ```
   
## Load

1. Download & install LMFlashprogrammer:
   [http://www.ti.com/tool/lmflashprogrammer](http://www.ti.com/tool/lmflashprogrammer)
2. Download & install drivers:
   [http://www.ti.com/general/docs/lit/getliterature.tsp?literatureNumber=spmc016a&fileType=zip](http://www.ti.com/general/docs/lit/getliterature.tsp?literatureNumber=spmc016a&fileType=zip)
3. Launch Programmer, select "uPY_TM4C123.bin" and load

## Debug

* Download CodeComposerStudio and import the source code as external project
* Configure your debugger to load the uPY_TM4C123.axf

## Connecting

You can connect to the REPL via UART over USB. You need to install the drivers for this to work!

Use these settings:
* Baudrate = 115200
* Data bits = 8
* Stop bits = 1
* Parity = none
* Flowcontrol = none

I recommend using **minicom** on Linux or **PuTTY** on Windows.

Alternatively you can use [**mpfshell**](https://github.com/wendlers/mpfshell):
```bash
python -m pip install mpfshell
```

## Usage

Currently, these Modules are available:
* umachine
  * Pin
  * SPI
  * UART
  * SDCard
* uos
---
### SD-Card

Connect the SD-Card in SPI-mode according to this table:  

| Board Pin | SD-Pin           |
| --------: | :--------------- |
|       PB3 | Card Detect (CD) |
|       PB4 | Clock (SCK)      |
|       PB5 | Chip Select (CS) |
|       PB6 | Data Out (DO)    |
|       PB7 | Data In (DI)     |

---
### Pin

See also: 
<http://docs.micropython.org/en/latest/library/machine.Pin.html>

    machine.Pin(id, mode, pull, *, value, drive, alt)

| Parameter | Function                                         | value range                                  | default |
| --------: | :----------------------------------------------- | :-------------------------------------------- | :----------- |
|        id | Pin ID as Number, String or Tuple \[port, pin\] | P\[A..F\]\[0..8\]                             | \-           |
|      mode | Pin mode                                        | IN, OUT, OPEN\_DRAIN, ALT                     | \-           |
|      pull | PullUp or  Down                                 | PULL\_NONE, PULL\_UP, PULL\_DOWN, OPEN\_DRAIN | PULL\_NONE   |
|     value | Pin value                                       | 0, 1                                          | 0            |
|     drive | Strength of the pad driver                          | LOW\_POWER, MED\_POWER, HIGH\_POWER           | LOW\_POWER   |
|       alt | ID of the alternative function                    | 0..15                                         | \-           |

#### Special function

    <Pin>.unlock() 

This function unlocks special pins on the Launchpad. This needs to be done to reassign another AF to these pins.
\<Pin\> is the object of the pin you wish to unlock.  
If you try to change a locked pin, you will see an error message.
---
### UART

See also:
<http://docs.micropython.org/en/latest/library/machine.Pin.html>



    machine.UART(id, baudrate, bits, parity, stop, *, ...)

|      Parameter | Function                            | value range | default |
| -------------: | :---------------------------------- | :----------- | :----------- |
|             id | UART ID als Zahl                    | 0..7         | \-           |
|       baudrate | Baudrate                            | bis 5Mbits   | \-           |
|           bits | word length                           | 5..8         | 8            |
|         parity | parity type                        | ODD, EVEN    | NONE         |
|           stop | stop bits                           | 1, 2         | 1            |
|           flow | flowcontrol (currently broken)                        | ~~RTS, CTS~~     | NONE         |
|        timeout | timeout in ms  | >= 0    | 10           |
|  timeout\_char | timeout per char in ms | >= 0    | 0            |
| read\_buf\_len | read buffer size               | 0    | 64           |

UART0 is used by the command line. Changing this unit may break the command line  
UART1 is currently not available
---
### SPI

See also:
<http://docs.micropython.org/en/latest/library/machine.SPI.html>

    SPI(id, mode, *, baudrate, polarity, phase, protocol, bits, firstbit, fss, dma, sck, mosi, miso)

|       Parameter | Function                        | value range               | default |
| --------------: | :------------------------------ | :------------------------- | :----------- |
|              id | Unit number           | 0..3                       | 0            |
|            mode | SPI Mode                       | MASTER, SLAVE, SLAVE\_OD   | MASTER       |
|        baudrate | Clock rate    | \<25M Master, \<6e6 Slave | 5e5          |
|           phase | SPI phase       | FIRST\_EDGE, SECOND\_EDGE  | FIRST\_EDGE  |
|        polarity | SPI polarity      | IDLE\_LOW, IDLE\_HIGH      | IDLE\_LOW    |
|        protocol | SSI protocol                       | SPI0..SPI3, MICROWIRE, TI  | SPI0         |
|            bits | word length                       | 4..16                      | 8            |
|        firstbit | bit order | MSB, LSB                   | MSB          |
|             fss | chip select/ frame control      | FSS\_SOFT, FSS\_HARD       | FSS\_HARD    |
|             ~~dma~~ | use DMA? (not implemented)              | ~~True, False~~                |     False         |
| ~~sck, mosi, miso~~ | (not implemented)       |                            |              |
 
If *protocol* is given, *phase* and *polarity* will be **ignored**!
