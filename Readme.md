# FPGA ESC
A litex SoC target for the TinyFPGA-BX platform implementing a usb wishbone bridge.

This SoC follows the Fomu LiteX target as closely as possible except for the system clock.

Vague intentions to make a brushless ESC with linux drivers


## Usage:
Download the Fomu toolchain and add its `bin` directory to your `$PATH`\
[https://github.com/im-tomu/fomu-toolchain]

Install the dependencies\
`pip3 install -r requirements.txt`

Build the thing\
`python3 integrated.py`

Upload the binary\
`tinyprog -p build/gateware/tinyfpga_bx.bin`


## Probable pitfalls:
* yosys and nextpnr should come from the Fomu toolchain - check these with `which yosys` etc, and shuffle your path accordingly
* dependency hell - open an issue, I want to fix this.

