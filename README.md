# spartan-edge-accelerator-arduino-qspi

Arduino code companion for the [Spartan Edge Accelerator Graphical System](https://github.com/smartperson/spartan-edge-accelerator-graphical-system) project.

The goal is to develop an application framework for the ESP32 that is included on Seeed's Spartan Edge Accelerator board.
The application does nothing practical, but helps to resolve unknowns and project risks:
* Proves that development in the Arudino IDE works for this chip on this board
* Sets up interrupts that are triggered by the spartan graphical system's VBLANK
* Initializes the QSPI controller on the on-board ESP32
* Sends high-speed data fast enough that it arrives during VBLANK that the spartan graphical system can use it to draw the next frame

How to use:
* Install the latest Arduino IDE
* Install support for the DOIT ESP32 DEVKIT V1
* Build and deploy to ESP32
* Generate bitstream and program the Xilinx FPGA using the [Spartan Edge Accelerator Graphical System](https://github.com/smartperson/spartan-edge-accelerator-graphical-system) project in Vivado
