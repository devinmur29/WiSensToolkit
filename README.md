# WiSensToolkit

Arduino library is in lib/WisensToolkit

## For Development

This library is set up to be developed with pioarduino (built on top of platformio).

### Installation

Install pioarduino via vscode: https://github.com/pioarduino/platform-espressif32 (follow steps under IDE preparation)

Clone this repo or download as a zip.

Open this folder. The pioarduino extension should recognize it and configure the environment/install necessary packages.

Install the CP210x driver if you haven't already, which you will need for your laptop to recognize the microcontrollers: https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads 

### Get started



Connect the esp32 and open the vscode command window (Ctrl + shift + P). Search "Set project port" and select the serial port of the connected device accordingly.

The upload button will flash the "nocalibrate.cpp" source to the device (this is the most up-to-date firmware)
![upload](upload.png)

### Known Issues

I don't think pioarduino likes being installed simultatenously with a pre-exisitng installation of platformio. If you already have platformio install on your machine, please uninstall it from vscode, delete the .platformio folder (usually under users/`<your name>`), and install pioarduino as described above.
