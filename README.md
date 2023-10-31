stm32-usbcdcacm : Open a usb cdcacm port for input and output from blue (stm32f103) and black (stm32f411) pill boards
===============================

Run `make` to build, then flash the `src/usb-in-out.bin` file.
Plug the pill into your PC using USB, and a virtual (ACM CDC, "/dev/ttyACMx") serial port should appear.

This started as a triple usb-uart converter by satoshinm [pill_serial: USB-to-serial x 3](https://github.com/satoshinm/pill_serial) which in turn was
heavily based on the [Black Magic Debug Probe firmware](https://github.com/blacksphere/blackmagic).

Their respective copyrights still apply, credit for solid useful code goes to them, blame for sloppy, buggy code is mine.
