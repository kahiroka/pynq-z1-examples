# PYNQ Z1 Examples

Precondition
- PYNQ-Z1 v2.3 SDCard image
- SDK branch: image\_v2.4

## hdmi loopback test
Connect hdmi-out to hdmi-in as prerequisite.

## arduino uart
Connect IO0/IO1/GND of PYNQ to RX/TX/GND of USB-UART cable or something. UART signal is 3.3v and its speed is 9600 bps. Python script and microblaze binary, which are based on arduino\_devmode.py and arduino\_mailbox.bin, need to be installed as below because Arduino pins are under control of microblaze processor if you use 'base.bit' overlay.

    pynq$ cd pynq-z1-examples/pynq/lib/arduino
    pynq$ make
    pynq$ cp arduino_uart.bin /usr/local/lib/python3.6/dist-packages/pynq/lib/arduino/
    pynq$ cp arduino_uart.py /usr/local/lib/python3.6/dist-packages/pynq/lib/arduino/

Add one line below to /usr/local/lib/python3.6/dist-packages/pynq/lib/arduino/\_\_init\_\_.py

    from .arduino_uart import Arduino_Uart
