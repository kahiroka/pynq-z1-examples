# PYNQ Z1 Examples

Precondition
- pynq\_z1\_v2.3.zip
- SDK branch: image\_v2.4

## hdmi loopback test
Connect hdmi-out to hdmi-in as prerequisite.

## arduino uart
Connect IO0/IO1 of PYNQ to RX/TX of USB serial or something. UART signal is 3.3v and its speed is 9600 bps. Python script and microblaze binary need to be installed as below because Arduino pins are under control of microblaze processor if you use 'base.bit' overlay.

    pynq$ cd pynq-z1-examples/pynq/lib/arduino
    pynq$ make
    pynq$ cp arduino_uart.bin /usr/local/lib/python3.6/dist-packages/pynq/lib/arduino/
    pynq$ cp arduino_uart.py /usr/local/lib/python3.6/dist-packages/pynq/lib/arduino/

Add one line below to /usr/local/lib/python3.6/dist-packages/pynq/lib/arduino/\_\_init\_\_.py

    from .arduino_uart import Arduino_Uart
