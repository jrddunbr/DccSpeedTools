# DCC Speed Tools

This firmware is one part of a project designed to assist with DCC speed matching.

## Electrical Hookup

This project currently expects the following:

* A supported dev board: STM32 Nucleo F446RE or Adafruit Metro ESP32-S3 N16R8 (or a device more powerful than your typical ATMega 328p)
* A Sparkfun or equivalent I2C MUX board (TCA9548A)
* A pair of Adafruit VL53L4CX ToF Distance sensors connected on MUX 0 and MUX 1
* A connection between the dev board and a computer with an open USB port.

## Setup (PlatformIO)

Install PlatformIO if you do not already have it:
https://docs.platformio.org/en/latest/core/installation.html

Then from the project root:

```bash
pio project init --ide clion
pio run
pio run -t upload
```

Use `pio run -e adafruit_metro_esp32s3` and `pio run -e adafruit_metro_esp32s3 -t upload` when targeting the Metro ESP32-S3.
If PlatformIO asks to install or update the `espressif32` platform, allow it so the ESP32-S3 toolchain is available.

## Neopixel State Colors

If the dev board exposes a NeoPixel (for example, the Metro ESP32-S3), the firmware displays the transit state machine on it:

* Dim white: idle/not armed yet (waiting for both sensors to be clear)
* Green: armed/ready (both sensors clear)
* Blue: waiting for the second sensor (transit in progress)
* Amber: blocked (object present, not waiting for second sensor)

## Track Length

Suggested straight track length: minimum 20cm, optimal 50cm or more.

This allows for more precision in the calculated speed.
At distances under 20cm, there is little time for the sensors to register the motion at higher speeds.
This is because the sensor takes a set time (roughly 12ms) to update with the distance measurement.
With a longer distance between sensors, 1% error or lower can be achieved.

## Serial Interface Protocol

A custom protocol defined in [protocol.md](protocol.md) provides computer interface using the CBOR binary data exchange standard to be used over serial.

The protocol is also used by a companion tool in a separate GitHub repository:
[DccSpeedToolsPC](https://github.com/jrddunbr/DccSpeedToolsPC)
