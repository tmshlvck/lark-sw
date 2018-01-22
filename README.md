# Lark Software Part

## Description

Project Lark is a varometer sensor component for glider pilots. Lark is an affordable built-in device that feeds data to a glide computer running on external smart phone or tablet.

The Lark HW & SW forms a device that physically connects to pitot-static system and compensated Total Energy pressure tube, continuously measures pressure in two absolute and one differential channels, computes total energy, airspeed and baro altitude and feeds data to an external glide computer over a WiFi connection. Lark also generates and produces audio output (vario beeping), using built-in speaker.

The software design, connectivity and core is heavily inspired by OpenVario project (http://openvario.org/) and I am reusing certain portions of the code. The protocol for connecting a glide computer is also taken from OpenVario. Currently XCSoar (https://www.xcsoar.org/) with OpenVario protocol is supported.

See Big Picture:
![lark-bigpic](https://user-images.githubusercontent.com/1937910/35197346-3f1961de-fede-11e7-9c7e-9617e9dce038.png)

## Software internals are build on ESP32 framework ESP-IDF.
Existing & planned components:
* MS5611 - taken from Crazyflie-Firmware, more rigorous rewrite pending.
* Audio Synthesis - inspired by OpenVario, but rewritten for ESP32 that has lower FPU performance and could not support the original waveshape synthesis computation.
* Network connectivity.
* TE vario computation - Taken from OpenVario, ported for FreeRTOS timing with ESP32 hi-res timer.
* Airspeed computation - TBD
* Controls - currently the HW prototype has one button for mute/unmute.
* Glider settings for STF - TBD.
* STF commander - TBD :-)

## Connecting XCSoar to Lark
* Turn off "Smart WiFi" on your phone
* Connect your smart phone / tablet to Lark AP, SSID: lark-vario, no password
* Accept the warning about a connection without Internet (and stay connected) 
* Go to Config -> Devices
* Add a new device: Port: TCP client, IP address: 192.168.4.1, TCP port: 4353, Driver: OpenVario
* The OpenVario Lark device has to be in the first position in the device list
* The Lark should connect at this point & monitoring shows NMEA sentences flowing from Lark

