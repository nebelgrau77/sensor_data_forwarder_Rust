### MPU6050 data over serial with RTIC

A Rust version of similar code examples for Arduino.

Reads sensor values: accelerometer X, Y, Z and gyroscope X,Y,Z, and prints them to serial in a format required by [EdgeImpulse data forwarder tool](https://docs.edgeimpulse.com/docs/cli-data-forwarder). 

In this example a USB-to-serial converter is connected to pins PA2 and PA3, and the MPU6050 sensor to pins PB9 and PB9. The board is STM32F411Cxx "Black pill".

Ideally this would run on a board that has a native serial-to-USB bridge, and a built-in sensor. 

Use `$ edge-impulse-data-forwarder --clean` to start.



