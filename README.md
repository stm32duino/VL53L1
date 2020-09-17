# VL53L1
Arduino library to support the VL53L1 Time-of-Flight ranging sensor with advanced multi-zone and multi-object detection.
This sensor uses I2C to communicate. An I2C instance is required to access to the sensor.
The APIs provide simple distance measure and multi-object detection in both polling and interrupt modes.

## API

This sensor uses I2C to communicate. And I2C instance is required to access to the sensor.

The API provides simple distance measure and multi-object detection.

## Examples

There are 2 examples with the VL53L1 library.

* VL53L1_Sat_HelloWorld: This example code is to show how to get multi-object detection and proximity
  values of the VL53L1 satellite sensor in polling mode.

* VL53L1_Sat_HelloWorld_Interrupt: This example code is to show how to get multi-object detection and proximity
  values of the VL53L1 satellite sensor in interrupt mode.

## Note

The maximum detection distance is influenced by the color of the target and
the indoor or outdoor situation due to absence or presence of external
infrared. The detection range can be comprise between ~40cm and ~400cm.
The library should work also with standard Arduino boards. In this case you just
need to adjust the code in the sketch in order to use the correct Wire instance and
the correct pin number for XSHUT and GPIO1 pins.

## Documentation

You can find the source files at  
https://github.com/stm32duino/VL53L1

The VL53L1 datasheet is available at  
https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l1.html
