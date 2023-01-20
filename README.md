# Real Lidarbot

ROS2 Foxy, running on Ubuntu server 20.04 on a Raspberry Pi 4, is employed in controlling a physical 2 wheeled drive vehicle. The vehicle is equipped with an RPLIDAR A1 sensor and an Intel RealSense D415 depth camera used for autonomous driving, obstacle avoidance and Simultaneous Localization and Mapping (SLAM) operations.

(Work in progress)

## Hardware
### Part list
The following components were used in this project:

| | Part |
| --| --|
|1| [Raspberry Pi 4 (4 GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)|
|2| SanDisk 64 GB SD Card|
|3| [Two wheel drive robot chassis kit (with wheel encoders)](https://www.amazon.com/perseids-Chassis-Encoder-Wheels-Battery/dp/B07DNYQ3PX/ref=sr_1_9?crid=3T8FVRRMPFCIX&keywords=two+wheeled+drive+robot+chassis&qid=1674141374&sprefix=two+wheeled+drive+robot+chas%2Caps%2C397&sr=8-9)|
|4| [Waveshare Motor Driver HAT](https://www.waveshare.com/wiki/Motor_Driver_HAT)|
|5| [2 x Photo interrupters for wheel encoders](https://www.aliexpress.com/item/32773600460.html?spm=a2g0o.order_list.order_list_main.5.21ef1802uhtGk4)|
|6| MPU6050 board|
|7| [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)|
|8| Intel RealSense D415 depth camera|
|9| [3D printed stands for RPLIDAR A1 and RPi 4](https://www.thingiverse.com/thing:3970110)|
|10| Mount for D415 depth camera|
|11| Powerbank for RPi 4 (minimum output: 5V 3A)|
|12| 3 Slot 18650 battery holder|
|13| 3 x 18650 batteries to power Motor Driver HAT|
|14| Female to Female Dupoint jumper cables|
|15| Spare wires|

Loose wires

Some other tools or parts used in the project are as follows:

| | Tool/Part |
| --| --|
|1| Soldering iron|
|2| 3D printer|
|3| Screwdriver set|
|4| Double-sided tape|

### Project Wiring and Assembly

The electronic components of the lidarbot are connected as shown below.

<p align="center">
  <img title='Wiring diagram' src=images/lidarbot_wiring.png width="800">
</p>

The MPU6050 board pins were connected to the Raspberry Pi 4 GPIO pins as follows:

| MPU6050 board | GPIO.BOARD| GPIO.BCM|
| ----------- | ------------| ------ |
| VCC         | 3.3V | 3.3V |
| GND         | GND | GND |
| SCL         | 05 | GPIO03 |
| SDA         | 03 | GPIO02 |

The right and left photo interrupter sensors are connected to GPIO pins as follows:

| Photo interrupter (R) | GPIO.BOARD | GPIO.BCM|
| ----------- | ------------| ------ |
| OUT | 18 | GPIO24 |
| VCC | 5V | 5V |
| GND | GND | GND |

| Photo interrupter (L) | GPIO.BOARD | GPIO.BCM|
| ----------- | ------------| ------ |
| OUT | 22 | GPIO25 |
| VCC | 5V | 5V |
| GND | GND | GND |

The screw terminal blocks on the Motor Driver HAT (shown below) are connected to the motor wires and battery holder cables as follows: 

| Motor Driver HAT pin | Connected to| 
| -- | -- |
| MA1 | (Left motor)| 
| MA2 | Black wire (Left motor)| 
| GND | Black wire (battery holder) | 
| VIN | Red wire (battery holder) | 
| MB1 | (Right motor)| 
| MB2 | Black wire (Right motor)| 

<p align="center">
  <img title='Motor Driver HAT' src=images/Motor_Driver_HAT.png width="500">
</p>

Should the wheel(s) move in the direction opposite of what is expected, exchange the respective motor cables screwed into the terminal blocks.

<p align="center">
  <img title='MPU6050' src=images/mpu6050.jpg width="400">
  <img title='Encoders' src=images/encoders.jpg width="400">
</p>

<br/>

<p align='center'>
  <img title='Top View' src=images/top_view.jpg width="400">
</p>

<p align="center">
  <img title='Side View' src=images/side_view.jpg width="400">
</p>


## Installation

## Package Overview

## Go to goal (with only wheel encoders)
### (0.0, 0.0) &rarr; (0.5, 0.5) &rarr; (0.0, 0.0)

<p align='center'>
    <img src='images/0.5_diag.gif'>
</p>

### (0.0, 0.0) &rarr; (0.8, 0.0) &rarr; (0.8, 0.8) &rarr; (0.0, 0.8) &rarr; (0.0, 0.0)


<p align='center'>
    <img src='images/0.8_square.gif'>
</p>

## Navigation (work in progress)