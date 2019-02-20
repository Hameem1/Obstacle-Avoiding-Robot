# Obstacle Avoiding Robot

## Abstract

An obstacle avoiding robot which uses fuzzy logic based motor control and is implemented using PIC18 series micro-controllers.

This code is designed to run on 3 separate micro-controllers.
- One master controller for the main logic. (PIC18F4550)
- Two slave controllers for each wheel. (PIC18F2431)

***

## Hardware Specifications

#### Component list

- Acrylic Sheet (For chassis)
- PCBs (Master board, Slave boards, Power control, H-bridges)
- Ultrasonic Sensors (x6)
- Geared motors (x2)
- LiPo BATTERY
- PIC18F4550
- PIC18F2431
- Wheels
- Switch
- LCD

#### The Basic Idea

- The robot takes obstacle distance readings from a semi circular array of 6 ultrasonic sensors at the front.

- All the obstacle distances for the sensors are displayed on a mounted LCD

- The master controller uses fuzzy logic to calculate its next move from these readings.

- To execute this move, it communicates instructions to the slave controllers to change their wheel speeds.

- Each slave controller implements a PID controller for the motor control and uses an H-bridge to actuate the motor.

- The motors are geared and fitted with Quadrature Encoder Interfaces which allow for distance tracking.

- All of this is powered by a LiPo battery fed to a voltage regulation circuit. A switch is used to turn on the battery.

