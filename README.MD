<H2>  Marlinspike Robotic Control Plane. </H2>
ORiginally, Object oriented C++ version of Mega2560 M and G code front end processing via USB port of Arduino. Then implemented as a Java class using Linux UART and GPIO userspace drivers. Finally, this implementation on the Pico, which probably wont be the last...
 
 Processes a variation of M and G code from CNC and 3D printing, to control a range of motor controllers and drivers
 and GPIO pins for smart machine and robotics hardware functionality.
 
 Geared toward the a microcontroller chip, such as Mega2560 or Pico, this code unifies the microcontroller platform and allows it to be easily accessed through the standard CNC-derived command set and asynchronous messaging loop. 
 
 As an example, it unifies the PWM functions across all available pins, and allows
 various motor controllers like H-bridge, half bridge, and smart controllers to be configured and controlled.
 
 It contains a main processing loop that receives M and G code commands like a CNC machine or 3D printer,
 then uses a pure C++ library for Pico SDK to handle the functions provided in the main processing loop.
 
 Servos, ADCs, Timers, Pin change interrupts, Analog and Digital IO pins, SPI, I2C, PWM, UARTs, and motor controllers are all unified and
 accessible through the main processing loop. In the primary loop, M and G code commands are processed such that this generic, multifunction, 
 realtime, robotic smart controller can be attached to a single board computer (SBC) laptop, or desktop USB port, or used standalone and accessed through a UART, 
 or potentially be attached via SPI, I2C or TWI to function as a realtime hardware controller.
 
 Using a few line of M code, high level configuration of attached sensors such as ultrasonic distance sensors, IMU's, motor drivers, LED arrays, and SPI and I2C devices can be easily performed.
 
 Once configured, data can be streamed and complex control of coordinated arrays of pins and sensors can be done in a high performance fashion.
 
 As an example: hall effect sensors can be attached to any motor driver through pins designated in the main
 processing loop, the other M and G code commands can be used to detect wheel rotations from that sensor, then that sensor can be set up to generate pin change interrupts under desired conditions, and GPIO pins can be designated as directional change pins for the attached assembly of motor drivers and sensors and odometry can be acquired by designating the proper encoder.
 
So in a nutshell: persistent, coordinated robotics functions can be achieved by sending a few lines of text.
 
A microcontroller running this code can be used to easily construct a robot, a 3D printer, a CNC machine, or any number of devices that need a generic smart controller capable of low level access to GPIO pins and hardware timers and ADCs etc.
 

In conjunction with this project, an associated project provides Java demultiplexing of the responses from the Microcontroller via the 
serial library to process the data acquisition and box it up for publishing on ROS topics.

You can even get started by attaching a flashed microcontroller to your PC, opening a terminal session to the USB port with minicom or PuTTY or any serial terminal, typically accessing ports like /dev/ttyACM0 or /dev/ttyUSB0 on Linux or COM4 or COM5 on Windows, at 115200 baud, 8 bits
no parity, and issuing commands directly to the microcontroller. Minicom works well under ubuntu. minicom -F /dev/ttyACM0 -b 115200.

Marlinspike Controller Command Reference

Overview

Marlinspike uses a structured set of M‑codes to declare, configure, and activate logical controllers inside the microcontroller subsystem.All controller configuration begins with M10, followed by one or more configuration commands.

This document describes the supported controller‑related M‑codes and their parameters.

1. M10 — Declare Controller Type and Slot

Syntax

M10 Z<slot> T<type>

DescriptionCreates a logical controller in a numbered slot.Subsequent M‑codes configure that controller.

Controller Types

Type

Name

Description

0

SmartController

Single PWM pin + encoder

1

H‑Bridge

PWM + direction pin

2

SplitBridge

Two PWM pins + enable

3

SwitchBridge

Two GPIO pins + enable

4

SwitchHBridge

One GPIO pin + direction

5

PWM

Standalone PWM driver

6

InputPin

Standalone GPIO input

7

OutputPin

Standalone GPIO output

8

DelayH‑Bridge

H‑Bridge with delayed activation

Examples

M10 Z0 T0   ; SmartController in slot 0
M10 Z1 T1   ; H‑Bridge in slot 1
M10 Z2 T2   ; SplitBridge in slot 2
M10 Z3 T3   ; SwitchBridge in slot 3
M10 Z0 T4   ; SwitchHBridge in slot 0
M10 Z0 T5   ; PWM driver in PWM slot 0
M10 Z1 T8   ; Delay H‑Bridge in slot 1

2. Controller Configuration M‑Codes

These commands configure the controller declared by M10.

2.1 M2 — SmartController (Type 0)

Syntax

M2 [Z<slot>] [C<channel>] [W<encoder pin>] [E<default dir>]

2.2 M3 — H‑Bridge (Type 1)

Syntax

M3 [Z<slot>] P<pwm pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F<frequency>] [H<duty>]

2.3 M4 — SplitBridge (Type 2)

Syntax

M4 [Z<slot>] P<pwm pin A> Q<pwm pin B> C<channel> D<enable pin> E<default dir> [W<encoder pin>] [F<frequency>] [H<duty>]

2.4 M5 — SwitchBridge / SwitchHBridge (Types 3 & 4)

SwitchBridge (Type 3)

M5 Z<slot> P<pin A> Q<pin B> C<channel> D<enable pin> E<default dir> [W<encoder>]

SwitchHBridge (Type 4)(no Q pin)

M5 Z<slot> P<pin> C<channel> D<direction pin> E<default dir> [W<encoder>]

2.5 M9 — PWM Driver (Type 5)

Syntax

M9 [Z<slot>] P<pwm pin> C<channel> D<enable pin> [F<frequency>] [H<duty>]

2.6 M16 — Delay H‑Bridge (Type 8)

Syntax

M16 [Z<slot>] P<pwm pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F<frequency>] [H<duty>]

3. Encoder Configuration

3.1 M14 — Analog Encoder

Syntax

M14 Z<slot> C<channel> P<pin> L<lo> H<hi> N<count>

3.2 M15 — Digital Encoder

Syntax

M15 Z<slot> C<channel> P<pin> S<state> N<count>

4. Standalone Pin Configuration

4.1 M41 — Output Pin

M41 P<pin>

4.2 M43 — Input Pin

M43 P<pin>

5. Examples

Differential Drive (Two H‑Bridges)

M10 Z0 T1
M3 Z0 P8  C1 D24 E0 W68

M3 Z0 P10 C2 D22 E0 W69

PWM LED Driver

M10 Z0 T5
M9  Z0 P13 C1 D30

SplitBridge Lift Actuator

M10 Z2 T2
M4  Z2 P6 Q7 C1 D32 E0

6. Notes

All controller configuration begins with M10.

Encoder configuration (M14/M15) may follow depending on controller type.

The firmware enforces construction invariants; invalid states cannot be created.

The host should treat controller creation/configuration as a transaction.
