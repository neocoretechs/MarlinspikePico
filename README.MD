<H2>  Marlinspike robotic controller platform. </H2>
ORiginally, Object oriented C++ version of Mega2560 M and G code front end processing via USB port of Arduino. Then implemented as a Java class using Linux UART and GPIO userspace drivers. Finally, this implementation on the Pico, which probably wont be the last...
 
 Processes a variation of M and G code from CNC and 3D printing to control a range of motor controllers and drivers
 and GPIO pins for smart machine and robotics hardware functionality.
 
 Geared toward the a microcontroller chip, such as Mega2560 or Pico, this code unifies the microcontroller platform and allows it to be easily accessed through the standard CNC-derived 'language' and asynchronous messaging loop. 
 
 As an example, it unifies the timers to deliver consistent PWM functions across all available pins, and allows
 various motor controllers like H-bridge, half bridge, and smart controllers to be controlled through UART, PWM, and GPIO functions.
 
 It contains a main processing loop that receives M and G code commands through the UART via USB or TTL, like a CNC machine or 3D printer,
 then uses a pure C++, rewritten, unified version of the Wiring library for arduino, ported to Pico SDK, to handle the functions provided in the main CNC-derived M and G code processing loop.
 
 Servos, ADCs, Timers, Pin change interrupts, Analog and Digital IO pins, SPI, I2C, PWM, UARTs, and motor controllers are all unified and
 accessible through the main processing loop. In the primary loop, M and G code commands are processed such that this generic, multifunction, 
 realtime, robotic smart controller can be attached to a single board computer (SBC), or used standalone and accessed through a UART, 
 or potentially be attached via SPI, I2C or TWI to function as a realtime hardware controller.
 
 Another example is the way in which ultrasonic sensors can be attached to any motor driver through main loop processing commands and used to
 set up a minimum distance to detect an object before issuing a command to shut down the motor driver.
 
 To continue the examples, through the processing loop, hall effect sensors can be attached to any motor driver through pins designated in the main
 processing loop M and G code commands and used to detect wheel rotations, then set up pin change interrupts for those sensors through other
 M and G code commands, then perform actions based on the wheel rotations and the interrupts generated and processed through other M and G code commands.
 
 A microcontroller running this code can be used to easily construct a robot, a 3D printer, a CNC machine, or any number of devices that need 
 a generic smart controller capable of low level access to GPIO pins and hardware timers and ADCs etc.
 
a.) A means to control low level robotics via serial communications to a Mega2560 board and provide an interface to the higher level busses.

The attached Microcontroller board provides the low level realtime data acquisition and control from the devices
and controllers so attached to it.

Much like 3D printers process M and G codes for generating movement and action, the attached board functions in the same way using a special protocol.
Timers, controllers, and pins are all first class objects in a compact subsystem, created and manipulated by the proper M and G codes.

In conjunction with this, project Marlinspike provides Java demultiplexing of the responses from the Microcontroller via the 
serial library to process the data acquisition and box it up for publishing on the ROS topic.

Values from ultrasonic sensors, digital and analog pins and other accessories can be set to stream real time data back over the USB port.
A full range of motor driver and smart motor controller functions for brushless and brushed DC motors is supported.

For debugging, open a terminal session to the USB port, typically /dev/ttyACM0 or /dev/ttyUSB0, at 115200 baud, 8 bits
no parity, and issue commands directly to the Mega 2560. Minicom works well under ubuntu. minicom -F /dev/ttyACM0 -b 115200.
