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
