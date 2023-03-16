# Voice-Controlled-Obstacle-Avoiding-Robot
The main aim of the robot is to scan the voice input  or joystick input and process it for required operation and to do different tasks over specified area by controlling the robotic vehicle via voice command. 




Abstract:
The main aim of the robot is to scan the voice input  or joystick input and process it for required operation and to do different tasks over specified area by controlling the robotic vehicle via voice command. The voice control technique consists of an Android app which communicates with the robot via Bluetooth module. Additionally, the robot will also have the proficiency to detect the obstacle, stops and avoid the obstacle by choosing different path with the help of Ultrasonic sensor. 


Introduction:
In this project, we will learn how to make Voice Controlled Robot Car Using Arduino. The robotic car can be controlled wirelessly via voice commands directly from the user. The robot can move forward, backward, left, and right and can also be stopped and can-do commands like light on and off, horn.

The Arduino voice-controlled robot car is interfaced with a Bluetooth module HC-05. We can give specific voice commands to the robot through an Android app installed on the phone. At the receiving side, a Bluetooth transceiver module receives the commands and forwards them to the Arduino and thus the robotic car is controlled.

Objective
Making the right hardware selection for your project depending on its functionalities.
· Prepare the circuit diagram to connect all the chosen components.
· Assemble all the project parts (mechanical and electronic assembly).
· Finally designing your own ARDUINO-based Voice-controlled Robot.


Hardware and Software Requirements
Arduino UNO 
Ultrasonic Sensor 
L293d Motor Driver Shield 
HC-05 Bluetooth module 
Servo motor 
Foam board 
IR sensor 
BO motor and wheels, etc... 

SOFTWARE REQUIREMENTS:

Arduino IDE software
Libraries to install in that:
Adafruit Motor driver shield V2 & V3(for working of motor driver shield)
Servo library is preinstalled in the software.


Implementation
	- Model (Optional) or
       -TINKERCAD circuit
 
 
•	+ 1 extra led and one extra buzzer+led at pin’s 12 and 13 respectively
 



Theory
	-Working logic of the project
ROLE OF ULTRASONIC SENSOR IN OBSTACLE AVOIDING ROBOT

WHAT IS ULTRASONIC SENSOR?
An ultrasonic sensor is an electronic device that measures the distance of a target object by emitting ultrasonic sound waves, and converts the reflected sound into an electrical signal.

ULTRASONIC SENSOR
 

CONNECTIONS OF ULTRASONIC SENSOR:
1.	Vcc - (connected to 5V pin in Arduino)
2.	Trig - (Transmitter which is connected to Analog pins in Arduino)
3.	Echo - (Receiver which is connected to Analog pins in Arduino)
4.	Gnd - (Gnd is ground which is connected to ground pin in Arduino)

Working of ultrasonic sensor in Obstacle Avoidance:

Ultrasonic sensor is used to sense the obstacles in the path by calculating the distance between the robot and the obstacle. If a robot finds any obstacle it changes its direction and continues moving.

How to build obstacle avoiding robot using Ultrasonic Sensor?

Before going to build the robot, it is important to understand how the ultrasonic sensor works because this sensor will have an important role in detecting obstacles. The basic principle behind the working of ultrasonic sensor is to note down the time taken by sensor to transmit ultrasonic beams and receive the ultrasonic beams after hitting the surface. 
The signal then hits the surface and returns back and captured by the receiver Echo pin of HC-SR04. The Echo pin had already made high at the time sending high.

 


The distance is calculated using the formula below:

The time taken by beam to return back is saved in variable and converted to distance using appropriate calculations

Distance= (Time x Speed of Sound in Air (343 m/s))/2


HC-05 Bluetooth module

HC-05 is a Bluetooth module which is designed for wireless communication. This module can be used in a master or slave configuration. It has range up to <100m which depends upon transmitter and receiver, atmosphere, geographic & urban conditions. It uses serial communication to communicate with devices. It communicates with microcontroller using serial port (USART).
 
Bluetooth serial modules allow all serial enabled devices to communicate with each other using Bluetooth.

Pin configuration:
 
Enable/key: This pin is used to toggle between Data Mode (set low) and AT command mode (set high). By default, it is in Data mode.
The two modes:
Data mode: Exchange of data between devices.
Command mode: It uses AT commands which are used to change setting of HC-05. To send these commands to module serial (USART) port is used.
Vcc: Powers the module. Connect to +5V or +3.3V Supply voltage.
GND: Ground Pin of module.
TXD: Transmit Serial data.  Everything received via Bluetooth will be given out by this pin as serial data.
RXD: Receive data serially.  Every serial data given to this pin will be broadcasted via Bluetooth.
State: The state pin is connected to on board LED, it can be used as feedback to check if Bluetooth is working properly.
LED: Indicates the status of Module
Blink once in 2 sec: Module has entered Command Mode
Repeated Blinking: Waiting for connection in Data Mode
Blink twice in 1 sec: Connection successful in Data Mode
Button: The command and data mode states are changeable through a button present on the module.
Here, we are not using the state pin and the enable/key pin.

HC-05 Default Settings:
Default Bluetooth Name: “HC-05”
Default Password: 1234 or 0000
Default Communication: Slave
Default Mode: Data Mode
Command Mode Baud Rate: 38400, 8, N, 1

Working of HC-05 Bluetooth module:

The HC-05 has two operating modes, one is the Data mode in which it can send and receive data from other Bluetooth devices and the other is the AT Command mode where the default device settings can be changed. We can operate the device in either of these two modes by using the key pin as explained in the pin description.
It is very easy to pair the HC-05 module with microcontrollers because it operates using the Serial Port Protocol (SPP). Simply power the module with +5V and connect the Rx pin of the module to the Txd of MCU and Txd pin of module to Rxd of MCU.
During power up the key pin can be grounded to enter into Command mode, if left free it will by default enter into the data mode. As soon as the module is powered you should be able to discover the Bluetooth device as “HC-05” then connect with it using the default password 1234 and start communicating with it. The name password and other default parameters can be changed.

Servo Motor
A servo motor is a rotary actuator that allows for precise control of angular position. It consists of a motor coupled to a sensor for position feedback. It also requires a servo drive to complete the system. The drive uses the feedback sensor to precisely control the rotary position of the motor.
 

Wire configuration:
 

Brown: Ground wire connected to the ground of system.
Red: Powers the motor typically +5V is used.
Orange: PWM signal is given in through this wire to drive the motor.

Working of servo motor:

The servo motor has some control circuits and a potentiometer connected to the output shaft. The pot can be seen on the right side of the circuit board allows the control circuitry to monitor the current angle of the servo motor.
If the shaft is at the correct angle, then the motor shuts off. If the circuit finds that the angle is not correct, it will turn the motor until it is at a desired angle. The output shaft of the servo is capable of traveling somewhere around 180 degrees. Usually, it is somewhere in the 210-degree range, however, it varies depending on the manufacturer. A normal servo is used to control an angular motion of 0 to 180 degrees. It is mechanically not capable of turning any farther due to a mechanical stop built on to the main output gear.
The power applied to the motor is proportional to the distance it needs to travel. So, if the shaft needs to turn a large distance, the motor will run at full speed. If it needs to turn only a small amount, the motor will run at a slower speed. This is called proportional control.

Communication of angle at which it should turn:

The control wire is used to communicate the angle. The angle is determined by the duration of a pulse that is applied to the control wire. This is called Pulse Coded Modulation. The servo expects to see a pulse every 20 milliseconds (.02 seconds). The length of the pulse will determine how far the motor turns. A 1.5 millisecond pulse, for example, will make the motor turn to the 90-degree position (often called the neutral position). If the pulse is shorter than 1.5 milliseconds, then the motor will turn the shaft closer to 0 degrees. If the pulse is longer than 1.5 milliseconds, the shaft turns closer to 180 degrees.
 

Arduino code:
For this you must have the library servo.h.
Servo - write()
Writes a value to the servo, controlling the shaft accordingly. On a standard servo, this will set the angle of the shaft (in degrees), moving the shaft to that orientation. On a continuous rotation servo, this will set the speed of the servo (with 0 being full-speed in one direction, 180 being full speed in the other, and a value near 90 being no movement).

Syntax:
 servo.write(angle)

Servo - attach()
Attach the Servo variable to a pin. 
Syntax:
servo.attach(pin) 
servo.attach(pin, min, max)
