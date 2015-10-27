An interface between Lego Mindstorms NXT and standard remote control receivers, servos and ESCs, with failsafe capability.  This enables software to be run on the NXT (e.g. written in Mindstorms NXT or [RobotC](http://www.robotc.net)) which can take input from the remote control receiver and other Mindstorms sensors and can control both the Mindstorms motors and standard remote control servos and Electronic Speed Controllers (ESCs) e.g. to implement an autopilot for a UAV or other autonomous robot.
# Development #
If you download and try any of this code please send a message to [Christopher Barnes](http://diydrones.com/profile/ChristopherBarnes) at DIY Drones to tell us of your interest and let us know how you get on.

# Software #
The software is written using the [arduino](http://www.arduino.cc) environment to run on an Atmel AVR ATMega328 micrcontroller. It was most recently built with Arduino version 0022.

**This is a [DIYDrones.com](http://www.diydrones.com) development team project.**
# Hardware #
Hardware will be available to run this software on, based on a derivative of Ardupilot.  At present it is possible to test the software and assist with the development using a standard Ardupilot board with a few minor modifications as detailed below.

## Prototype ##
  * Connect the NXT Sensor cable as follows:
|Pin 1|N/A|
|:----|:--|
|Pin 2|GND|
|Pin 3|GND|
|Pin 4|VCC|
|Pin 5|JP1 Pin 6 PC5(SCL)|
|Pin 6|JP1 Pin 5 PC4(SDA)|

  * Pullup resistors are required from JP1 pin 5 (SDA) and pin 6 (SCL) to VCC (which is available at lots of locations on the Ardupilot PCB) suggested value 82KOhm. (If you don't have this value try the next lower value that you have available.)
  * Connect JP6 pin 5 (TReset) to pin 6 (GND) to hold the ATTiny in reset so that the ATMega can drive the multiplexer control signal directly.
  * To power the ARduNXT prototype from the Lego Mindstorms NXT connect the NXT Sensor cable pin 4 to VCC and remove (unsolder) the link on JP7 completely to avoid the NXT trying to power any connected servos or any other conflict over the source of VCC. _you will need to provide a separate source of power for servos and/or Radio Control receiver._
  * Connect "MUX IN 3" (S3B) to JP3 pin 3 (ATMega PB0, Arduino D8) - this is the same as the fairly common Ardupilot modification to get a Throttle Control PWM Servo Output.
  * Connect "MUX IN 4" (S4B) to JP3 pin 2 (ATMega PD7, Arduino D7) - this gives the ArduNXT control of the 4th PWM Servo Output channel.
  * Connect the Remote Control Input for Channel 3 "S3A" to JP3 pin 1 (ATMega PD6, Arduino D6) - this enables the ArduNXT to receive this channel.
  * Connect the Remote Control Input for Channel 4 "S4A" to JP4 pin 1 (ATMega PB3, Arduino D11) - this enables the ArduNXT to receive this channel.

### LEDs ###
The LEDs have the following functions on the prototype hardware:
|LED1|Red|Power|
|:---|:--|:----|
|LED2|Green|n/a  |
|LED3|Yellow|NXT Communications|
|LED4|Blue|GPS Status (Flashing=Searching, On=Good fix)|
|LED5|Red|Multiplexer (On=ArduNXT in contol)|

### GPS ###
The current code works with uBlox 5 GPS modules using UBX binary messages, but it will be straightforward to integrate NMEA or other binary protocols which have already been coded in the main Ardupilot development. (Please be careful to ensure that your GPS module has the correct supply voltage as some are designed to run from 3.3V rather than the 5V of the ATMega.)

### DSM2 SPM9545 Satellite Receiver ###
As an alternative to using a conventional Remote Control receiver with an individual PWM signal per channel (for which ArduNXT is limited to using only 4 channels), you can connect an [SPM9545 DSM2 Satellite Receiver](http://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM9545) and have up to 7 channels of Remote Control.
**Advantages** of this are the increased number of channels available and reduced cost.
**Disadvantages** of this are that it connects to the serial port in place of the GPS module so you can't use both at the same time.

When this receiver is connected the blue LED is used as an indication of the status of the DSM2 signal.

### Spektrum DX5e Transmitter ###
If you use this receiver with the Spektrum DX5e Transmitter, which is billed as being a 5 channel system, you actually get **6 channels!** :-) The extra channel communicates the position of the Trainer switch.  Only just found this out - delighted.  Please try it, no hack required.