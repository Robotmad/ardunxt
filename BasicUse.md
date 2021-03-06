# Introduction #
ArduNXT uses I2C communication in order for the Lego Mindstorms NXT to be able to read and write data.  Access is provided to raw and processed versions of the information where appropriate, enabling part of the processing load to be shared with the ArduNXT.

It supports:
  * 4 Channels of PWM Servo Control Output
  * 4 Channels of PWM Remote Control Input

ArduNXT is fully compliant with the standards for NXT I2C based sensors, reporting the following strings (try scanning it with the RobotC sample program "I2C Scanner":
|Parameter|Address|Value|
|:--------|:------|:----|
|Version  |0x00   |V1.00|
|Manufacturer|0x08   |DIYDrone|
|Device Name|0x10   |ArduNXT|

While the ArduNXT is compatible with the NXT programming environment it is much easier and quicker to develop code using RobotC.  When using RobotC ArfuNXT can support faster IIC operation with the "sensorI2CCustomFast" configuraiton.

# Details #

## Device Slave Address ##
All messages start with the device slave address which is defined in the source code by ARDUNXT\_I2C\_ADDRESS which has a default value of 1.

Most Lego Sensors use an address of 0x01 as there is only one sensor connected per NXT IIC Sensor interface port. (The address is a 7 bit field, the LSBit of the addressing byte in IIC is used to indicate read or write operations, thus for a read the byte is 0x02 and for write it is 0x03.)

However, when ArduNXT used in Mindsensors NXT Servo Sensor compatibility mode the address is 0x58 (i.e. 0xB0/1 when combined with direction bit.) This mode is selected at compilation time by the following definition:
#define MINDSENSORS\_NXT\_SERVO\_COMPATIBLE

## Addresses ##
|Parameter|Direction|Address|Default|
|:--------|:--------|:------|:------|
|Status   |Read     |0x40   |0x00   |
|Command  |Write    |0x41   |-      |
|Servo 0 Position (LSB)|Write    |0x42   |?      |
|Servo 0 Position (MSB)|Write    |0x43   |?      |
|Servo 1 Position (LSB)|Write    |0x44   |?      |
|Servo 1 Position (MSB)|Write    |0x45   |?      |
|Servo 2 Position (LSB)|Write    |0x46   |?      |
|Servo 2 Position (MSB)|Write    |0x47   |?      |
|Servo 3 Position (LSB)|Write    |0x48   |?      |
|Servo 3 Position (MSB)|Write    |0x49   |?      |
|Remote Control Ch 1 Input (LSB)|Read     |0x63   |0      |
|Remote Control Ch 1 Input (MSB)|Read     |0x64   |0      |
|Remote Control Ch 2 Input (LSB)|Read     |0x65   |0      |
|Remote Control Ch 2 Input (MSB)|Read     |0x66   |0      |
|Remote Control Ch 3 Input (LSB)|Read     |0x67   |0      |
|Remote Control Ch 3 Input (MSB)|Read     |0x68   |0      |
|Remote Control Ch 4 Input (LSB)|Read     |0x69   |0      |
|Remote Control Ch 4 Input (MSB)|Read     |0x6A   |0      |

### Reading or Writing multiple bytes ###
The NXT reads and writes data from/to memory mapped registers in the ARduNXT, using an 8 bit register address. Multiple bytes can be read or written at a time, with sutiable NXT code, as the register address is automatically incremented after each byte has been read/written.

While a data transfer is in progress the shared memory is not updated by the ArduNXT, this is to avoid any multi-byte values from changing between reading the individual bytes.


---
