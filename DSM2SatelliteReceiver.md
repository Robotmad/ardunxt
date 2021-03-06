# Introduction #

This page describes how to connect, configure and use an SPM9545 Satellite Receiver for Remote Control of ArduNXT.


# Details #

## Hardware ##
The SPM9545 Satellite Receiver needs a 3V3 power supply which for now you will need to provide yourself.  Hopefully this will be included in future production version of the ArduNXT hardware. So all you need to connect are the ground and signal wires from the receiver to the GND and RX pins on the prototype ArduNXT (on my prototype I have connected them using the connector intended for the EM406 GPS module).

Here you will find a [Labelled diagram of SPM9545 Satellite Receiver](http://www.diydrones.com/profiles/blog/show?id=705844%3ABlogPost%3A64228) connections.

## Binding ##
You will also need to bind the satellite receiver with your transmitter before using it with ArduNXT (so it is best to buy it as part of an AR6200 receiver package which comes with a main receiver).  This is just a temporary limitation which will be resolved through future software development (code is in the file **DSM2Receiver.pde**).

## Software ##
The Satellite Receiver serial data uses a rate of 115200 Baud, as the same rate must be used for input and output this becomes the baud rate for diagnostic output.
A package of software just to demonstrate the capability of this Satellite Receiver has been prepared for download [DSM2Demo.zip](http://code.google.com/p/ardunxt/downloads/detail?name=DSM2Demo.zip)