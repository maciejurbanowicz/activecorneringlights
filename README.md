# activecorneringlights
Active Cornering Lights for Motorcycles

## Introduction
The Active Cornering Lights (ACL) for Motorcycles is the system aimed at improving safety and comfort when riding a motorcycle in bad weather conditions, at night and twisty roads.
It incorporates hardware and software components. It has the following features:
* Turn on and off both (left and right) lights simutlinaseoly,
* Turn on and off left or right light when the rider drives on the corners,
* Full control over the threshold roll angles for every light,
* Full control over the performance and accuracy of measuring of the roll angles.

Important note: the author of the project does not take any responsibility, nor liability for any damages or breaching regultions or any laws. You use it at your own risk.

## System design
The ACL consists of:
### Hardware:
- Arduino Nano 33 BLE,
- 3-position switch ON(1)-OFF(0)-ON(2),
- 2 relays controlled by the Arduino board,
- Harness,
- Connectors,
- Box.
### Software:
- A control software RTOS,
- Madgwick Filter.
### Diagrams
Note: this section will be updated soon.
## Usage
The Arduino IDE has been used to code, compile and test the code. You can download it from https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE. Please update the values of: sampleFreqDef and betaDef in the source code of Madgwick filter according to your needs. The sampleFreqDef is 119 Hz for Arduino Nano 33 BLE, and the betaDef can be adjusted to the needs. The default value provides a smooth and accurate angles with low noise, however the performance is low. For better performance it is suggested to keep the value at around 1.
