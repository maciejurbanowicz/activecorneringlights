/*-------------------------------------------------------------------------------------------
// Definitions in MadgwickAHRS.cpp. The values need to be adjusted according to the IMU you have and the accuracy and performance you want to achieve.

#define sampleFreqDef   119.0f          // sample frequency in Hz
#define betaDef         0.4f            // 2 * proportional gain
*/


#include <mbed.h>
#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>

using namespace mbed;
using namespace rtos;
 
using namespace std::chrono_literals;

//Madgwick
Madgwick filter;

//Arrays for accelerometer and gyro
std::array<float, 3> A = {0,0,0}; //A - accelerometer [g]
std::array<float, 3> G = {0,0,0}; //G - gyro [dps]

float roll;
//float pitch, heading; //pitch and heading values

//Roll angleL thresholds
const float rollONanglenegative = -7.0;
const float rollONanglepositive = 7.0;
const unsigned long rollONangledelaytime = 700;

//Relay board
DigitalOut LightPinLeft(P1_11); //left channel ID
DigitalOut LightPinRight(P1_12); //right channel ID
//#define LightPinLeft 2 //left channel ID
//#define LightPinRight 3 //right channel ID

//Switch
DigitalIn switch1(P0_21); //the main switch connected to D8
DigitalIn switch2(P0_27); //the secondary switch connected to D9
//#define switch1 8; //the main switch connected to D8
//#define switch2 9; //the secondary switch connected do D9

volatile int switch1status = 1; //default value for the switch, 1 = OFF, 0 = ON
volatile int switch2status = 1; //default value for the switch, 1 = OFF, 0 = ON
int switch1statustemp = 0; //temporary switch status 1
int switch2statustemp = 0; //temporary switch status 2

//Lights
int lightLstatus = 0;
int lightRstatus = 0;

//Threads
Thread TRTOScontrol;
Thread Ttelemetry;

//LED on-board for checking ON/OFF of lights
DigitalOut led(LED1);

/* RTOScontrol reads raw values from IMU (Acceleration and Gyroscope) and uses the Madgwick filter to provide roll angle. Pitch and heading are also available. */

void RTOScontrol() {
  for (;;) {
    // read raw data from IMU
    IMU.readAcceleration(A[0], A[1], A[2]);
    IMU.readGyroscope(G[0], G[1], G[2]);
    // convert from raw data to gravity and degrees/second units
    for (int i = 0; i<3; i++) {
      A[i] = (A[i]*4.0) / 32768.0;
      G[i] = (G[i]*2000.0) / 32768.0;
    }

    // update the filter, which computes orientation
    filter.updateIMU(G[0], G[1], G[2], A[0], A[1], A[2]);

    //get roll angle after filter
    roll = filter.getRoll();
    //pitch = filter.getPitch();
    //heading = filter.getYaw();
    {
      if (switch1status == 1 && switch2status == 1) {
        LightPinLeft.write(0);
        LightPinRight.write(0);
        led.write(0);       
        lightLstatus = 0;
        lightRstatus = 0;
      }
      if (switch1status == 0 && switch2status == 1) {       
        {
          if (roll < rollONanglenegative) {
            LightPinLeft.write(1);
            LightPinRight.write(0);
            led.write(1);
            lightLstatus = 1;
            lightRstatus = 0;
            //ThisThread::sleep_for(rollONangledelaytime);
          }
              
          if (roll > rollONanglepositive) {
            LightPinLeft.write(0);
            LightPinRight.write(1);
            led.write(1);
            lightLstatus = 0;
            lightRstatus = 1;
            //ThisThread::sleep_for(rollONangledelaytime);
          }
              
          if (roll > rollONanglenegative && roll < rollONanglepositive) {
            LightPinLeft.write(0);
            LightPinRight.write(0);
            led.write(0);
            lightLstatus = 0;
            lightRstatus = 0;
          }
        }
      }
      if (switch1status == 1 && switch2status == 0) {
        LightPinLeft.write(1);
        LightPinRight.write(1);
        led.write(1);     
        lightLstatus = 1;
        lightRstatus = 1;
      }
    }
  }
}

/* This is the new SwitchesReading function. It supports a single switch ON-OFF-ON. */

void SwitchesReading() {
  switch1statustemp = 0;
  switch2statustemp = 0;
  int iterations = 1000;
  int threshold = 50;
  for (int i = 0 ; i<iterations; i++) {
    switch1statustemp = switch1statustemp + digitalRead(switch1);
    switch2statustemp = switch2statustemp + digitalRead(switch2);
  }
  if (switch1statustemp > threshold) {
    switch1status = 1; //OFF
  }
  else {
    switch1status = 0; //ON
  }
  if (switch2statustemp > threshold) {
    switch2status = 1; //OFF
  }
  else {
    switch2status = 0; //ON
  }
}

void telemetry() {
  for (;;) {
    Serial.print("Switch 1 = ");Serial.print(switch1status);
    Serial.print(" | Switch 2 = ");Serial.print(switch2status);
    Serial.print(" | Switch 1 RAW = ");Serial.print(digitalRead(switch1));
    Serial.print(" | Switch 2 RAW = ");Serial.print(digitalRead(switch2));
    Serial.print(" | Light L = ");Serial.print(lightLstatus);
    Serial.print(" | Light R = ");Serial.print(lightRstatus);
    Serial.print(" | switch1statustemp = ");Serial.print(switch1statustemp);
    Serial.print(" | switch2statustemp = ");Serial.println(switch2statustemp);
  }
}

void serialInit(int serialinit) {
  Serial.begin(serialinit);
  while (!Serial); {
    ;
  }
  Serial.println("Serial initialized");
}

void IMUInit() {
  IMU.begin();
  Serial.println("IMU initialized");
}

void setup() {
  serialInit(9600);
  IMUInit();
  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);

  //SwitchesReading(); //Initial reading of the statuses of the switches.

  attachInterrupt(digitalPinToInterrupt(switch1),SwitchesReading,CHANGE);
  attachInterrupt(digitalPinToInterrupt(switch2),SwitchesReading,CHANGE);

  //threads
  TRTOScontrol.start(RTOScontrol);
  Ttelemetry.start(telemetry);
}

void loop() {
  // empty loop
}