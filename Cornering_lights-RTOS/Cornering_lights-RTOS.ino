/*-------------------------------------------------------------------------------------------
// Definitions in MadgwickAHRS.cpp. Set up the values as follows

#define sampleFreqDef   119.0f          // sample frequency in Hz
#define betaDef         1.0f            // 2 * proportional gain
*/


#include <mbed.h>
#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>

using namespace mbed;
using namespace rtos;
 
using namespace std::chrono_literals;

//Madgwick
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

//Arrays for accelerometer and gyro
std::array<float, 3> A = {0,0,0}; //A - accelerometer [g]
std::array<float, 3> G = {0,0,0}; //G - gyro [dps]

float roll;
//float pitch, heading;

//Roll angleL thresholds
const float rollONanglenegative = -8.0;
const float rollONanglepositive = 8.0;
const unsigned long rollONangledelaytime = 500;

//time
unsigned long microsNow;

//sample frequency
//float samplefrequency = 119.0;

//Relay board
DigitalOut LightPinLeft(P1_11); //left channel ID
DigitalOut LightPinRight(P1_12); //right channel ID

//Switch
DigitalIn switch1(P0_21); //the main switch connected to D8
DigitalIn switch2(P0_27); //the secondary switch connected to D9

int switch1status = 1; //default value for the switch, OFF
int switch2status = 1; //default value for the switch, OFF

//Lights
int lightLstatus = 0;
int lightRstatus = 0;
int lightcorneringstatus = 0;

//Threads
Thread TSwitchesReading;
Thread TIMUreadings;
Thread TLightsController;
//Thread Ttelemetry;

//Dev
DigitalOut led(LED1);

void IMUreadings() {
  for (;;) {
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {

      // read raw data from IMU
      IMU.readAcceleration(A[0], A[1], A[2]);
      IMU.readGyroscope(G[0], G[1], G[2]);
      // convert from raw data to gravity and degrees/second units
      for (int i = 0; i<3; i++) {
        A[i] = (A[i]*4.0) / 32768.0;
      }

      for (int j = 0; j<3; j++) {
        G[j] = (G[j]*2000.0) / 32768.0;
      }

      // update the filter, which computes orientation
      filter.updateIMU(G[0], G[1], G[2], A[0], A[1], A[2]);

      //get roll angle after filter
      roll = filter.getRoll();
      //pitch = filter.getPitch();
      //heading = filter.getYaw();

      // increment previous time, so we keep proper pace
      microsPrevious = microsPrevious + microsPerReading;
    }
  }
}

/* Old SwticherReading function. It was used for the two-switches 0-1 configuration.
void SwitchesReading() {
  for (;;) {
    switch1status = switch1.read();
    switch2status = switch2.read();

    //if the main switch is off (status 1), then both lights are off
    if (switch1status == 1 || (switch1status == 1 && switch2status == 1) || (switch1status == 1 && switch2status == 0)) {
      lightLstatus = 0;
      lightRstatus = 0;
      lightcorneringstatus = 0;
    }
    else {
      //if the main switch is on (status 0), then check the status of the second switch
      //if the second switch is off (status 1), then the conrering light system is on (status 1)
      if (switch1status == 0 && switch2status == 1) {
        lightcorneringstatus = 1;
      }
      //if the second switch is on (status 0), then the cornering light system is off (status 0), and the lights are on
      if (switch1status == 0 && switch2status == 0) {
        lightcorneringstatus = 0;
        lightLstatus = 1;
        lightRstatus = 1;
      }
    }
  }
} */

/* This is the new SwitchesReading function. It supports a single switch ON-OFF-ON. */

void SwitchesReading() {
  for (;;) {
    switch1status = switch1.read();
    switch2status = switch2.read();

    //if the 1st and 2nd switches ere off (status 1), then both lights are off, the automatic cornering system is disabled
    if (switch1status == 1 && switch2status == 1) {
      lightLstatus = 0;
      lightRstatus = 0;
      lightcorneringstatus = 0;
    }
    //if the 1st switch is on (status 0) and the 2nd switch is off (status 1), then the conrering light system is on (status 1)
    if (switch1status == 0 && switch2status == 1) {
      lightcorneringstatus = 1;
    }
    //if the 1st switch is off (status 1), and the 2nd switch is on (status 0), then the cornering light system is off (status 0), and the both lights are on
    if (switch1status == 1 && switch2status == 0) {
      lightcorneringstatus = 0;
      lightLstatus = 1;
      lightRstatus = 1;
      }
    }
}

void LightsController() {
  for (;;) {
    if (lightcorneringstatus == 0) {
      if (lightLstatus == 1 && lightRstatus == 1) { 
        LightPinLeft.write(1);
        LightPinRight.write(1);
        led.write(1);       
        lightLstatus = 1;
        lightRstatus = 1;
      }
      else {
        LightPinLeft.write(0);
        LightPinRight.write(0);
        led.write(0);       
        lightLstatus = 0;
        lightRstatus = 0;        
      }
    }
    else {
      if (roll < rollONanglenegative) {
        LightPinLeft.write(1);
        LightPinRight.write(0);
        led.write(1);
        lightLstatus = 1;
        lightRstatus = 0;
        ThisThread::sleep_for(rollONangledelaytime);
      }
      if (roll > rollONanglepositive) {
        LightPinLeft.write(0);
        LightPinRight.write(1);
        led.write(1);
        lightLstatus = 0;
        lightRstatus = 1;
        ThisThread::sleep_for(rollONangledelaytime);
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
}

void telemetry() {
  for (;;) {
    Serial.print("Switch 1 = ");Serial.print(switch1status);Serial.print(" | Switch 2 status = ");Serial.print(switch2status);Serial.print(" | Roll = ");Serial.print(roll);Serial.print(" | Light cornering = ");Serial.print(lightcorneringstatus);Serial.print(" | Left = ");Serial.print(lightLstatus);Serial.print(" | Right = ");Serial.println(lightRstatus);
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
  //serialInit(115200);
  IMUInit();
  switch1.mode(PullUp);
  switch2.mode(PullUp);

  //threads
  TIMUreadings.start(IMUreadings);
  TSwitchesReading.start(SwitchesReading);
  TLightsController.start(LightsController);
  //Ttelemetry.start(telemetry);
}

void loop() {
  // empty loop

}