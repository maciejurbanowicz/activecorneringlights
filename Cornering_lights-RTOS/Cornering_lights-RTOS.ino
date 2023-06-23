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

//Switch
//DigitalIn switch1(P0_21); //the main switch connected to D8
//DigitalIn switch2(P0_27); //the secondary switch connected to D9
int switch1 = 8; //the main switch connected to D8
int switch2 = 9; //the secondary switch connected do D9

int switch1status = 1; //default value for the switch, 1 = OFF, 0 = ON
int switch2status = 1; //default value for the switch, 1 = OFF, 0 = ON

//Lights
int lightLstatus = 0;
int lightRstatus = 0;

//Threads
//Thread TSwitchesReading;
Thread TIMUreadings;
Thread TLightsController;
//Thread Ttelemetry;

//LED on-board for checking ON/OFF of lights
DigitalOut led(LED1);

/* IMUreadings reads raw values from IMU (Acceleration and Gyroscope) and uses the Madgwick filter to provide roll angle. Pitch and heading are also available. */

void IMUreadings() {
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
  }
}

/* This is the new SwitchesReading function. It supports a single switch ON-OFF-ON. */

void SwitchesReading() {
  //for (;;) {
    {
    //switch1status = switch1.read();
    switch1status = digitalRead(switch1);
    //switch2status = switch2.read();
    switch2status = digitalRead(switch2);
    }
}

void LightsController() {
  for (;;) {
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
    if (switch1status == 1 && switch2status == 0) {
		  LightPinLeft.write(1);
      LightPinRight.write(1);
      led.write(1);       
      lightLstatus = 1;
      lightRstatus = 1;
		}
  }
}

void telemetry() {
  for (;;) {
    Serial.print("Switch 1 = ");Serial.print(switch1status);
    Serial.print(" | Switch 2 = ");Serial.print(switch2status);
    Serial.print(" | Roll = ");Serial.print(roll);
    Serial.print(" | Left = ");Serial.print(lightLstatus);
    Serial.print(" | Right = ");Serial.println(lightRstatus);
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
  //serialInit(9600);
  IMUInit();
  //switch1.mode(PullUp);
  pinMode(switch1, INPUT_PULLUP);
  //switch2.mode(PullUp);
  pinMode(switch2, INPUT_PULLUP);

  SwitchesReading(); //Initial reading of the statuses of the switches.

  attachInterrupt(digitalPinToInterrupt(switch1),SwitchesReading,CHANGE);
  attachInterrupt(digitalPinToInterrupt(switch2),SwitchesReading,CHANGE);

  //threads
  TIMUreadings.start(IMUreadings);
  //TSwitchesReading.start(SwitchesReading);
  TLightsController.start(LightsController);
  //Ttelemetry.start(telemetry);
}

void loop() {
  // empty loop
}
