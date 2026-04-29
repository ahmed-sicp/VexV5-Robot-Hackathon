#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor leftMotor = motor(PORT1, ratio18_1, false);

motor rightMotor = motor(PORT10, ratio18_1, false);

inertial IMU = inertial(PORT5);

controller Controller1 = controller(primary);


// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  IMU & Encoder Tuner - VEXCode V5 Basic C++               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// BotValues struct
struct BotValues {
  float leftRatio;
  float rightRatio;
  float trackWidth;

  BotValues(float l, float r, float tw)
      : leftRatio(l), rightRatio(r), trackWidth(tw) {}
};

int main() {
  // Calibrate IMU
  IMU.calibrate();
  Brain.Screen.printAt(1, 30, "Calibrating IMU...");
  while (IMU.isCalibrating()) {
    wait(10, msec);
  }
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 30, "IMU Ready! Tuning Opmode");
  wait(1000, msec);

  // Reset encoders and IMU heading
  leftMotor.resetPosition();
  rightMotor.resetPosition();
  IMU.resetRotation();

  // Placeholder tuning values — adjust these!
  BotValues tuning_vals(1.0f, 1.0f, 12.5f);

  // Main loop
  while (true) {
    float l_deg   = (float)leftMotor.position(degrees);
    float r_deg   = (float)rightMotor.position(degrees);
    float imu_deg = -(float)IMU.rotation(degrees); // CW positive

    // Encoder-based heading
    float l_dist  = l_deg * tuning_vals.leftRatio;
    float r_dist  = r_deg * tuning_vals.rightRatio;
    float enc_rad = (r_dist - l_dist) / tuning_vals.trackWidth;
    float enc_deg = enc_rad * (180.0f / M_PI);

    // Brain screen output
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(1,  30, "IMU & Encoder Tuner");
    Brain.Screen.printAt(1,  60, "IMU Deg: %.2f", imu_deg);
    Brain.Screen.printAt(1,  90, "ENC Deg: %.2f", enc_deg);
    Brain.Screen.printAt(1, 120, "DIFF:    %.2f", imu_deg - enc_deg);
    Brain.Screen.printAt(1, 150, "X = Reset");

    // X button resets encoders and IMU heading
    if (Controller1.ButtonX.pressing()) {
      leftMotor.resetPosition();
      rightMotor.resetPosition();
      IMU.resetRotation();
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(1, 60, "-- RESET --");
      wait(500, msec);
    }

    wait(50, msec);
  }
}