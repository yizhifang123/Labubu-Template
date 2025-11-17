#include "vex.h"
#include <cmath>

using namespace vex;

competition Competition;

/**
 * Runs once when the robot is powered on.  Use this to configure sensors,
 * calibrate the inertial, and spin up background tasks.
 */
void runPreAutonomous() {
  configureDevices();
  initializeMotion();

  // Set an initial pose.  Change these numbers after placing the robot on
  // the field so autonomous starts from the correct location.
  setPose(0.0, 0.0, getInertialHeading());
}

/**
 * Example autonomous routine demonstrating how the higher-level motion
 * helpers work together.  Replace these commands with your own routine.
 */
void runAutonomous() {
  // Drive forward 24 inches while holding the current heading.
  driveTo(24.0, 1000);

  // Swing turn to face the goal.
  swing(correct_angle + 45.0, 1, 2000);


  // Finish with a tight turn toward the alliance stake.
  turnToPoint(48.0, 0.0, 1, 1500);
}

/**
 * Driver control loop.  This template uses an arcade-style drive by default
 * but you can adjust the mix to your preferences.
 */
void runDriver() {
  while (true) {

    // Split-arcade drive:
    // - Left stick vertical (Axis3) controls forward/backward speed
    // - Right stick horizontal (Axis1) controls turning
    double forward = Controller1.Axis3.position(percentUnits::pct);
    double turn = Controller1.Axis1.position(percentUnits::pct);

    // Basic deadband to avoid drift when sticks rest near 0.
    if (std::fabs(forward) < 5.0) forward = 0.0;
    if (std::fabs(turn) < 5.0) turn = 0.0;

    // Scale turning so small stick inputs are 40% softer while full deflection stays 100%.
    // const double turnMagnitude = std::fabs(turn);
    // const double turnScale = 0.6 + 0.4 * (turnMagnitude / 100.0);
    // turn *= turnScale;

    // Mix forward and turn into left/right volt outputs.
    const double leftVoltage = (forward + turn) * 0.12;
    const double rightVoltage = (forward - turn) * 0.12;
    driveChassis(leftVoltage, rightVoltage);

    wait(20, msec);
  }
}

void pre_auton() {
  runPreAutonomous();
}

void autonomous() {
  runAutonomous();
}

void usercontrol() {
  runDriver();
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
