#include "vex.h"
#include "robot-config.h"

using namespace vex;

// ---------------------------------------------------------------------------//
// Device Instances                                                           //
// ---------------------------------------------------------------------------//

brain Brain;
controller Controller1 = controller(primary);

motor left_chassis1 = motor(PORT5, gearSetting::ratio6_1, true);
motor left_chassis2 = motor(PORT6, gearSetting::ratio6_1, true);
motor left_chassis3 = motor(PORT7, gearSetting::ratio6_1, false);
motor_group left_chassis(left_chassis1, left_chassis2, left_chassis3);

motor right_chassis1 = motor(PORT3, gearSetting::ratio6_1, false);
motor right_chassis2 = motor(PORT4, gearSetting::ratio6_1, false);
motor right_chassis3 = motor(PORT11, gearSetting::ratio6_1, true);
motor_group right_chassis(right_chassis1, right_chassis2, right_chassis3);

motor* leftEncoderMotor = &left_chassis1;
motor* rightEncoderMotor = &right_chassis1;

inertial InertialSensor = inertial(PORT11);
rotation HorizontalTracker = rotation(PORT3, false);
rotation VerticalTracker   = rotation(PORT4, false);

// ---------------------------------------------------------------------------//
// Tunable Parameters                                                         //
// ---------------------------------------------------------------------------//

DrivetrainDimensions drivetrainDimensions{};
MotionLimits motionLimits{};
MotionGains motionGains{};
TrackingOptions trackingOptions{};

// ---------------------------------------------------------------------------//
// Internal Helpers                                                           //
// ---------------------------------------------------------------------------//

namespace {

void configureDriveMotors() {
  left_chassis.setStopping(brakeType::coast);
  right_chassis.setStopping(brakeType::coast);

  leftEncoderMotor = &left_chassis1;
  rightEncoderMotor = &right_chassis1;
}

} // namespace

// ---------------------------------------------------------------------------//
// Configuration                                                              //
// ---------------------------------------------------------------------------//

void configureDevices() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrating inertial...");

  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(20, msec);
  }

  Brain.Screen.nextRow();
  Brain.Screen.print("Configuring drive...");
  configureDriveMotors();

  Brain.Screen.nextRow();
  Brain.Screen.print("Resetting trackers...");
  HorizontalTracker.resetPosition();
  VerticalTracker.resetPosition();

  // Default options: heading hold enabled, odom sensors off until configured.
  trackingOptions.enableHeadingHold = true;
  trackingOptions.useHorizontalTracker = false;
  trackingOptions.useVerticalTracker = false;

  Brain.Screen.nextRow();
  Brain.Screen.print("Robot ready. Adjust tuning in robot-config.h");
}

