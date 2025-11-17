#include "vex.h"
#include "robot-config.h"

using namespace vex;

// ---------------------------------------------------------------------------//
// Device Instances                                                           //
// ---------------------------------------------------------------------------//

brain Brain;
controller Controller1 = controller(primary);

motor left_chassis1 = motor(PORT20, gearSetting::ratio6_1, true);
motor left_chassis2 = motor(PORT8, gearSetting::ratio6_1, true);
motor left_chassis3 = motor(PORT10, gearSetting::ratio6_1, true);
motor_group left_chassis(left_chassis1, left_chassis2, left_chassis3);

motor right_chassis1 = motor(PORT14, gearSetting::ratio6_1, false);
motor right_chassis2 = motor(PORT3, gearSetting::ratio6_1, false);
motor right_chassis3 = motor(PORT5, gearSetting::ratio6_1, false);
motor_group right_chassis(right_chassis1, right_chassis2, right_chassis3);

motor* leftEncoderMotor = &left_chassis1;
motor* rightEncoderMotor = &right_chassis1;

inertial InertialSensor = inertial(PORT15);
rotation HorizontalTracker = rotation(PORT6, false);
rotation VerticalTracker   = rotation(PORT6, false);

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

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Configuring drive...");
  configureDriveMotors();

  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Resetting trackers...");
  HorizontalTracker.resetPosition();
  VerticalTracker.resetPosition();

  // Default options: heading hold disabled, odom sensors off until configured.
  trackingOptions.enableHeadingHold = false;
  trackingOptions.useHorizontalTracker = false;
  trackingOptions.useVerticalTracker = false;

  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Robot ready. Adjust tuning in robot-config.h");
}

