#pragma once

#include "v5.h"
#include "v5_vcs.h"

/**
 * The device declarations live in this header so every source file can access
 * consistent hardware objects.  Update the port assignments to match your
 * robot and call `configureDevices()` from `main()` once at startup.
 */
extern vex::brain Brain;
extern vex::controller Controller1;

extern vex::motor left_chassis1;
extern vex::motor left_chassis2;
extern vex::motor left_chassis3;
extern vex::motor_group left_chassis;
extern vex::motor right_chassis1;
extern vex::motor right_chassis2;
extern vex::motor right_chassis3;
extern vex::motor_group right_chassis;
extern vex::motor* leftEncoderMotor;
extern vex::motor* rightEncoderMotor;

extern vex::inertial InertialSensor;
extern vex::rotation HorizontalTracker;
extern vex::rotation VerticalTracker;

/**
 * Describes the drivetrain geometry so the motion algorithms can reason
 * about distances.  All units are inches and represent physical measurements.
 */
struct DrivetrainDimensions {
  double wheelDiameterIn = 4.0;      ///< Wheel diameter in inches
  double gearRatio = 1.0;            ///< Motor rotations : wheel rotations
  double trackWidthIn = 11.0;        ///< Distance between left/right wheels
  double horizontalTrackerDiameterIn = 2.75;
  double verticalTrackerDiameterIn = 2.75;
  double horizontalTrackerOffsetIn = 0.0; ///< + is to the left of robot center
  double verticalTrackerOffsetIn = 0.0;   ///< + is forward from robot center
};

extern DrivetrainDimensions drivetrainDimensions;

/**
 * Slew rate, voltage limits, and default PID gains live together so they are
 * easy to tweak.  Voltages are specified in V5 **volts** (max 12).
 */
struct MotionLimits {
  double maxVoltage = 12.0;
  double minVoltage = 1.5;
  double slewStepForward = 2.0;  ///< Maximum increase per loop (volts)
  double slewStepReverse = 2.5;  ///< Maximum decrease per loop (volts)
};

extern MotionLimits motionLimits;

/**
 * Holds the PID gains that the motion routines use.  The same PID class powers
 * turning, driving, and heading corrections, so we expose the values here.
 */
struct MotionGains {
  double driveKp = 0.12;
  double driveKi = 0.0;
  double driveKd = 0.005;

  double turnKp = 0.45;
  double turnKi = 0.0;
  double turnKd = 0.36;

  double headingHoldKp = 0.16;
  double headingHoldKi = 0.0;
  double headingHoldKd = 0.002;
};

extern MotionGains motionGains;

/**
 * Toggle individual subsystems so teams can start simple and scale up.  The
 * odometry routines automatically pick the correct tracking strategy based on
 * which sensors are enabled.
 */
struct TrackingOptions {
  bool useHorizontalTracker = false;
  bool useVerticalTracker = false;
  bool enableHeadingHold = true;
};

extern TrackingOptions trackingOptions;

/**
 * Configure sensor calibration and motor properties.  Call this once inside
 * `pre_auton()` before any motion code runs.
 */
void configureDevices();


