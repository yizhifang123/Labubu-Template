#include "motor-control.h"

#include "pid.h"
#include "robot-config.h"
#include "utils.h"

#include <algorithm>
#include <atomic>
#include <cmath>

using namespace vex;

std::atomic<bool> is_turning{false};
double correct_angle = 0.0;
double xpos = 0.0;
double ypos = 0.0;
double heading = 0.0;

// RW-Template global variables for motion chaining
double prev_left_output = 0.0;
double prev_right_output = 0.0;

namespace {

double wheelCircumference() {
  return M_PI * drivetrainDimensions.wheelDiameterIn;
}

double motorDegreesToInches(double degrees) {
  const double motorRotations = degrees / 360.0;
  const double wheelRotations = motorRotations / std::max(drivetrainDimensions.gearRatio, 1e-6);
  return wheelRotations * wheelCircumference();
}

double trackerDegreesToInches(double degrees, double diameter) {
  const double rotations = degrees / 360.0;
  return rotations * M_PI * diameter;
}

class SlewRateLimiter {
 public:
  SlewRateLimiter(double stepUp, double stepDown)
      : m_stepUp(stepUp), m_stepDown(stepDown) {}

  double filter(double target) {
    const double delta = target - m_current;
    if (delta > m_stepUp) {
      m_current += m_stepUp;
    } else if (delta < -m_stepDown) {
      m_current -= m_stepDown;
    } else {
      m_current = target;
    }
    return m_current;
  }

  void reset(double value = 0.0) { m_current = value; }

 private:
  double m_stepUp;
  double m_stepDown;
  double m_current = 0.0;
};

void setDriveVoltage(double left, double right) {
  left = clamp(left, -motionLimits.maxVoltage, motionLimits.maxVoltage);
  right = clamp(right, -motionLimits.maxVoltage, motionLimits.maxVoltage);
  left_chassis.spin(vex::directionType::fwd, left, vex::voltageUnits::volt);
  right_chassis.spin(vex::directionType::fwd, right, vex::voltageUnits::volt);
}

double computeDt(timer& t) {
  const double dt = std::max(t.time(sec), 1e-3);
  t.reset();
  return dt;
}

void updatePoseFromArc(double deltaForward, double deltaHeadingRad) {
  if (std::fabs(deltaHeadingRad) < 1e-6) {
    // Straight line
    xpos += deltaForward * std::sin(degToRad(heading));
    ypos += deltaForward * std::cos(degToRad(heading));
  } else {
    const double headingRad = degToRad(heading);
    const double radius = deltaForward / deltaHeadingRad;
    const double midHeading = headingRad + deltaHeadingRad / 2.0;

    xpos += radius * (std::sin(midHeading + deltaHeadingRad / 2.0) - std::sin(midHeading - deltaHeadingRad / 2.0));
    ypos += radius * (std::cos(midHeading - deltaHeadingRad / 2.0) - std::cos(midHeading + deltaHeadingRad / 2.0));
  }
  heading = wrapAngleDeg(heading + radToDeg(deltaHeadingRad));
}

template <typename Function>
void startThread(Function&& fn) {
  (void)thread(fn);
}

// Helper functions matching RW-Template implementation
void scaleToMin(double& left_output, double& right_output, double min_output) {
  if (std::fabs(left_output) <= std::fabs(right_output) && left_output < min_output && left_output > 0) {
    right_output = right_output / left_output * min_output;
    left_output = min_output;
  } else if (std::fabs(right_output) < std::fabs(left_output) && right_output < min_output && right_output > 0) {
    left_output = left_output / right_output * min_output;
    right_output = min_output;
  } else if (std::fabs(left_output) <= std::fabs(right_output) && left_output > -min_output && left_output < 0) {
    right_output = right_output / left_output * -min_output;
    left_output = -min_output;
  } else if (std::fabs(right_output) < std::fabs(left_output) && right_output > -min_output && right_output < 0) {
    left_output = left_output / right_output * -min_output;
    right_output = -min_output;
  }
}

void scaleToMax(double& left_output, double& right_output, double max_output) {
  if (std::fabs(left_output) >= std::fabs(right_output) && left_output > max_output) {
    right_output = right_output / left_output * max_output;
    left_output = max_output;
  } else if (std::fabs(right_output) > std::fabs(left_output) && right_output > max_output) {
    left_output = left_output / right_output * max_output;
    right_output = max_output;
  } else if (std::fabs(left_output) > std::fabs(right_output) && left_output < -max_output) {
    right_output = right_output / left_output * -max_output;
    left_output = -max_output;
  } else if (std::fabs(right_output) > std::fabs(left_output) && right_output < -max_output) {
    left_output = left_output / right_output * -max_output;
    right_output = -max_output;
  }
}

}  // namespace

void maintainHeadingTarget(double target) {
  correct_angle = wrapAngleDeg(target);
}

void initializeMotion() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  initialized = true;

  stopChassis(brakeType::coast);
  resetChassis();

  xpos = 0.0;
  ypos = 0.0;
  correct_angle = getInertialHeading();
  heading = correct_angle;

  if (trackingOptions.useHorizontalTracker && trackingOptions.useVerticalTracker) {
    startThread([] { trackXYOdomWheel(); });
  } else if (trackingOptions.useHorizontalTracker) {
    startThread([] { trackXOdomWheel(); });
  } else if (trackingOptions.useVerticalTracker) {
    startThread([] { trackYOdomWheel(); });
  } else {
    startThread([] { trackNoOdomWheel(); });
  }
}

void driveChassis(double left_power, double right_power) {
  setDriveVoltage(left_power, right_power);
}

double getInertialHeading() {
  return wrapAngleDeg(InertialSensor.rotation(degrees));
}

double normalizeTarget(double angle) {
  // RW-Template implementation using while loops
  if (angle - getInertialHeading() > 180) {
    while (angle - getInertialHeading() > 180) angle -= 360;
  } else if (angle - getInertialHeading() < -180) {
    while (angle - getInertialHeading() < -180) angle += 360;
  }
  return angle;
}

void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  // RW-Template implementation
  stopChassis(vex::brakeType::coast);
  is_turning.store(true, std::memory_order_seq_cst);
  double threshold = 1;
  PID pid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);

  // Normalize and set PID target
  turn_angle = normalizeTarget(turn_angle);
  pid.setTarget(turn_angle);
  pid.setOutputLimits(-max_output, max_output);
  pid.setIntegralLimit(0);  // setIntegralMax(0) in RW-Template
  pid.setTolerance(threshold, threshold * 4.5, 250);  // Combined tolerance setup

  // Draw baseline for visualization (RW-Template style)
  double draw_amplifier = 230 / std::fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, std::fabs(turn_angle) * draw_amplifier, 600, std::fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // PID loop for turning
  double start_time = Brain.timer(msec);
  timer sample;
  double output;
  double current_heading = getInertialHeading();
  double previous_heading = 0;
  int index = 1;
  
  if(exit == false && correct_angle < turn_angle) {
    // Turn right without stopping at end
    while (getInertialHeading() < turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      double dt = computeDt(sample);
      output = pid.step(current_heading, dt);
      // Draw heading trace
      Brain.Screen.drawLine(index * 3, std::fabs(previous_heading) * draw_amplifier, (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // Clamp output
      if(output < motionLimits.minVoltage) output = motionLimits.minVoltage;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  } else if(exit == false && correct_angle > turn_angle) {
    // Turn left without stopping at end
    while (getInertialHeading() > turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      double dt = computeDt(sample);
      output = pid.step(current_heading, dt);
      Brain.Screen.drawLine(index * 3, std::fabs(previous_heading) * draw_amplifier, (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < motionLimits.minVoltage) output = motionLimits.minVoltage;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(-output, output);
      wait(10, msec);
    }
  } else {
    // Standard PID turn
    while (!pid.isSettled() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      double dt = computeDt(sample);
      output = pid.step(current_heading, dt);
      Brain.Screen.drawLine(index * 3, std::fabs(previous_heading) * draw_amplifier, (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  }
  if(exit) {
    stopChassis(vex::brakeType::hold);
  }
  correct_angle = turn_angle;
  is_turning.store(false, std::memory_order_seq_cst);
}

void stopChassis(brakeType type) {
  left_chassis.stop(type);
  right_chassis.stop(type);
}

void resetChassis() {
  // RW-Template uses setPosition instead of resetPosition
  left_chassis.setPosition(0, degrees);
  right_chassis.setPosition(0, degrees);
}

double getLeftRotationDegree() {
  return leftEncoderMotor->position(degrees);
}

double getRightRotationDegree() {
  return rightEncoderMotor->position(degrees);
}

void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  // RW-Template implementation
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  stopChassis(vex::brakeType::coast);
  is_turning.store(true, std::memory_order_seq_cst);
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_slew_fwd = drive_direction > 0 ? motionLimits.slewStepForward : motionLimits.slewStepReverse;
  double max_slew_rev = drive_direction > 0 ? motionLimits.slewStepReverse : motionLimits.slewStepForward;
  bool min_speed = false;
  
  // Simplified chaining logic (RW-Template has dir_change_start/end variables we don't have)
  if(!exit) {
    // For chaining without direction change variables, use default behavior
    max_slew_fwd = 24;  // Large slew for smooth chaining
    max_slew_rev = 24;
    min_speed = true;
  }

  distance_in = distance_in * drive_direction;
  PID pid_distance(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);

  // Configure PID controller
  pid_distance.setTarget(distance_in);
  pid_distance.setOutputLimits(-max_output, max_output);
  pid_distance.setIntegralLimit(3);
  pid_distance.setTolerance(threshold, 5.0, 250);

  double start_time = Brain.timer(msec);
  timer sample;
  double left_output = 0, right_output = 0;
  double current_distance = 0, current_angle = 0;

  // Main PID loop for driving straight (matching RW-Template logic)
  while (((!pid_distance.isSettled() && Brain.timer(msec) - start_time <= time_limit_msec && exit) || 
          (exit == false && current_distance < distance_in && Brain.timer(msec) - start_time <= time_limit_msec))) {
    // Calculate current distance and heading
    double wheel_circ = wheelCircumference();
    current_distance = (std::fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_circ) + 
                        std::fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_circ)) / 2;
    double dt = computeDt(sample);
    left_output = pid_distance.step(current_distance, dt) * drive_direction;
    right_output = left_output;

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, motionLimits.minVoltage);
    }
    if(!exit) {
      left_output = 24 * drive_direction;
      right_output = 24 * drive_direction;
    }

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check (manual slew rate like RW-Template)
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::brakeType::hold);
  }
  is_turning.store(false, std::memory_order_seq_cst);
}

void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  // RW-Template implementation
  // Store initial encoder values for both sides
  double start_right = getRightRotationDegree(), start_left = getLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;

  // Normalize the target angle to be within +/-180 degrees of the current heading
  result_angle_deg = normalizeTarget(result_angle_deg);
  result_angle = (result_angle_deg - correct_angle) * M_PI / 180.0;

  // Calculate arc lengths for inner and outer wheels
  double track_width = drivetrainDimensions.trackWidthIn;
  in_arc = std::fabs((std::fabs(center_radius) - (track_width / 2)) * result_angle);
  out_arc = std::fabs((std::fabs(center_radius) + (track_width / 2)) * result_angle);
  ratio = in_arc / std::max(out_arc, 1e-6);

  stopChassis(vex::brakeType::coast);
  is_turning.store(true, std::memory_order_seq_cst);
  double threshold = 0.5;

  // Determine curve and drive direction
  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || 
      (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }

  // Slew rate and minimum speed logic for chaining (simplified - RW-Template has dir_change variables)
  double max_slew_fwd = drive_direction > 0 ? motionLimits.slewStepForward : motionLimits.slewStepReverse;
  double max_slew_rev = drive_direction > 0 ? motionLimits.slewStepReverse : motionLimits.slewStepForward;
  bool min_speed = false;
  if(!exit) {
    max_slew_fwd = 24;  // Large slew for smooth chaining
    max_slew_rev = 24;
    min_speed = true;
  }

  // Initialize PID controller for arc distance
  PID pid_out(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);

  pid_out.setTarget(out_arc);
  pid_out.setOutputLimits(-max_output, max_output);
  pid_out.setIntegralLimit(0);
  pid_out.setTolerance(0.3, 0.9 * 4.5, 250);

  double start_time = Brain.timer(msec);
  timer sample;
  double left_output = 0, right_output = 0;
  double current_right = 0, current_left = 0;
  double wheel_circ = wheelCircumference();

  // Main control loop for each curve/exit configuration (RW-Template has 4 cases)
  if (curve_direction == -1 && exit == true) {
    // Left curve, stop at end
    while (!pid_out.isSettled() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = std::fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_circ);
      double dt = computeDt(sample);
      right_output = pid_out.step(current_right, dt) * drive_direction;
      left_output = right_output * ratio;

      // Enforce minimum output if chaining
      if(min_speed) {
        scaleToMin(left_output, right_output, motionLimits.minVoltage);
      }

      // Enforce maximum output
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == 1 && exit == true) {
    // Right curve, stop at end
    while (!pid_out.isSettled() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_left = std::fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_circ);
      double dt = computeDt(sample);
      left_output = pid_out.step(current_left, dt) * drive_direction;
      right_output = left_output * ratio;

      if(min_speed) {
        scaleToMin(left_output, right_output, motionLimits.minVoltage);
      }

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == -1 && exit == false) {
    // Left curve, chaining (do not stop at end)
    while (current_right < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = std::fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_circ);
      double dt = computeDt(sample);
      right_output = pid_out.step(current_right, dt) * drive_direction;
      left_output = right_output * ratio;

      if(min_speed) {
        scaleToMin(left_output, right_output, motionLimits.minVoltage);
      }

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else {
    // Right curve, chaining (do not stop at end)
    while (current_left < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = std::fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_circ);
      double dt = computeDt(sample);
      left_output = pid_out.step(current_left, dt) * drive_direction;
      right_output = left_output * ratio;

      if(min_speed) {
        scaleToMin(left_output, right_output, motionLimits.minVoltage);
      }

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  }
  // Stop the chassis if required
  if(exit == true) {
    stopChassis(vex::brakeType::hold);
  }
  // Update the global heading
  correct_angle = result_angle_deg;
  is_turning.store(false, std::memory_order_seq_cst);
}

void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  // RW-Template implementation
  stopChassis(vex::brakeType::coast);
  is_turning.store(true, std::memory_order_seq_cst);
  double threshold = 1;
  PID pid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);

  swing_angle = normalizeTarget(swing_angle);
  pid.setTarget(swing_angle);
  pid.setOutputLimits(-max_output, max_output);
  pid.setIntegralLimit(0);  // setIntegralMax(0) in RW-Template
  pid.setTolerance(threshold, threshold * 4.5, 250);  // Combined tolerance setup

  // Draw the baseline for visualization
  double draw_amplifier = 230 / std::fabs(swing_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, std::fabs(swing_angle) * draw_amplifier, 600, std::fabs(swing_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // Start the PID loop
  double start_time = Brain.timer(msec);
  timer sample;
  double output;
  double current_heading = correct_angle;
  double previous_heading = 0;
  int index = 1;
  int choice = 1;

  // Determine which side to swing and direction
  if(swing_angle - correct_angle < 0 && drive_direction == 1) {
    choice = 1;
  } else if(swing_angle - correct_angle > 0 && drive_direction == 1) {
    choice = 2;
  } else if(swing_angle - correct_angle < 0 && drive_direction == -1) {
    choice = 3;
  } else {
    choice = 4;
  }

  // Swing logic for each case, chaining (exit == false)
  if(choice == 1 && exit == false) {
    // Swing left, forward
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      double dt = computeDt(sample);
      output = pid.step(current_heading, dt);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, std::fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < motionLimits.minVoltage) output = motionLimits.minVoltage;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(vex::brakeType::hold);
      right_chassis.spin(vex::directionType::fwd, output * drive_direction, vex::voltageUnits::volt);
      wait(10, msec);
    }
  } else if(choice == 2 && exit == false) {
    // Swing right, forward
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      double dt = computeDt(sample);
      output = pid.step(current_heading, dt);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, std::fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < motionLimits.minVoltage) output = motionLimits.minVoltage;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(vex::directionType::fwd, output * drive_direction, vex::voltageUnits::volt);
      right_chassis.stop(vex::brakeType::hold);
      wait(10, msec);
    }
  } else if(choice == 3 && exit == false) {
    // Swing left, backward
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      double dt = computeDt(sample);
      output = pid.step(current_heading, dt);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, std::fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < motionLimits.minVoltage) output = motionLimits.minVoltage;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(vex::directionType::fwd, output * drive_direction, vex::voltageUnits::volt);
      right_chassis.stop(vex::brakeType::hold);
      wait(10, msec);
    }
  } else if(choice == 4 && exit == false) {
    // Swing right, backward
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec && exit == false) {
      current_heading = getInertialHeading();
      double dt = computeDt(sample);
      output = pid.step(current_heading, dt);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, std::fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < motionLimits.minVoltage) output = motionLimits.minVoltage;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(vex::brakeType::hold);
      right_chassis.spin(vex::directionType::fwd, output * drive_direction, vex::voltageUnits::volt);
      wait(10, msec);
    }
  }

  // PID loop for exit == true (stop at end)
  while (!pid.isSettled() && Brain.timer(msec) - start_time <= time_limit_msec && exit == true) {
    current_heading = getInertialHeading();
    double dt = computeDt(sample);
    output = pid.step(current_heading, dt);

    // Draw heading trace
    Brain.Screen.drawLine(
        index * 3, std::fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    // Clamp output
    if(output > max_output) output = max_output;
    else if(output < -max_output) output = -max_output;

    // Apply output to correct side based on swing direction
    switch(choice) {
    case 1:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, -output * drive_direction, volt);
      break;
    case 2:
      left_chassis.spin(vex::directionType::fwd, output * drive_direction, vex::voltageUnits::volt);
      right_chassis.stop(vex::brakeType::hold);
      break;
    case 3:
      left_chassis.spin(vex::directionType::fwd, -output * drive_direction, vex::voltageUnits::volt);
      right_chassis.stop(vex::brakeType::hold);
      break;
    case 4:
      left_chassis.stop(vex::brakeType::hold);
      right_chassis.spin(vex::directionType::fwd, output * drive_direction, vex::voltageUnits::volt);
      break;
    }
    wait(10, msec);
  }
  if(exit == true) {
    stopChassis(vex::brakeType::hold);
  }
  correct_angle = swing_angle;
  is_turning.store(false, std::memory_order_seq_cst);
}

void trackNoOdomWheel() {
  double prevLeft = getLeftRotationDegree();
  double prevRight = getRightRotationDegree();
  double prevHeading = degToRad(getInertialHeading());

  while (true) {
    const double currentLeft = getLeftRotationDegree();
    const double currentRight = getRightRotationDegree();
    const double currentHeading = degToRad(getInertialHeading());

    const double deltaLeft = motorDegreesToInches(currentLeft - prevLeft);
    const double deltaRight = motorDegreesToInches(currentRight - prevRight);
    const double deltaForward = (deltaLeft + deltaRight) / 2.0;
    const double deltaHeading = currentHeading - prevHeading;

    heading = wrapAngleDeg(radToDeg(currentHeading));
    updatePoseFromArc(deltaForward, deltaHeading);

    prevLeft = currentLeft;
    prevRight = currentRight;
    prevHeading = currentHeading;

    wait(20, msec);
  }
}

void trackXYOdomWheel() {
  double prevHeading = degToRad(getInertialHeading());
  double prevHorizontal = HorizontalTracker.position(degrees);
  double prevVertical = VerticalTracker.position(degrees);

  while (true) {
    const double headingRad = degToRad(getInertialHeading());
    const double horizontal = HorizontalTracker.position(degrees);
    const double vertical = VerticalTracker.position(degrees);

    const double deltaHeading = headingRad - prevHeading;
    const double deltaHorizontal = trackerDegreesToInches(horizontal - prevHorizontal, drivetrainDimensions.horizontalTrackerDiameterIn);
    const double deltaVertical = trackerDegreesToInches(vertical - prevVertical, drivetrainDimensions.verticalTrackerDiameterIn);

    double deltaXLocal;
    double deltaYLocal;

    if (std::fabs(deltaHeading) < 1e-6) {
      deltaXLocal = deltaHorizontal;
      deltaYLocal = deltaVertical;
    } else {
      const double sinTerm = 2.0 * std::sin(deltaHeading / 2.0);
      deltaXLocal = sinTerm * ((deltaHorizontal / deltaHeading) + drivetrainDimensions.horizontalTrackerOffsetIn);
      deltaYLocal = sinTerm * ((deltaVertical / deltaHeading) + drivetrainDimensions.verticalTrackerOffsetIn);
    }

    const double averageHeading = prevHeading + deltaHeading / 2.0;
    xpos += deltaYLocal * std::sin(averageHeading) + deltaXLocal * std::cos(averageHeading);
    ypos += deltaYLocal * std::cos(averageHeading) - deltaXLocal * std::sin(averageHeading);
    heading = wrapAngleDeg(radToDeg(headingRad));

    prevHeading = headingRad;
    prevHorizontal = horizontal;
    prevVertical = vertical;

    wait(10, msec);
  }
}

void trackXOdomWheel() {
  double prevHeading = degToRad(getInertialHeading());
  double prevHorizontal = HorizontalTracker.position(degrees);
  double prevLeft = getLeftRotationDegree();
  double prevRight = getRightRotationDegree();

  while (true) {
    const double headingRad = degToRad(getInertialHeading());
    const double horizontal = HorizontalTracker.position(degrees);
    const double left = getLeftRotationDegree();
    const double right = getRightRotationDegree();

    const double deltaHeading = headingRad - prevHeading;
    const double deltaHorizontal = trackerDegreesToInches(horizontal - prevHorizontal, drivetrainDimensions.horizontalTrackerDiameterIn);
    const double deltaForward = (motorDegreesToInches(left - prevLeft) + motorDegreesToInches(right - prevRight)) / 2.0;

    double deltaXLocal;
    double deltaYLocal;

    if (std::fabs(deltaHeading) < 1e-6) {
      deltaXLocal = deltaHorizontal;
      deltaYLocal = deltaForward;
    } else {
      const double sinTerm = 2.0 * std::sin(deltaHeading / 2.0);
      deltaXLocal = sinTerm * ((deltaHorizontal / deltaHeading) + drivetrainDimensions.horizontalTrackerOffsetIn);
      deltaYLocal = sinTerm * ((deltaForward / deltaHeading) + drivetrainDimensions.trackWidthIn / 2.0);
    }

    const double averageHeading = prevHeading + deltaHeading / 2.0;
    xpos += deltaYLocal * std::sin(averageHeading) + deltaXLocal * std::cos(averageHeading);
    ypos += deltaYLocal * std::cos(averageHeading) - deltaXLocal * std::sin(averageHeading);
    heading = wrapAngleDeg(radToDeg(headingRad));

    prevHeading = headingRad;
    prevHorizontal = horizontal;
    prevLeft = left;
    prevRight = right;

    wait(10, msec);
  }
}

void trackYOdomWheel() {
  double prevHeading = degToRad(getInertialHeading());
  double prevVertical = VerticalTracker.position(degrees);

  while (true) {
    const double headingRad = degToRad(getInertialHeading());
    const double vertical = VerticalTracker.position(degrees);

    const double deltaHeading = headingRad - prevHeading;
    const double deltaVertical = trackerDegreesToInches(vertical - prevVertical, drivetrainDimensions.verticalTrackerDiameterIn);

    double deltaYLocal;
    if (std::fabs(deltaHeading) < 1e-6) {
      deltaYLocal = deltaVertical;
    } else {
      const double sinTerm = 2.0 * std::sin(deltaHeading / 2.0);
      deltaYLocal = sinTerm * ((deltaVertical / deltaHeading) + drivetrainDimensions.verticalTrackerOffsetIn);
    }

    const double averageHeading = prevHeading + deltaHeading / 2.0;
    xpos += deltaYLocal * std::sin(averageHeading);
    ypos += deltaYLocal * std::cos(averageHeading);
    heading = wrapAngleDeg(radToDeg(headingRad));

    prevHeading = headingRad;
    prevVertical = vertical;

    wait(10, msec);
  }
}

void turnToPoint(double x, double y, int direction, double time_limit_msec) {
  // RW-Template implementation
  stopChassis(vex::brakeType::coast);
  is_turning.store(true, std::memory_order_seq_cst);
  double threshold = 1, add = 0;
  if(direction == -1) {
    add = 180; // Add 180 degrees if turning to face backward
  }
  // Calculate target angle using atan2 and normalize
  double turn_angle = normalizeTarget(radToDeg(std::atan2(x - xpos, y - ypos))) + add;
  PID pid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);

  pid.setTarget(turn_angle);
  pid.setOutputLimits(-motionLimits.maxVoltage, motionLimits.maxVoltage);
  pid.setIntegralLimit(0);  // setIntegralMax(0) in RW-Template
  pid.setTolerance(threshold, threshold * 4.5, 500);  // Combined tolerance setup

  // Draw the baseline for visualization
  double draw_amplifier = 230 / std::fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, std::fabs(turn_angle) * draw_amplifier, 
                        600, std::fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // Start the PID loop
  double start_time = Brain.timer(msec);
  timer sample;
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  while (!pid.isSettled() && Brain.timer(msec) - start_time <= time_limit_msec) {
    // Continuously update target as robot moves
    pid.setTarget(normalizeTarget(radToDeg(std::atan2(x - xpos, y - ypos))) + add);
    current_heading = getInertialHeading();
    double dt = computeDt(sample);
    output = pid.step(current_heading, dt);

    // Draw heading trace
    Brain.Screen.drawLine(
        index * 3, std::fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, std::fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    driveChassis(output, -output);
    wait(10, msec);
  }  
  stopChassis(vex::hold);
  correct_angle = getInertialHeading();
  is_turning.store(false, std::memory_order_seq_cst);
}

void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  // RW-Template implementation
  stopChassis(vex::brakeType::coast);
  is_turning.store(true, std::memory_order_seq_cst);
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? motionLimits.slewStepForward : motionLimits.slewStepReverse;
  double max_slew_rev = dir > 0 ? motionLimits.slewStepReverse : motionLimits.slewStepForward;
  bool min_speed = false;
  
  // Simplified chaining logic (RW-Template has dir_change variables we don't have)
  if(!exit) {
    max_slew_fwd = 24;
    max_slew_rev = 24;
    min_speed = true;
  }

  PID pid_distance(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);

  // Set PID target for distance
  pid_distance.setTarget(std::hypot(x - xpos, y - ypos));
  pid_distance.setOutputLimits(-max_output, max_output);
  pid_distance.setIntegralLimit(0);
  pid_distance.setTolerance(threshold, 5.0, 250);

  double start_time = Brain.timer(msec);
  timer sample;
  double left_output = 0, right_output = 0;
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;

  // Main PID loop for moving to point (matching RW-Template logic)
  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    // Continuously update targets as robot moves
    double current_distance = std::hypot(x - xpos, y - ypos);
    double target_heading_angle = radToDeg(std::atan2(x - xpos, y - ypos));
    pid_distance.setTarget(current_distance);
    current_angle = getInertialHeading();
    
    // Calculate drive output based on heading and distance (adapting RW-Template logic)
    double dt = computeDt(sample);
    // RW-Template uses update(0) which uses error = target - 0, so we pass 0 as measurement
    // Our PID uses step(measurement, dt) where error = target - measurement
    // To match RW-Template's update(0), we pass 0 as measurement
    double distance_output = pid_distance.step(0, dt);
    double angle_to_target = target_heading_angle + add - current_angle;
    left_output = distance_output * std::cos(degToRad(angle_to_target)) * dir;
    right_output = left_output;
    
    // Check if robot has crossed the perpendicular line to the target
    double normalized_angle = normalizeTarget(current_angle + add);
    perpendicular_line = ((ypos - y) * -std::cos(degToRad(normalized_angle)) <= 
                          (xpos - x) * std::sin(degToRad(normalized_angle)) + exittolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, motionLimits.minVoltage);
    }

    right_output = left_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check (manual slew rate like RW-Template)
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
  if(exit == true) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::brakeType::hold);
  }
  correct_angle = getInertialHeading();
  is_turning.store(false, std::memory_order_seq_cst);
}

void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn) {
  // RW-Template implementation
  stopChassis(vex::brakeType::coast);
  is_turning.store(true, std::memory_order_seq_cst);
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? motionLimits.slewStepForward : motionLimits.slewStepReverse;
  double max_slew_rev = dir > 0 ? motionLimits.slewStepReverse : motionLimits.slewStepForward;
  bool min_speed = false;
  
  // Simplified chaining logic (RW-Template has dir_change variables we don't have)
  if(!exit) {
    max_slew_fwd = 24;
    max_slew_rev = 24;
    min_speed = true;
  }

  PID pid_distance(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);

  pid_distance.setTarget(0); // Target is dynamically updated
  pid_distance.setOutputLimits(-max_output, max_output);
  pid_distance.setIntegralLimit(3);
  pid_distance.setTolerance(threshold, 5.0, 250);

  double start_time = Brain.timer(msec);
  timer sample;
  double left_output = 0, right_output = 0, slip_speed = 0, overturn_value = 0;
  double exit_tolerance = 3;
  bool perpendicular_line = false, prev_perpendicular_line = true;
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  // Main PID loop for boomerang path (matching RW-Template logic)
  while (!pid_distance.isSettled() && Brain.timer(msec) - start_time <= time_limit_msec) {
    hypotenuse = std::hypot(xpos - x, ypos - y); // Distance to target
    // Calculate carrot point for path leading
    carrot_x = x - hypotenuse * std::sin(degToRad(a + add)) * dlead;
    carrot_y = y - hypotenuse * std::cos(degToRad(a + add)) * dlead;
    double carrot_distance = std::hypot(carrot_x - xpos, carrot_y - ypos);
    pid_distance.setTarget(carrot_distance * dir);
    current_angle = getInertialHeading();
    
    // Calculate drive output based on carrot point (adapting RW-Template logic)
    double dt = computeDt(sample);
    // RW-Template uses update(0) which uses error = target - 0, so we pass 0 as measurement
    // Our PID uses step(measurement, dt) where error = target - measurement
    // To match RW-Template's update(0), we pass 0 as measurement
    double distance_output = pid_distance.step(0, dt);
    double carrot_angle = radToDeg(std::atan2(carrot_x - xpos, carrot_y - ypos));
    double angle_to_carrot = carrot_angle + add - current_angle;
    left_output = distance_output * std::cos(degToRad(angle_to_carrot));
    right_output = left_output;
    
    // Check if robot has crossed the perpendicular line to the target
    double normalized_a = normalizeTarget(a);
    perpendicular_line = ((ypos - y) * -std::cos(degToRad(normalized_a)) <= 
                          (xpos - x) * std::sin(degToRad(normalized_a)) + exit_tolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, motionLimits.minVoltage);
    }

    // Limit slip speed for smoother curves (simplified - RW-Template uses chase_power)
    // RW-Template: slip_speed = sqrt(chase_power * getRadius(x_pos, y_pos, carrot_x, carrot_y, current_angle) * 9.8);
    double radius = getRadius(xpos, ypos, carrot_x, carrot_y, current_angle);
    slip_speed = std::sqrt(0.8 * std::fabs(radius) * 9.8);  // Simplified chase_power to 0.8
    if(left_output > slip_speed) {
      left_output = slip_speed;
    } else if(left_output < -slip_speed) {
      left_output = -slip_speed;
    }

    // Overturn logic for sharp turns
    overturn_value = std::fabs(left_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check (manual slew rate like RW-Template)
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::brakeType::hold);
  }
  correct_angle = a;
  is_turning.store(false, std::memory_order_seq_cst);
}

void getPose(double& x, double& y, double& heading_out) {
  x = xpos;
  y = ypos;
  heading_out = heading;
}

void setPose(double x, double y, double heading_in) {
  xpos = x;
  ypos = y;
  heading = wrapAngleDeg(heading_in);
  correct_angle = heading;
  InertialSensor.setHeading(correct_angle, degrees);
  HorizontalTracker.resetPosition();
  VerticalTracker.resetPosition();
}

