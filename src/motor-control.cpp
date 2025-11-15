#include "motor-control.h"

#include "pid.h"
#include "robot-config.h"
#include "utils.h"

#include <algorithm>
#include <atomic>
#include <cmath>

using namespace vex;

std::atomic<bool> headingCorrectionEnabled{false};
bool is_turning = false;
double correct_angle = 0.0;
double xpos = 0.0;
double ypos = 0.0;
double heading = 0.0;

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
  left_chassis.spin(fwd, left, voltageUnits::volt);
  right_chassis.spin(fwd, right, voltageUnits::volt);
}

void maintainHeadingTarget(double target) {
  correct_angle = wrapAngleDeg(target);
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

}  // namespace

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

  if (trackingOptions.enableHeadingHold) {
    headingCorrectionEnabled = true;
    startThread([] { correctHeading(); });
  }

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
  const double current = getInertialHeading();
  const double wrapped = wrapAngleDeg(angle - current);
  return wrapAngleDeg(current + wrapped);
}

void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  timer total;
  timer sample;

  turn_angle = normalizeTarget(turn_angle);
  PID turnPid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);
  turnPid.setTarget(turn_angle);
  turnPid.setOutputLimits(-max_output, max_output);
  turnPid.setIntegralLimit(4.0);
  turnPid.setTolerance(1.0, 5.0, 150);

  is_turning = true;

  while (total.time(msec) <= time_limit_msec && !turnPid.isSettled()) {
    const double dt = computeDt(sample);
    const double heading = getInertialHeading();
    double output = turnPid.step(heading, dt);

    if (std::fabs(output) < motionLimits.minVoltage) {
      output = std::copysign(motionLimits.minVoltage, output);
    }

    setDriveVoltage(output, -output);
    wait(10, msec);
  }

  if (exit) {
    stopChassis(brakeType::hold);
  }

  maintainHeadingTarget(turn_angle);
  is_turning = false;
}

void stopChassis(brakeType type) {
  left_chassis.stop(type);
  right_chassis.stop(type);
}

void resetChassis() {
  left_chassis.resetPosition();
  right_chassis.resetPosition();
}

double getLeftRotationDegree() {
  return leftEncoderMotor->position(degrees);
}

double getRightRotationDegree() {
  return rightEncoderMotor->position(degrees);
}

void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  timer total;
  timer sample;

  const bool forward = distance_in >= 0.0;
  const double direction = forward ? 1.0 : -1.0;
  const double targetDistance = std::fabs(distance_in);

  PID distancePid(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);
  distancePid.setTarget(targetDistance);
  distancePid.setOutputLimits(-max_output, max_output);
  distancePid.setIntegralLimit(6.0);
  distancePid.setTolerance(0.5, 5.0, 150);

  PID headingPid(motionGains.headingHoldKp, motionGains.headingHoldKi, motionGains.headingHoldKd);
  headingPid.setTarget(correct_angle);
  headingPid.setOutputLimits(-max_output, max_output);
  headingPid.setIntegralLimit(2.0);
  headingPid.setTolerance(0.5, 5.0, 100);

  SlewRateLimiter leftSlew(motionLimits.slewStepForward, motionLimits.slewStepReverse);
  SlewRateLimiter rightSlew(motionLimits.slewStepForward, motionLimits.slewStepReverse);

  resetChassis();
  is_turning = true;

  while (total.time(msec) <= time_limit_msec) {
    const double dt = computeDt(sample);
    const double leftDistance = std::fabs(motorDegreesToInches(getLeftRotationDegree()));
    const double rightDistance = std::fabs(motorDegreesToInches(getRightRotationDegree()));
    const double travelled = (leftDistance + rightDistance) / 2.0;

    double baseVoltage = distancePid.step(travelled, dt) * direction;
    if (!exit && travelled >= targetDistance) {
      break;
    }

    double correction = 0.0;
    if (trackingOptions.enableHeadingHold) {
      headingPid.setTarget(correct_angle);
      correction = headingPid.step(getInertialHeading(), dt);
    }

    double leftVoltage = clamp(baseVoltage + correction, -max_output, max_output);
    double rightVoltage = clamp(baseVoltage - correction, -max_output, max_output);

    leftVoltage = leftSlew.filter(leftVoltage);
    rightVoltage = rightSlew.filter(rightVoltage);

    if (std::fabs(leftVoltage) < motionLimits.minVoltage) {
      leftVoltage = std::copysign(motionLimits.minVoltage, leftVoltage);
    }
    if (std::fabs(rightVoltage) < motionLimits.minVoltage) {
      rightVoltage = std::copysign(motionLimits.minVoltage, rightVoltage);
    }

    setDriveVoltage(leftVoltage, rightVoltage);

    if (distancePid.isSettled() && exit) {
      break;
    }

    wait(10, msec);
  }

  if (exit) {
    stopChassis(brakeType::hold);
  }

  maintainHeadingTarget(getInertialHeading());
  is_turning = false;
}

void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  if (std::fabs(center_radius) < 1e-3) {
    turnToAngle(result_angle_deg, time_limit_msec, exit, max_output);
    return;
  }

  timer total;
  timer sample;

  const double targetHeading = normalizeTarget(result_angle_deg);
  const double headingChange = wrapAngleDeg(targetHeading - correct_angle);
  const double headingRad = degToRad(std::fabs(headingChange));
  const bool turningLeft = center_radius < 0;

  const double outerRadius = std::fabs(center_radius) + drivetrainDimensions.trackWidthIn / 2.0;
  const double innerRadius = std::max(std::fabs(center_radius) - drivetrainDimensions.trackWidthIn / 2.0, 1e-3);

  const double outerArc = headingRad * outerRadius;
  const double innerArc = headingRad * innerRadius;
  const double ratio = innerArc / std::max(outerArc, 1e-6);
  const double driveDirection = headingChange >= 0 ? 1.0 : -1.0;

  PID outerPid(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);
  outerPid.setTarget(outerArc);
  outerPid.setOutputLimits(-max_output, max_output);
  outerPid.setIntegralLimit(6.0);
  outerPid.setTolerance(0.5, 5.0, 150);

  PID headingPid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);
  headingPid.setTarget(targetHeading);
  headingPid.setOutputLimits(-max_output, max_output);
  headingPid.setIntegralLimit(2.0);
  headingPid.setTolerance(1.0, 5.0, 150);

  resetChassis();
  is_turning = true;

  while (total.time(msec) <= time_limit_msec) {
    const double dt = computeDt(sample);
    const double leftDistance = std::fabs(motorDegreesToInches(getLeftRotationDegree()));
    const double rightDistance = std::fabs(motorDegreesToInches(getRightRotationDegree()));

    const double outerTravel = turningLeft ? rightDistance : leftDistance;

    double outerCommand = outerPid.step(outerTravel, dt) * driveDirection;
    double innerCommand = outerCommand * ratio;
    double headingCorrection = headingPid.step(getInertialHeading(), dt);

    double leftVoltage = turningLeft ? innerCommand : outerCommand;
    double rightVoltage = turningLeft ? outerCommand : innerCommand;

    leftVoltage += headingCorrection;
    rightVoltage -= headingCorrection;

    if (std::fabs(leftVoltage) < motionLimits.minVoltage) {
      leftVoltage = std::copysign(motionLimits.minVoltage, leftVoltage);
    }
    if (std::fabs(rightVoltage) < motionLimits.minVoltage) {
      rightVoltage = std::copysign(motionLimits.minVoltage, rightVoltage);
    }

    setDriveVoltage(leftVoltage, rightVoltage);

    if (outerPid.isSettled() && exit) {
      break;
    }

    wait(10, msec);
  }

  if (exit) {
    stopChassis(brakeType::hold);
  }

  maintainHeadingTarget(targetHeading);
  is_turning = false;
}

void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  timer total;
  timer sample;

  const double targetHeading = normalizeTarget(swing_angle);
  const double delta = wrapAngleDeg(targetHeading - correct_angle);
  const bool turningRight = delta > 0.0;
  const bool forward = drive_direction >= 0.0;

  const bool holdRight = (turningRight && forward) || (!turningRight && !forward);

  PID swingPid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);
  swingPid.setTarget(targetHeading);
  swingPid.setOutputLimits(-max_output, max_output);
  swingPid.setIntegralLimit(2.0);
  swingPid.setTolerance(1.0, 5.0, 150);

  is_turning = true;

  while (total.time(msec) <= time_limit_msec && !swingPid.isSettled()) {
    const double dt = computeDt(sample);
    double output = swingPid.step(getInertialHeading(), dt);

    if (std::fabs(output) < motionLimits.minVoltage) {
      output = std::copysign(motionLimits.minVoltage, output);
    }

    if (holdRight) {
      right_chassis.stop(brakeType::hold);
      left_chassis.spin(fwd, output * (forward ? 1.0 : -1.0), voltageUnits::volt);
    } else {
      left_chassis.stop(brakeType::hold);
      right_chassis.spin(fwd, -output * (forward ? 1.0 : -1.0), voltageUnits::volt);
    }

    wait(10, msec);
  }

  if (exit) {
    stopChassis(brakeType::hold);
  }

  maintainHeadingTarget(targetHeading);
  is_turning = false;
}

void correctHeading() {
  PID holdPid(motionGains.headingHoldKp, motionGains.headingHoldKi, motionGains.headingHoldKd);
  holdPid.setOutputLimits(-motionLimits.maxVoltage, motionLimits.maxVoltage);
  holdPid.setIntegralLimit(2.0);
  holdPid.setTolerance(0.5, 5.0, 200);

  timer sample;

  while (true) {
    if (headingCorrectionEnabled.load() && !is_turning) {
      holdPid.setTarget(correct_angle);
      const double dt = computeDt(sample);
      double correction = holdPid.step(getInertialHeading(), dt);

      if (std::fabs(correction) < motionLimits.minVoltage ||
          std::fabs(holdPid.error()) < 0.5) {
        correction = 0.0;
      }

      setDriveVoltage(correction, -correction);
    } else {
      holdPid.resetIntegral();
      sample.reset();
    }

    wait(20, msec);
  }
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

void turnToPoint(double x, double y, int dir, double time_limit_msec) {
  const double angle = radToDeg(std::atan2(x - xpos, y - ypos));
  const double target = dir >= 0 ? angle : wrapAngleDeg(angle + 180.0);
  turnToAngle(target, time_limit_msec, true, motionLimits.maxVoltage);
}

void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  timer total;
  timer sample;

  PID distancePid(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);
  distancePid.setTarget(0.0);
  distancePid.setOutputLimits(-max_output, max_output);
  distancePid.setIntegralLimit(6.0);
  distancePid.setTolerance(0.75, 5.0, 150);

  PID headingPid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);
  headingPid.setOutputLimits(-max_output, max_output);
  headingPid.setIntegralLimit(2.0);
  headingPid.setTolerance(2.0, 6.0, 150);

  SlewRateLimiter leftSlew(motionLimits.slewStepForward, motionLimits.slewStepReverse);
  SlewRateLimiter rightSlew(motionLimits.slewStepForward, motionLimits.slewStepReverse);

  is_turning = true;

  while (total.time(msec) <= time_limit_msec) {
    const double dt = computeDt(sample);
    const double dx = x - xpos;
    const double dy = y - ypos;
    const double distance = std::hypot(dx, dy);

    if (distance < 0.75 && exit) {
      break;
    }

    const double desiredHeading = wrapAngleDeg(radToDeg(std::atan2(dx, dy)) + (dir >= 0 ? 0.0 : 180.0));
    headingPid.setTarget(desiredHeading);

    const double rawOutput = distancePid.step(distance, dt);
    const double directionSign = dir >= 0 ? 1.0 : -1.0;
    const double translational = -rawOutput * directionSign;
    const double correction = headingPid.step(getInertialHeading(), dt);

    double leftVoltage = translational + correction;
    double rightVoltage = translational - correction;

    if (overturn) {
      const double excess = std::fabs(leftVoltage) - max_output;
      if (excess > 0) leftVoltage -= excess * std::copysign(1.0, leftVoltage);
      const double excessRight = std::fabs(rightVoltage) - max_output;
      if (excessRight > 0) rightVoltage -= excessRight * std::copysign(1.0, rightVoltage);
    }

    leftVoltage = leftSlew.filter(clamp(leftVoltage, -max_output, max_output));
    rightVoltage = rightSlew.filter(clamp(rightVoltage, -max_output, max_output));

    if (std::fabs(leftVoltage) < motionLimits.minVoltage) {
      leftVoltage = std::copysign(motionLimits.minVoltage, leftVoltage);
    }
    if (std::fabs(rightVoltage) < motionLimits.minVoltage) {
      rightVoltage = std::copysign(motionLimits.minVoltage, rightVoltage);
    }

    setDriveVoltage(leftVoltage, rightVoltage);

    if (!exit && distance < 1.0) {
      break;
    }

    wait(10, msec);
  }

  if (exit) {
    stopChassis(brakeType::hold);
  }

  maintainHeadingTarget(getInertialHeading());
  is_turning = false;
}

void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn) {
  timer total;
  timer sample;

  const double clampedLead = clamp(dlead, 0.0, 1.0);
  const double finalHeading = normalizeTarget(a);

  PID distancePid(motionGains.driveKp, motionGains.driveKi, motionGains.driveKd);
  distancePid.setTarget(0.0);
  distancePid.setOutputLimits(-max_output, max_output);
  distancePid.setIntegralLimit(6.0);
  distancePid.setTolerance(0.5, 5.0, 150);

  PID headingPid(motionGains.turnKp, motionGains.turnKi, motionGains.turnKd);
  headingPid.setOutputLimits(-max_output, max_output);
  headingPid.setIntegralLimit(2.0);
  headingPid.setTolerance(2.0, 6.0, 150);

  is_turning = true;

  while (total.time(msec) <= time_limit_msec) {
    const double dt = computeDt(sample);
    const double dx = x - xpos;
    const double dy = y - ypos;
    const double distance = std::hypot(dx, dy);

    if (distance < 0.75 && exit) {
      break;
    }

    const double carrotDistance = distance * clampedLead;
    const double leadHeading = degToRad(finalHeading);
    const double carrotX = x - carrotDistance * std::sin(leadHeading);
    const double carrotY = y - carrotDistance * std::cos(leadHeading);

    const double carrotDx = carrotX - xpos;
    const double carrotDy = carrotY - ypos;
    const double carrotDist = std::hypot(carrotDx, carrotDy);

    const double desiredHeading = wrapAngleDeg(radToDeg(std::atan2(carrotDx, carrotDy)) + (dir >= 0 ? 0.0 : 180.0));
    headingPid.setTarget(desiredHeading);

    const double rawOutput = distancePid.step(carrotDist, dt);
    const double directionSign = dir >= 0 ? 1.0 : -1.0;
    const double translational = -rawOutput * directionSign;
    const double correction = headingPid.step(getInertialHeading(), dt);

    double leftVoltage = translational + correction;
    double rightVoltage = translational - correction;

    if (overturn) {
      const double excessL = std::fabs(leftVoltage) - max_output;
      if (excessL > 0) leftVoltage -= excessL * std::copysign(1.0, leftVoltage);
      const double excessR = std::fabs(rightVoltage) - max_output;
      if (excessR > 0) rightVoltage -= excessR * std::copysign(1.0, rightVoltage);
    }

    leftVoltage = clamp(leftVoltage, -max_output, max_output);
    rightVoltage = clamp(rightVoltage, -max_output, max_output);

    if (std::fabs(leftVoltage) < motionLimits.minVoltage) {
      leftVoltage = std::copysign(motionLimits.minVoltage, leftVoltage);
    }
    if (std::fabs(rightVoltage) < motionLimits.minVoltage) {
      rightVoltage = std::copysign(motionLimits.minVoltage, rightVoltage);
    }

    setDriveVoltage(leftVoltage, rightVoltage);

    if (!exit && carrotDist < 1.0) {
      break;
    }

    wait(10, msec);
  }

  if (exit) {
    stopChassis(brakeType::hold);
  }

  maintainHeadingTarget(finalHeading);
  is_turning = false;
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

