#include "pid.h"
#include "utils.h"
#include <algorithm>
#include <cmath>

PID::PID(double kp, double ki, double kd)
  : m_kp(kp), m_ki(ki), m_kd(kd),
    m_target(0.0), m_error(0.0), m_prevError(0.0),
    m_integral(0.0), m_derivative(0.0), m_output(0.0),
    m_minOutput(-12.0), m_maxOutput(12.0),
    m_integralLimit(1e6),
    m_errorTolerance(0.5), m_derivativeTolerance(0.5),
    m_settleTimeMsec(100), m_timeInTolerance(0) {}

void PID::setGains(double kp, double ki, double kd) {
  m_kp = kp;
  m_ki = ki;
  m_kd = kd;
}

void PID::setTarget(double target) {
  m_target = target;
}

void PID::setOutputLimits(double minOutput, double maxOutput) {
  m_minOutput = minOutput;
  m_maxOutput = maxOutput;
}

void PID::setIntegralLimit(double integralLimit) {
  m_integralLimit = std::fabs(integralLimit);
}

void PID::setTolerance(double errorTolerance, double derivativeTolerance, int settleTimeMsec) {
  m_errorTolerance = std::fabs(errorTolerance);
  m_derivativeTolerance = std::fabs(derivativeTolerance);
  m_settleTimeMsec = std::max(settleTimeMsec, 0);
}

void PID::reset() {
  m_error = 0.0;
  m_prevError = 0.0;
  m_integral = 0.0;
  m_derivative = 0.0;
  m_output = 0.0;
  m_timeInTolerance = 0;
}

void PID::resetIntegral() {
  m_integral = 0.0;
}

double PID::step(double measurement, double dtSeconds) {
  m_error = m_target - measurement;
  m_integral += m_error * dtSeconds;
  if (std::fabs(m_integral) > m_integralLimit) {
    m_integral = std::copysign(m_integralLimit, m_integral);
  }

  if (dtSeconds > 1e-6) {
    m_derivative = (m_error - m_prevError) / dtSeconds;
  } else {
    m_derivative = 0.0;
  }

  m_output = m_kp * m_error + m_ki * m_integral + m_kd * m_derivative;
  m_output = clamp(m_output, m_minOutput, m_maxOutput);

  if (std::fabs(m_error) < m_errorTolerance &&
      std::fabs(m_derivative) < m_derivativeTolerance) {
    m_timeInTolerance += static_cast<int>(dtSeconds * 1000.0);
  } else {
    m_timeInTolerance = 0;
  }

  m_prevError = m_error;
  return m_output;
}

bool PID::isSettled() const {
  return m_timeInTolerance >= m_settleTimeMsec;
}

