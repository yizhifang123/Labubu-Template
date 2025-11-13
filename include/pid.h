#pragma once

#include <cstdint>

/**
 * Simple PID controller with a few quality-of-life helpers inspired by LemLib
 * and EZ-Template.  The interface is intentionally compact so new students can
 * reason about the math while still supporting advanced exit conditions.
 */
class PID {
 public:
  PID(double kp = 0.0, double ki = 0.0, double kd = 0.0);

  void setGains(double kp, double ki, double kd);
  void setTarget(double target);
  void setOutputLimits(double minOutput, double maxOutput);
  void setIntegralLimit(double integralLimit);
  void setTolerance(double errorTolerance, double derivativeTolerance, int settleTimeMsec);

  void reset();
  void resetIntegral();

  double step(double measurement, double dtSeconds);

  bool isSettled() const;
  double error() const { return m_error; }
  double target() const { return m_target; }
  double output() const { return m_output; }

 private:
  double m_kp;
  double m_ki;
  double m_kd;

  double m_target;
  double m_error;
  double m_prevError;
  double m_integral;
  double m_derivative;
  double m_output;

  double m_minOutput;
  double m_maxOutput;
  double m_integralLimit;

  double m_errorTolerance;
  double m_derivativeTolerance;
  int m_settleTimeMsec;
  int m_timeInTolerance;
};

