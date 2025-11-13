#include "utils.h"
#include <algorithm>
#include <cmath>

double degToRad(double deg) {
  return deg * M_PI / 180.0;
}

double radToDeg(double rad) {
  return rad * 180.0 / M_PI;
}

double wrapAngleDeg(double angle) {
  while (angle <= -180.0) {
    angle += 360.0;
  }
  while (angle > 180.0) {
    angle -= 360.0;
  }
  return angle;
}

double clamp(double value, double minValue, double maxValue) {
  return std::min(std::max(value, minValue), maxValue);
}

double getRadius(double x, double y, double x1, double y1, double headingDeg) {
  const double dx = x1 - x;
  const double dy = y1 - y;
  const double headingRad = degToRad(headingDeg);

  const double forwardComponent = std::cos(headingRad) * dy + std::sin(headingRad) * dx;
  const double lateralComponent = -std::sin(headingRad) * dy + std::cos(headingRad) * dx;

  if (std::fabs(lateralComponent) < 1e-6) {
    return std::copysign(1e6, forwardComponent); // effectively straight
  }

  return forwardComponent * forwardComponent / (2.0 * lateralComponent);
}

