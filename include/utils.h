#pragma once

#include <cmath>

/**
 * Lightweight math helpers used across the motion code.  These functions are
 * intentionally simple so new programmers can read the implementation and
 * understand the geometry behind the robot's movement.
 */

constexpr double inchesToMeters(double inches) {
  return inches * 0.0254;
}

constexpr double metersToInches(double meters) {
  return meters / 0.0254;
}

double degToRad(double deg);
double radToDeg(double rad);

/**
 * Returns the signed radius of curvature for a point relative to the robot.
 * A positive radius indicates a right-hand turn, negative for left-hand.
 */
double getRadius(double x, double y, double x1, double y1, double headingDeg);

/**
 * Wraps an angle into the range (-180, 180].
 */
double wrapAngleDeg(double angle);

/**
 * Clamp helper mirroring std::clamp but with shorter syntax for beginners.
 */
double clamp(double value, double minValue, double maxValue);

