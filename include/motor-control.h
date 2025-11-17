#pragma once

#include "robot-config.h"
#include <atomic>

/**
 * This header mirrors the RW-Template API so existing autonomous code can be
 * dropped in, while the implementation underneath borrows structure ideas
 * from LemLib and EZ-Template.  Every function is fully documented to keep
 * things beginner friendly.
 */

extern std::atomic<bool> is_turning;
extern double correct_angle;
extern double xpos;
extern double ypos;
extern double heading;

void initializeMotion();

void driveChassis(double left_power, double right_power);

double getInertialHeading();
double normalizeTarget(double angle);
void maintainHeadingTarget(double target);

void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);

void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);

void getPose(double& x, double& y, double& heading);
void setPose(double x, double y, double heading);

