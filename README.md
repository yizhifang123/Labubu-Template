# Labubu Template

This project is a beginner-friendly VEX V5 competition template inspired by RW-Template, LemLib, and EZ-Template. 

## Project Layout

- `include/robot-config.h` – Hardware declarations, chassis geometry, PID gains, and helper structs.
- `src/robot-config.cpp` – Port assignments, sensor calibration, and default configuration flags.
- `include/motor-control.h` – Motion API that mirrors RW-Template (`driveTo`, `turnToAngle`, `boomerang`, etc.).
- `src/motor-control.cpp` – Core motion logic: PID, slew limiting, odometry threads, and high-level routines.
- `include/pid.h` / `src/pid.cpp` – Lightweight PID controller with settle detection and clamp helpers.
- `include/utils.h` / `src/utils.cpp` – Math helpers (angle wrapping, unit conversions, curve radius).
- `src/main.cpp` – Competition entry point with sample `pre_auton`, `autonomous`, and driver control loops.

## Quick Start

1. **Update Ports:** Open `src/robot-config.cpp` and adjust the `motor` and sensor constructors to match your robot.
2. **Tune Dimensions:** Set wheel diameter, track width, and optional tracker diameters in `include/robot-config.h`.
3. **Calibrate Sensors:** The `configureDevices()` function runs in `pre_auton()`; make sure the robot is stationary during startup.
4. **Optional Odometry:** Enable `trackingOptions.useHorizontalTracker` and/or `useVerticalTracker` in `configureDevices()` after wiring rotation sensors.
5. **Build & Download:** Use VEXcode VS Code extension or standard VEX CLI makefile to compile and deploy.

## Key Functions

All motion helpers live in `include/motor-control.h` and use inches/degrees for inputs unless noted.

| Function | Purpose |
| --- | --- |
| `driveChassis(leftVolt, rightVolt)` | Direct voltage control for tank drive |
| `turnToAngle(angle, time_ms, exit, maxVolt)` | PID turn-to-heading with settle detection |
| `driveTo(distance_in, time_ms, exit, maxVolt)` | Straight-line movement with heading hold |
| `curveCircle(targetHeading, radius_in, time_ms, exit, maxVolt)` | Smooth arc driving using inner/outer PID |
| `swing(angle, direction, time_ms, exit, maxVolt)` | One-side hold swing turns |
| `turnToPoint(x, y, dir, time_ms)` | Face a field coordinate |
| `moveToPoint(x, y, dir, time_ms, exit, maxVolt, overturn)` | Pathfollow-lite point drive with heading correction |
| `boomerang(x, y, dir, finalAngle, lead, time_ms, exit, maxVolt, overturn)` | Curved “carrot chase” motion for dynamic approach |
| `setPose({x, y, heading})` / `getPose()` | Reset or read odometry pose state |

Each routine accepts optional chaining parameters (`exit`, `overturn`) so you can link multiple moves. The internal PID class supports output limits, integral caps, and automatic settle timers to keep tuning approachable.

## Autonomous Workflow

1. Call `setPose()` after placing the robot on the field so the odometry threads start at the correct coordinate.
2. Use combinations of `driveTo`, `turnToPoint`, `moveToPoint`, or `boomerang` inside `runAutonomous()` to script routines.
3. For complex paths, chain motions by setting `exit = false` on intermediate commands, then finish with `exit = true`.

## Best Practices for Accurate Autonomous Movements

To achieve the most precise and reliable autonomous routines, follow these guidelines when using motion functions:

### 1. Initial Pose Setup

**Always set the starting pose immediately after calibration:**
```cpp
void runPreAutonomous() {
  configureDevices();  // Calibrates IMU
  initializeMotion();
  setPose(0.0, 0.0, getInertialHeading());  // Match robot's actual field position
}
```

**Key Points:**
- Set pose to match the robot's physical placement on the field (field coordinates, not robot-relative)
- The heading should match the IMU reading after calibration
- Update pose values when changing starting positions between matches

### 2. Function Selection for Different Scenarios

**Use `driveTo()` for:**
- Straight-line movements in robot-relative coordinates
- Simple forward/backward drives
- When you know the exact distance to travel

**Use `turnToAngle()` for:**
- Simple point turns to a specific heading
- Rotations when you know the exact angle (e.g., 90°, 180°)

**Use `turnToPoint()` for:**
- Facing a specific field coordinate
- When you know where you want to look, not the exact angle

**Use `moveToPoint()` for:**
- Navigating to coordinates when you need to maintain a specific heading direction
- Point-to-point movement with heading correction
- When you want the robot to arrive at a point facing a particular direction (`dir` parameter: +1 forward, -1 backward)

**Use `boomerang()` for:**
- Smooth curved approaches to a target
- When you want to approach a point from an angle and end facing a specific direction
- More dynamic paths that adapt during movement
- Useful for picking up objects or approaching game elements smoothly

**Use `curveCircle()` for:**
- Smooth arc movements without precise coordinate targets
- Moving in a curve to a specific heading

**Use `swing()` for:**
- Efficient turns while moving (one wheel holds, one turns)
- Quick heading adjustments without stopping

### 3. Motion Chaining with `exit` Parameter

**Best Practice: Use `exit = false` for intermediate moves, `exit = true` for final moves**

```cpp
// Good: Chain multiple movements smoothly
driveTo(24.0, 3000, false);      // Don't stop, continue immediately
turnToAngle(90.0, 2000, false);  // Still chaining
driveTo(12.0, 2000, true);       // Final move, stop and settle

// Bad: Each move stops and starts
driveTo(24.0, 3000, true);   // Unnecessary stop
turnToAngle(90.0, 2000, true); // Another stop
driveTo(12.0, 2000, true);   // Yet another stop
```

**Why this matters:**
- `exit = false`: Motion completes when settled or timeout, but allows smooth transition
- `exit = true`: Motion stops with brakes applied, ensuring precise positioning
- Chaining reduces accumulated timing errors and creates smoother paths

### 4. Time Limits: Safety vs. Precision

**Always set reasonable time limits as a safety timeout:**
```cpp
driveTo(24.0, 3000);  // 3000ms = 3 second timeout
```

**Guidelines:**
- Estimate time needed: `distance / speed + buffer`
- Too short: Robot times out before completing movement
- Too long: If robot gets stuck, wastes valuable autonomous time
- Good default: Estimate + 50% buffer (e.g., 2 second move → 3000ms timeout)

### 5. Heading Correction and Accuracy

**Enable heading hold for straight-line accuracy:**
```cpp
void runAutonomous() {
  headingCorrectionEnabled = trackingOptions.enableHeadingHold;  // Enable for straight drives
  
  driveTo(48.0, 4000);  // Heading correction keeps robot straight
}
```

**How it works:**
- Background PID continuously corrects heading drift during movements
- Uses IMU readings to maintain `correct_angle` (set after each motion completes)
- Automatically mixes correction into `driveTo()` and other movements
- Disabled during active turns to prevent conflicts

**When to disable:**
- If correction fights manual tuning
- For specific maneuvers where heading drift is intentional
- During complex sequences where correction causes issues

### 6. Tuning for Accuracy

**Start with conservative PID gains and tune systematically:**

1. **Test `driveTo()` first:**
   ```cpp
   driveTo(24.0, 3000);  // Simple test
   ```
   - Tune `motionGains.driveKp`, `driveKi`, `driveKd` in `robot-config.h`
   - Robot should reach target without overshoot
   - Should settle smoothly without oscillation

2. **Test `turnToAngle()` next:**
   ```cpp
   turnToAngle(90.0, 2000);  // Quarter turn test
   ```
   - Tune `motionGains.turnKp`, `turnKi`, `turnKd`
   - Should reach angle accurately and settle quickly

3. **Verify heading correction:**
   - Drive long distances (48+ inches) in straight line
   - Robot should maintain heading without drift
   - Tune `motionGains.headingHoldKp`, `headingHoldKi`, `headingHoldKd` if needed

### 7. Using Odometry vs. Encoders

**This template supports both approaches:**

**Encoder-based (default, no tracking wheels):**
- Uses IMU + drive encoders
- Good for simple movements and robots without tracking wheels
- Less accurate for complex paths requiring precise positioning

**Odometry with tracking wheels (recommended for competition):**
- Enable in `configureDevices()`:
  ```cpp
  trackingOptions.useHorizontalTracker = true;  // For X position
  trackingOptions.useVerticalTracker = true;    // For Y position
  ```
- Provides accurate field coordinate tracking
- Essential for `moveToPoint()` and `boomerang()` precision
- Requires accurate measurement of tracker diameters and offsets

### 8. Common Patterns for Reliable Routines

**Pattern 1: Simple Move Sequence**
```cpp
void runAutonomous() {
  driveTo(24.0, 3000);
  turnToAngle(90.0, 2000);
  driveTo(12.0, 2000);
}
```

**Pattern 2: Coordinate-Based Navigation**
```cpp
void runAutonomous() {
  setPose(0.0, 0.0, 0.0);  // Starting position
  
  moveToPoint(36.0, 24.0, 1, 4000);  // Go to point facing forward
  turnToPoint(48.0, 12.0, 1, 2000);  // Face next target
  boomerang(48.0, 12.0, 1, 0.0, 0.4, 3000);  // Smooth approach
}
```

**Pattern 3: Complex Chained Movement**
```cpp
void runAutonomous() {
  // Pick up object
  driveTo(18.0, 2000, false);  // Approach
  // ... activate intake ...
  driveTo(6.0, 1500, true);    // Final approach, stop
  
  // Turn and deliver
  turnToAngle(180.0, 2000, false);
  driveTo(-30.0, 3000, true);  // Drive backward
}
```

### 9. Debugging Tips

**If movements are inaccurate:**

1. **Check physical dimensions in `robot-config.h`:**
   - Wheel diameter must match actual wheels
   - Track width must be accurately measured
   - Tracker diameters/offsets if using odometry

2. **Verify sensor calibration:**
   - IMU must be stationary during `configureDevices()`
   - Ensure IMU is mounted securely (no vibration)

3. **Test individual movements:**
   - Run one function at a time to isolate issues
   - Check if `driveTo()` is accurate before testing `moveToPoint()`

4. **Monitor pose updates:**
   - Use `getPose()` between movements to verify odometry
   - Print values to Brain screen or console

5. **Adjust PID gains incrementally:**
   - Change one gain at a time
   - Test after each change
   - Document successful values

### 10. Competition Day Checklist

- [ ] Robot dimensions (wheel diameter, track width) are accurate
- [ ] IMU calibration completes successfully every run
- [ ] Starting pose matches actual robot placement
- [ ] Time limits are appropriate (not too tight, not wasteful)
- [ ] Motion chaining (`exit` parameters) is correct for your routine
- [ ] Heading correction is enabled for straight drives
- [ ] Odometry sensors (if used) are properly configured and calibrated

Following these practices will help you achieve consistent, accurate autonomous routines that perform reliably under competition conditions.

## Driver Control

`runDriver()` in `src/main.cpp` demonstrates an arcade drive example that scales joystick values to volts. You can swap this for tank, split arcade, or custom controls using the same `driveChassis()` helper.





**Driver control usage (`src/main.cpp`):**

```44:59:src/main.cpp
if (Controller1.ButtonR1.pressing()) {
  intakeSpin(12.0);
} else if (Controller1.ButtonR2.pressing()) {
  intakeSpin(-12.0);
} else {
  intakeStop();
}
```

### Example: Pneumatic Clamp

**`include/robot-config.h`:**

```20:24:include/robot-config.h
extern vex::pneumatic clamp;
```

**`src/robot-config.cpp`:**

```30:35:src/robot-config.cpp
pneumatic clamp = pneumatic(PORT8); // smart port with V5 pneumatics module
```

**Helper:**

```cpp
void setClamp(bool extended) {
  clamp.set(extended);
}
```

**Driver control usage:**

```44:59:src/main.cpp
if (Controller1.ButtonL1.pressing()) {
  setClamp(true);
} else if (Controller1.ButtonL2.pressing()) {
  setClamp(false);
}
```

Refer to the VEX API documentation for additional device constructors and options (e.g. smartdrive, drivetrain, pneumatics, servos).^[ [VEX drivetrain API](https://api.vex.com/v5/home/cpp/Drivetrain.html) ][ [VEX smartdrive API](https://api.vex.com/v5/home/cpp/SmartDrive.html) ][ [VEX pneumatics API](https://api.vex.com/v5/home/cpp/Pneumatics.html) ]

## Troubleshooting & Tips

- If heading hold fights manual driving, disable it by setting `trackingOptions.enableHeadingHold = false`.
- Always re-measure physical dimensions after changing wheels or gearing; accurate geometry is critical for odometry.
- Start with low PID gains and increase slowly. Use the built-in `headingCorrectionEnabled` flag to toggle background heading hold during tuning.
- `README.md` will evolve as your team adds mechanisms—document new subsystems and share tuning values with the rest of the team.

