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

## Driver Control

`runDriver()` in `src/main.cpp` demonstrates an arcade drive example that scales joystick values to volts. You can swap this for tank, split arcade, or custom controls using the same `driveChassis()` helper.

## Adding Subsystems (Intake, Pneumatics, etc.)

1. **Declare devices in `include/robot-config.h`:** add `extern` statements for each motor, pneumatic, or sensor you want to share across files.
2. **Define them in `src/robot-config.cpp`:** construct the matching `motor`, `motor_group`, or `pneumatic` objects with the correct ports/gearings, then run any one-time setup inside `configureDevices()`.
3. **Write helpers (optional):** create small functions (e.g. `intakeSpin`, `setClamp`) so autonomous and driver-control code stays readable.
4. **Use them in `runAutonomous()` / `runDriver()`:** call the helpers or spin the motors directly based on joystick buttons, timers, or sequencing needs.

Declaring the hardware once in `robot-config.{h,cpp}` keeps the project synchronized—every subsystem can be accessed anywhere without risk of double-instantiating devices.

### Example: Intake Motors

**`include/robot-config.h`:**

```15:20:include/robot-config.h
extern vex::motor intakeMotorLeft;
extern vex::motor intakeMotorRight;
extern vex::motor_group intake;
```

**`src/robot-config.cpp`:**

```20:30:src/robot-config.cpp
motor intakeMotorLeft = motor(PORT12, vex::gearSetting::ratio6_1, false);
motor intakeMotorRight = motor(PORT13, vex::gearSetting::ratio6_1, true);
motor_group intake(intakeMotorLeft, intakeMotorRight);
```

**Helper (`src/subsystems.cpp` for example):**

```cpp
void intakeSpin(double voltage) {
  intake.spin(vex::fwd, voltage, vex::voltageUnits::volt);
}

void intakeStop(vex::brakeType mode = vex::brakeType::coast) {
  intake.stop(mode);
}
```

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

