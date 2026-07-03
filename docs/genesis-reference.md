# 78181A Genesis Reference — "Push Back" Repository

**Repo:** https://github.com/NicksonC1/78181A-Push-Back
**Team:** 78181A Genesis
**Framework:** PROS (VEX V5)
**Key Insight:** The "Genesis" library is a customized fork of **LemLib**, a well-known open-source VEX PROS library.

---

## What This Project Is

A competition-ready VEX PROS codebase with two layers:
1. **Genesis Library** (`include/genesis/`, `src/genesis/`) — reusable robot control library (LemLib fork)
2. **Robot Application** (`src/main.cpp`, `src/config.cpp`, etc.) — team-specific hardware, auton routes, driver control

This is the same two-layer pattern WinLib uses, but Genesis relies heavily on OOP (classes, inheritance, polymorphism) which makes it harder for beginners to follow.

---

## Project Structure

```
78181A-Push-Back/
  include/
    genesis/
      api.hpp                   Master include for the library
      asset.hpp                 Macro for embedding path files
      chassis/
        chassis.hpp             Core Chassis class (the big one)
        odom.hpp                Odometry free functions
        trackingWheel.hpp       TrackingWheel class
      driveCurve.hpp            Abstract DriveCurve + ExpoDriveCurve
      exitcondition.hpp         ExitCondition class
      pid.hpp                   PID controller class
      pose.hpp                  Pose class (x, y, theta)
      timer.hpp                 Timer utility
      util.hpp                  Math utilities
      logger/                   Logging system
    config.h                    Hardware declarations (extern)

  src/
    genesis/
      chassis/
        chassis.cpp             Chassis constructor, calibrate, setPose
        odom.cpp                Odometry background task
        opcontrol.cpp           tank(), arcade(), curvature() drive modes
        trackingWheel.cpp       Sensor distance reading
        motions/
          moveToPoint.cpp       Drive to (x, y)
          moveToPose.cpp        Drive to (x, y, theta) with curved approach
          turnToHeading.cpp     Turn to absolute heading
          turnToPoint.cpp       Turn to face a point
          swingToHeading.cpp    Swing turn (one side locked)
          swingToPoint.cpp      Swing turn to face a point
          pursuit.cpp           Pure Pursuit path following
      pid.cpp                   PID implementation
      pose.cpp                  Pose math
      util.cpp                  Utility functions
      driveCurve.cpp            Exponential drive curve
      exitcondition.cpp         Exit condition logic

    config.cpp                  Hardware definitions (motor ports, sensors)
    main.cpp                    Auton routes, chassis config, utilities
    opcontrol.cpp               PROS entry point functions
    brainScreenLVGL.cpp         Brain screen UI

  static/                       Pure Pursuit path files (.txt)
```

---

## What Each File Contains

### Library Core

| File | What It Does |
|------|-------------|
| **chassis.hpp** | The `Chassis` class — holds drivetrain motors, PID controllers, exit conditions, odom sensors. Has methods for every motion type. This is the "god object" that WinLib avoids. |
| **odom.hpp/cpp** | Free functions (not a class!) for odometry: `init()`, `update()`, `getPose()`, `setPose()`. Runs a background task every 10ms. Uses arc-based integration. |
| **trackingWheel.hpp/cpp** | `TrackingWheel` class — wraps different sensor types (ADI encoder, rotation sensor, motor group) behind one interface. Reads distance traveled. |
| **pid.hpp/cpp** | `PID` class — standard P+I+D with anti-windup (resets integral on sign change or when error > windupRange). |
| **pose.hpp/cpp** | `Pose` class — (x, y, theta) with operator overloads (+, -, *, /), distance(), angle(), lerp(), rotate(). Nearly identical to WinLib's Pose. |
| **exitcondition.hpp/cpp** | `ExitCondition` class — triggers when input stays within a range for N milliseconds. Same concept as WinLib's. |
| **timer.hpp/cpp** | `Timer` class — millisecond countdown. Same concept as WinLib's. |
| **util.hpp/cpp** | Free functions: `angleError()`, `slew()`, `getCurvature()`, `ema()`, `sgn()`. |
| **driveCurve.hpp/cpp** | Abstract `DriveCurve` base class with virtual `curve()` method. `ExpoDriveCurve` implements an exponential joystick curve for smoother driver control. (OOP inheritance — WinLib avoids this) |
| **asset.hpp** | Macro that embeds text files (Pure Pursuit paths) into the binary at compile time. |

### Motion Functions (the important ones for understanding)

#### `moveToPoint(x, y, timeout, params)`
- **What:** Drives robot to a target (x, y) coordinate
- **How:** Runs a loop every 10ms:
  1. Gets current pose from odometry
  2. Calculates distance to target (lateral error) and heading error (angular error)
  3. Feeds errors into lateral PID and angular PID
  4. Combines: `leftMotor = lateralPower + angularPower`, `rightMotor = lateralPower - angularPower`
  5. Exits when ExitCondition triggers or timeout expires
- **WinLib equivalent:** This is what our `moveTo()` will do, simplified

#### `moveToPose(x, y, theta, timeout, params)` — "Boomerang Controller"
- **What:** Drives to (x, y) AND arrives facing heading theta
- **How:** Uses a "carrot point" — an intermediate target that creates a curved path so the robot approaches from the correct angle. Also limits speed based on path curvature to prevent wheel slip.
- **WinLib status:** IMPLEMENTED (added for the 2026–27 Override season) as `Chassis::boomerang` in `chassis/movement/boomerang.cpp` — a minimal teaching version: it keeps the carrot + collapse-on-close + final-heading lock, but drops the curvature-based slip-speed limit and the far-away `sgn(cos)` speed trick, and uses constant kP. See `CHANGES_FROM_REFERENCE.md` → *Movement / Motions*.

#### `turnToHeading(theta, timeout, params)`
- **What:** Rotates robot in place to face a specific heading
- **How:** Angular PID only. Left motors get +power, right motors get -power.
- **WinLib equivalent:** This is what our `turnTo()` will do

#### `turnToPoint(x, y, timeout, params)`
- **What:** Rotates robot to face toward a specific (x, y) coordinate
- **How:** Calculates target heading as `atan2(dy, dx)`, then does same thing as turnToHeading
- **WinLib equivalent:** This is what our `turnToPoint()` will do

#### `swingToHeading / swingToPoint`
- **What:** Turns the robot but locks one side of the drivetrain (like swinging a door on a hinge)
- **WinLib status:** Not planned initially, but could be added later as a teaching extension

#### `follow(path, lookahead, timeout)` — Pure Pursuit
- **What:** Follows a pre-defined curved path by chasing a "lookahead point" ahead on the path
- **How:** Reads waypoints from a text file, uses circle-line intersection math to find the target, calculates curvature to convert into differential wheel speeds
- **WinLib status:** NOT implementing (too complex)

### Motion System Design Patterns (Genesis uses, WinLib avoids)

| Pattern | Genesis | WinLib |
|---------|---------|--------|
| **Chassis class** | Central object that owns everything | No chassis class; free functions |
| **Motion queue + mutex** | Motions can be queued and run async | Motions are blocking (synchronous) |
| **Parameter structs** | `{.forwards = false, .maxSpeed = 60}` | Simple function parameters |
| **DriveCurve inheritance** | Abstract base class + subclasses | Not needed |
| **Async tasks** | Each motion spawns a `pros::Task` | No tasks; runs in main thread |

---

## Odometry — How It Works

The odometry system tracks the robot's (x, y, theta) position on the field.

### Sensors Used
- **Tracking wheels:** Unpowered wheels with rotation sensors that measure distance traveled
- **IMU (Inertial Measurement Unit):** Gyroscope that measures heading (rotation angle)
- Genesis uses: 1 vertical tracking wheel + 1 IMU (port 21 with a calibration scalar of 1.01152)

### The Algorithm (runs every 10ms in a background task)
1. Read new distances from tracking wheels and new heading from IMU
2. Calculate how much the robot moved since last tick (`deltaDistance`) and how much it rotated (`deltaHeading`)
3. If the robot turned: use arc math to figure out the actual displacement (the robot followed a curve, not a straight line)
4. If the robot went straight: displacement = deltaDistance
5. Rotate the local displacement into global coordinates using the robot's heading
6. Add to global (x, y, theta)

### Key Detail: Arc-Based Integration
When a robot turns while driving, it follows a circular arc, not a straight line. The math accounts for this:
- `localX = 2 * sin(deltaHeading/2) * (deltaX/deltaHeading + horizontalOffset)`
- `localY = 2 * sin(deltaHeading/2) * (deltaY/deltaHeading + verticalOffset)`
- Then rotate by `(heading - deltaHeading/2)` to get global displacement

---

## Application Layer (Team-Specific Code)

### Hardware (config.cpp)
- 6 drive motors (3L, 3R), blue cartridge (600 RPM)
- 2 intake motors
- 3 distance sensors, 2 optical sensors
- Pneumatic pistons: loader, hook, state1, state2, middle
- Custom IMU class that multiplies readings by a calibration scalar

### Notable Utility Functions (in main.cpp)
- `cdrift(leftVolt, rightVolt, timeout)` — raw motor power for timed movements (wall pushes, alignment)
- `linear(distance, timeout)` — drive a relative distance from current pose
- `resetWalls()` — uses distance sensors to triangulate position relative to field walls (corrects odometry drift)
- `chain(waypoints)` — iterates through (x,y) pairs doing turnToPoint + moveToPoint for each
- Color sorting — optical sensor detects ring color, rejects wrong color via piston
- Anti-jam — monitors intake motor velocity, briefly reverses if stalled

### Autonomous Route Example (from their code)
```cpp
chassis.setPose(-47.5, -58, 180);              // set starting position
chassis.moveToPoint(-47.5, -26, 3000);         // drive forward
chassis.turnToHeading(120, 800);               // turn
chassis.moveToPoint(-25, -46, 2000);           // drive to next position
chassis.moveToPoint(-7, -46, 3000,             // motion chain (doesn't fully stop)
    {.minSpeed = 60});
```

---

## Key Takeaways for WinLib

1. **PID values reference:** Genesis uses lateral `kP=7.5, kD=6` and angular `kP=2.75, kD=17.5` (no integral term). Good starting points for tuning.
2. **Odometry is free functions in Genesis too** — `odom.hpp` uses the same pattern WinLib plans to use.
3. **The Chassis class is the main complexity** — by removing it, WinLib becomes dramatically simpler.
4. **Motion chaining** (minSpeed parameter) is a nice feature but adds complexity. WinLib starts without it.
5. **Distance sensor wall resets** are a competition-level feature worth considering later in the season.
