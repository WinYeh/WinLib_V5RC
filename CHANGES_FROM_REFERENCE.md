# Changes from Reference Libraries

This document tracks how **WinLib** intentionally diverges from its reference implementations. It exists so that anyone reading WinLib alongside Genesis or LemLib (including future-Winyeh, the 2026 team, and any agent helping with this code) can see the *why* behind every difference at a glance.

## References

- **Genesis** (78181A's fork of LemLib, the immediate inspiration): https://github.com/NicksonC1/78181A-Push-Back
- **LemLib** (upstream): https://github.com/LemLib/LemLib
- **Pilons 5225A odometry paper** (the math behind `Odom.cpp`): http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

WinLib's north star is teachability for a high-school team in their first PROS season. Most divergences below trade competitive features (or generic flexibility) for code a 10th-grader can read top-to-bottom.

> **Note for the agent:** every time you change WinLib in a way that makes it diverge further from Genesis/LemLib — adding a file, removing a class member, changing an algorithm, simplifying a fallback path — append a new entry to this document. See `CLAUDE.md` § *Tracking Divergences* for when and how.

---

## Architecture

### Split ownership — `Chassis` owns sensors + the public API; `odom.cpp` owns the running math
- **Reference:** LemLib/Genesis put **everything** inside the `Chassis` class — sensors, the tracking task, the pose state, and `getPose()` all live as Chassis members.
- **WinLib:** Movement, opcontrol, chassis-level utilities, AND the `OdomSensors` configuration all live on the `Chassis` class. But the **odometry running state** (`odomPose`, `odomSpeed`, `prevVertical`, etc.) and the tracking task itself stay in `odom.cpp` as module-level statics. The public odom API is namespace-level free functions (`WinLib::getPose`, `setPose`, `update`, `init`). `odom.cpp` reads the sensors by `#include "config.h"` and reaching through the global `chassis.odomSensors` — there's only one copy of the OdomSensors in the whole program.
- **Why:** Two competing goals. (1) Odom math is self-contained and reads cleanly on its own — folding it into Chassis would mean every odom edit also loads the Chassis mental model. So the math + running state stay in `odom.cpp`. (2) Sensor configuration is robot-specific and lives naturally with the rest of the chassis config — making it a Chassis member means `chassis.calibrate()` can handle IMU calibration and wheel reset in one call, and the user has one obvious place to read or change it. The compromise: `odom.cpp` depends on the application's `config.h` (mild layering violation), but in exchange there's zero data duplication and zero sensor-injection boilerplate.

### Blocking motions only — no motion queue, no async
- **Reference:** LemLib enqueues motions; they run on a background `pros::Task`.
- **WinLib:** Movement functions block until the motion finishes. Subsystem-parallel work (intake, lift) is done with plain `pros::Task` at the user's discretion.
- **Why:** Students can step through an auton routine in their head without modeling a scheduler.

### No command-based programming system
- **Reference:** LemLib has been experimenting with command-based abstractions.
- **WinLib:** Deferred until writing real routes surfaces a concrete need.

### Opcontrol reduced to a single arcade method
- **Reference:** Genesis/LemLib offers three opcontrol drive helpers — `tank(left, right, disableDriveCurve)`, `arcade(throttle, turn, disableDriveCurve, desaturateBias)`, and `curvature(throttle, turn, disableDriveCurve)`. Each one runs the joystick input through a configurable `DriveCurve` (deadband + exponential curve) before sending to the motors.
- **WinLib:** Only `Chassis::arcade(int throttle, int turn)`. Classic arcade mixing (`left = throttle + turn`, `right = throttle - turn`) with **raw** joystick input — no deadband, no drive curve, no scaling. The conversion to motor voltage is one linear line per side (`mV = joystick * 12000 / 127`), and `pros::Motor::move_voltage` clips overflow automatically. `tank` and `curvature` are not included.
- **Why:** Teaching focus. New PROS drivers can read the one method top-to-bottom and understand exactly what the joystick is doing to the wheels. Tank drive is the same arcade math with different input wiring (the driver can replicate it in their own opcontrol loop if they want); curvature drive is a niche control scheme that introduces lookahead complexity for marginal driver-experience gain. Drive curves return later in their own file (see *Pending / Deferred*).

### Motion set is intentionally limited
- **Reference:** LemLib includes pure pursuit, arc/curvature primitives, ramsete-style controllers, etc.
- **WinLib:** Only **lateral**, **angular** (turn-to-heading / turn-to-point), and **boomerang**.
- **Why:** Path generation and lookahead tuning are too much teaching burden for the value at this level. Boomerang earns its keep for the 2026–27 Override season.

### Config simplified — POD struct for `Drivetrain`, hybrid constructor for `ControllerSettings`
- **Reference:** LemLib's `Drivetrain`, `OdomSensors`, `ControllerSettings`, etc. are full classes with explicit constructors and 6–9 positional flat fields (you have to remember the order).
- **WinLib:** `Drivetrain` is a POD struct (data only, no methods). `ControllerSettings` keeps the same flat-field storage as Genesis but takes a `PID` + two `ExitCondition` objects in its constructor — the constructor pulls the numeric values out via getters and stores them as flat floats (see the *Controllers / PID* section). Small *utility* classes remain (`PID`, `Timer`, `ExitCondition`, `Pose`, `TrackingWheel`).
- **Why:** Different shapes for different jobs. `Drivetrain` is a bag of motors and numbers, so a POD struct fits. `ControllerSettings` needs flat fields for fast/direct access from movement code (`settings.kP`, `settings.smallError`), but constructing it with named primitive objects (`PID(10,0,3,3)`, `ExitCondition(1,100)`) is far more readable than a 9-float positional blob. Hybrid is the best of both worlds.

---

## Sensors

### `TrackingWheel` accepts only `pros::Rotation`
- **Reference:** LemLib's `TrackingWheel` has constructors for `pros::Rotation`, `pros::ADIEncoder`, and `pros::MotorGroup` (the last lets a powered drive wheel pretend to be a tracking wheel).
- **WinLib:** Single constructor — `pros::Rotation*` only.
- **Why:** ADI encoders aren't shipped with modern VEX kits. Motor-encoder "tracking wheels" slip, so they don't earn their complexity.

### `TrackingWheel` has no `getType()` method
- **Reference:** Returns an enum distinguishing real tracking wheels from motor-encoder substitutes.
- **WinLib:** Removed — only one kind of tracking wheel exists, so the distinction is meaningless.

### `TrackingWheel` distance math — divisor and units
- **Reference (and a common Genesis bug):** Various forks divide encoder ticks by `360` or `60`, which assumes the sensor reports degrees or RPM.
- **PROS reality:** `pros::Rotation::get_position()` and `get_velocity()` both report **centidegrees** / **centidegrees per second**. One full wheel rotation = 36,000.
- **WinLib:** Divisor is `36000`. Diameter is stored in inches; the function multiplies by `25.4` so the **output is in mm / mm/s**. Header comments and the function docs reflect this.

### New: `DSR` (Distance Sensor Reset) — absolute position fix from 4 distance sensors
- **Reference:** Genesis/LemLib has no equivalent. Their odometry is tracking-wheel + IMU only; once accumulated drift is in the pose, there's no built-in way to scrub it.
- **WinLib:** New class `WinLib::DSR` in `chassis/DSR.hpp` / `.cpp`. Wraps 4 `pros::Distance*` sensors (one on each side of the robot) plus the per-sensor mm offset from the robot's tracking center. `dsr.reset()` uses the IMU heading + ray–wall intersection math to figure out which wall each sensor is hitting (works at **any** heading, not just cardinal), then picks the closest valid reading per axis and overwrites the odom (x, y) via `WinLib::setPose()`. Heading is left untouched (the IMU is the source of truth for heading). Held by `Chassis` as a public member; `Chassis::resetLocalPosition()` is now a thin delegate to `dsr.reset()`.
- **Why:** Odometry drifts. After 60+ seconds of an auton route, the (x, y) error can be measured in inches. Distance sensors give an absolute fix against the field walls — DSR is the team's "scrub the drift" button. The any-heading design matters because real auton routes don't pause at perfect cardinal angles for the sake of sensor calibration.

### `OdomSensors` holds one wheel per axis, not a pair
- **Reference:** LemLib stores `vertical1`, `vertical2`, `horizontal1`, `horizontal2` so the Pilons differential heading formula has two parallel wheels to subtract.
- **WinLib:** Single `vertical`, single `horizontal`, single `imu`.
- **Why:** Modern teaching teams don't run two-parallel-wheels-per-axis setups, and the IMU is a more reliable heading source for our use case.

---

## Odometry (`Odom.cpp`)

### Heading-source priority reduced to 2
- **Reference (Pilons / LemLib):** Horizontal wheels → Vertical wheels → IMU → Drivetrain encoders.
- **WinLib:** Horizontal wheel → IMU. (Vertical-wheel and drivetrain-encoder branches deleted.)
- **Why:** Without two parallel wheels per axis, the differential heading formula doesn't work — translation contaminates the reading from a single vertical wheel and gives nonsense heading. Drivetrain motor encoders slip too much to be a credible odometry source.

### `update()` consolidated to a single sensor-read pass
- **Reference:** Two separate read blocks — one to compute heading deltas, one to pick the "best" wheel for position tracking.
- **WinLib:** One read at the top of `update()`. The "choose best wheel" block is gone (only one wheel per axis to choose from).

### **Bug fixed during this consolidation**
The inherited two-block layout updated `prevVertical` / `prevHorizontal` *between* the two reads. That was fine for Genesis's two-wheel-per-axis design because the second read targeted a *different* wheel. Once the code was simplified to a single wheel per axis, both reads hit the same sensor, and `deltaY` / `deltaX` in the position section always evaluated to **zero** — the robot's position never updated from forward driving. The single-read flow eliminates this.

### `getLocalSpeed()` and `estimatePose()` removed
- **Reference (Genesis):** Declared but never implemented.
- **WinLib:** Dropped from the public API. `odomLocalSpeed` is still computed internally so a future Kalman-filter pass can use it.

### Drivetrain instance in the odom module
- **Reference (Genesis):** Module-level `Drivetrain drive` so the motor-encoder fallback path can access wheel diameter / gear ratio.
- **WinLib:** Deleted. The Drivetrain belongs to movement code, not odometry.

---

## Controllers / PID

### `windupRange` is actually wired through
- **Reference:** Genesis declares `windupRange` on `ControllerSettings` and passes it down, but the `PID` class itself never reads it — the field is stored and ignored when computing output. Effectively dead config.
- **WinLib:** Wired through. The `PID` constructor takes a 4th param (`windupRange`, default `0`) and `PID::compute()` uses it as an anti-windup deadband — only accumulates the integral when `|error| < windupRange`. A `windupRange` of `0` disables anti-windup and preserves the original "always accumulate" behavior for callers that don't care.
- **Why:** No reason to carry a field if the code ignores it. Either implement the feature or drop the field — we picked implement, since anti-windup is a real PID tuning lever the team will eventually want.

### `slew` (max-acceleration rate-limit) removed
- **Reference:** Genesis's `ControllerSettings` has a `slew` field intended to cap how fast the controller output can change per tick.
- **WinLib:** Removed from `ControllerSettings`. `PID::compute()`'s output is used directly with no rate limit.
- **Why:** Slew limiting adds another tuning knob without a clear teaching payoff at this level. If a motion is too jerky, the right fix is usually tuning `kP` / `kD` or capping `maxSpeed`, not adding a slew limit. Revisit if a real motion actually needs it.

### `ControllerSettings` — readable construction, flat storage (hybrid design)
- **Reference:** Genesis's `ControllerSettings` is a class with 8 flat fields (`kP, kI, kD, windupRange, smallError, smallErrorTimeout, largeError, largeErrorTimeout` — slew is dropped in WinLib). Construction is positional, 8+ raw numbers with no field hints.
- **WinLib:** Same flat-field storage, but **the constructor takes objects, not floats** — a `PID` and two `ExitCondition` instances. The constructor reads each object's values via getters (`pid.getkP()`, `smallExit.getRange()`, etc.) and copies them into its own flat float fields. The `PID` and `ExitCondition` arguments are NOT stored; they're construction-time conveniences only.
  ```cpp
  WinLib::ControllerSettings lateralSettings(
      WinLib::PID(10, 0, 3, 3),          // kP, kI, kD, windupRange
      WinLib::ExitCondition(1, 100),     // small: 1 inch, 100 ms
      WinLib::ExitCondition(3, 500)      // large: 3 inches, 500 ms
  );
  // Internally stored as flat floats: kP=10, kI=0, kD=3, windupRange=3,
  //                                   smallError=1, smallErrorTimeout=100,
  //                                   largeError=3, largeErrorTimeout=500
  ```
- **Why:** Three wins. (1) **Readable construction** — `ExitCondition(1, 100)` makes the (range, timeout) pairing visible; the old flat form could silently compile with the two swapped. (2) **Flat storage for fast access** — movement code can read `settings.kP` or mutate `settings.smallError` directly, without going through getters or object indirection. (3) **One-line mid-route tuning** — `chassis.lateralSettings.smallError = 0.5;` works from any auton.
- **Required infrastructure:** This design requires `PID::getkP()`, `getkI()`, `getkD()`, `getWindupRange()` and `ExitCondition::getRange()`, `getTime()` — all added to those classes.

### Motor-power unit: volts (0–12 V), not PWM (0–127)
- **Reference:** Genesis/LemLib use the PROS PWM range — `pros::Motor::move(int)` taking -127..127. `maxSpeed = 127`, `minSpeed = 0`.
- **WinLib:** Movement-param motor-power fields (`maxSpeed`, `minSpeed`) are in **volts** with `12.0` as the hardware ceiling. Inside Chassis motion methods, drive the motors with `pros::Motor::move_voltage(mV)` (mV = volts × 1000). Opcontrol helpers (`tank`/`arcade`/`curvature`) still take joystick-shaped `int` args (-127..127) for ergonomic reasons and convert to volts internally.
- **Why:** PID gains tuned in PWM units don't transfer cleanly to voltage and vice versa. Picking one unit end-to-end (voltage) means tuning intuition stays consistent, and `kP * error → volts` reads correctly without a hidden 127-vs-12000 scaling factor.

### `ExitCondition::range` and `time` no longer `const` — runtime-tunable
- **Reference:** Genesis's `ExitCondition` declares its fields as `const float range; const int time;`, so the values are locked at construction.
- **WinLib:** `const` removed, plus explicit setters: `setRange(float)` and `setTime(int)`. Both also call `reset()` so the timer doesn't carry over with mismatched thresholds.
- **Why:** Auton routes sometimes need a different "good enough" definition at different points in the same match — e.g. tighten the small-error threshold for a final scoring approach, then loosen for the next cruise leg. With `const` fields, the user had to build a new `ExitCondition` and copy-assign; opening up the fields turns it into a one-liner like `chassis.lateralSettings.smallExit.setRange(0.5)`.

---

## Path Following / Asset System

> **Status:** Pure pursuit and other path-following motions are currently **forbidden** per `CLAUDE.md` § *Design Constraints* (lateral / angular / boomerang only). This section exists so that *if* that rule is ever relaxed, the LemLib asset machinery does **not** come along with it.

### No `asset` struct, no `ASSET(x)` / `ASSET_LIB(x)` macros
- **Reference:** Genesis/LemLib ships `include/genesis/asset.hpp` defining an `asset` struct and `ASSET(x)` / `ASSET_LIB(x)` macros. These declare `extern "C"` linker symbols (`_binary_static_<name>_start` / `_binary_static_<name>_size`) produced by `objcopy` rules in `firmware/hot-cold-asset.mk`. The purpose is to embed SD-card text files (path waypoints) into the firmware binary at compile time so `Chassis::follow(const asset& path, ...)` and the internal `getData(const asset&)` parser in `pursuit.cpp` can read them at runtime without filesystem I/O.
- **WinLib:** None of it. No `asset.hpp`, no `ASSET` / `ASSET_LIB` macros, no references to `_binary_static_*_start` / `_binary_static_*_size`, no `static/` directory, no `objcopy` rule in the Makefile, no `hot-cold-asset.mk`, no `#include "asset.hpp"` anywhere.
- **Why:** In the reference team's current season code, `chassis.follow(` is called **zero** times — every prior call lives in archived files, and the `ASSET(...)` declarations in their `main.cpp` are orphaned. Skipping this whole subsystem loses no feature the team actually uses; it only removes inherited infrastructure that has no caller. Linker-embedded binaries are also a teaching nightmare for 10th-graders.

### Path API (if/when path following is ever added) — use `std::vector<Pose>`, not assets
- **Reference signature:** `Chassis::follow(const asset& path, float lookahead, int timeout, bool forwards = true, bool async = true)`.
- **WinLib signature (forward-looking, not yet implemented):**
  ```cpp
  void WinLib::follow(const std::vector<Pose>& waypoints,
                      float lookahead,
                      int   timeout,
                      bool  forwards = true);
  ```
  - **Free function**, not a member of a `Chassis` class — matches the rest of WinLib's chassis API.
  - **No `async` parameter** — WinLib motions are blocking-only (`CLAUDE.md` § *Blocking motions*). Drop it.
  - **Caller passes `std::vector<WinLib::Pose>` directly**, e.g.:
    ```cpp
    std::vector<WinLib::Pose> middlePath = {{0, 0, 0}, {12, 12, 0}, {24, 24, 0}};
    WinLib::follow(middlePath, 9, 1200, false);
    ```
- **Why:** Bypassing the asset system means students don't have to learn linker symbols, `objcopy`, or a custom Makefile to author a path. If SD-card path loading is ever genuinely needed, a runtime `std::ifstream("/usd/<name>.txt")` parser is the preferred shape — easier to teach, keeps the Makefile clean.

---

## Pending / Deferred (do not change without asking the user)

These are tracked in `CLAUDE.md` § *Deferred Decisions*; listed here so this doc stays a one-stop reference:

- **Odometry thread-safety** (`pros::Mutex` around `odomPose` / `odomSpeed`).
- ~~**Odom sensor injection**~~ — **resolved**. `OdomSensors` is now a member of `Chassis`, initialized via the constructor, and `odom.cpp` reads through `chassis.odomSensors` by `#include "config.h"`.
- **Kalman filter pass** over local speed.
- **Command-based abstraction.**
- **Async motion support / motion queue.** Currently disallowed. Flagged by Winyeh as a possible future addition once blocking-only routes show their limits. Until then: no `pros::Task`-backed motions, no queue, no `bool async` params, no `waitUntilDone`/`cancelMotion`.
- **Dual-IMU averaging.** The robot has two IMUs declared (`imu1`, `imu2`) but only `imu1` is wired into `OdomSensors.imu`. Averaging both for better heading accuracy is the intent. Naive average of two angles breaks at the 0°/360° wraparound (`1° + 359°` averages to 180°, should be 0°). Two viable solutions: (a) since odom uses `pros::Imu::get_rotation()` (unbounded — accumulates past 360°), a plain mean works correctly as long as both IMUs are reset together at calibration; (b) compute a circular average via `atan2(sin(a) + sin(b), cos(a) + cos(b))`. Implementation shape: add a second `pros::Imu*` field to `OdomSensors`, then average inside `Odom.cpp::update()`. Not yet built — needs an explicit ask.

---

## Changelog format (for future entries)

When you add a new entry, prefer this shape:

```
### <Short title of the divergence>
- **Reference:** <what Genesis/LemLib does>
- **WinLib:** <what we do instead>
- **Why:** <one or two sentences, focused on the teaching/design tradeoff>
```

Keep entries scoped to *intentional* divergences. Don't catalogue every code-cleanup edit — only the ones a future reader would be confused or surprised by if they compared the two libraries side-by-side.
