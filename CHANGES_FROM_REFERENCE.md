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

### Hybrid ownership — `Chassis` class for movement, free functions for odom
- **Reference:** LemLib/Genesis put **everything** inside the `Chassis` class — the odometry task is started by `chassis.calibrate()`, the pose lives in `chassis.pose` (a member), and `chassis.getPose()` reads from that member.
- **WinLib:** Movement, opcontrol, and chassis-level utilities live on the `Chassis` class (`chassis.moveToPoint(...)`, `chassis.tank(...)`, etc.). **Odometry stays as namespace-level free functions** in `WinLib::` (`getPose`, `setPose`, `update`, `init`) with module-level static state inside `odom.cpp`. The `Chassis` class **calls** the odom API; it does not own the pose.
- **Why:** Two reasons. (1) The odom math is self-contained and reads cleanly on its own; folding it into Chassis would mean every odom edit also loads the Chassis mental model. (2) Splitting ownership lets odom evolve independently — e.g. the deferred sensor-injection mechanism doesn't have to touch the Chassis constructor.

### Blocking motions only — no motion queue, no async
- **Reference:** LemLib enqueues motions; they run on a background `pros::Task`.
- **WinLib:** Movement functions block until the motion finishes. Subsystem-parallel work (intake, lift) is done with plain `pros::Task` at the user's discretion.
- **Why:** Students can step through an auton routine in their head without modeling a scheduler.

### No command-based programming system
- **Reference:** LemLib has been experimenting with command-based abstractions.
- **WinLib:** Deferred until writing real routes surfaces a concrete need.

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
- **Odom sensor injection** — `WinLib::init()` currently leaves `odomSensors` as `(nullptr, nullptr, nullptr)`. The injection mechanism is being designed separately.
- **Kalman filter pass** over local speed.
- **Command-based abstraction.**
- **Async motion support / motion queue.** Currently disallowed. Flagged by Winyeh as a possible future addition once blocking-only routes show their limits. Until then: no `pros::Task`-backed motions, no queue, no `bool async` params, no `waitUntilDone`/`cancelMotion`.

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
