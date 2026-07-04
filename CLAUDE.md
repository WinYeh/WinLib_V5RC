# Claude Agent Guidelines for WinLib Project

## About the User

- **Name:** Winyeh
- **Role:** Mentor/teacher building this project to teach a VEX team entering 10th grade (summer 2026)
- **Experience Level:** High school student with 2 seasons of VEXcode experience; transitioning to PROS because competitive top teams use it for its advantages. This is their first PROS project.
- **Goal:** Create a teaching-focused library + project that the team can understand, modify, and maintain themselves during the season — not just use as a black box.

## Project Context

This is a VEX robotics project using the PROS framework, structured with two intentional layers:

1. **WinLib Library Layer** — reusable, robot-agnostic code (sensors, drivetrain logic, utilities)
   - Lives in `include/WinLib/` and `src/WinLib/`
   - Should be kept independent from any specific robot configuration

2. **Robot Application Layer** — robot-specific code that uses WinLib
   - Lives in `src/main.cpp`, `src/config.cpp`, `include/config.h`
   - Wires up hardware (ports, motors, sensors) and calls WinLib functions
   - Declares three robots, each in its own namespace (`test`, `dr4b`, `ace`), each with a complete device set and its own `WinLib::Chassis`. A single global pointer `WinLib::Chassis* Chs` names the active robot (`setActiveChassis(...)`, called once in `initialize()`); WinLib library code reads the active robot through `Chs`.

### Reference Project
- Based on study of team 78181A Genesis's repo: https://github.com/NicksonC1/78181A-Push-Back
- That repo uses a LemLib fork ("Genesis" library) with heavy OOP (Chassis class, DriveCurve hierarchy, motion queue, etc.)
- WinLib intentionally simplifies this for teachability

### Design Constraints
- **Chassis class for movement/opcontrol; free functions for odometry.** A `Chassis` class is the public API for autonomous motions, opcontrol drive, and chassis-level utilities (`calibrate`, `setBrakeMode`, etc.). Students coming from VEXcode are already used to `chassis.driveFor(...)`-style calls, so the `chassis.` prefix is a feature (it tells you what's being acted on), not noise. Odometry stays as free functions in the `WinLib::` namespace (`getPose`, `setPose`, `update`, `init`) with module-level *running state* (`odomPose`, `odomSpeed`, `prevVertical`, etc.) inside `odom.cpp`. The **sensor configuration** (`OdomSensors`) lives on `Chassis` as a member — single source of truth — and `odom.cpp` reads it by including `config.h` and reaching through the active-chassis pointer `Chs` (`Chs->odomSensors`). The `Chassis` class **calls** the odom API when it needs the robot's pose. Small self-contained utility classes are still allowed (PID, Timer, ExitCondition, Pose, TrackingWheel). POD config structs (`Drivetrain`, `OdomSensors`) are preferred over OOP for plain data carriers — no methods, no inheritance. `ControllerSettings` is a thin composition class that holds a `PID` and one `ExitCondition` (the settle window), plus an optional `AsymptoticGains` curve — no logic of its own, just a named bundle of subcomponents. No class inheritance hierarchies.
- **Lateral, Angular, Boomerang, and Swing motions:** Boomerang is allowed (drive-to-pose with a target heading via a carrot point) — added for the 2026-2027 Override season, where the complex field layout rewards score-on-the-move autonomous routes. **Swing motions (`swingToHeading` / `swingToPoint`) are now on the menu too** (reversal of the earlier "no swing" decision — see the `chassis.hpp` note below and the Deferred Decisions entry): a swing locks one side of the drivetrain and powers only the other, pivoting around a stationary wheel to change heading while carving forward. **Model swings on LemLib's implementation, not Genesis** — LemLib is the upstream library Genesis forked from, and its swing code is the cleaner reference. **No pure pursuit, no arc/curvature primitives** — these still add too much teaching burden (path generation, lookahead tuning, intersection math) for marginal benefit.
- **Blocking motions:** Movement functions run synchronously (no async tasks, no motion queue). Students can read autonomous routes top-to-bottom. (Async support may be revisited in the future — see *Deferred Decisions*.)
- **Volts (0–12 V) is the motor-power unit.** Movement param defaults (`maxSpeed`, `minSpeed`) are in volts, with `12.0` as the hardware ceiling. Inside Chassis motion methods, drive the motors with `pros::Motor::move_voltage(mV)` (mV = volts × 1000), not `move(pwm)`. Opcontrol helpers (`tank`/`arcade`/`curvature`) still take joystick-shaped int args (-127..127) and convert internally.
- **Teachability over performance:** Every design choice should prioritize "can a 10th grader understand this?" over competitive optimization.

### Planned Architecture
Chassis-related code lives under a `chassis/` subfolder, separate from general utilities. Status labels below: **[done]**, **[needs fixes]**, **[to add]**.

**Headers — `include/WinLib/`**
- **[done]** `pid.hpp`, `pose.hpp`, `timer.hpp`, `exitcondition.hpp`, `util.hpp` — small self-contained utilities.
- (No separate `chassis/config.hpp`.) The config types — `Drivetrain` POD (motor groups, track width, wheel diameter, gear ratio, `horizontalDrift`) and `ControllerSettings` (composition class holding one `PID` and one `ExitCondition` — the settle window). `ControllerSettings` constructor reads the inputs via getters and stores the values as flat floats (`kP`, `kI`, `kD`, `windupRange`, `exitRange`, `exitTimeout`) for direct mutation by movement code. Both live inside `chassis/chassis.hpp` alongside the `Chassis` class itself. (`OdomSensors` still lives separately in `chassis/OdomSensors.hpp`.)
- **[done]** `chassis/OdomSensors.hpp` — holds the `Omniwheel` diameter constants, the `TrackingWheel` class (wraps a `pros::Rotation` sensor only — ADI encoders / motor groups intentionally unsupported, since modern VEX doesn't use them), and the `OdomSensors` class (vertical wheel, horizontal wheel, IMU pointers). Units: wheel diameter is passed in inches and stored as mm; offset is in mm; `getDistanceTraveled()` returns mm.
- **[done]** `chassis/Odom.hpp` — free-function declarations: `getPose`, `setPose`, `getSpeed`, `update`, `init`. Odom is a standalone module; the `Chassis` class calls these from its movement methods. (`getLocalSpeed`/`estimatePose` were removed; a Kalman filter is a future maybe.)
- **[done]** `chassis/DSR.hpp` — Distance Sensor Reset class. Holds 4 `pros::Distance*` pointers (front/back/left/right) + their mm offsets from robot center. `reset()` reads all 4 sensors plus the IMU heading, uses ray–wall intersection math to figure out which wall each sensor hits, and overwrites the odom (x, y) using the closest valid reading per axis. Works at any heading (not just cardinal). Assumes the standard VEX field (3600 × 3600 mm, origin at center). Also exposes `leftReading()`/`rightReading()` (mm, or -1 if null/invalid) so wall-following motions like `moveByWall` can read a side wall without duplicating the sensor pointers.
- **[done]** `chassis/chassis.hpp` — declares `class WinLib::Chassis`. Constructor: `Chassis(Drivetrain, OdomSensors, ControllerSettings lateral, ControllerSettings angular, DSR)`. Methods: `calibrate`, `setBrakeMode`, `resetPosition` (delegates to `dsr.reset()`), the unit helpers `motorRPM`/`MMTodeg`/`degToMM`, the raw drive helpers `move_voltage`/`move_percentage`, the motions `moveToPoint`, `moveFor`, `turnToHeading`, `turnBy`, `turnToPoint`, `moveToPose`, `moveByWall`, and `arcade` (only opcontrol method — `tank`/`curvature` deliberately omitted). `moveByWall` adds a `WallSide` enum + `WallParams` struct here, and two `DSR` side-reading accessors (`leftReading`/`rightReading`). Slimming from the Genesis copy-paste is complete: no `asset`/`motionPlus`/`driveCurve` includes, no `*Plus` methods, no motion queue, no `bool async` params, no `slew`, no duplicate `AngularDirection` enum. (`turnToPoint` is declared but not yet implemented; `moveToPoint` and `moveToPose` now are — see the movement/ note. Swing motions `swingToHeading`/`swingToPoint` are now planned but not yet declared or implemented — model them on LemLib, not Genesis; see the Deferred Decisions entry.)

**Sources — `src/WinLib/`**
- **[done]** `pid.cpp`, `pose.cpp`, `timer.cpp`, `exitcondition.cpp`, `util.cpp`.
- **[done]** `chassis/OdomSensors.cpp`.
- **[done]** `chassis/DSR.cpp` — Distance Sensor Reset implementation. Reads IMU heading + 4 distance sensors, runs ray–wall intersection per sensor, picks closest valid X-info and Y-info readings, calls `WinLib::setPose()` with the corrected pose.
- **[mostly done]** `chassis/Odom.cpp` — Pilons-style tracking in a `pros::Task` at ~10ms, with two selectable algorithms: `OdomMode::TW2` (two tracking wheels, heading from the IMU only) and `OdomMode::VPD` (single vertical wheel + drivetrain, heading from an IMU/drivetrain-differential blend via `blendByTrust`). Both build a local step and share one `integrateStep` projection. Sensors are read through the active-chassis pointer (`Chs->odomSensors`); a per-sensor glitch guard replaces any non-finite reading with its last good value. Resolved since the original note: sensor injection (now via `Chs`) and the unused `deltaVertical`/`deltaHorizontal` locals (gone with `integrateStep`). Remaining: thread-safety — a `pros::Mutex` around the pose publish/reads, deferred (see Deferred Decisions).
- **[done]** `chassis/chassis.cpp` — the small "glue" methods of `Chassis`: constructor, `calibrate` (calibrates the IMU, resets the tracking wheels, then starts the odom task), `setBrakeMode`, `resetPosition`, plus the unit helpers `motorRPM`/`MMTodeg`/`degToMM` and the raw drive helpers `move_voltage`/`move_percentage`.
- **[done]** `chassis/opcontrol.cpp` — `Chassis::arcade` only. Single-method opcontrol with the classic arcade mixing (`left = throttle + turn`, `right = throttle - turn`), raw joystick input converted linearly to motor voltage.
- `chassis/movement/` — one `.cpp` file per motion (each defines a single `Chassis` method; the linker stitches them together, so each file stays short and focused on one motion's math).
  - **[done]** `moveFor.cpp` — drives a set distance **and** holds a target heading `theta` via a parallel angular PID (heading-corrected straight drive), so it is not a pure distance move. Distance progress comes from the drivetrain motor encoders (converted by `MMTodeg`), not odom.
  - **[done]** `turnToHeading.cpp` — turn to an **absolute** heading (shortest path via `angleError`, or a forced direction).
  - **[done]** `turnBy.cpp` — **relative** turn by N degrees, tracked on the unbounded heading as a magnitude (`angle` must be positive). Used for multi-rotation odom validation; `turnToHeading` can't express a full revolution.
  - **[done]** `moveToPoint.cpp` — drive to a **point** (x, y); no final heading. Aims the robot's leading end at the point and drives, with `cos(angularError)` scaling whose sign auto-reverses an overshoot; stops steering inside a small radius to avoid atan2 spin. Consumes odom (x, y).
  - **[done]** `moveToPose.cpp` — drive to a full **pose** (x, y, θ) via the boomerang controller (carrot point + `lead`). Consumes odom (x, y). (Renamed from `boomerang.cpp` to match LemLib's public name; the algorithm is still "the boomerang controller".)
  - **[done]** `moveByWall.cpp` — drive an encoder-measured distance while hugging a side wall at a fixed `standoff` (via a `DSR` side distance sensor), with an optional handoff to IMU heading-hold once aligned. Ported from 14683A `move_new_wall` (see CHANGES). Does NOT consume odom position — encoders + distance sensor + IMU only.
  - **[to add / deferred]** `turn_to_point.cpp` — the remaining odom-(x,y)-based motion. Declared in `chassis.hpp` but not implemented; deferred until a route needs it.

  Note: `moveToPoint`/`moveToPose`/`turnToPoint` are the *only* motions that consume odom **position** — `moveFor`/`turnToHeading`/`turnBy` use encoders + IMU heading, so they're immune to odom x/y drift.

### Deferred Decisions
- **Command-based programming system (async task management).** Explicitly deferred. The plan is to first write real autonomous routes using the standard pattern — blocking chassis motions plus `pros::Task` for parallel subsystem work (intake, lift, etc.) — and revisit whether a command-based abstraction is worth building only after that experience surfaces a real need. Do not propose or build command-based infrastructure unless the user explicitly asks.
- **Odometry thread-safety (`pros::Mutex`).** The tracking task writes `odomPose`/`odomSpeed` every ~10ms while movement code reads them via `getPose`/`getSpeed`, so a reader can catch a half-updated ("torn") pose — e.g. a new `x` paired with an old `y`. The fix is a single `pros::Mutex` (ideally via `std::lock_guard`) wrapping the publish step in `update()` **and** the reads in every getter/setter — a lock only works if both sides take it. Deferred for now: the race costs at most one stale axis for one 10ms tick, acceptable for current routes. Revisit if a movement ever misbehaves in a way traceable to a torn pose read. Do not add the mutex unless the user asks.
- ~~**Odom sensor injection.**~~ **Resolved.** `OdomSensors` now lives on `Chassis` and is initialized via the constructor. `odom.cpp` reads through the active-chassis pointer `Chs->odomSensors` by `#include "config.h"`. Acceptable layering compromise — odom depends on the application's `Chs`, but the upside is zero data duplication and zero setup boilerplate.
- **Drive curves (`DriveCurve` / `ExpoDriveCurve`).** Joystick→motor input shaping (deadband + exponential curve) for opcontrol. Genesis ships this as a small class hierarchy bundled into the Chassis constructor. WinLib temporarily drops it — `Chassis::arcade` uses raw linear joystick input until autonomous motion work settles. Will return in a dedicated `driveCurve.hpp` / `.cpp` once the auton motions are in. Do not add drive-curve code to `chassis.hpp` in the meantime.
- **Async motion support / motion queue.** WinLib is currently blocking-only by design (motions run synchronously, no `pros::Task`, no queue, no `cancelMotion`/`waitUntilDone`/`async` param). The user has flagged this as a possible future addition — if/when chained-motion routes outgrow the blocking model, this becomes a real conversation. For now, the rule stands: no async, no queue, no `bool async = true` params. Revisit only on explicit request.
- **Drivetrain-encoder odometry fallback (reversal of earlier decision).** Originally we dropped drivetrain motor encoders from the odom math on the grounds that "modern VEX uses tracking wheels, not motor encoders." That decision is now reversed: drivetrain motor encoders will be added back so that a robot with **only one tracking wheel** (or zero) can still localize. The motor encoders act as the fallback when the dedicated tracking wheel for that axis is missing. **Sensor-priority rule that comes with this:** for each axis (forward and sideways), prefer the dedicated tracking wheel if present; otherwise fall back to the drivetrain motor encoders (averaged across the left and right groups, with the gear ratio applied to convert revs → wheel travel). The vertical (forward/back) axis is the main use case — left+right motor encoders trivially give you forward travel. The horizontal (sideways) axis can NOT be measured from drivetrain encoders alone on a non-mecanum drivetrain, so if the horizontal tracking wheel is missing, the horizontal delta is just assumed zero (which is what the current algorithm already does). **Trigger:** do NOT implement this until the two-tracking-wheel algorithm has been verified accurate across a full 15-second autonomous routine on the real robot. Re-introducing the fallback before then would muddy any odom-accuracy debugging — you wouldn't know whether a positional error came from the tracking-wheel math or the encoder-fallback math. Once accuracy is confirmed, the shape of the change is: (a) re-add `pros::MotorGroup*` references on the `Drivetrain` POD (already there — `leftMotors`, `rightMotors` exist), (b) extend `Odom.cpp::update()` to read motor encoders when `verticalWheel == nullptr`, applying `wheelDiameter`, `rpm`, and gear ratio from `Drivetrain`, (c) document in code which sensor source was selected for that tick (a small `enum class OdomSource { TrackingWheel, MotorEncoder, None }` if it helps debugging). Heading priority does NOT change — still IMU only. (Reference Odom.cpp comment "Dual parallel vertical wheels and drivetrain tracking were dropped" is now outdated for the second clause; update when the code lands.)

  **Update (2026-06-19):** A separate **VPD** odom mode (`OdomMode::VPD` / `OdomUpdate_VPD`) now uses the drivetrain encoders — but as its own algorithm, not the fallback described above. VPD derives *heading* from the left/right encoder difference fused with the IMU (`blendByTrust`), with forward from the vertical wheel (drivetrain average as a fallback) and sideways assumed zero. The fallback described here — motor-encoder *forward* travel inside the **TW2** algorithm when the vertical wheel is null, heading still IMU-only — is still **not** implemented. (The outdated `Odom.cpp` comment noted above was removed when the shared `integrateStep` refactor landed.)
- **Dual-IMU averaging.** The robot has two IMUs (`imu1`, `imu2`) declared in `config.cpp`, but only `imu1` is wired into `OdomSensors`. `imu2` sits idle. The intent is to average both readings for better heading accuracy, but a naive `(a + b) / 2` of two angles breaks across the 0°/360° wraparound (e.g. `imu1 = 1°, imu2 = 359°` averages to 180° but should be 0°). Two reasonable paths exist when we tackle this: (a) since odom uses `pros::Imu::get_rotation()` which is **unbounded** (accumulates past 360° without wrapping), a simple `(a + b) / 2` actually works without a wraparound bug — as long as both IMUs are reset together at calibration; (b) compute a proper **circular average** via `atan2(sin(a) + sin(b), cos(a) + cos(b))`, robust regardless of bounded vs unbounded inputs. Path (a) is simpler and almost certainly correct for our case, but requires a clear statement of the assumption. The right shape: extend `OdomSensors` with a second `pros::Imu*` and average inside `Odom.cpp::update()`. Do not implement unless the user asks.
- **Swing motions (`swingToHeading` / `swingToPoint`) — approved, not yet built (reversal of the earlier "no swing" decision).** Genesis-era WinLib deliberately dropped swings; that is now reversed for the Override season (see Design Constraints line and the `chassis.hpp` note). A swing locks one side of the drivetrain and powers only the other, pivoting around a stationary wheel to change heading while carving forward — useful for turning *and* moving around an obstacle at once. **Reference LemLib's implementation, not Genesis** — LemLib is the upstream library Genesis forked from, and its swing code is the cleaner one to learn from. Shape of the change when it lands: declare `swingToHeading`/`swingToPoint` on `Chassis`, add one `.cpp` per motion under `chassis/movement/` (like `turnToHeading.cpp`), reuse the angular `ControllerSettings` + `ExitCondition` pattern, and drive only one side of the drivetrain. This differs from a normal turn only in the mixing step (one side held at 0). It is on the roadmap ([`docs/future/roadmap.md`](docs/future/roadmap.md) §5) but not started; do not build until the user asks. When it lands, log it in `CHANGES_FROM_REFERENCE.md`.

## How Claude Should Respond

### Tone & Teaching Style
- Explain concepts as if talking to a smart high school student who is new to the topic
- **Always include an analogy** labeled `[Analogy]` that explains the concept as if to a 5-year-old
- Keep explanations friendly, encouraging, and clear — avoid jargon without first explaining it
- Use short sentences and concrete real-world comparisons

### Code Style
- Keep code simple and readable — avoid over-engineering
- Prefer clarity over cleverness
- Add comments to non-obvious logic
- Respect the two-layer architecture: WinLib should not depend on robot-specific config
- Movement and opcontrol code lives as methods of the `Chassis` class. Odometry stays as free functions in the `WinLib::` namespace. Don't mix the two — Chassis methods should call into the odom API, not duplicate odom state.
- Movement functions should be straightforward: create PID, loop until done, stop motors

### Things to Avoid
- Do not use dense academic language
- Do not assume prior knowledge of C++ concepts without explaining them first
- Do not skip analogies when the user asks for them
- Do not move the odom module's *running state* (`odomPose`, `odomSpeed`, `prevVertical`, etc.) into `Chassis` member variables. That state stays in `odom.cpp` as module-level statics, updated by the tracking task. (`OdomSensors`, on the other hand, **is** owned by `Chassis` — single source of truth — and `odom.cpp` reads through `Chs->odomSensors`.)
- Do not add curvature/arc/pure pursuit motions. (Boomerang and swing motions are allowed for the Override season — model swings on LemLib.)
- Do not add async motions or motion queues

### Tracking Divergences from Reference Libraries
`CHANGES_FROM_REFERENCE.md` (in the project root) is the running log of every place WinLib intentionally differs from Genesis 78181A and LemLib. It exists so a future reader can compare the two libraries side-by-side and see *why* every difference was made.

**Read it before** adding new chassis / movement / odometry code, changing a public API signature in those modules, or porting any non-trivial structure (struct, macro, build rule, header) from the reference. Some divergences are *forward-looking traps* — e.g. the `asset` / `ASSET()` macro system is forbidden even though pure pursuit itself is currently forbidden — and you will not find them anywhere else in this codebase.

**When to update it:**
- A new file is added under `include/WinLib/` or `src/WinLib/` that has no Genesis/LemLib counterpart, or whose counterpart was meaningfully simplified.
- A class member, method, parameter, or return type is removed or renamed relative to the reference.
- An algorithm or fallback path is simplified or replaced (e.g. the heading-priority simplification, the `Odom::update()` consolidation).
- A bug in the inherited code is fixed and the fix changes observable behavior.

**When NOT to update it:**
- Pure formatting / comment / whitespace cleanups.
- Bug fixes to code WinLib itself wrote (i.e. not inherited from the reference).
- Work-in-progress edits that haven't settled — wait until the divergence is intentional and stable.

**How to update it:**
- Append a new entry in the appropriate section (Architecture, Sensors, Odometry, etc.), using the changelog template at the bottom of that file (`### Title` → `Reference:` → `WinLib:` → `Why:`).
- Do not delete old entries when behavior changes again — instead, add a new entry that supersedes the old one, and leave the original as a historical record.

After making any change that fits the "When to update" list above, mention in your reply that `CHANGES_FROM_REFERENCE.md` was updated (or ask the user whether the change is intentional enough to log, if uncertain).
