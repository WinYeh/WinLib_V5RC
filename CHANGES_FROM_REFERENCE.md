# Changes from Reference Libraries

This document tracks how **WinLib** intentionally diverges from its reference implementations. It exists so that anyone reading WinLib alongside Genesis or LemLib (including future-Winyeh, the 2026 team, and any agent helping with this code) can see the *why* behind every difference at a glance.

## References

- **Genesis** (78181A's fork of LemLib, the immediate inspiration): https://github.com/NicksonC1/78181A-Push-Back
- **LemLib** (upstream): https://github.com/LemLib/LemLib
- **Pilons 5225A odometry paper** (the math behind `Odom.cpp`): http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
- **14683A Push Back** (Winyeh's own prior-season VEXcode code — source of `moveByWall`, ported from its `move_new_wall()`): https://github.com/WinYeh/PushBack_14683A

WinLib's north star is teachability for a high-school team in their first PROS season. Most divergences below trade competitive features (or generic flexibility) for code a 10th-grader can read top-to-bottom.

> **Note for the agent:** every time you change WinLib in a way that makes it diverge further from Genesis/LemLib — adding a file, removing a class member, changing an algorithm, simplifying a fallback path — append a new entry to this document. See `CLAUDE.md` § *Tracking Divergences* for when and how.

---

## Architecture

### Split ownership — `Chassis` owns sensors + the public API; `odom.cpp` owns the running math
- **Reference:** LemLib/Genesis put **everything** inside the `Chassis` class — sensors, the tracking task, the pose state, and `getPose()` all live as Chassis members.
- **WinLib:** Movement, opcontrol, chassis-level utilities, AND the `OdomSensors` configuration all live on the `Chassis` class. But the **odometry running state** (`odomPose`, `odomSpeed`, `prevVertical`, etc.) and the tracking task itself stay in `odom.cpp` as module-level statics. The public odom API is namespace-level free functions (`WinLib::getPose`, `setPose`, `update`, `init`). `odom.cpp` reads the sensors by `#include "config.h"` and reaching through the global `chassis.odomSensors` — there's only one copy of the OdomSensors in the whole program.
- **Why:** Two competing goals. (1) Odom math is self-contained and reads cleanly on its own — folding it into Chassis would mean every odom edit also loads the Chassis mental model. So the math + running state stay in `odom.cpp`. (2) Sensor configuration is robot-specific and lives naturally with the rest of the chassis config — making it a Chassis member means `chassis.calibrate()` can handle IMU calibration and wheel reset in one call, and the user has one obvious place to read or change it. The compromise: `odom.cpp` depends on the application's `config.h` (mild layering violation), but in exchange there's zero data duplication and zero sensor-injection boilerplate.

### Multi-robot config via namespaces + a single active-chassis pointer `Chs` (supersedes the "global `chassis`" detail above)
- **Reference:** LemLib/Genesis declare one `Chassis chassis;` global; library code refers to it by name. There is no concept of multiple robots sharing one codebase.
- **WinLib:** `config.h`/`config.cpp` declare **three** robots, each in its own namespace (`test`, `dr4b`, `ace`), and each carries a complete, independent device set + its own `WinLib::Chassis`. A single global pointer `WinLib::Chassis* Chs` (plus a `setActiveChassis(WinLib::Chassis&)` setter) names whichever robot is active. `main.cpp`'s `initialize()` calls `setActiveChassis(ace::chassis)` once, before the odom task starts. The robot-agnostic library files (`Odom.cpp`, `moveFor.cpp`) now read through `Chs->odomSensors` / `Chs->drivetrain` instead of a hard-named global `chassis`. A `if (Chs == nullptr) return;` guard sits at the top of `OdomUpdate()`. Subsystems are deliberately **not** abstracted by `Chs` — every robot's subsystems differ in type and API, so subsystem code still names its robot (`ace::Cascade::Ctr()`).
- **Why:** One pointer can swap between the three chassis precisely because they're all the same `WinLib::Chassis` type — switching robots becomes one line in `initialize()` instead of editing every call site. Subsystems can't ride along (no shared type), which is the honest boundary of the idea. The runtime cost (all three robots' device handles constructed at boot) is negligible for PROS, whose Motor/Rotation/Distance objects are lightweight handles.

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
- **WinLib:** New class `WinLib::DSR` in `chassis/DSR.hpp` / `.cpp`. Wraps 4 `pros::Distance*` sensors (one on each side of the robot) plus the per-sensor mm offset from the robot's tracking center. `dsr.reset()` uses the IMU heading + ray–wall intersection math to figure out which wall each sensor is hitting (works at **any** heading, not just cardinal), then picks the closest valid reading per axis and overwrites the odom (x, y) via `WinLib::setPose()`. Heading is left untouched (the IMU is the source of truth for heading). Held by `Chassis` as a public member; `Chassis::resetPosition()` is now a thin delegate to `dsr.reset()`.
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

### Two selectable odom modes — `OdomMode { TW2, VPD }`
- **Reference (Pilons / LemLib):** One odometry pipeline with a fixed sensor-priority cascade; no user-visible algorithm selector.
- **WinLib:** `OdomUpdate()` is a dispatcher over an explicit `OdomMode` stored on `OdomSensors`:
  - **TW2** — the original two-tracking-wheel + IMU algorithm, unchanged (`OdomUpdate_TW2`).
  - **VPD** — single vertical tracking wheel + drivetrain (`OdomUpdate_VPD`): forward from the vertical wheel (drivetrain average as a fallback), heading from the drivetrain left/right difference `(ΔS_L − ΔS_R)/trackWidth` **fused with the IMU** via `blendByTrust(imuΔ, driveΔ, imuTrust)`, sideways assumed zero. Both modes hand their local step to a shared `integrateStep()` (the chord-at-average-heading projection).
- **Why:** The team wanted a version needing only one tracking wheel, with an IMU-independent heading cross-check/fallback (when `imuTrust → 0`, or the IMU is null, VPD heading is pure drivetrain). Explicit mode beats LemLib's infer-from-which-sensors-are-non-null here: the two modes share nearly the same sensor set, so sensor *presence* can't disambiguate them, and the named mode is self-documenting for students.
- **Supersedes (kept above as history):** "Heading-source priority reduced to 2" — VPD adds drivetrain-differential heading (fused), so heading is IMU-only *only* in TW2. And part of "Drivetrain instance in the odom module" — VPD *does* read the drivetrain, but through `chassis.drivetrain` via the existing `config.h` reach-through, not a duplicated module-level instance.
- **Supporting changes:** `blendByTrust()` (util) — credibility-weighted fusion sharing `ema`'s arithmetic; `Chassis::degToMM()` + `motorRPM()` — encoder degrees → mm; `moveFor` switched from taring the drive encoders to a start-offset so it no longer corrupts the now-shared encoders; `imuTrust` field added to `OdomSensors`.

### Non-finite sensor read is sanitized per-sensor — NaN-poisoning guard in both update modes
- **Reference (Pilons / LemLib):** No finiteness check. A failed sensor read returns `PROS_ERR_F` (`== INFINITY`); the integrator runs `sin`/`cos` on it, producing `NaN`. Because any arithmetic with `NaN` is `NaN`, the pose is poisoned **permanently** — `getPose()` returns `(nan, nan, nan)` for the rest of the run.
- **WinLib:** Both `OdomUpdate_TW2` and `OdomUpdate_VPD` call `std::isfinite()` on each raw reading and, if one is non-finite, replace *that reading* with the sensor's last good value (so its delta is 0 this tick) — the healthy sensors keep updating the pose. (An intermediate version bailed the *whole* update if *any* read was bad, but that froze ALL of odom — `theta` stuck at 0 — the moment a single motor/sensor was unplugged, since one dead drivetrain port makes `get_position()` return `INFINITY`. Per-sensor sanitizing degrades gracefully instead.)
- **Why:** A failed read (loose port, brownout, unplugged motor) shouldn't brick odometry. Letting it through poisons the pose with `NaN` permanently (`getPose()` → `(nan, nan, nan)`, seen in practice during VPD validation); skipping the whole tick freezes the pose entirely (also seen — `theta` stuck at 0). Sanitizing per-sensor keeps the good sensors live and only drops the bad one's contribution for that one tick.

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

### `ControllerSettings` collapsed to ONE exit window (supersedes the two-window entry above)
- **Reference:** Genesis's `ControllerSettings` carries two exit windows — `smallError`/`smallErrorTimeout` (tight final-approach) and `largeError`/`largeErrorTimeout` (looser "good enough" window used in combination with a minSpeed floor).
- **WinLib:** Reduced to a single window — flat fields `exitRange` and `exitTimeout`. The constructor takes one `ExitCondition`, not two.
  ```cpp
  WinLib::ControllerSettings lateralSettings(
      WinLib::PID(10, 0, 3, 3),         // kP, kI, kD, windupRange
      WinLib::ExitCondition(1, 100)     // settle: within 1 inch for 100 ms
  );
  // Internally stored as flat floats: kP=10, kI=0, kD=3, windupRange=3,
  //                                   exitRange=1, exitTimeout=100
  ```
  The "looser good-enough at low speed" role is now served entirely by `LateralParams::earlyExitRange` / `AngularParams::earlyExitRange` (instantaneous threshold, only fires when `params.minSpeed > 0`).
- **Why:** Two windows were rarely both used. In practice routes either tuned the small window and ignored the large one, or set `minSpeed > 0` and relied on `earlyExitRange` anyway. Carrying both meant four extra fields and two extra `ExitCondition` calls at every config site — pure overhead for teaching teams who only need to think about "when am I done." If a real route ever needs the dual-window behavior, build it locally inside that motion's `.cpp` from two ad-hoc `ExitCondition` instances rather than baking it into the shared config.
- **Movement-code consequence:** `turn_to_heading.cpp` (and the future motion files) now construct a single `ExitCondition exit(angularSettings.exitRange, angularSettings.exitTimeout)` and check `exit.getExit()` once per loop, plus the `earlyExitRange + minSpeed` short-circuit.

### Motor-power unit: volts (0–12 V), not PWM (0–127)
- **Reference:** Genesis/LemLib use the PROS PWM range — `pros::Motor::move(int)` taking -127..127. `maxSpeed = 127`, `minSpeed = 0`.
- **WinLib:** Movement-param motor-power fields (`maxSpeed`, `minSpeed`) are in **volts** with `12.0` as the hardware ceiling. Inside Chassis motion methods, drive the motors with `pros::Motor::move_voltage(mV)` (mV = volts × 1000). Opcontrol helpers (`tank`/`arcade`/`curvature`) still take joystick-shaped `int` args (-127..127) for ergonomic reasons and convert to volts internally.
- **Why:** PID gains tuned in PWM units don't transfer cleanly to voltage and vice versa. Picking one unit end-to-end (voltage) means tuning intuition stays consistent, and `kP * error → volts` reads correctly without a hidden 127-vs-12000 scaling factor.

### `ExitCondition::range` and `time` no longer `const` — runtime-tunable
- **Reference:** Genesis's `ExitCondition` declares its fields as `const float range; const int time;`, so the values are locked at construction.
- **WinLib:** `const` removed, plus explicit setters: `setRange(float)` and `setTime(int)`. Both also call `reset()` so the timer doesn't carry over with mismatched thresholds.
- **Why:** Auton routes sometimes need a different "good enough" definition at different points in the same match — e.g. tighten the small-error threshold for a final scoring approach, then loosen for the next cruise leg. With `const` fields, the user had to build a new `ExitCondition` and copy-assign; opening up the fields turns it into a one-liner like `chassis.lateralSettings.smallExit.setRange(0.5)`.

### Gain scheduling — asymptotic kP curve ported as a formula, not a class
- **Reference (Genesis):** `AsymptoticGainsPlus` (a class with `getGain`/`setGain`/`configure` and a stored running `setpoint`) feeds `kP` into `PIDPlus`, which holds it **by reference** so mutating the gains object updates every PID that points at it. Tuned via `setTurnPIDPlus(...)` / `setLateralPIDPlus(...)`, configured once in `MotionPlusTuning::apply()`. Full write-up in `docs/future/pidplus_and_asymptotic_gains.md`.
- **WinLib:** Ported only the **formula** — `WinLib::asymptoticGain(setpoint, initial, final, knee, power)`, a free function in `pid.hpp` / `pid.cpp`. No `AsymptoticGainsPlus` class, no reference indirection, no `setKp`/`tick` split, no reconfigure API. The four curve numbers live in a plain `AsymptoticGains` POD; a `ControllerSettings` optionally carries one as `std::optional<AsymptoticGains> gains` (by value, constructed inline at the config site). Scheduling is **per-motion locked** (the reference page's "case a"): a motion reads its initial error once, computes `kP` via `asymptoticGain` when `gains` has a value (else uses the constant `kP` from the PID), and holds that `kP` for the whole motion. Wired into `turnToHeading` and `moveFor`; both axes ship a curve (angular because turns needed it; lateral enabled by request). `moveToPose` and `moveToPoint` stay on the constant-`kP` fallback (matching Genesis's `movePosePlus`, which hardcodes `setKp` to fixed values).
- **Why:** The full Genesis machinery exists for a feature only one-and-a-half axes use, and the by-reference indirection buys runtime gain-sharing that WinLib doesn't need — it rebuilds a fresh PID every motion anyway. The formula alone delivers the real benefit (one set of numbers → snappy small motions, gentle big ones, no per-call `kP` hand-tuning) at a fraction of the teaching cost. `std::optional` leaves the original constant-`kP` path working untouched when no curve is supplied. Knee/setpoint units follow each motion's own error units — heading degrees for turns, motor degrees for `moveFor`.

---

## Movement / Motions

### `moveFor` measures distance from motor encoders, not odom — `Chassis::MMTodeg`
- **Reference:** LemLib/Genesis lateral moves track progress through **odometry** — the motion compares the odom pose against the target (in inches).
- **WinLib:** `moveFor` reads the **drivetrain motor encoders** directly. It tares both groups at the start, converts the commanded distance (mm) to motor degrees with `Chassis::MMTodeg`, then runs the lateral PID on `target − averageEncoderDegrees`. `MMTodeg` computes wheel circumference in mm (`wheelDiameter × 25.4 × π`) and derives the gear ratio at runtime from the cartridge color (`leftMotors->get_gearing()` → 100/200/600 rpm) ÷ `drivetrain.rpm`. (An earlier version of this helper had the unit constant and gear factor wrong — `2.54` instead of `25.4`, plus a hardcoded `0.75` — which made every `moveFor` overshoot by ≈4.5×; corrected.)
- **Why:** `moveFor`/`turnToHeading` are meant to work on day one, before odom is tuned. "Drive until the wheels have turned N degrees" is something a student can fully reason about without the odom pipeline; odom-based motions (`moveToPoint`, etc.) come later, once the pose is trusted.

### `moveFor` also holds a target heading — `theta` parameter + parallel angular PID
- **Reference:** Lateral distance moves take a distance and keep the robot straight via an *internal* correction to whatever heading the robot started at.
- **WinLib:** `moveFor(float distance, float theta, int timeout, LateralParams)` takes an **explicit target heading** `theta`. Each tick it runs a second (angular) PID toward `theta` and mixes the two outputs — `left = lateral + angular`, `right = lateral − angular` — so the robot actively steers to the commanded absolute heading while driving the distance. The angular correction is clamped to ±2 V so it can't overpower the drive.
- **Why:** Makes the motion's intent explicit ("drive 1.5 m while facing 0°") and lets a route correct drift to an *absolute* heading instead of only preserving whatever heading the previous motion left behind. This expands `moveFor` beyond the "simple distance move" originally sketched in `CLAUDE.md` § *Planned Architecture*.

### `LateralParams::forwards` is an `int` sign (±1), not a `bool`
- **Reference:** LemLib/Genesis motions use `bool forwards = true`.
- **WinLib:** `LateralParams::forwards` is an `int` holding `+1` (forwards) or `-1` (backwards), used directly as a sign multiplier on the target: `MMTodeg(distance) * params.forwards`. (`AngularParams` and `MoveToPoseParams` keep their `bool forwards`, since there the flag selects a *facing*, not a math sign.)
- **Why:** The value doubles as the arithmetic it drives — `* -1` cleanly negates the target for a reverse move, whereas a `bool` promotes to `0` and would zero the target out entirely. One token, no `if`.

### New: `turnBy(angle, timeout, AngularParams)` — relative in-place turn
- **Reference:** Genesis/LemLib expose only absolute turns (`turnToHeading` / `turnToPoint`). There is no relative "rotate N degrees from here" primitive, and because absolute turns run on the wrapped shortest-path error, a full revolution (or any move ≥180°) can't be commanded.
- **WinLib:** Adds `Chassis::turnBy(float angle, int timeout, AngularParams params)` in `chassis/movement/turnBy.cpp`. It tracks progress against the **unbounded** accumulated heading (`getPose().theta`, which never wraps): it snapshots `start = getPose().theta`, then each tick computes `error = angle − |getPose().theta − start|` — the *magnitude* of rotation so far, no `angleError`, no wrap. This handles a full revolution and beyond (`360`, `720`, …) and is **sign-robust** (progress is a magnitude, so the robot converges no matter which physical direction it spins). Trade-off of the magnitude form: `angle` must be **positive** — a negative `angle` can never be reached, since `|…| ≥ 0` — and `params.direction` is unused. Shares the same PID / exit-condition / gain-schedule / debug structure as `turnToHeading`.
- **Why:** Odom validation needs exact, repeatable full rotations — e.g. the offset-measurement method `offset = D / (2π·N)`, where the systematic offset arc grows linearly with N while random drift only grows like √N, so more rotations sharpen the estimate. `turnToHeading` can't express that. A separate relative motion keeps each method's contract clear (absolute heading vs. relative rotation) instead of overloading `turnToHeading` with a magic full-turn case.

### New: `moveToPose(x, y, theta, timeout, MoveToPoseParams)` — minimal drive-to-pose (the "boomerang" controller)
- **Reference:** LemLib/Genesis `moveToPose` (the "boomerang controller") uses a carrot point AND: (a) while far from the target, scales lateral output by the *sign* of `cos(headingError)` only (`out *= sgn(scalar)`) so the robot commits to the curve at full speed; (b) a curvature-based slip-speed clamp `maxSlipSpeed = sqrt(horizontalDrift · radius · 9.8)` from `getCurvature(pose, carrot)`; (c) direction locking (`if (forwards && !close) out = max(out, 0)`); and (d) gain scheduling on kP.
- **WinLib:** `Chassis::moveToPose` in `chassis/movement/moveToPose.cpp` (named to match LemLib's public API; renamed from `boomerang`/`BoomerangParams`, though the algorithm is still "the boomerang controller"). Keeps the core idea — the carrot `target − (sinθ, cosθ)·lead·distTarget` (note **(sin, cos)**, matching the project's compass heading convention where 0°=+Y, CW, per `Odom.cpp`), collapse-on-`close` (`carrot = target` inside a latched 190 mm ≈ 7.5 in radius), and the steering swap (aim at the carrot while far, lock onto the final `theta` while close). **Drops** for teachability: the `sgn(cos)` far-away trick (uses plain always-on `cos(headingError)` scaling instead, like `moveToPoint`), the `horizontalDrift` curvature clamp (respects `maxSpeed` + desaturation only), and gain scheduling (constant kP — verified this matches Genesis's `movePosePlus`, which hardcodes `setKp(1)` on lateral and `setKp(180)` on turn, so its asymptotic curve evaluates to a constant too). Backwards approach (`forwards = false`) is handled by aiming the robot's back (`heading + 180°`) and negating the lateral term, rather than the reference's direction-clamp. Exit reuses the **lateral** settings' window on remaining distance converted to motor degrees (`MMTodeg(distTarget)`), gated on `close`; `earlyExitRange` is in mm.
- **Why:** Boomerang earns its spot for the 2026–27 Override season (score-on-the-move routes), but the two dropped features — the sign-only speed trick and the slip-speed model — add real reading burden (curvature math, a friction constant, a far/near speed-policy split) for gains that only matter at high speed on tight arcs. A 10th grader can read this version top-to-bottom: build carrot → face it → drive at it → collapse it → lock the final heading. Both dropped features are documented here so they can be re-added later without rediscovery.

### New: `moveToPoint(x, y, timeout, LateralParams)` — drive to a point (no heading)
- **Reference:** LemLib's `moveToPoint` — drive to an (x, y) with no target heading, using a dedicated `MoveToPointParams { forwards, maxSpeed, minSpeed, earlyExitRange }`.
- **WinLib:** `Chassis::moveToPoint` in `chassis/movement/moveToPoint.cpp`. Same behavior, but **reuses the existing `LateralParams`** instead of a new param struct — its fields already match LemLib's `MoveToPointParams`, except `forwards` is the `int` ±1 that `moveFor` already uses (`>= 0` = front-first, `< 0` = back-first). Each tick: aim the leading end at the point (`atan2(dx, dy)` in the project's (sin, cos) compass convention), drive distance scaled by `cos(angularError)` (whose sign auto-reverses an overshoot with no special case), and stop steering — zeroing only the angular output — inside a 75 mm radius to avoid atan2 spin right on the point. Constant kP, like `moveToPose`. Exit reuses the **lateral** window on remaining distance in motor degrees (`MMTodeg(distTarget)`), gated on `close`.
- **Why:** LemLib splits point-vs-pose for a real reason: most "just get there" moves don't care about the final heading, and paying `moveToPose`'s carrot/`lead` complexity for them is wasteful. `moveToPoint` is the cheap, common case; reach for `moveToPose` only when the ending orientation matters. Reusing `LateralParams` avoids a near-duplicate struct for a 10th grader to keep straight.

### New: `moveByWall(distance, side, standoff, timeout, WallParams)` — wall-following straight drive
- **Reference:** 14683A's VEXcode `move_new_wall()` (two overloads) in `src/movement.cpp` of [PushBack_14683A](https://github.com/WinYeh/PushBack_14683A). Drives an encoder-measured distance while a PD controller holds a side distance sensor at a target reading; the 2nd overload switches to IMU heading-hold once `|wallError| < 2.5`. Straight speed uses a nonlinear `kP·|error|^e` curve; the side is selected by passing `target_L`/`target_R` with the unused one set to 0; distances are in cm.
- **WinLib:** `Chassis::moveByWall` in `chassis/movement/moveByWall.cpp`, built on the `moveFor` template (encoder distance via `MMTodeg`, lateral PID from `lateralSettings`, `ExitCondition` + timeout). The correction term holds a side-wall `standoff` via a PD controller (`WallParams::turnKp`/`turnKd`), clamped to ±1 V, mixed `left = lat ± turn` with the turn term flipped for reverse. Optional heading handoff kept: when `WallParams::targetHead` is set and `|wallError| < alignThreshold`, the correction switches to a proportional IMU heading-hold. Reads the side sensor through new `DSR::leftReading()`/`rightReading()` accessors (DSR already owns the distance sensors).
- **Divergences from the reference:** (1) **linear PID** for the drive, not `|error|^e` — consistent with `moveFor`, teachable, and avoids the very-gentle-near-target stall; (2) **`WallSide` enum + `standoff`** instead of the `target_L`/`target_R`-with-a-zero trick; (3) **mm** throughout (`pros::Distance::get()` is already mm — no cm `/10`); (4) a missing/invalid side reading zeroes the correction for that tick (drive straight) rather than assuming a value.
- **Why:** The team needs "drive along this wall at a fixed gap" for Push Back routes, and it's a natural reuse of the `moveFor` skeleton plus the distance sensors DSR already carries. Keeping the port on WinLib's linear-PID + params-struct conventions means it reads like the other motions.

### New: `swingToHeading` / `swingToPoint` — one-side-locked swing turns
- **Reference:** LemLib `swingToHeading(theta, DriveSide lockedSide, timeout, SwingToHeadingParams, async)` and `swingToPoint(x, y, DriveSide lockedSide, timeout, SwingToPointParams, async)`. A swing runs the normal angular PID but powers only the unlocked side and brakes the locked side, so the robot pivots around the locked wheels. `SwingTo*Params` = `direction`, `maxSpeed`, `minSpeed`, `earlyExitRange` (+ `forwards` on the point variant). Modeled on LemLib, **not Genesis** (per `docs/future/roadmap.md` §5).
- **WinLib:** `Chassis::swingToHeading`/`swingToPoint` in `chassis/movement/swingToHeading.cpp` / `swingToPoint.cpp`, built on the `turnToHeading` template (angular PID, `angleError`, `ExitCondition`, gain-schedule kP, minSpeed/earlyExitRange). The **only** change is the mix: the free side gets the PID output — sign chosen (`left = +output`, `right = -output`) to preserve `turnToHeading`'s rotation sense so `direction` behaves identically — and the locked side is `brake()`-held (brake mode HOLD set up front). `swingToPoint` recomputes the bearing to the point (`atan2(dx, dy)` in the (sin,cos) convention) **every tick** (the robot translates as it swings) and applies the `forwards` 180° flip with an `fmod` wrap (the moveToPoint lesson). New `DriveSide { LEFT, RIGHT }` enum names the locked side.
- **Divergences / choices:** (1) **reuses the existing `AngularParams`** — it already matches LemLib's swing params — rather than adding `SwingTo*Params` structs; (2) **blocking**, no `async` (WinLib rule).
- **Why:** Swings change heading while carving around an obstacle (Override's cluttered field), and they're a near-free reuse of `turnToHeading` — only the motor mix differs — so they stay readable and share the same tuning (`angularSettings`).

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
- ~~**Odom sensor injection**~~ — **resolved**. `OdomSensors` is now a member of `Chassis`, initialized via the constructor, and `odom.cpp` reads through the active-chassis pointer `Chs->odomSensors` by `#include "config.h"`.
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
