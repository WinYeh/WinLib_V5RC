# PIDPlus + AsymptoticGainsPlus — Reference Page

> **Status: Reference only — not implemented in this library.** See Section 8 for when to revisit.

This page exists so that if WinLib ever needs **gain scheduling** (changing the PID's `kP` based on how far it has to move), the student maintaining it can read this and decide whether to adopt the system Team 78181A built — or pick something simpler.

All cited code lives in [78181A Push Back / Genesis](https://github.com/NicksonC1/78181A-Push-Back).

---

## 1. The problem PIDPlus solves

Classic LemLib `PID` stores three numbers — `kP`, `kI`, `kD` — and they're **locked at construction**. That means *one* set of gains has to work for:

- a **1°** final heading correction (needs high `kP` to actually move at all), AND
- a **180°** swing across the field (a high `kP` here will saturate the motors and overshoot).

You can't win both fights at once. PIDPlus lets the same controller use a `kP` that **changes based on how far it's being asked to move**. Small motion → high `kP` (precise settling). Big motion → low `kP` (smooth ramp, no overshoot).

That's gain scheduling.

---

## 2. Class structure — what's different from classic

In Genesis (`include/genesis/motionPlus.hpp`), the two relevant declarations are:

```cpp
class AsymptoticGainsPlus {
public:
    AsymptoticGainsPlus(float i, float f, float k, float p);
    void  configure(float i, float f, float k, float p);
    void  setGain(float setpoint);
    float getGain() const;
private:
    float i, f, k, p;
    float setpoint = 0;
};

class PIDPlus {
public:
    PIDPlus(AsymptoticGainsPlus& kP,
            float kI = 0, float kD = 0,
            float integralDeadband = 0,
            bool  integralSignReset = false);
    void  configure(float kI, float kD,
                    float integralDeadband = 0,
                    bool  integralSignReset = false);
    float tick(float error);
    void  reset(float initialError);
    void  setKp(float setpoint);
    float getError() const;
private:
    AsymptoticGainsPlus& kP;   // ← stored as a REFERENCE
    float kI;
    float kD;
    float integralDeadband;
    // ... running state ...
    bool  integralSignReset;
};
```

Two things to notice vs. the classic PID:

1. **`kI` and `kD` are non-const.** You can change them at runtime via `configure(...)` instead of throwing the PID away and building a new one.
2. **`kP` is not a number — it's a reference to a separate object** (an `AsymptoticGainsPlus`). When the PID needs `kP`, it calls `kP.getGain()`. This indirection is the whole trick — change the gains object and every PID holding a reference to it sees the new value.

---

## 3. The AsymptoticGainsPlus formula

This is the heart of it. From `src/genesis/motionPlus.cpp`:

```
getGain() = (f - i) * |setpoint|^p / (|setpoint|^p + k^p) + i
```

Four knobs:

- **`i` (initial)** — the `kP` value when `setpoint` is near 0
- **`f` (final)** — the `kP` value when `setpoint` is very large ("asymptote" means *a value the curve approaches but never quite reaches*)
- **`k` (knee)** — the `setpoint` at which the curve is exactly at the **midpoint** `(i + f) / 2`
- **`p` (power)** — how *sharply* the curve transitions around the knee. Higher `p` = sharper bend.

The shape is a smooth S-curve from `i` (at setpoint 0) toward `f` (at setpoint infinity), crossing the midpoint at `setpoint = k`.

---

## 4. Worked numerical example

Using the reference team's actual **turn axis** values (`i = 450`, `f = 220`, `k = 28`, `p = 1.5`):

| setpoint (°) | `getGain()` | Physical meaning |
|---:|---:|---|
| 1   | **448.46** | Tiny correction → near `i` (high `kP`, snaps to target) |
| 10  | **409.54** | Still small swing → still near `i` |
| 28  | **335.00** | Knee — exactly halfway between 450 and 220 |
| 90  | **254.01** | Big swing → curve heading toward `f` |
| 180 | **233.22** | Long sweep → almost at `f` (low `kP`, gentle) |

Walking through `1°`:

```
|1|^1.5 = 1
28^1.5  ≈ 148.16
getGain = (220 − 450) × 1 / (1 + 148.16) + 450
        = −230 / 149.16 + 450
        = −1.54 + 450
        ≈ 448.46
```

The big picture: **small angles get high `kP` for precision settling. Big angles get low `kP` so the robot ramps in instead of slamming.**

---

## 5. Setpoint vs error — the most confusing thing

This trips people up. The PIDPlus has **two different inputs** that look interchangeable but aren't:

- **`setpoint`** is what `setKp(setpoint)` takes. It picks *which* `kP` value off the asymptotic curve.
- **`error`** is what `tick(error)` takes. It's what the PID multiplies by `kP` to get the output.

They are **not the same input**. Three patterns appear in Genesis code:

### (a) Per-motion locked scheduling
Motion calls `setKp(initialError)` **once at start**. The `kP` is fixed for the whole motion, based on the original task size.
```cpp
// turnHeadingPlus (src/genesis/chassis/motions/motionPlus.cpp:473)
turnControllerPlus.reset(angleErrorPlus(target, currentHeadingPlus(*this)));
turnControllerPlus.setKp(sides == 2 ? angleErrorPlus(target, currentHeadingPlus(*this))
                                    : angleErrorPlus(target, currentHeadingPlus(*this)) / 2.5f);
```

### (b) Per-tick live scheduling
Motion calls `setKp(currentError)` **inside the loop**. `kP` changes as the error shrinks. This is what `headingCorrectControllerPlus` does inside lateral motions like `moveDistPlus`.

### (c) Decoupled / hardcoded — effectively turns scheduling off
Motion calls `setKp(literal_constant)`. `kP` is fixed at the curve's value at that point, regardless of actual error.
```cpp
// movePointPlus (src/genesis/chassis/motions/motionPlus.cpp:551)
lateralControllerPlus.reset(pose.distance(end));
lateralControllerPlus.setKp(1);     // ← hardcoded 1 — locks kP near i
turnControllerPlus.reset(facePlus(pose, end));
turnControllerPlus.setKp(180);      // ← hardcoded 180 — locks kP near f
```

If you remember nothing else from this page: **`setKp` picks which gain to use; `tick` consumes the error. They are different jobs that happen to take numbers in the same units.**

---

## 6. How the reference team actually uses it (the reality check)

Two surprises hidden in the actual codebase:

**Only the turn axis uses scheduling.** From `MotionPlusTuning::apply()` in `src/main.cpp`:

| Axis | i | f | k | p | Scheduling active? |
|---|---:|---:|---:|---:|---|
| Lateral | 6500 | 6500 | 1 | 1 | **No** — `i == f`, formula collapses to constant 6500 |
| Turn | 450 | 220 | 28 | 1.5 | **Yes** |
| Heading-correct | 200 | 200 | 1 | 1 | **No** — same collapse |

With `i == f`, the formula always returns `i` no matter what. The machinery is built for all three controllers but only enabled on turns.

**Most "Plus" motions don't exploit scheduling either.** `movePointPlus` and `movePosePlus` hardcode `setKp(1)` / `setKp(180)` (case (c) above) — so for those motions, the asymptotic curve is just a fancy way to pick a constant. The motions that genuinely benefit are `turnHeadingPlus`, `turnPointPlus`, and the per-tick `headingCorrectControllerPlus` calls inside lateral moves.

So: the team built the full system, configured it for three controllers, and only really used it on **one and a half**.

---

## 7. The tuning interface

Per axis, eight numbers — four reshape the gain curve, four reshape the PID's integral/derivative. From `src/genesis/chassis/motions/motionPlus.cpp` (~line 213):

```cpp
void Chassis::setTurnPIDPlus(float kPInitial, float kPFinal, float kPKnee, float kPPower,
                             float kI, float kD,
                             float integralDeadband, bool integralSignReset) {
    turnKpPlus.configure(kPInitial, kPFinal, kPKnee, kPPower);
    turnControllerPlus.configure(kI, kD, integralDeadband, integralSignReset);
}
```

(`setLateralPIDPlus` and `setHeadingCorrectPIDPlus` have identical shapes against their own gain objects.)

The team calls these **once**, in `MotionPlusTuning::apply()` from `initialize()`. Even though the API allows mid-auton retuning, they never do that in practice.

---

## 8. When (if ever) to adopt this in our library

**Default answer: don't.** WinLib v1 ships one constant `kP` per axis. Try it on the actual robot first.

Only consider scheduling if you genuinely see this pattern:
- Set `kP` high enough for 1° corrections to settle → big swings overshoot
- Set `kP` low enough for big swings to look clean → small corrections sit there doing nothing

And even then, **try a two-piece map first**:

```cpp
float effectiveKp = (std::fabs(error) < threshold) ? kPSmall : kPLarge;
```

Two numbers and an `if`. A student can read it, tune it, and reason about it without a graphing calculator. The full asymptotic formula has **four** parameters that all interact, and you really do need to plot the curve to understand what a tweak does. That's a lot of complexity for one axis (turns) that the reference team's lateral motions don't even use.

**Only reach for the full asymptotic system** if a two-piece map proves too coarse — for example, if you actually need a smooth `kP` ramp across many setpoint sizes and the discontinuity at the threshold causes visible jerking.

---

## 9. What we explicitly chose NOT to port from Plus, and why

- **The `AsymptoticGainsPlus`-reference-from-`PIDPlus` indirection.** Powerful — change one gains object and every PID using it updates. But it adds a class, a reference field, and a `setKp(setpoint)` call site for a feature only one axis uses. Cost > benefit at this stage.
- **The `setLateralPIDPlus` / `setTurnPIDPlus` / `setHeadingCorrectPIDPlus` reconfigure API.** WinLib already gets the same flexibility "for free" by leaving `PID::kP`, `kI`, `kD` non-const — you can just assign to them. One less API surface to maintain.
- **The `ExitPlus` static-predicate exit system.** WinLib kept the classic stateful `ExitCondition` with `getExit() == true means done`. Easier polarity to reason about for a student than `static bool error(...)` calls that return true the *instant* the condition is met without any dwell.
- **The `MotionPlusTuning::apply()` config block pattern.** Tuning numbers in WinLib live directly in `ControllerSettings` in `config.cpp` — one config location, not two. If you have to look in two places to find the gains, you'll forget the second place exists.

---

## Verification note

Specific facts on this page were checked against the Genesis repo on the date of writing:
- Class declarations: `include/genesis/motionPlus.hpp` (verified verbatim)
- Function bodies: `src/genesis/motionPlus.cpp` (constructor, `getGain`, `tick`, `setKp`, `reset`, `configure` — verified verbatim)
- Motion call sites: `src/genesis/chassis/motions/motionPlus.cpp` (`setLateralPIDPlus` ~line 213, the `setKp` lines in `moveDistPlus`/`movePointPlus`/`turnHeadingPlus` — verified)
- Tuning numbers: `src/main.cpp` (`MotionPlusTuning::apply()` lateral 6500/6500/1/1, turn 450/220/28/1.5, heading 200/200/1/1 — verified)

If you're reading this in a future season and Genesis has moved on, re-check before trusting the line numbers.
