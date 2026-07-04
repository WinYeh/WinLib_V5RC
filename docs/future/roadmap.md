# WinLib — Future Map (Roadmap)

> **Status: Living roadmap.** Some items here have already shipped — each near-term item now carries its own **Status:** line (✅ built · 🔨 in progress · ⬜ not started · ❌ dropped). This is the "where WinLib is headed" map so that whoever picks the library up next season knows what's done, what's coming, in roughly what order, and *why* each piece matters. Items are listed in **development priority** — closer to the top = sooner and more certain; closer to the bottom = later and less certain.

This page is a map, not a manual. Each entry is a short "what it is / why we want it / how it should work." When one of these actually gets built, it earns its own detailed page (like [`pidplus_and_asymptotic_gains.md`](pidplus_and_asymptotic_gains.md) in this same folder) and the entry here shrinks to a one-line "→ done, see that page."

**[Analogy]** Think of this like the "coming soon" poster outside a movie theater. The posters tell you what's on the way and roughly when, but you can't buy a ticket yet. Some films are already filming (near the top); some are just an idea a director pitched (near the bottom).

---

## Part 1 — Near-term (the current season's work)

These are the features we expect to build *soon*, because real autonomous routes will need them. They mostly fall into three buckets: **finish the motions**, **make odometry trustworthy**, and **write it all down clearly**.

### 1. Verify the Boomerang controller through stress testing

> **Status: ✅ Built & first-pass verified.** `moveToPoint` and `moveToPose` are implemented (`chassis/movement/moveToPoint.cpp`, `moveToPose.cpp`) and confirmed working on the real robot (commit *"tested & verified that both moveToPoint & moveToPose functions are ready to be used"*). The `boomerang` name was renamed to `moveToPose` to match LemLib. **Still ongoing:** wider cross-position stress testing as real routes get written — one clean run isn't the same as reliable-from-every-corner (see the paper-airplane analogy below).

`moveToPoint` and `moveToPose` are the motions that steer by odometry **position** (x, y) instead of just encoder distance + IMU heading. Before we trust them in a match, they need **stress testing** — running them over and over, from many start positions, and measuring how far off the robot ends up.

- **`moveToPoint`** — drive to an (x, y) target, heading is whatever falls out of the drive.
- **`moveToPose`** — drive to an (x, y) target *and* arrive facing a specific heading (this is the Boomerang / "carrot point" motion).

Why "stress test" and not just "test once"? Because these motions **read odometry position every tick**, so any drift in the odom math shows up directly as the robot missing its target. A motion that works once from the origin can fail badly from the far corner. We only trust it after it lands accurately, repeatedly, from everywhere.

**[Analogy]** It's like testing a paper-airplane design. Throwing it once and watching it fly straight doesn't prove anything — maybe you got lucky. You throw it twenty times, from different spots, and only then do you decide the design is good.

> **Depends on:** trustworthy odometry (see #3). If the pose is drifting, you can't tell whether a missed target is the *motion's* fault or the *odom's* fault. That's why odom stability is listed as its own bucket.

### 2. ~~Move tuning parameters into the config files first~~ — DROPPED

> **Status: ❌ No longer required.** The current split — baseline gains in `config.cpp`'s `ControllerSettings`, per-move overrides at the call site — turned out to be fine in practice. Consolidating everything into config isn't worth doing; this item is closed and won't be picked up.

~~Right now some motion parameters (max speed, min speed, exit ranges, PID gains) are scattered — some in `config.cpp`'s `ControllerSettings`, some passed inline at the call site in `main.cpp`. The plan: **every default lives in the config file, in one place**, so a student tunes a robot by editing one file.~~

### 3. Solve the odometry-instability problems

> **Status: 🔨 Both pieces built; verification ongoing.** DSR and `moveByWall` are both implemented. What's left is the *trust* work — confirming they actually keep the pose honest across a full match, same stress-testing the motions need.

Odometry drifts. Wheels slip, sensors jitter, and small errors add up over a 15-second match until the robot *thinks* it's somewhere it isn't. Two features attack this directly:

- **DSR (Distance Sensor Reset)** — ✅ *built* (`chassis/DSR.hpp` / `DSR.cpp`). It uses the four distance sensors + IMU heading to figure out which walls the robot sees and **snaps the odom (x, y) back to the truth**. This is the big lever for fixing drift. It still needs the same stress-testing treatment as the motions.
- **`moveByWall`** — ✅ *built* (`chassis/movement/moveByWall.cpp`; declared on `Chassis` with a `WallSide` enum + `WallParams`). Drive until the robot is a known standoff distance from a chosen wall, then use that wall contact as a hard, physical "you are exactly here" reset. Walls don't drift — they're the most reliable reference point on the field.
  - **Reference:** modeled on WinYeh's own **Push Back** season repo — [WinYeh/PushBack_14683A](https://github.com/WinYeh/PushBack_14683A) (Winyeh's team 14683A repo from the Push Back season, *not* the 78181A Genesis reference).

**[Analogy]** Imagine walking across a dark room with your eyes closed, counting steps to guess where you are (that's raw odometry — the guess drifts a little with every step). Now imagine you reach out and touch the wall. Instantly you know *exactly* where you are, no guessing. DSR and `moveByWall` are both "touch the wall" moves for the robot.

### 4. `moveUntilVolt`

> **Status: ⬜ Not started.** No `moveUntilVolt` declaration or `.cpp` yet.

A motion that drives forward until the motors have to push *hard* — i.e. the voltage/current the motors draw spikes because the robot ran into something solid (a wall, a stack of blocks, a mobile goal). Instead of "drive exactly 500 mm," you say "drive forward until you hit something," which is perfect for aligning against a physical object.

**[Analogy]** Like pushing a shopping cart forward with your eyes closed until it bumps the checkout counter. You don't measure the distance — you just push until you feel resistance, and now you're lined up against the counter.

### 5. `swingToPoint` / `swingToHeading`

> **Status: ✅ Done.** Implemented as `chassis/movement/swingToHeading.cpp` / `swingToPoint.cpp`, modeled on LemLib. Reuse `AngularParams` + `angularSettings`; a `DriveSide` enum names the locked side; the locked side is brake-held while the free side is driven by the angular PID. See `CHANGES_FROM_REFERENCE.md`.

A **swing turn** locks one side of the drivetrain and only powers the other, so the robot pivots around a stationary wheel instead of spinning in place. It carves a wide arc. Useful when you want to change heading *and* move around an obstacle at the same time.

- `swingToHeading` — swing until facing an absolute heading.
- `swingToPoint` — swing until facing an (x, y) target.

- **Reference:** model these on **LemLib**'s swing motions (`swingToHeading` / `swingToPoint`), not Genesis. LemLib is the upstream library Genesis itself forked from, and its swing implementation is the cleaner one to learn from.

**[Analogy]** A normal turn (`turnToHeading`) is spinning in place like a figure skater. A swing turn is like paddling a canoe on one side only — you both turn *and* drift forward around the paddle.

### 6. Route examination — the two ways to chain motions

> **Status: 🔨 Mechanisms exist; guidance ongoing.** Both hand-off styles already work (`ExitCondition` and the `minSpeed` + `earlyExitRange` early-exit). What's left is writing real routes and settling the "when to use which" guidance below into practice.

When you write an autonomous *route*, you string many motions back-to-back. How one motion "hands off" to the next changes how smooth and how reliable the route is. WinLib supports two hand-off styles, and a route author should understand the trade-off:

- **Exit-condition chaining (more consistent).** Each motion runs until its `ExitCondition` says "close enough, and I've *stayed* close enough for a moment," then stops fully before the next begins. Predictable and repeatable, at the cost of a tiny pause between moves.
- **`minSpeed` chaining (faster, less consistent).** By giving a motion a `minSpeed` floor and an `earlyExitRange`, the robot bails out of the motion *early while still moving*, carrying its momentum into the next move. Smoother and quicker, but harder to make land in exactly the same spot every time.

The takeaway for route authors: **start with exit-condition chaining** to get a route that *works reliably*, then selectively switch specific hand-offs to `minSpeed` where you need the speed and can afford the variability.

**[Analogy]** Exit-condition chaining is coming to a **full stop** at each stop sign before driving on — safe and predictable. `minSpeed` chaining is a **rolling stop** — faster, keeps your momentum, but you have less control and it won't be identical every time.

### 7. TLDR / Documentation page for WinLib

> **Status: ⬜ Not started.** No getting-started page yet. Arguably the most important remaining near-term item (see below).

A single **crystal-clear** getting-started page written for a 10th grader who has never touched this library. Not a reference dump — a friendly tour: here's how you declare a robot, here's how you write an autonomous route, here's how you drive in opcontrol, here are the five motions you'll actually use. Short, concrete, example-first.

This is arguably the **most important** near-term item, because the whole point of WinLib is that the team can *understand and maintain it themselves*. A library nobody can read is a black box, and black boxes are exactly what we're avoiding.

**[Analogy]** Like the one-page "quick start" sheet that comes on top of the thick manual in a new appliance box. Nobody reads the 90-page manual first — they read the single page that says "do these three things and it works."

---

## Part 2 — Long-term (later seasons, "most likely → most unlikely")

These are bigger, further-off ideas. They're ordered by how likely we are to actually build them, most likely first. None of these should be started without Winyeh explicitly asking — several are flagged as *deferred* in CLAUDE.md for good reasons.

### A. LVGL screen UI (most likely of the long-shots)

Use **LVGL** (the graphics library already vendored in `include/liblvgl/`) to build a real on-brain-screen interface — imagined as something like a **phone's control panel**: tap tiles to pick the autonomous route, toggle settings, watch live sensor readouts. Right now the brain screen just prints text lines.

**[Analogy]** Turning the robot's little screen from a plain typed-out receipt into the touch-screen home screen of a phone, with buttons you can actually tap.

### B. Command-based programming

A system for describing autonomous routes as composable "commands" that can run in sequence *or in parallel* (e.g. "drive to the goal **while** spinning up the intake"). This is a large architectural addition.

> **Explicitly deferred in CLAUDE.md.** The plan is to first write real routes the simple, blocking way (plus a `pros::Task` for parallel subsystem work) and only revisit command-based infrastructure if that experience proves it's genuinely needed. **Do not build this unless Winyeh explicitly asks.**

**[Analogy]** Right now a route is a recipe you follow one step at a time, top to bottom. Command-based programming is like a head chef who can say "start the sauce, and *at the same time* get someone chopping onions" — steps that overlap and combine. Powerful, but a much more complicated kitchen to run.

### C. PID tuning graph (least likely)

A live graph of the PID controller's error over time, so tuning is *visual* instead of squinting at printed numbers. Two possible paths: mimic a **MATLAB**-style plot, or actually pipe the robot's telemetry out to MATLAB (or another plotting tool) over a serial connection.

**[Analogy]** Tuning PID by reading printed numbers is like adjusting a shower's temperature blindfolded, one small turn at a time, feeling with your hand. A tuning graph is taking the blindfold off and *watching* a thermometer — you see instantly whether your last tweak helped or overshot.

---

## How to keep this map honest

- When a Part 1 item ships, replace its section body with a one-liner pointing to its real doc + code, and (if it changed a public API or diverged from the reference) log it in `CHANGES_FROM_REFERENCE.md`.
- If a long-term item gets promoted (e.g. Winyeh decides to build command-based programming), move it up into Part 1 and remove the "deferred" warning only after CLAUDE.md's Deferred Decisions section is updated to match. The two documents must not disagree.
- Re-order Part 2 whenever the "most likely → most unlikely" ranking changes. It's a living guess, not a contract.
