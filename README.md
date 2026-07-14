# WinLib

**A teaching-focused VEX robotics library for PROS.**

WinLib is a small, readable C++ library that gives a VEX V5 robot the two things
every competitive robot needs: it can **drive itself** during the autonomous
period (turn to angles, drive to points on the field, follow walls, swing around
obstacles) and it can **track where it is** on the field (odometry). It is built
on top of [PROS](https://pros.cs.purdue.edu/), the professional C++ framework
that top VEX teams use instead of the block-based VEXcode.

> **The whole point of WinLib is that you can read it.** It is written for a high
> school team stepping into PROS for the first time. Every design choice asks one
> question — *"can a 10th grader understand this?"* — and picks understandable
> over clever. WinLib is not a black box you press "go" on; it is a recipe book
> you are meant to open, read top-to-bottom, and modify yourself.

**[Analogy]** Most robot libraries are like a microwave: you press a button and
food gets hot, but you have no idea what happens inside. WinLib is more like a
recipe card taped to the fridge — it does the same job, but you can *read* every
step, change the ingredients, and learn to cook on your own.

---

## Who this is for

A VEX team entering their **first PROS season**, coming from VEXcode. You should
be comfortable with the *idea* of code (variables, loops, functions) but you do
**not** need to know C++ deeply yet — the code and the guides explain the C++ as
they go.

## The two layers

WinLib is split into two clean halves so the reusable brain stays separate from
the one specific robot it happens to be bolted into:

| Layer | Lives in | What it is |
|-------|----------|------------|
| **WinLib library** | `include/WinLib/`, `src/WinLib/` | Robot-**agnostic** code: PID, odometry, the `Chassis` class and all its movements. Knows nothing about your specific ports. |
| **Robot application** | `config.h`, `config.cpp`, `main.cpp` | Robot-**specific** wiring: which motor is on which port, wheel sizes, PID gains, and the autonomous routes you write. |

Each robot is declared in its own namespace (`test`, `dr4b`, `ace`, …) with a
complete, independent device set and its own `WinLib::Chassis`. A single global
pointer, `Chs`, aims at whichever robot is active this run — you set it once in
`initialize()` with `setActiveChassis(...)`, and the library reads through it.

**[Analogy]** The library layer is a **driver** who knows how to drive *any* car.
The application layer is the **specific car** — its engine, its mirror positions,
its quirks. `Chs` is you pointing at one car in the parking lot and saying "that's
the one I'm driving today."

## What WinLib can do

**Autonomous motions** (all blocking — they run top-to-bottom so a route reads
like a to-do list):

| Motion | What it does |
|--------|--------------|
| `moveFor` | Drive a set distance while holding a heading (straight line). |
| `turnToHeading` | Turn in place to an absolute compass heading. |
| `turnBy` | Turn in place by a relative number of degrees (can exceed 360°). |
| `moveToPoint` | Drive to an (x, y) point on the field. Uses odometry. |
| `moveToPose` | Drive to an (x, y) point **and** arrive facing a heading (the "boomerang" controller). Uses odometry. |
| `moveByWall` | Drive along a wall at a fixed distance, using a distance sensor. |
| `swingToHeading` / `swingToPoint` | Swing turns — lock one side of the drive and pivot around it, turning *and* moving at once. |

**Driver control:** `arcade` — one-stick-throttle, one-stick-turn joystick drive.

**Odometry:** a background task that fuses tracking wheels + IMU (two selectable
algorithms) to always know the robot's (x, y, heading). Plus **DSR** — a
"touch the wall" position reset using distance sensors to erase drift.

## Getting started

New to the library? **Start with the student website** — a friendly, example-first
walkthrough of building your own project with WinLib (declare a robot, write your
first autonomous route, drive in opcontrol, tune the PID):

> 🌐 **Student guide:** https://claude.ai/code/artifact/7eb1c665-8dbe-47ad-9f4b-f33de6e039d4
> *(private by default — open it and use the page's share menu to share it with the team)*

Then dig into these in-repo docs:

- **[`CLAUDE.md`](CLAUDE.md)** — the design bible: every architectural rule and
  *why* it exists.
- **[`docs/future/roadmap.md`](docs/future/roadmap.md)** — what is built, what is
  coming, and in what order.
- **[`CHANGES_FROM_REFERENCE.md`](CHANGES_FROM_REFERENCE.md)** — every place WinLib
  intentionally differs from its reference libraries (Genesis 78181A / LemLib),
  with the reasoning.

## Built on the shoulders of

WinLib was written by studying two excellent libraries and deliberately
*simplifying* them for teachability:

- **[LemLib](https://github.com/LemLib/LemLib)** — the upstream open-source library.
- **[Genesis (78181A)](https://github.com/NicksonC1/78181A-Push-Back)** — a
  competitive fork of LemLib.

WinLib keeps the good ideas (a `Chassis` API, odometry, the boomerang controller,
swing turns) and drops the heavy machinery (async motion queues, pure-pursuit path
following, deep class hierarchies) that would get in a first-year student's way.

## Status

Pre-release (v0.0.1), actively developed for the 2026–2027 Override season. Core
motions and odometry are implemented; several features are still being verified on
the real robot. See the [roadmap](docs/future/roadmap.md) for the live status of
every piece.
