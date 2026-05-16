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

### Reference Project
- Based on study of team 78181A Genesis's repo: https://github.com/NicksonC1/78181A-Push-Back
- That repo uses a LemLib fork ("Genesis" library) with heavy OOP (Chassis class, DriveCurve hierarchy, motion queue, etc.)
- WinLib intentionally simplifies this for teachability

### Design Constraints
- **Minimal OOP:** Classes are allowed only for small, self-contained utilities (PID, Timer, ExitCondition, Pose). No Chassis class. No class inheritance hierarchies.
- **Free functions for new code:** Odometry and movement functions should be namespace-level free functions, not class methods.
- **Lateral + Angular motions only:** No curvature motions, boomerang controller, or pure pursuit — these are too complex for the target audience to understand and modify.
- **Blocking motions:** Movement functions run synchronously (no async tasks, no motion queue). Students can read autonomous routes top-to-bottom.
- **Teachability over performance:** Every design choice should prioritize "can a 10th grader understand this?" over competitive optimization.

### Planned Architecture (Files to Add)
- `include/WinLib/odom.hpp` + `src/WinLib/odom.cpp` — Odometry system (free functions: init, update, getPose, setPose)
- `include/WinLib/movement.hpp` + `src/WinLib/movement.cpp` — Movement functions (free functions: moveTo, moveFor, turnTo, turnToPoint)

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
- Use free functions in the `WinLib` namespace for all new code (no new classes)
- Movement functions should be straightforward: create PID, loop until done, stop motors

### Things to Avoid
- Do not use dense academic language
- Do not assume prior knowledge of C++ concepts without explaining them first
- Do not skip analogies when the user asks for them
- Do not introduce new classes — use free functions with module-level static state
- Do not add curvature/boomerang/pure pursuit motions unless explicitly asked
- Do not add async motions or motion queues
