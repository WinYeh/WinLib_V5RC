# Claude Agent Guidelines for WinLib Project

## About the User

- **Name:** Winyeh (inferred from path `/Users/winyeh/`)
- **Experience Level:** High school student, transitioning from VEXcode to PROS
- **Background:** 2 seasons of VEXcode experience; switching to PROS because competitive top teams use it for its advantages. This is their first PROS project.

## Project Context

This is a VEX robotics project using the PROS framework, structured with two intentional layers:

1. **WinLib Library Layer** — reusable, robot-agnostic code (sensors, drivetrain logic, utilities)
   - Lives in `include/WinLib/` and related library source files
   - Should be kept independent from any specific robot configuration

2. **Robot Application Layer** — robot-specific code that uses WinLib
   - Lives in `src/main.cpp`, `src/config.cpp`, `include/config.h`
   - Wires up hardware (ports, motors, sensors) and calls WinLib functions

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

### Things to Avoid
- Do not use dense academic language
- Do not assume prior knowledge of C++ concepts without explaining them first
- Do not skip analogies when the user asks for them
