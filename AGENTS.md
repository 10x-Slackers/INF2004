# AGENTS

## Project context

This is a **university coursework project** done by undergraduates. It is not a production system and will never be deployed to real users.

It is an autonomous robotic car built in C on µT-Kernel 3.0 (RP2040 dual-core SMP port) for the Raspberry Pi Pico W, built with GNU make from `build_make/`.

## Research First

- Always research before implementing a task — both the codebase and online. Understand existing patterns, dependencies, and conventions before writing code.
- Refer to the latest documentation and existing implementations of any libraries or frameworks in use.
- Check `docs/` for architectural and planning details relevant to the task. Start from the index in `docs/README.md`.

## Ask When Unsure

- Clarify ambiguities before proceeding — ask rather than guess.
- When in doubt about scope, priorities, or critical decisions, surface the uncertainty and let the user decide.

## Engineering posture

Optimize for **low line count, readability, and simplicity** — not for production hardening. When a tradeoff exists:

- **Prefer fewer lines of code.** Skip defensive checks, input validation, retry logic, and abstractions that a production codebase would require.
- **Prefer readability.** A short, obvious solution beats a clever, dense one.
- **Security and robustness may be traded for simplicity.** It is acceptable to a certain degree to trust inputs, skip edge-case handling, and omit safeguards (rate limiting, sanitization layers, error boundaries, etc.) if removing them makes the code clearer and shorter.
- **Don't over-engineer.** No speculative extensibility, no config layers for options nobody uses yet, no premature abstraction.

## Conventions

- The repository is based on the course template (`upstream` remote, https://github.com/sirfonzie/INF2004_ProjectTemplate). Its `README.md` explains the build options and the `app_program/` layout.
- Application code lives in `app_program/`, one folder per subsystem (`subsystemN/`), with drivers as `drv_<name>.c/.h` inside the owning subsystem's folder. It follows the Barr C Coding Standard. Layout is enforced by `.clang-format` through pre-commit.
- Kernel sources (`kernel/`, `lib/`, `device/`, `include/`) keep their upstream style. Do not reformat them.
- Follow the task priorities, hardware ownership and pin map in `docs/ARCHITECTURE.md` and `docs/HARDWARE.md`, and record pins, priorities and interfaces in `TEAM.md`. Update those files when a change alters them.
- Record tests in `testing/` as described in `testing/README.md`.
- Keep docs timeless. Do not write current-state or progress information into them.

## Checks

- After every task, check that the firmware still builds without new warnings. From the repository root:

```sh
make -C build_make -j$(nproc)
make -C build_make TEST=1 -j$(nproc)
```
