# AGENTS

The main guide for working in this repository, for people and coding agents alike.

## Project context

This is a **university coursework project** done by undergraduates. It is not a production system and will never be deployed to real users.

It is an autonomous robotic car built in C on µT-Kernel 3.0 (RP2040 dual-core SMP port) for the Raspberry Pi Pico W, built with GNU make from `build_make/`. Set-up is in `README.md`; the design and reference docs are indexed in `docs/README.md`.

## Research

- Before implementing, read the code and `docs/` relevant to the task so changes follow existing patterns. Scale this to the task; a small fix needs only the files it touches.
- Check µT-Kernel, Pico SDK and hardware APIs against their documentation or the vendored sources rather than memory.

## When to ask

- Ask before deciding anything that changes scope, priorities, the design in `docs/`, or is hard to reverse.
- For smaller ambiguities, pick the option most consistent with the existing code and state the assumption in your summary.

## Engineering posture

Optimize for **low line count, readability, and simplicity**, not production hardening. When a tradeoff exists:

- **Prefer fewer lines of code.** Skip defensive checks, input validation, retry logic, and abstractions a production codebase would require.
- **Prefer readability.** A short, obvious solution beats a clever, dense one.
- **Security and robustness may be traded for simplicity.** Within reason, trust inputs, skip edge-case handling and omit safeguards if that makes the code clearer and shorter.
- **Don't over-engineer.** No speculative extensibility, no config layers for options nobody uses yet, no premature abstraction.

## Ownership

- Each subsystem (`app_program/subsystemN/`, `testing/subsystemN/`) belongs to one student (`TEAM.md` section 1). Change another student's subsystem through a pull request they review.
- Shared files (`app_program/app_main.c`, `app.h`, `app_fault.h`, `app_program/shared/`, `TEAM.md`) change by team agreement.
- Claim pins, peripherals, task priorities and interfaces in `TEAM.md` before coding them, and keep `docs/ARCHITECTURE.md` and `docs/HARDWARE.md` in step when a change alters them.

## Code conventions

- Application code lives in `app_program/`, one folder per subsystem, with drivers as `drv_<name>.c/.h` in the owning subsystem's folder. `docs/DEVELOPMENT.md` shows how to add a task and which kernel objects to use.
- Follow the Barr C Coding Standard: braces on every body, fixed-width integer types, named constants, module prefixes on public names. The deviations all subsystems share are listed at the top of `app_program/app.h`; a subsystem lists its own in its header. Layout is applied by clang-format through pre-commit (`pre-commit install` once).
- Each peripheral has one owning task; other tasks get its data through the owner's interface. Motors are driven only through the motion API.
- Size application data to its range (Change Requirement 1) and list each field in `docs/DATA_REPRESENTATION.md`.
- Use `app_start_cyclic()` for periodic tasks: `tk_dly_tsk()` adds a kernel tick (10 ms) to every wait.
- Kernel sources (`kernel/`, `lib/`, `device/`, `include/`) keep their upstream style. Do not reformat them.
- Keep docs timeless. Do not write current-state or progress information into them.

## Tests

- Add a test where it catches something: a regression, or logic that is easy to get wrong. Automate it with `TEST_CHECK` in `subsystemN_test.c`, or as a host program in `testing/subsystemN/host/`, when it can be.
- Record the tests that matter as evidence in `testing/` (see `testing/README.md`): what was run, on which commit, and the result, with a log when there is console output worth keeping.

## Checks

After each change, check that both images still build without new warnings. From the repository root:

```sh
make -C build_make -j$(nproc)
make -C build_make TEST=1 -j$(nproc)
```
