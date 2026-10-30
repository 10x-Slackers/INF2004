# Documentation

Reference and guides for the INF2004 autonomous robotic car (µT-Kernel 3.0 on Raspberry Pi Pico W + Cytron Robo Pico).

| Document                                         | Read it when you need                                                                            |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------------ |
| [PROJECT_BRIEF.md](PROJECT_BRIEF.md)             | What the car must do, how it is assessed, who owns what, and the team's requirement IDs (FR/NFR) |
| [HARDWARE.md](HARDWARE.md)                       | The pin map, wiring, electrical limits and a quick reference for each sensor and actuator        |
| [ARCHITECTURE.md](ARCHITECTURE.md)               | The RTOS task set, hardware ownership, inter-task interfaces and the statecharts                 |
| [MOTION_CONTROL.md](MOTION_CONTROL.md) | Buddy 2 requirement mapping, timing and binary contract |
| [DEVELOPMENT.md](DEVELOPMENT.md)                 | How to build, flash, add a task, and the conventions application code follows                    |
| [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md) | The minimum-bit rules for application data and the table of every field's range and bits         |

These describe this project and are derived from the course brief, the Week 6 design review and the component datasheets. The course template's own guide (build options, `app_program/` layout) is the [root README](../README.md); team ownership is in [TEAM.md](../TEAM.md) and test records in [testing/](../testing/README.md).

Buddy 2: [operating guide](../README.md#buddy-2--motion-control).
