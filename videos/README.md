# Videos

Short videos of tests and demos on the real car. A video shows what a log cannot: the car following the line, stopping, passing an obstacle. Only record one when the log alone doesn't show that the test passed. Auto, host and RAM tests, and anything fully shown by console output, don't need one.

## What to record

- **Short:** 15–60 seconds, one test or one feature per video.
- **Say what it is:** show or say the test ID (`SYS-08`, `INT-09`, …) at the start.
- **Show the commit:** run `git rev-parse --short HEAD` before recording and keep it in shot for a moment, or put it in the table below.
- **Show the result:** keep the car and, if it matters, the console or dashboard in view.
- **Landscape**, steady, good light.

## How to share it: unlisted YouTube only

1. Upload to YouTube with visibility **Unlisted**: anyone with the link can watch, but it doesn't show up in search. Not *Private* (the link won't work for the team or the instructor) and not *Public*.
2. Copy the link (`https://youtu.be/...`).
3. Add a row to the table below, and put the same link in the *Log / evidence* column of the test's result row in `testing/.../TEST_LOG.md`.

Don't commit video files or attach them on GitHub: a video stays in the git history for good and slows every clone. `.gitignore` ignores common video files for this reason.

## Video index

Newest at the bottom.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
