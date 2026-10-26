"""Rebuild identical logic with full-width reference fields, not old firmware."""
from pathlib import Path
import os
import re
import shlex
import shutil
import subprocess

root = Path(__file__).resolve().parents[3]
reference = root / "build/host/reference"
source = reference / "app_program/subsystem2"
source.mkdir(parents=True, exist_ok=True)
for original in (root / "app_program/subsystem2").glob("*.[ch]"):
    content = re.sub(r"unsigned int (\w+)\s*:\s*\d+;",
                     r"uint32_t \1;", original.read_text())
    (source / original.name).write_text(content)
test_dir = reference / "testing/subsystem2/host"
test_dir.mkdir(parents=True, exist_ok=True)
shutil.copyfile(root / "testing/subsystem2/host/test_motion.c",
                test_dir / "test_motion.c")
command = shlex.split(os.environ.get("HOST_CC", "cc"))
command += ["-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-pthread", "-O2"]
command += shlex.split(os.environ.get("HOST_CFLAGS", ""))
command += [str(source / name) for name in
            ["motion_control.c", "ctrl_math.c", "console_motion.c"]]
command += [str(test_dir / "test_motion.c"), "-o", str(reference / "test")]
subprocess.run(command, check=True)
print("FULL-WIDTH REFERENCE: identical algorithms, unsigned fields widened to "
      "uint32_t; not a historical firmware baseline.", flush=True)
subprocess.run([str(reference / "test")], check=True)

# Optional ARM comparison retains the framework unchanged and varies only
# Buddy 2 application storage. All objects are built in a separate tree.
import sys
if "--firmware" in sys.argv:
    firmware_root = root / "build/representation-reference"
    firmware_root.mkdir(parents=True, exist_ok=True)
    for child in root.iterdir():
        if child.name not in {"build", ".git", "app_program"}:
            target = firmware_root / child.name
            if not target.exists():
                target.symlink_to(child, target_is_directory=child.is_dir())
    shutil.copytree(root / "app_program", firmware_root / "app_program",
                    dirs_exist_ok=True)
    for path in (firmware_root / "app_program/subsystem2").glob("*.[ch]"):
        path.write_text(re.sub(r"unsigned int (\w+)\s*:\s*\d+;",
                               r"uint32_t \1;", path.read_text()))
    profile = "smp1-usb_cdc-cyw43-none-mqtt0-crash0-test0"
    image = "mtk3pico_smp1_usb_cdc_wifi.elf"
    for label, directory in [("packed", root),
                              ("full-width reference", firmware_root)]:
        log = root / "build/host" / (label.replace(" ", "-") + ".log")
        with log.open("w") as output:
            result = subprocess.run(
                ["make", "-j8", "SMP=1", "TEST=0", "CONSOLE=usb_cdc",
                 "WIFI=cyw43", "NET=none", "MQTT=0", "CRASH_REPORT=0"],
                cwd=directory, stdout=output, stderr=subprocess.STDOUT)
        if result.returncode:
            print(log.read_text()[-4000:])
            raise SystemExit(result.returncode)
        print(label, flush=True)
        subprocess.run(["arm-none-eabi-size",
                        str(directory / "build" / profile / image)],
                       check=True)
