"""Rebuild identical logic with full-width reference fields, not old firmware."""
from pathlib import Path
import os
import re
import shlex
import shutil
import tempfile
import subprocess

root = Path(__file__).resolve().parents[3]
with tempfile.TemporaryDirectory(prefix="s2-reference-") as folder:
    reference = Path(folder)
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

    # Compare real ARM static storage for the same portable application modules.
    # This is a representation baseline, never a claim of historical hardware.
    compiler = shutil.which("arm-none-eabi-gcc")
    size_tool = shutil.which("arm-none-eabi-size")
    if compiler and size_tool:
        for label, directory in [("packed", root / "app_program/subsystem2"),
                                 ("full-width", source)]:
            total = 0
            for name in ["motion_control", "ctrl_math", "console_motion"]:
                obj = reference / (label + "-" + name + ".o")
                subprocess.run([compiler, "-std=c99", "-O2", "-mcpu=cortex-m0plus",
                                "-mthumb", "-Wall", "-Wextra", "-Werror", "-c",
                                str(directory / (name + ".c")), "-o", str(obj)],
                               check=True)
                fields = subprocess.check_output([size_tool, str(obj)],
                                                 text=True).splitlines()[1].split()
                total += int(fields[1]) + int(fields[2])
            print(f"ARM {label}: control/math/console static RAM = {total} bytes")
    else:
        raise RuntimeError("ARM compiler and size tool required for RAM comparison")
