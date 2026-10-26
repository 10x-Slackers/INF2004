"""Focused source checks; this is not a complete BARR-C compliance checker."""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[3]
failures = []
for path in sorted((root / "app_program/subsystem2").glob("*.[ch]")):
    text = path.read_text()
    for number, line in enumerate(text.splitlines(), 1):
        if "\t" in line or len(line) > 80 or line.rstrip() != line:
            failures.append(f"{path.name}:{number}: layout")
    code = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)
    if re.search(r"\b(?:signed\s+int|int\d+_t)\s+\w+\s*:", code):
        failures.append(f"{path.name}: signed bit-field")
    if re.search(r"\b(?:float|double|goto|continue|auto|register)\b", code):
        failures.append(f"{path.name}: forbidden keyword or real-time float")
    if re.search(r"\b(?:malloc|calloc|realloc|free|atoi|sprintf|gets)\s*\(", code):
        failures.append(f"{path.name}: forbidden function")
    if not text.endswith("/* End of file. */\n"):
        failures.append(f"{path.name}: missing end marker")
    if path.suffix == ".h" and "#ifndef " not in text:
        failures.append(f"{path.name}: missing header guard")

if failures:
    print("\n".join(failures))
    sys.exit(1)
print("PASS: Buddy 2 layout, signed-field, keyword, allocation and header checks")
print("Manual BARR-C review and documented port deviations remain necessary.")
