#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Build the FM-1 Teensy terminal: build/terminal.bin (app) and build/fm1-teensy-terminal.fwsc (the package to install).
#   teensy-terminal/build.sh
# Needs the JieLi toolchain ($JIELI_TOOLCHAIN, default ~/.jieli/toolchain), JieLi's AC79 SDK ($AC79_SDK, default
# ~/fw-AC79_AIoT_SDK: three of its files go in every package) and a Felucca checkout for the update loader and the
# package writer ($FELUCCA_UPSTREAM, default ~/teensy-fm1/vendor/felucca-upstream: hugelton/Felucca).
set -e
TC="${JIELI_TOOLCHAIN:-$HOME/.jieli/toolchain}"
export JIELI_TOOLCHAIN="$TC"
export AC79_SDK="${AC79_SDK:-$HOME/fw-AC79_AIoT_SDK}"
UP="${FELUCCA_UPSTREAM:-$HOME/teensy-fm1/vendor/felucca-upstream}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BSP="$HERE/bsp"
OUT="${TERMINAL_OUT:-$HERE/build}"
rm -rf "$OUT"; mkdir -p "$OUT"
CC="$TC/pi32v2/bin/clang -target pi32v2"
$CC -c "$BSP/chip/crt0.S" -o "$OUT/crt0.o"
$CC -c "$BSP/chip/fm1_vec.S" -o "$OUT/vec.o"
$CC -c "$BSP/chip/fm1_isr.S" -o "$OUT/isr.o"
$CC -Os -ffunction-sections -fno-builtin -Wall -Wno-unused-function \
    -I"$BSP/chip" -I"$BSP/boards/fm1" $TERMINAL_CFLAGS -c "$HERE/terminal.c" -o "$OUT/terminal.o"
"$TC/pi32v2/bin/ld" -T "$HERE/app.ld" "$OUT/crt0.o" "$OUT/vec.o" "$OUT/isr.o" "$OUT/terminal.o" -o "$OUT/terminal.elf"
"$TC/common/bin/objcopy" -O binary -j .text "$OUT/terminal.elf" "$OUT/text.bin"
"$TC/common/bin/objcopy" -O binary -j .data "$OUT/terminal.elf" "$OUT/data.bin"
"$TC/common/bin/objcopy" -O binary -j .ram_text "$OUT/terminal.elf" "$OUT/rt.bin"
"$TC/common/bin/objdump" -h "$OUT/terminal.elf" > "$OUT/sections.txt"
python3 - "$TC" "$OUT" "$UP" <<'PY'
import re, subprocess, sys, os
tc, out, up = sys.argv[1:]
# the app image, as hwtest's build.sh makes it
syms = subprocess.run([f"{tc}/common/bin/objdump", "-t", f"{out}/terminal.elf"], capture_output=True, text=True, check=True).stdout
def sym(n):
    return int(re.search(r"^([0-9a-f]+) .*\s" + n + "$", syms, re.M).group(1), 16)
load, rt_load, start = sym("_data_load"), sym("_rt_load"), sym("_start")
assert start == 0x02000120, hex(start)
img = bytearray(open(f"{out}/text.bin", "rb").read())
data = open(f"{out}/data.bin", "rb").read()
rt = open(f"{out}/rt.bin", "rb").read()
for at, part in sorted([(rt_load, rt), (load, data)]):
    if part:
        assert at - 0x02000120 >= len(img), "sections overlap"
        img += b"\xff" * (at - 0x02000120 - len(img)) + part
img += b"\xff" * (-len(img) % 4)
open(f"{out}/terminal.bin", "wb").write(img)
bss_end = sym("_bss_end")
print(f"terminal.bin: {len(img)} bytes; RAM used up to {bss_end:#x} (RAM is 0x1c08000..0x1c20000)")
assert bss_end <= 0x01C20000, "RAM overflow"
# the update loader and the package: Felucca's own build code (tools/build.py, tools/fm1pkg_make.py)
sys.path.insert(0, f"{up}/tools")
os.chdir(up)
import build, fm1pkg_make
ota = build.build_loader()
pkg = fm1pkg_make.ufw(fm1pkg_make.flash_image(bytes(img), fm1pkg_make.KEY), ota, "FM-1_1300")
open(f"{out}/fm1-teensy-terminal.fwsc", "wb").write(pkg)
print(f"package: {out}/fm1-teensy-terminal.fwsc  {len(pkg)} bytes, identity FM-1_1300")
PY
# hand the package to the Windows side (the desktop install .bat reads it from there)
WIN="/mnt/c/Users/blair/teensy/fm1"
if [ -z "$TERMINAL_OUT" ] && [ -d "$WIN" ]; then cp "$OUT/fm1-teensy-terminal.fwsc" "$OUT/terminal.bin" "$WIN/" && echo "copied to $WIN"; fi
