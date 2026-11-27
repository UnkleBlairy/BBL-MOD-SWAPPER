#!/usr/bin/env bash
# Build the Teensy image and the ROM library.
#   teensy-fm1.hex (copied to C:\Users\blair\teensy\): slot 0 (0x60000000) Felucca + the firmware menu,
#     slot 1 (0x60200000) BBL Jam Box, slot 2 (0x60400000) JIANT FM, each with its tag (src/roms.rs); slot 3 empty.
#   roms/<STEM>.R1 .R2 .R3: every ROM built for every slot, a 512-byte header + the image, for the card's ROMS/
#     (C:\Users\blair\teensy\roms\, uploaded by the Desktop "Teensy Upload ROMs.bat").
set -euo pipefail
. ~/.cargo/env
cd "$(dirname "$0")"
OBJCOPY=$(find "$(rustc --print sysroot)" -name llvm-objcopy -type f | head -1)

TFM1_FW=felucca cargo build --release
"$OBJCOPY" -O ihex target/thumbv7em-none-eabihf/release/teensy-fm1 target/slot0-felucca.hex

# name    stem    menu name      version   default slot   save memory in the slot (build.rs Fw.nor; 0 = in RAM)
ROMS="jambox JAMBOX BBL_JAM_BOX 0.9 1 1048576
jiant JIANT JIANT_FM 0.9.18 2 458752
sloop SLOOP SLOOP 2.8.5 3 1048576
x0x X0X X0X 1.2.2 3 0"
mkdir -p roms
echo "$ROMS" | while read -r fw stem name ver home nor; do
    for s in 1 2 3; do
        dir=target/$fw; [ "$s" = "$home" ] || dir=target/$fw-s$s
        TFM1_FW=$fw TFM1_SLOT=$s CARGO_TARGET_DIR=$dir cargo build --release
        "$OBJCOPY" -O ihex $dir/thumbv7em-none-eabihf/release/teensy-fm1 target/$fw-s$s.hex
        python3 - target/$fw-s$s.hex roms/$stem.R$s "$s" "$stem" "${name//_/ }" "$ver" "$nor" <<'PY'
import sys, zlib, struct
src, out, slot, stem, name, ver, nor = sys.argv[1:8]
slot, nor = int(slot), int(nor)
BASE = [0x60000000, 0x60200000, 0x60400000, 0x60600000]
END = [0x60200000, 0x60400000, 0x60600000, 0x607C0000]
mem = {}; base = 0
for l in open(src).read().split():
    n, ad, t = int(l[1:3], 16), int(l[3:7], 16), int(l[7:9], 16)
    if t == 4: base = int(l[9:13], 16) << 16
    elif t == 0:
        for i, b in enumerate(bytes.fromhex(l[9:9 + 2 * n])): mem[base + ad + i] = b
lo, hi = min(mem), max(mem) + 1
assert lo == BASE[slot], hex(lo)
assert hi <= END[slot] - 0x1000 - nor, 'image runs into the save memory / tag: %x' % hi
img = bytearray(b'\xff' * (hi - lo))
for a, b in mem.items(): img[a - lo] = b
assert struct.unpack_from('<I', img, 0x1000)[0] == 0x402000D1 and struct.unpack_from('<I', img, 0x1014)[0] == lo + 0x1000, 'boot header'
h = bytearray(512)
h[0:8] = b'FM1ROM01'
struct.pack_into('<IIII', h, 8, slot, len(img), zlib.crc32(img), lo)
h[24:24 + len(name)] = name.encode(); h[56:56 + len(ver)] = ver.encode(); h[72:72 + len(stem)] = stem.encode()
struct.pack_into('<I', h, 80, nor)
open(out, 'wb').write(bytes(h) + bytes(img))
print('%s: slot %d, %d KB of %d, crc %08x' % (out, slot, len(img) // 1024, (END[slot] - 0x1000 - lo) // 1024, zlib.crc32(img)))
PY
    done
done

# one hex: slot 0's records (its end record dropped), then slots 1 and 2 with their tags (the ROM headers)
python3 - target/slot0-felucca.hex roms/JAMBOX.R1 roms/JIANT.R2 teensy-fm1.hex <<'PY'
import sys
a = [l for l in open(sys.argv[1]).read().split() if l[7:9] != '01']
# slot 0's boot header: the flash is 8 MB, not teensy4-fcb's 2 MB (FCB sflashA1Size, at 0x60000050), so slot 1 is readable
base = 0; done = False
for i, l in enumerate(a):
    n, ad, t = int(l[1:3], 16), int(l[3:7], 16), int(l[7:9], 16)
    if t == 4: base = int(l[9:13], 16) << 16
    if t == 0 and base + ad <= 0x60000050 and 0x60000054 <= base + ad + n:
        rec = bytearray.fromhex(l[1:-2]); o = 4 + 0x60000050 - (base + ad)
        assert rec[o:o+4] == bytes.fromhex('00002000'), rec[o:o+4].hex()
        rec[o:o+4] = (0x800000).to_bytes(4, 'little')
        a[i] = ':' + rec.hex().upper() + '%02X' % (-sum(rec) & 0xFF); done = True
assert done, 'FCB flash size not found'
END = [0x60200000, 0x60400000, 0x60600000, 0x607C0000]
def rec(t, ad, data):
    r = bytearray([len(data), ad >> 8, ad & 0xFF, t]) + data
    return ':' + r.hex().upper() + '%02X' % (-sum(r) & 0xFF)
def records(addr, data):
    out = []; hi = None
    for o in range(0, len(data), 16):
        x = addr + o
        if x >> 16 != hi:
            hi = x >> 16; out.append(rec(4, 0, hi.to_bytes(2, 'big')))
        chunk = data[o:o + 16]
        if chunk != b'\xff' * len(chunk): out.append(rec(0, x & 0xFFFF, chunk))
    return out
b = []
for f in sys.argv[2:-1]:
    d = open(f, 'rb').read(); h, img = d[:512], d[512:]
    slot = int.from_bytes(h[8:12], 'little'); lo = int.from_bytes(h[20:24], 'little')
    b += records(lo, img) + records(END[slot] - 0x1000, h)
open(sys.argv[-1], 'w').write('\n'.join(a + b + [':00000001FF']) + '\n')
PY
arm-none-eabi-size target/thumbv7em-none-eabihf/release/teensy-fm1 target/jambox/thumbv7em-none-eabihf/release/teensy-fm1 target/jiant/thumbv7em-none-eabihf/release/teensy-fm1
# optional: copy the results to a Windows folder (WSL). Set TFM1_COPY_TO, or keep the porter's own default if it exists.
COPY_TO="${TFM1_COPY_TO:-/mnt/c/Users/blair/teensy}"
if [ -d "$COPY_TO" ]; then
    mkdir -p "$COPY_TO/roms"
    cp teensy-fm1.hex "$COPY_TO/"
    cp roms/*.R? "$COPY_TO/roms/"
    echo "-> $COPY_TO: teensy-fm1.hex and roms/"
fi
echo "-> teensy-fm1.hex and roms/ (copy roms/<STEM>.R1 .R2 .R3 to the SD card's ROMS folder)"
