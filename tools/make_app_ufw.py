"""Offline: package an SDK-built app (sdk.elf) into a .ufw for radio 2, on the res-link layout.

Usage: py -I make_app_ufw.py BASE_RESLINK.ufw SDK.elf OUT.ufw WORKDIR [DUMP_xip.bin]

app.bin is made the way the SDK's post-build does it (cpu/br23/tools/download.c, standard case):
  .text .data .data_code, then the overlays .overlay_aec/_wav/_ape/_flac/_m4a/_amr/_dts/_fm/_mp3/_wma (aec, fm, mp3
  and wma with trailing 0xFF removed), then bank.bin (only if the ELF has a .bank section; it is reported).
Before using it, the ELF's program headers are checked: .text must load at 0x01E00100 (ENTRY), and each later piece
must load right after the previous one (LMA = previous LMA + size), so the concatenation is what the CPU expects.

Into the res-link image (base sha d8d75d9b...): app.bin replaces the bytes at SoC 0x4100; the rest up to 0x7BE1C is
filled with 0xFF; the app.bin entry (SoC 0x4020) gets the new size and data CRC; app_area_head (0x4000) keeps size
0x77E4C (chain -> V0.05's resources at 0x7BE4C) and gets a new data CRC over [0x4020, 0x7BE4C). The VM / PRCT /
BTIF entries, the tone header and table at 0x7BE4C, uboot, isd_config and ota.bin are untouched. Then: re-encrypt,
container CRCs, a check that only the app area changed, and (with the dump) the whole chain on the composed flash.
Writes WORKDIR/app.bin and WORKDIR/verify/flash_dec.bin + flash.bin for emu_verify.
"""
import hashlib
import os
import struct
import sys

BASE_SHA = "d8d75d9b497405f99e6ea6d86b3c46aa52782c8175e1b1e95246889e95125bd9"
CHIPKEY, APP_START = 0xB50F, 0x3000
AREA, APP_HDR, APP_DATA, APP_END, TONE, LIMIT = 0x4000, 0x4020, 0x4100, 0x7BE1C, 0x7BE4C, 0x7C000
ENTRY = 0x01E00100
VA0 = 0x01E00000

base_path, elf_path, out_path, work = sys.argv[1:5]
dump_path = sys.argv[5] if len(sys.argv) > 5 else None
os.makedirs(work, exist_ok=True)


def crc16(d, c=0):
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else c << 1
            c &= 0xFFFF
    return c


def enc(b, key):
    out = bytearray(len(b)); k = key
    for i, x in enumerate(b):
        out[i] = x ^ (k & 0xFF)
        k = ((k << 1) ^ (0x1021 if k & 0x8000 else 0)) & 0xFFFF
    return bytes(out)


def blk_key(off):
    return CHIPKEY ^ (((off - APP_START) >> 2) & 0xFFFF)


# ---------------- ELF -> app.bin
elf = open(elf_path, "rb").read()
assert elf[:4] == b"\x7fELF"
shoff, = struct.unpack_from("<I", elf, 0x20)
phoff, = struct.unpack_from("<I", elf, 0x1C)
phentsize, phnum = struct.unpack_from("<HH", elf, 0x2A)
shentsize, shnum, shstrndx = struct.unpack_from("<HHH", elf, 0x2E)
sh = [struct.unpack_from("<IIIIIIIIII", elf, shoff + i * shentsize) for i in range(shnum)]
shstr = sh[shstrndx][4]
secname = lambda s: elf[shstr + s[0]:elf.index(b"\0", shstr + s[0])].decode()
secs = {secname(s): s for s in sh}
phs = [struct.unpack_from("<IIIIIIII", elf, phoff + i * phentsize) for i in range(phnum)]   # type off vaddr paddr filesz memsz flags align


def lma(s):
    """Load address of a section: via the PT_LOAD segment that contains its file offset."""
    for p in phs:
        if p[0] == 1 and p[1] <= s[4] < p[1] + max(p[4], 1):
            return p[3] + (s[4] - p[1])
    return s[3]


# The flash (load) positions come from the linker script's own symbols (cpu/br23/sdk.ld):
#   data_begin = text_begin + text_size; aec_begin = text+data; wav_begin = aec_begin + aec_size; ... fm_begin
# The startup code / overlay loader read these symbols, so app.bin must put each piece exactly there.
symtab = next(s for s in sh if s[1] == 2)
sstr = sh[symtab[6]][4]
SYM = {}
for k in range(symtab[5] // 16):
    n, v, sz, info, o, shx = struct.unpack_from("<IIIBBH", elf, symtab[4] + k * 16)
    SYM[elf[sstr + n:elf.index(b"\0", sstr + n)].decode(errors="replace")] = v
text = secs[".text"]
assert text[3] == ENTRY == SYM["text_begin"], f".text at {text[3]:#x}, text_begin {SYM.get('text_begin', 0):#x}"
assert SYM["text_size"] == text[5] and SYM["data_begin"] == ENTRY + text[5]
assert SYM["data_size"] == secs[".data"][5]
assert SYM.get("psram_text_size", 0) == 0, "psram text not handled"   # (bank_stub_* lives inside .text)
pieces = [(".text", ENTRY, text[5]), (".data", SYM["data_begin"], SYM["data_size"])]
for ov in ("aec", "wav", "ape", "flac", "m4a", "amr", "dts", "fm", "mp3", "wma"):
    if f"{ov}_begin" in SYM:
        pieces.append((f".overlay_{ov}", SYM[f"{ov}_begin"], SYM[f"{ov}_size"]))
    else:
        s = secs.get(f".overlay_{ov}")
        assert s is None or s[1] == 8 or not elf[s[4]:s[4] + s[5]].strip(b"\xff\x00"), f".overlay_{ov} has data but no _begin symbol"
end = max(b + s for _, b, s in pieces)
app = bytearray(b"\xff" * (end - ENTRY))
cursor = ENTRY
print("app.bin pieces (flash address from the linker symbols):")
for name, begin, size in sorted(pieces, key=lambda p: p[1]):
    s = secs[name]
    body = elf[s[4]:s[4] + s[5]] if s[1] != 8 else b""
    assert size <= len(body) or size == 0, f"{name}: symbol size {size:#x} > section {len(body):#x}"
    assert begin == cursor, f"{name} at {begin:#x}, expected {cursor:#x} (gap or overlap)"
    app[begin - ENTRY:begin - ENTRY + size] = body[:size]
    print(f"  {name:<16} flash {begin:#010x} size {size:#7x}  (section VMA {s[3]:#010x}, {len(body):#x} bytes)")
    cursor = begin + size
open(os.path.join(work, "app.bin"), "wb").write(app)
room = APP_END - APP_DATA
print(f"app.bin {len(app)} bytes ({len(app) / 1024:.1f} KB), room {room} ({100 * len(app) / room:.1f}% used)")
assert len(app) <= room, "app.bin too big for the res-link layout"

# ---------------- base image
u = bytearray(open(base_path, "rb").read())
assert hashlib.sha256(u).hexdigest() == BASE_SHA, "base is not the res-link image"
hdr = enc(u[:0x40], 0xFFFF)
count = struct.unpack_from("<H", hdr, 8)[0]
ents = [bytearray(enc(u[0x40 + i * 0x50:0x40 + (i + 1) * 0x50], 0xFFFF)) for i in range(count)]
fe = next(e for e in ents if e[0x40:0x49] == b"flash.bin")
f_off, f_size = struct.unpack_from("<II", fe, 8)
raw = bytearray(u[f_off:f_off + f_size])
plain = bytearray()
for off in range(0, len(raw), 32):
    blk = bytes(raw[off:off + 32])
    plain += enc(blk, blk_key(off)) if off >= APP_START else blk
P = lambda soc: soc - 0x1000
old = bytes(plain)

e = plain[P(APP_HDR):P(APP_HDR) + 32]
assert e[16:23] == b"app.bin" and struct.unpack_from("<I", e, 4)[0] == 0x100
assert struct.unpack_from("<I", plain, P(AREA) + 8)[0] == TONE - AREA

plain[P(APP_DATA):P(APP_END)] = bytes(app) + b"\xff" * (room - len(app))
struct.pack_into("<I", plain, P(APP_HDR) + 8, len(app))
struct.pack_into("<H", plain, P(APP_HDR) + 2, crc16(plain[P(APP_DATA):P(APP_DATA) + len(app)]))
struct.pack_into("<H", plain, P(APP_HDR), crc16(plain[P(APP_HDR) + 2:P(APP_HDR) + 32]))
struct.pack_into("<H", plain, P(AREA) + 2, crc16(plain[P(AREA) + 32:P(TONE)]))
struct.pack_into("<H", plain, P(AREA), crc16(plain[P(AREA) + 2:P(AREA) + 32]))
assert plain[P(0x4040):P(APP_DATA)] == old[P(0x4040):P(APP_DATA)], "VM/PRCT/BTIF entries changed"
assert plain[P(APP_END):] == old[P(APP_END):], "bytes at/after 0x7BE1C changed"
assert plain[:P(AREA)] == old[:P(AREA)], "bytes below 0x4000 changed"

new_raw = bytearray(raw)
for off in range(APP_START, len(raw), 32):
    new_raw[off:off + 32] = enc(bytes(plain[off:off + 32]), blk_key(off))
assert new_raw[:APP_START] == raw[:APP_START]
assert new_raw[P(APP_END) // 32 * 32 + 32:] == raw[P(APP_END) // 32 * 32 + 32:]

# ---------------- chain check on the composed flash (needs the dump for the resources above 0x7C000)
if dump_path:
    xip = open(dump_path, "rb").read()
    comp = bytes(plain) + xip[LIMIT - 0x4000:]
    C = lambda soc, n: comp[P(soc):P(soc) + n]
    ent, walked = AREA, []
    while True:
        h = C(ent, 32)
        hc, dc, off, size, attr, res, idx = struct.unpack_from("<HHIIBBH", h)
        nm = h[16:32].split(b"\0")[0].split(b"\xff")[0].decode()
        assert crc16(h[2:32]) == hc and crc16(C(ent + 32, size - 32)) == dc, f"chain CRC at {ent:#x} ({nm})"
        walked.append(nm)
        if idx:
            break
        ent += size
    assert walked == ["app_area_head", "tone", "cfg_tool.bin", "eq_cfg_hw.bin"], walked
    print("chain OK:", " -> ".join(walked))
    vd = os.path.join(work, "verify")
    os.makedirs(vd, exist_ok=True)
    dump_raw = bytearray()
    for off in range(LIMIT - 0x1000, 0x100000 - 0x1000, 32):
        soc = off + 0x1000
        dump_raw += enc(xip[soc - 0x4000:soc - 0x4000 + 32], blk_key(off))
    open(os.path.join(vd, "flash_dec.bin"), "wb").write(comp)
    open(os.path.join(vd, "flash.bin"), "wb").write(bytes(new_raw) + bytes(dump_raw))

u[f_off:f_off + f_size] = new_raw
struct.pack_into("<H", fe, 4, crc16(new_raw))
for i, en in enumerate(ents):
    u[0x40 + i * 0x50:0x40 + (i + 1) * 0x50] = enc(bytes(en), 0xFFFF)
hdr = bytearray(hdr)
struct.pack_into("<H", hdr, 2, crc16(u[0x40:0x40 + count * 0x50]))
struct.pack_into("<H", hdr, 0, crc16(hdr[2:]))
u[:0x40] = enc(bytes(hdr), 0xFFFF)
open(out_path, "wb").write(u)
print("wrote", out_path, len(u), "bytes, sha256", hashlib.sha256(u).hexdigest())
