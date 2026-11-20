"""Read-only identify + backup of a STOCK Baofeng UV-5R Mini over its programming cable.

Usage:
  py mini_read.py COM3 ident  OUTDIR    # handshake only: prints/saves the F and M replies
  py mini_read.py COM3 chirp  OUTDIR    # CHIRP regions -> raw + decrypted + CHIRP-loadable .img
  py mini_read.py COM3 cal    OUTDIR    # 0xF000-0xF27F calibration area (raw)
  py mini_read.py COM3 scan   OUTDIR    # every 64-byte block 0x0000-0xFFFF (raw), stops at first refusal

Protocol = CHIRP's UV17Pro family, UV5RMini class: 115200 8N1, ident b"PROGRAMCOLORPROU",
ack 0x06, magics F(16) M(15) SEND!(1), read frame 'R' + addr16 + len8, reply = 4-byte echo + 64 data
bytes XOR'd with symbol "CO 7".

Only the ident, the three handshake magics and 4-byte 'R' read frames can leave this script:
send() refuses anything else, so there is no code path that writes to the radio.
Turn the radio on normally (main screen, not a menu) before running.
"""
import os
import struct
import sys
import time

import serial

PORT, MODE, OUTDIR = sys.argv[1:4]
IDENT = b"PROGRAMCOLORPROU"
MAGICS = [(b"\x46", 16), (b"\x4d", 15),
          (b"\x53\x45\x4E\x44\x21\x05\x0D\x01\x01\x01\x04\x11\x08\x05"
           b"\x0D\x0D\x01\x11\x0F\x09\x12\x09\x10\x04\x00", 1)]
HANDSHAKE = {IDENT} | {m for m, _ in MAGICS}
BLOCK = 0x40
CHIRP_REGIONS = [(0x0000, 0x8040), (0x9000, 0x0040), (0xA000, 0x01C0)]
CHIRP_MODEL = b"UV-5R Mini"
KEY = b"CO 7"


def crypt(buf):
    out = bytearray()
    for i, b in enumerate(buf):
        k = KEY[i % 4]
        if k != 0x20 and b not in (0, 0xFF, k, k ^ 0xFF):
            b ^= k
        out.append(b)
    return bytes(out)


os.makedirs(OUTDIR, exist_ok=True)
s = serial.Serial(PORT, 115200, timeout=0.5, rtscts=False, xonxoff=False)


def send(frame):
    is_read = len(frame) == 4 and frame[0] == 0x52
    if frame not in HANDSHAKE and not is_read:
        raise SystemExit(f"REFUSED to send {frame[:8].hex()}")
    s.write(frame)


def handshake():
    s.reset_input_buffer()
    time.sleep(0.05)
    send(IDENT)
    ack = s.read(1)
    if ack != b"\x06":
        raise SystemExit(f"No ACK to ident (got {ack.hex() or 'nothing'}). "
                         "Radio on at main screen? Close CHIRP. Try power-cycling the radio.")
    replies = {}
    for magic, n in MAGICS:
        send(magic)
        replies[magic[:1]] = s.read(n)
    return replies


def read_blocks(addrs, stop_on_fail=True):
    data, log = bytearray(), []
    for addr in addrs:
        frame = struct.pack(">BHB", 0x52, addr, BLOCK)
        send(frame)
        d = s.read(BLOCK + 4)
        if len(d) != BLOCK + 4 or d[:4] != frame:
            log.append(f"{addr:#06x} FAIL: got {len(d)} bytes, header {d[:4].hex()}")
            print(log[-1])
            if stop_on_fail:
                break
            continue
        data += d[4:]
        log.append(f"{addr:#06x} ok{' blank' if all(b == 0xFF for b in d[4:]) else ''}")
    return data, log


def save(name, blob, log=None):
    p = os.path.join(OUTDIR, name)
    with open(p, "wb") as f:
        f.write(blob)
    if log is not None:
        with open(p + ".log", "w") as f:
            f.write("\n".join(log) + "\n")
    print(f"saved {len(blob)} bytes -> {p}")


try:
    r = handshake()
    f_reply, m_reply, send_reply = r[b"F"], r[b"M"], r[b"S"]
    print(f"F reply ({len(f_reply)}): {f_reply.hex()}  {f_reply!r}")
    print(f"M reply ({len(m_reply)}): {m_reply.hex()}  {m_reply!r}")
    print(f"SEND! reply: {send_reply.hex() or 'nothing'}")
    with open(os.path.join(OUTDIR, "ident.txt"), "w") as f:
        f.write(f"F {f_reply.hex()} {f_reply!r}\nM {m_reply.hex()} {m_reply!r}\n"
                f"SEND {send_reply.hex()}\n")

    if MODE == "ident":
        pass
    elif MODE == "chirp":
        raw, log = bytearray(), []
        for start, size in CHIRP_REGIONS:
            d, lg = read_blocks(range(start, start + size, BLOCK))
            raw += d
            log += lg
            if len(d) != ((size + BLOCK - 1) // BLOCK) * BLOCK:
                raise SystemExit("CHIRP region read incomplete; see log")
        save("chirp_regions_raw.bin", raw, log)
        # Rebuild CHIRP's image: decrypt, trim the last block of each region to its size
        img, pos = bytearray(), 0
        for start, size in CHIRP_REGIONS:
            padded = ((size + BLOCK - 1) // BLOCK) * BLOCK
            img += crypt(raw[pos:pos + padded])[:size]
            pos += padded
        save("chirp_regions_decrypted.bin", img)
        save("UV-5R_Mini_stock.img", bytes(img) + CHIRP_MODEL)
    elif MODE == "cal":
        d, log = read_blocks(range(0xF000, 0xF280, BLOCK))
        save("calibration_F000-F27F_raw.bin", d, log)
    elif MODE == "scan":
        d, log = read_blocks(range(0x0000, 0x10000, BLOCK))
        save("scan_0000-FFFF_raw.bin", d, log)
    else:
        raise SystemExit("mode must be ident, chirp, cal or scan")
finally:
    s.close()
