"""UV-5R Mini firmware updater over the CH340 cable. A Python port of Codeplug Studio's
src/integrations/radio-io/radios/uv5r-mini/firmwareUpdate/{frames,session,outgoing,updater}.ts.

Usage (radio on the UPDATE... screen: hold PTT-A + '=' while switching on):
  py mini_fw_update.py COM3 identify
      Listen 2 s, then ping once a second until the stock app answers. Stops at the first answer.
      Sends only pings. Nothing on the radio is touched.
  py mini_fw_update.py COM3 flash IMAGE.ufw SHA256
      The full update. IMAGE must hash to SHA256 or nothing is sent.

Frames: AA 55 | len u16 LE (= 1 + data) | cmd | data | CRC-16/XMODEM u16 LE over all bytes before it.
Commands: 1 rate, 2 data {off,len}, 3 result, 4 length, 6 ping. Link 9600 8N1, DTR/RTS LOW.
The radio drives everything: the host only pings, names one of two rates, answers reads with the exact
image bytes asked for (<= 0x1000, inside the image), and echoes the radio's own cmd 3 / cmd 4.
Every outgoing frame passes check_outgoing(); anything else raises before it reaches the port.
"""
import hashlib
import struct
import sys
import time

import serial

APP_BAUD, LOADER_BAUD, MAX_DATA = 9600, 38400, 0x1000
BAUD, DATA, RESULT, LENGTH, PING = 1, 2, 3, 4, 6
LISTEN_S, PING_INTERVAL_S, PING_TOTAL_S, PONG_SETTLE_S, IDLE_S = 2.0, 1.0, 30.0, 0.3, 60.0

RESULT_TEXT = {
    0x00: "OK",
    0x01: "app: image is not for this radio's key (nothing written)",
    0x83: "refused: file isn't for this radio's firmware",
    0x86: "already installed / can't replace (nothing erased if before cmd 4)",
    0x87: "nothing written; radio boots the installed app",
    0x8A: "app: image is for another stock version (nothing written)",
    0x8F: "loader: other flash layout (nothing erased)",
    0x90: "loader: image runs past the install area (nothing erased)",
}


def crc16(data):
    c = 0
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else c << 1
            c &= 0xFFFF
    return c


def frame(cmd, data=b""):
    body = b"\xAA\x55" + struct.pack("<HB", 1 + len(data), cmd) + data
    return body + struct.pack("<H", crc16(body))


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.bad = []
        self.junk = 0

    def feed(self, b):
        self.buf += b
        out = []
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0:
                keep = 1 if self.buf[-1:] == b"\xAA" else 0
                self.junk += len(self.buf) - keep
                self.buf = bytearray(b"\xAA") if keep else bytearray()
                return out
            self.junk += i
            del self.buf[:i]
            if len(self.buf) < 4:
                return out
            ln = self.buf[2] | self.buf[3] << 8
            if ln == 0:
                self.bad.append(bytes(self.buf[:2]).hex())
                del self.buf[:2]
                continue
            total = ln + 6
            if len(self.buf) < total:
                return out
            f = bytes(self.buf[:total])
            del self.buf[:total]
            if (f[-2] | f[-1] << 8) != crc16(f[:-2]):
                self.bad.append(f.hex())
            else:
                out.append((f[4], f[5:-2]))


class Session:
    def __init__(self, image):
        self.image = image
        self.rate = APP_BAUD
        self.phase = "listen"
        self.continued = False
        self.area = None
        self.result = None
        self.result_phase = None
        self.reads = 0
        self.last = {}
        self.length_heard = False

    def ping(self):
        if self.phase == "listen":
            self.phase = "ping"
        return [frame(PING)], None, None

    def on_frame(self, cmd, data):
        self.last[cmd] = bytes(data)
        if cmd == BAUD:
            return self.on_baud(data)
        if cmd == DATA:
            return self.on_data(data)
        if cmd == LENGTH:
            self.length_heard = True
            if self.phase == "loader" and len(data) >= 4:
                self.area = struct.unpack_from("<I", data)[0]
                log(f"LENGTH {self.area:#x}: the radio is erasing its firmware area")
                return [frame(LENGTH, data)], None, None
            return [], None, None
        if cmd == RESULT:
            if not data:
                return [], None, None
            if self.result is None:
                self.result, self.result_phase = data[0], self.phase
            self.phase = "done"
            return ([frame(RESULT, data)] if self.result_phase == "loader" else []), None, None
        log(f"ignored cmd {cmd} in {self.phase}: {bytes(data).hex()}")
        return [], None, None

    def loader_rate(self):
        self.rate = LOADER_BAUD
        return [frame(BAUD, struct.pack("<I", LOADER_BAUD))], LOADER_BAUD, None

    def on_baud(self, data):
        if self.phase == "listen":
            self.continued, self.phase = True, "loader"
            log("loader talked first: continuing an interrupted update")
            return self.loader_rate()
        if self.phase == "ping":
            if not data:
                self.phase = "app"
                log("pong: application is in update mode")
                return [frame(BAUD, struct.pack("<I", APP_BAUD))], None, None
            return [], None, None
        if self.phase == "app":
            if self.reads > 0:
                self.phase = "loader"
                log("hand-over: app reset into the loader")
                return self.loader_rate()
            return [], None, None
        if self.phase == "loader":
            return [frame(BAUD, struct.pack("<I", LOADER_BAUD))], None, None
        return [], None, None

    def on_data(self, data):
        if self.phase in ("listen", "done") or len(data) < 8:
            return [], None, None
        off, ln = struct.unpack_from("<II", data)
        self.reads += 1
        if off + ln > len(self.image):
            return [], None, f"read {off:#x}+{ln:#x} is outside the image"
        if ln > MAX_DATA:
            return [], None, f"read of {ln:#x} bytes is over the frame limit"
        return [frame(DATA, bytes(data[:8]) + self.image[off:off + ln])], None, None


def check_outgoing(f, s):
    if len(f) < 7 or f[:2] != b"\xAA\x55":
        raise SystemExit("REFUSED: not a frame")
    ln = f[2] | f[3] << 8
    if ln < 1 or len(f) != ln + 6 or (f[-2] | f[-1] << 8) != crc16(f[:-2]):
        raise SystemExit("REFUSED: bad length/CRC")
    cmd, data = f[4], f[5:-2]
    if cmd == PING:
        ok = not data
    elif cmd == BAUD:
        ok = len(data) == 4 and struct.unpack("<I", data)[0] in (APP_BAUD, LOADER_BAUD)
    elif cmd == DATA:
        off, n = struct.unpack_from("<II", data)
        ok = (n <= MAX_DATA and off + n <= len(s.image) and len(data) == 8 + n
              and data[8:] == s.image[off:off + n])
    elif cmd in (RESULT, LENGTH):
        ok = s.last.get(cmd) == bytes(data)
    else:
        ok = False
    if not ok:
        raise SystemExit(f"REFUSED to send cmd {cmd}: {f[:24].hex()}")


LOG = []


def log(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    LOG.append(line)
    print(line, flush=True)


def open_port(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, APP_BAUD, 0.05
    s.dtr = False
    s.rts = False
    s.open()
    s.dtr = False
    s.rts = False
    return s


def main():
    port, mode = sys.argv[1], sys.argv[2]
    if mode == "identify":
        image = b""
        if len(sys.argv) > 3:
            global PING_TOTAL_S
            PING_TOTAL_S = float(sys.argv[3])
    elif mode == "flash":
        path, want = sys.argv[3], sys.argv[4].lower()
        image = open(path, "rb").read()
        got = hashlib.sha256(image).hexdigest()
        if not got.startswith(want) or len(want) < 16:
            raise SystemExit(f"image sha256 {got} does not match {want}. Nothing sent.")
        log(f"image ok: {path} {len(image)} bytes sha256 {got[:16]}")
        if len(sys.argv) > 5:
            global LISTEN_S
            LISTEN_S = float(sys.argv[5])
    else:
        raise SystemExit("mode must be identify or flash")

    s = Session(image)
    p = Parser()
    ser = open_port(port)

    link = {"rate": APP_BAUD, "switched_at": None}

    def send(frames, switch):
        nbytes = 0
        for f in frames:
            check_outgoing(f, s)
            ser.write(f)
            nbytes += len(f)
        ser.flush()
        if switch:
            # Let the reply physically leave the CH340 at the old rate before changing it (10 bits/byte + margin).
            time.sleep(nbytes * 10 / link["rate"] + 0.04)
            ser.reset_input_buffer()
            p.buf.clear()
            ser.baudrate = switch
            link["rate"] = switch
            link["switched_at"] = time.monotonic()
            log(f"link now {switch} baud")

    def back_to_app_rate():
        ser.reset_input_buffer()
        p.buf.clear()
        ser.baudrate = APP_BAUD
        link["rate"] = APP_BAUD
        link["switched_at"] = None
        log("no hello at 38400: back to 9600 to wait for the loader to restart its hello")

    def next_frame(timeout):
        end = time.monotonic() + timeout
        while True:
            if p.pending:
                return p.pending.pop(0)
            if time.monotonic() > end:
                return None
            got = ser.read(256)
            if got:
                p.pending.extend(p.feed(got))
                for h in p.bad:
                    log(f"bad-crc frame dropped: {h[:40]}")
                p.bad.clear()

    p.pending = []
    started = time.monotonic()
    try:
        # 1. listen
        log("listening 2 s (sending nothing)")
        first = next_frame(LISTEN_S)
        if first is not None:
            cmd, data = first
            if mode == "identify":
                log(f"radio talked first: cmd {cmd} {bytes(data).hex()} (loader resuming?) - stopping")
                return
            send(*s.on_frame(cmd, data)[:2])
        else:
            # 2. ping until the pong
            pings, pong = 0, None
            t_end = time.monotonic() + PING_TOTAL_S
            while time.monotonic() < t_end and pong is None:
                send(*s.ping()[:2])
                pings += 1
                log(f"ping {pings}")
                f = next_frame(PING_INTERVAL_S)
                while f is not None and not (f[0] == BAUD and not f[1]):
                    log(f"heard cmd {f[0]} {bytes(f[1]).hex()[:40]} while pinging")
                    f = next_frame(0.2)
                pong = f
            if pong is None:
                log("NO RADIO IN UPDATE MODE ANSWERED. Nothing was sent that the radio acted on.")
                return
            if pings > 1:
                while next_frame(PONG_SETTLE_S) is not None:
                    pass
            log("PONG: application answered (update mode confirmed)")
            if mode == "identify":
                log("identify only: stopping here. Nothing on the radio was touched.")
                return
            frames, switch, flagged = s.on_frame(BAUD, b"")
            send(frames, switch)

        # 3. the radio drives: app reads, hand-over, loader reads, length, result
        last_report = 0
        retries = 0
        silent_since = time.monotonic()
        while s.phase != "done":
            awaiting_hello = link["switched_at"] is not None
            in_loader_at_fast = (s.phase == "loader" and link["rate"] == LOADER_BAUD
                                 and s.area is not None and not awaiting_hello)
            f = next_frame(1.5 if awaiting_hello else (4.0 if in_loader_at_fast else IDLE_S))
            if f is None:
                if p.junk:
                    log(f"(heard {p.junk} non-frame bytes at {link['rate']} baud)")
                    p.junk = 0
                if awaiting_hello and retries < 5:
                    retries += 1
                    back_to_app_rate()
                    continue
                if in_loader_at_fast and time.monotonic() - silent_since < IDLE_S:
                    # Did the radio reset? A restarted loader says hello once at 9600.
                    log("loader quiet for 4 s: checking 9600 for a restarted loader")
                    back_to_app_rate()
                    hello = next_frame(3.0)
                    if hello is not None and hello[0] == BAUD:
                        log("LOADER RESTARTED (radio reset during the update): answering {38400}")
                        s.last[BAUD] = bytes(hello[1])
                        frames, switch, _ = s.loader_rate()
                        send(frames, switch)
                        retries = 0
                        continue
                    if hello is not None:
                        log(f"at 9600 heard cmd {hello[0]} {bytes(hello[1]).hex()[:32]}")
                    # Its single hello may already have gone by while we listened at 38400 (it reads as
                    # non-frame bytes there). A restarted loader waits for {38400} at 9600: offer it.
                    log("no hello heard: offering {38400} at 9600 in case the loader restarted unseen")
                    frames, switch, _ = s.loader_rate()
                    send(frames, switch)
                    retries = 0
                    continue
                raise SystemExit(f"No frame from the radio for {IDLE_S:.0f} s (phase {s.phase}). "
                                 "See recovery: switch off/on with no keys, run flash again.")
            silent_since = time.monotonic()
            link["switched_at"] = None
            if s.phase == "loader" and f[0] == BAUD and link["rate"] == APP_BAUD:
                # The loader restarted its hello at 9600 (it gave up waiting at 38400): answer it again.
                s.last[BAUD] = bytes(f[1])
                log("loader hello at 9600 again: answering {38400}")
                frames, switch, _ = s.loader_rate()
                send(frames, switch)
                continue
            frames, switch, flagged = s.on_frame(*f)
            if flagged:
                raise SystemExit(f"REFUSED: {flagged}. Nothing more was sent.")
            send(frames, switch)
            if f[0] == DATA and time.monotonic() - last_report > 2:
                off, ln = struct.unpack_from("<II", f[1])
                log(f"{s.phase} read #{s.reads}: {off:#x}+{ln:#x}"
                    + (f" / area {s.area:#x}" if s.area else ""))
                last_report = time.monotonic()
        code = s.result
        log(f"RESULT {code:#04x} in {s.result_phase}: {RESULT_TEXT.get(code, 'unknown')}"
            f" (length heard: {s.length_heard}, reads {s.reads}, {time.monotonic() - started:.0f} s)")
        if code == 0 and s.result_phase == "loader":
            log("Done. The radio restarts by itself; check the power-on screen and Radio Info.")
    finally:
        time.sleep(0.5)  # let the last frame (e.g. the result echo) leave the cable before closing
        ser.close()
        with open("mini_fw_update.log", "a") as fh:
            fh.write("\n".join(LOG) + "\n\n")


main()
