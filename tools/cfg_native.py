"""Install native Mini source into an already configured minitest3 SDK.

Does not run packaging, download tools, or touch a connected radio.
"""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[1]
sdk = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "fw/sdk"
app = sdk / "apps/soundbox/app_main.c"
raw = app.read_bytes()
marker = b"void app_main()\r\n{"
if b"mini_app_main();" not in raw:
    if marker not in raw:
        marker = b"void app_main()\n{"
    assert raw.count(marker) == 1, "Unexpected app_main entry"
    raw = raw.replace(marker, marker + b"\n    extern void mini_app_main(void);\n    mini_app_main();\n    return;\n", 1)
    app.write_bytes(raw)
source = sdk / "apps/soundbox/mini_app.c"
source.write_bytes((root / "fw/native/mini_app.c").read_bytes())
for name in ("mini_rf.c", "mini_rf.h", "ui_assets.h", "app_text.h", "app_shared.h", "app_fox.h", 
             "app_search.h", "mini_settings.h"):
    (sdk / "apps/soundbox" / name).write_bytes((root / "fw/native" / name).read_bytes())
# This native app only uses SPI1. The SDK's generic SPI unit otherwise requires
# platform declarations for SPI0/SPI2 even when both board drivers are disabled.
spi = sdk / "cpu/br23/spi.c"
raw = spi.read_bytes()
for number in (0, 2):
    raw = re.sub(rb"(#define SPI" + str(number).encode() + rb"_ENABLE\s+)1\b",
                 rb"\g<1>0", raw)
spi.write_bytes(raw)
make = sdk / "Makefile"
raw = make.read_bytes()
if b"apps/soundbox/mini_app.c" not in raw:
    needle = b"apps/soundbox/app_main.c"
    assert raw.count(needle) == 1
    raw = raw.replace(needle, b"apps/soundbox/mini_app.c \\\n\t" + needle)
    make.write_bytes(raw)
raw = make.read_bytes()
if b"apps/soundbox/mini_rf.c" not in raw:
    raw = raw.replace(b"apps/soundbox/mini_app.c", b"apps/soundbox/mini_rf.c \\\n\tapps/soundbox/mini_app.c", 1)
    make.write_bytes(raw)
# The Mini battery-divider wiring is not the SDK demo board's divider.
# Prevent an incorrect demo-board ADC reading from switching off our app.
cfg = sdk / "apps/soundbox/board/br23/board_ac695x_demo/board_ac695x_demo_cfg.h"
raw = cfg.read_bytes()
raw, n = re.subn(rb"(#define\s+TCFG_SYS_LVD_EN\s+)1\b", rb"\g<1>0", raw)
assert n == 1 or re.search(rb"#define\s+TCFG_SYS_LVD_EN\s+0\b", raw)
cfg.write_bytes(raw)

# No demo-board audio/LED callbacks while reporting a completed update.
cfg = sdk / "apps/soundbox/include/app_config.h"
raw = cfg.read_bytes()
raw = re.sub(rb"(?m)^#define (UPDATE_(?:VOICE|LED)_REMIND)\s*\r?$",
             rb"/* Native Mini: \1 disabled. */", raw)
cfg.write_bytes(raw)

# A failed UART read must not report success with an untouched destination.
# Preserve the minitest3 supported API handover, and tighten read validation.
uart = sdk / "apps/common/update/uart_update.c"
raw = uart.read_bytes()
assert b"app_active_update_task_init(&info)" in raw, "Apply cfg_minitest.py first"
if b"Native Mini: reject incomplete reads" not in raw:
    needle = b"        memcpy(&file_cmd, protocal_frame.data.data, sizeof(file_cmd));"
    assert raw.count(needle) == 1
    raw = raw.replace(needle,
        b"        /* Native Mini: reject incomplete reads before copying. */\n"
        b"        if (protocal_frame.data.length != sizeof(file_cmd) + relen) {\n"
        b"            continue;\n        }\n" + needle)
    needle = b"    putchar('R');\r\n    return relen;"
    if needle not in raw:
        needle = needle.replace(b"\r\n", b"\n")
    assert raw.count(needle) == 1
    raw = raw.replace(needle, b"    putchar('R');\n    return (u32)-1;")
    uart.write_bytes(raw)

# Keep read-only diagnostics separate from the updater protocol. Decoder only
# queues work; the UART task replies. Never interrupt an active fast transfer.
raw = uart.read_bytes()
if b"mini_diag_reply" not in raw:
    needle = b"                    case CMD_UART_UPDATE_READY:"
    assert raw.count(needle) == 1
    raw = raw.replace(needle,
        b"                    case 0x70: /* Native read-only diagnostic */\n"
        b"                        if (update_baudrate == 9600 && protocal_frame.data.length == 1)\n"
        b"                            os_taskq_post_msg(THIS_TASK_NAME, 1, 0x70);\n"
        b"                        break;\n" + needle)
    needle = b"        case MSG_UART_UPDATE_READY:"
    assert raw.count(needle) == 1
    raw = raw.replace(needle,
        b"        case 0x70: {\n"
        b"            extern void mini_diag_reply(void);\n"
        b"            if (update_baudrate == 9600) mini_diag_reply();\n"
        b"            break;\n        }\n" + needle)
    uart.write_bytes(raw)
print("Native Mini installed; update handover retained; read failures propagated.")

raw = uart.read_bytes()
if b"mini_nor_request" not in raw:
    needle = b"                    case 0x70:"
    assert raw.count(needle) == 1
    raw = raw.replace(needle,
        b"                    case 0x71: { /* Native read-only NOR request */\n"
        b"                        extern void mini_nor_request(const u8 *);\n"
        b"                        if (update_baudrate == 9600 && protocal_frame.data.length == 7)\n"
        b"                            mini_nor_request(protocal_frame.data.data + 1);\n"
        b"                        break;\n                    }\n" + needle)
    uart.write_bytes(raw)

raw = uart.read_bytes()
if b"mini_nor_write_request" not in raw:
    needle = b"                    case 0x70:"
    assert raw.count(needle) == 1
    raw = raw.replace(needle,
        b"                    case 0x72: case 0x73: case 0x74: { /* Native UV Channels link (N15) */\n"
        b"                        extern void mini_nor_write_request(u8, const u8 *, unsigned);\n"
        b"                        if (update_baudrate == 9600)\n"
        b"                            mini_nor_write_request(protocal_frame.data.data[0], protocal_frame.data.data + 1,\n"
        b"                                                   protocal_frame.data.length - 1);\n"
        b"                        break;\n                    }\n" + needle)
    uart.write_bytes(raw)

# N16: the PC channel app's programming session (UV-5RM custom firmware protocol, 115200 baud) on the same port.
# The radio alternates the idle port between 9600 and 115200 (mini_app.c hop_service); a valid updater frame holds
# 9600, and "PROGRAMBF-5RH" seen at 115200 starts a session that owns the port until it ends.
raw = uart.read_bytes()
if b"mini_cps_feed" not in raw:
    needle = b"    last_rx_ms = now;\n"
    if needle not in raw: needle = needle.replace(b"\n", b"\r\n")
    assert raw.count(needle) == 1
    raw = raw.replace(needle, needle +
        b"    { extern int mini_cps_feed(const u8 *b, u16 n); if (mini_cps_feed(buf, len)) return; }\n")
    needle = b"                if (crc0 == crc) {\n"
    if needle not in raw: needle = needle.replace(b"\n", b"\r\n")
    assert raw.count(needle) == 1
    raw = raw.replace(needle, needle +
        b"                    { extern void mini_uart_frame_seen(void); mini_uart_frame_seen(); }\n")
    needle = b"static void clock_critical_exit(void)\n{\n    uart_update_set_baud(update_baudrate);\n}"
    if needle not in raw: needle = needle.replace(b"\n", b"\r\n")
    assert raw.count(needle) == 1
    raw = raw.replace(needle,
        b"static void clock_critical_exit(void)\n{\n"
        b"    extern u32 mini_uart_hw_baud_now(void);\n"
        b"    uart_update_set_baud(update_baudrate == 9600 ? mini_uart_hw_baud_now() : update_baudrate);\n}")
    raw += (b"\n/* Native Mini (N16): raw send and hardware baud for the PC channel app session. */\n"
            b"void mini_uart_raw_send(const u8 *buf, u32 len)\n{\n"
            b"    uart_set_dir(0);\n    uart_update_write((u8 *)buf, len);\n    uart_set_dir(1);\n}\n"
            b"void mini_uart_hw_baud(u32 baud) { uart_update_set_baud(baud); }\n"
            b"u32 mini_uart_update_baud(void) { return update_baudrate; }\n")
    uart.write_bytes(raw)

# N20: an update must never start while the radio transmits (the loader runs none of our code, so a keyed transmitter
# would stay keyed for the whole update). mini_update_prepare() blocks new TX, waits up to 8 s for a transmission to
# end, and refuses the hand-over if it doesn't.
raw = uart.read_bytes()
if b"mini_update_prepare" not in raw:
    needle = b"            app_active_update_task_init(&info);\n"
    if needle not in raw: needle = needle.replace(b"\n", b"\r\n")
    assert raw.count(needle) == 1
    raw = raw.replace(needle,
        b"            { extern int mini_update_prepare(void); if (!mini_update_prepare()) break; }\n" + needle)
    uart.write_bytes(raw)

# Recover from a truncated or corrupt UART frame (e.g. cable/RF noise). The
# original SDK trusts the length and can wait forever or overrun its buffer.
raw = uart.read_bytes()
if b"Native frame bounds" not in raw:
    raw = raw.replace(b'#include "asm/clock.h"', b'#include "asm/clock.h"\n#include "asm/timer.h"', 1)
    needle = b"    u16 crc, crc0, i, ch;"
    assert raw.count(needle) == 1
    raw = raw.replace(needle, needle +
        b"\n    /* Native frame bounds and inter-chunk timeout. */\n"
        b"    static u32 last_rx_ms;\n    u32 now = timer_get_ms();\n"
        b"    if (rx_cnt && (u32)(now - last_rx_ms) > PACKET_TIMEOUT) rx_cnt = 0;\n"
        b"    last_rx_ms = now;\n")
    needle = b"        } else if (rx_cnt < 4) {\r\n            protocal_frame.raw_data[rx_cnt++] = ch;"
    if needle not in raw: needle = needle.replace(b"\r\n", b"\n")
    assert raw.count(needle) == 1
    raw = raw.replace(needle, needle +
        b"\n            if (rx_cnt == 4 && (!protocal_frame.data.length ||\n"
        b"                protocal_frame.data.length > PROTOCAL_SIZE)) rx_cnt = 0;\n")
    uart.write_bytes(raw)
