"""Read native Mini telemetry. Sends only the empty diagnostic request 0x70.

Usage: py mini_native_diag.py COM3
No pin-driving, firmware-writing, reset, or update commands are sent.
"""
import json
import serial
import struct
import sys
import time

def crc16(b):
    c = 0
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ (0x1021 if c & 0x8000 else 0)) & 0xffff
    return c

request = bytes.fromhex('aa55010070')
request += struct.pack('<H', crc16(request))
s = serial.Serial()
s.port, s.baudrate, s.timeout = sys.argv[1], 9600, 0.1
s.dtr = s.rts = False
s.open()
try:
    s.reset_input_buffer()
    b = bytearray()
    end = time.monotonic() + 8
    next_send = 0.0
    while time.monotonic() < end:
        # N16+: the radio's idle port alternates 9600/115200, so resend until it answers.
        if not b and time.monotonic() >= next_send:
            s.write(request)
            s.flush()
            next_send = time.monotonic() + 0.7
        b += s.read(256)
        start = b.find(b'\xaa\x55')
        if start < 0 or len(b) < start + 4:
            continue
        n = int.from_bytes(b[start+2:start+4], 'little')
        if not 1 <= n <= 528:
            raise SystemExit('Invalid response length')
        if len(b) < start + n + 6:
            continue
        f = bytes(b[start:start+n+6])
        if f == request:                     # our own request echoed by the cable: skip it, keep listening
            del b[:start+n+6]
            continue
        if crc16(f[:-2]) != int.from_bytes(f[-2:], 'little'):
            raise SystemExit('Response CRC mismatch')
        if f[4] != 0x70 or n not in (49,81,89,97,101,105,109,121,125,129,145,149):   # 145: N34+, 149: N36+
            raise SystemExit('Unexpected diagnostic response: ' + f.hex())
        d = struct.unpack('<%dI' % ((n-1)//4), f[5:-2])
        names = ['magic', 'build', 'ticks', 'rf_chip', 'spi_errors', 'pr_out_dir_die',
                 'pb_out', 'pb_dir', 'pb_die', 'spi1_con', 'iomap_con1', 'key_ptt']
        names += ['frequency_hz','frequency_word_readback','rf_state','rf_gpio','rssi','monitor','audible','tune_ok']
        names += ['nor_jedec','nor_errors']
        out = {}
        if len(d) >= 24:  # N08+: squelch / calibration
            out = {'squelch_reg': hex(d[22] & 0xffff), 'squelch_level': (d[22] >> 16) & 0xff,
                   'cal_loaded': bool(d[22] >> 24), 'freq_word_written': d[23] & 0x3ffffff, 'xtal_index': d[23] >> 26}
        if len(d) >= 25:  # N09+: receive tone
            if d[1] >= 10:  # N10 packing: kind 0-1, matched 2, value 3-15
                kind = d[24] & 3; matched = bool(d[24] & 4); val = (d[24] >> 3) & 0x1fff
            else:
                kind = d[24] & 0xf; matched = bool(d[24] & 0x8000); val = (d[24] >> 4) & 0x7ff
            out |= {'rx_tone': ['none', f'CTCSS {val/10:.1f}', f'DCS {val:03o}N', f'DCS {val:03o}I'][kind] if kind < 4 else hex(d[24]),
                    'tone_matched': matched, 'rf_status_0c': hex(d[24] >> 16)}
        if len(d) >= 26:  # N11+: settings store at NOR 0xFE000
            out |= {'settings_saves': d[25] & 0xffff, 'settings_failures': (d[25] >> 16) & 0xff,
                    'settings_next_slot': (d[25] >> 24) & 0x7f, 'settings_locked': bool(d[25] >> 31)}
        if len(d) >= 27:  # N12+: TX test state
            out |= {'tx_keyed': bool(d[26] & 1), 'tx_power': 'HIGH' if d[26] & 2 else 'LOW', 'pa_cal_loaded': bool(d[26] & 4),
                    'tx_hold': bool(d[26] & 8), 'tx_count': (d[26] >> 4) & 0xff, 'tx_aborts': (d[26] >> 12) & 0xf,
                    'last_pa_reg36': hex(d[26] >> 16)}
        if len(d) >= 30:  # N16+: TEXT packet counters since TEXT opened, PC link state
            out |= {'text_sync': d[27] & 0xff, 'text_good': (d[27] >> 8) & 0xff, 'text_bad': (d[27] >> 16) & 0xff,
                    'text_sent': d[27] >> 24, 'text_replies_seen': d[28] & 0xff, 'text_delivered': (d[28] >> 8) & 0xff,
                    'pc_port_115200_now': bool(d[29] & 1), 'pc_alternation_off': bool(d[29] & 2),
                    'pc_session_state': (d[29] >> 2) & 3, 'pc_sessions': (d[29] >> 8) & 0xff, 'pc_frames': d[29] >> 16}
        if len(d) >= 31:  # N19+: FREQ ERR (other radio's carrier minus ours)
            fe = d[30] & 0xffffff
            fe = fe - 0x1000000 if fe & 0x800000 else fe
            out |= {'freq_err_state': ['none', 'measured', 'no carrier', 'measuring'][min(d[30] >> 24, 3)], 'freq_err_hz': fe}
        if len(d) >= 32:  # N21+: the Mini's own REG 0x3D / 0x73, read when TEXT last opened (0 = not yet)
            out |= {'mini_reg3d_before_text': hex(d[31] >> 16), 'mini_reg73_before_text': hex(d[31] & 0xffff)}
        if len(d) >= 36:  # N34+: the RF chip's power-on values of the Mini-only init registers
            vals = [d[32 + k // 2] >> (16 * (k % 2)) & 0xffff for k in range(7)]
            out |= {'rf_poweron': {r: hex(v) for r, v in zip(['0x17', '0x25', '0x3a', '0x42', '0x4c', '0x4d', '0x4e'], vals)}}
        if len(d) >= 37:  # N36+: battery (PB3 >> 2), smoothed, bars (6 = critical, 255 = not yet), lowest threshold
            out |= {'battery_raw': d[36] & 0xff, 'battery_avg': (d[36] >> 8) & 0xff, 'battery_bars': (d[36] >> 16) & 0xff,
                    'battery_threshold_low': d[36] >> 24}
        print(json.dumps({k: v if k in ('build','ticks','spi_errors','frequency_hz','frequency_word_readback','rssi','monitor','audible','tune_ok') else hex(v)
                          for k, v in zip(names,d)} | out, indent=2))
        break
    else:
        raise SystemExit('No native diagnostic reply (nothing changed). Raw: ' + b.hex())
finally:
    s.close()
