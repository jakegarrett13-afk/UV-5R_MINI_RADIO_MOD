/* N26: SETTINGS, laid out as the UV-5RM custom firmware's settings app (apps/settings/common.rs): groups CURRENT
 * CHANNEL, ALL CHANNELS, AUDIO, DISPLAY, KEYS, SYSTEM; MENU edits, UP/DOWN change (live), MENU keeps, EXIT puts the
 * old value back. CURRENT CHANNEL edits the VFO (saved with the settings) or, in channel mode, the stored channel
 * (written to its slot when MENU confirms, as save_channel_item). Not on the Mini yet, so not listed: ANI (DTMF ID),
 * VOX, scrambler, dual watch (TDR), battery save, 1750 Hz tone, RIT, battery calibration, observer position.
 * Included into mini_app.c after draw_list. */
static bool tune_vfo(u32 hz);
static bool api_chan_write(u16 num, const u8 *buf, u16 len);
static bool api_chan_read(u16 num, u8 *out, u16 len);
static bool channel_step(int dir, bool from_start);
static void settings_tick(bool now);

/* ---- the new settings (saved in "MS2" records next to the "MS1" ones, same sector and store) ---- */
static bool vfo_narrow[2], vfo_busy[2];
static u8 vfo_shift[2];                 /* 0 off, 1 +, 2 - */
static u16 vfo_offset_khz[2] = {600, 5000};
static u16 vfo_txtone[2];               /* index into tone_at (0 = none) */
static u8 mic_gain;                     /* 0 = calibration value, 1..64 = REG 0x7D value 0..63 */
static bool tx_inhibit, ste_on;
static u8 scan_mode = 1;                /* 0 TIME, 1 CARRIER, 2 STOP (5RM ScanMd) */
static u8 ch_disp = 2;                  /* 0 FREQ, 1 NAME, 2 NAME + FREQ */
static u8 roger_mode;                   /* 0 off, 1 ROGER, 2 COIN, 3 MORSE K, 4 CHIRP */
static u8 rptrl;                        /* extra carrier after release, x 100 ms */
static u8 light_mode = 2;               /* 0 always on, 5 / 10 / 20 / 30 / 60 s */
static u8 autolock;                     /* 0 off, 10 / 30 / 60 s */
enum { KF_NONE, KF_WN, KF_MONITOR, KF_MODE, KF_TEXT, KF_SCAN, KF_POWER, KF_LIGHT, KF_COUNT };
static const char *const kf_name[KF_COUNT] = {"NONE", "W/N", "MONITOR", "MODE", "TEXT", "SCAN", "POWER", "LIGHT"};
static u8 kf_side_sh = KF_POWER, kf_side_lg = KF_LIGHT, kf_ptta = KF_TEXT, kf_band_sh = KF_MODE, kf_band_lg = KF_NONE;
#define FLOCK_COUNT 10
static const char *const flock_name[FLOCK_COUNT] = {"OFF", "FCC HAM", "GB HAM", "400-430", "400-438", "PMR446", "GMRS/FRS", "CA HAM", "CE HAM", "ALL LOCK"};
static u8 flock;                        /* 0 = the Mini's hardware TX range 136-174 / 400-480 MHz */
static bool boot_logo = true, boot_snd = true;   /* N27: BOOT SND, the Mini's own start-up chime */
static const u16 light_s[6] = {0, 5, 10, 20, 30, 60};
static const u8 autolock_s[4] = {0, 10, 30, 60};

/* TX band lock (5RM band_lock_label presets); always inside the Mini's 136-174 / 400-480 MHz. */
static bool flock_allows(u32 hz)
{
    u32 m = hz / 1000;                   /* kHz */
    switch (flock) {
    case 1: return (m >= 144000 && m <= 148000) || (m >= 420000 && m <= 450000);
    case 2: return (m >= 144000 && m <= 146000) || (m >= 430000 && m <= 440000);
    case 3: return m >= 400000 && m <= 430000;
    case 4: return m >= 400000 && m <= 438000;
    case 5: return hz >= 446000000u && hz <= 446200000u;
    case 6: return (hz >= 462550000u && hz <= 462725000u) || (hz >= 467550000u && hz <= 467725000u);
    case 7: return (m >= 144000 && m <= 148000) || (m >= 430000 && m <= 450000);
    case 8: return (m >= 144000 && m <= 146000) || (m >= 430000 && m <= 440000);
    case 9: return false;
    default: return true;
    }
}
static u32 vfo_tx_hz(unsigned side)
{
    u32 hz = vfo_hz[side], off = (u32)vfo_offset_khz[side] * 1000u;
    return vfo_shift[side] == 1 ? hz + off : vfo_shift[side] == 2 ? (hz > off ? hz - off : 0) : hz;
}
static bool cur_narrow(void) { return mr_mode ? ch.narrow : vfo_narrow[vfo_sel]; }
static bool cur_busy(void) { return mr_mode ? ch.busy : vfo_busy[vfo_sel]; }

/* ---- MS2 record: magic[4], offset A/B kHz (u16), TX tone A/B (u16 index), flags (narrow A/B, busy A/B, TX inhibit,
 * STE), shift (A bits 0-1, B 2-3), mic, then +1-encoded bytes (0 = default) for scan mode, display, roger, tail,
 * light, auto lock, the five key functions, TX lock, boot screen; 2 reserved; CRC16 of bytes 0-29. ---- */
static const u8 ext_magic[4] = {'M', 'S', '2', 0};
static u8 ext_saved[SET_SLOT];
static bool ext_have;
static void ext_build(u8 *r)
{
    u16 c; unsigned i;
    memset(r, 0, SET_SLOT);
    memcpy(r, ext_magic, 4);
    for (i = 0; i < 2; ++i) {
        r[4 + 2 * i] = (u8)vfo_offset_khz[i]; r[5 + 2 * i] = (u8)(vfo_offset_khz[i] >> 8);
        r[8 + 2 * i] = (u8)vfo_txtone[i]; r[9 + 2 * i] = (u8)(vfo_txtone[i] >> 8);
    }
    r[12] = vfo_narrow[0] | (vfo_narrow[1] << 1) | (vfo_busy[0] << 2) | (vfo_busy[1] << 3) | (tx_inhibit << 4) | (ste_on << 5);
    r[13] = (u8)((vfo_shift[0] & 3) | ((vfo_shift[1] & 3) << 2));
    r[14] = mic_gain;
    r[15] = scan_mode + 1; r[16] = ch_disp + 1; r[17] = roger_mode + 1; r[18] = rptrl + 1; r[19] = light_mode + 1;
    r[20] = autolock + 1; r[21] = kf_side_sh + 1; r[22] = kf_side_lg + 1; r[23] = kf_ptta + 1; r[24] = kf_band_sh + 1;
    r[25] = kf_band_lg + 1; r[26] = flock + 1; r[27] = boot_logo + 1; r[28] = boot_snd + 1;
    c = crc16_ccitt(r, 30); r[30] = (u8)c; r[31] = (u8)(c >> 8);
}
static void ext_apply(const u8 *r)
{
    unsigned i;
    for (i = 0; i < 2; ++i) {
        u16 off = r[4 + 2 * i] | (r[5 + 2 * i] << 8), t = r[8 + 2 * i] | (r[9 + 2 * i] << 8);
        vfo_offset_khz[i] = off; vfo_txtone[i] = t < TONES ? t : 0;
        vfo_narrow[i] = (r[12] >> i) & 1; vfo_busy[i] = (r[12] >> (2 + i)) & 1;
        vfo_shift[i] = (r[13] >> (2 * i)) & 3; if (vfo_shift[i] > 2) vfo_shift[i] = 0;
    }
    tx_inhibit = (r[12] >> 4) & 1; ste_on = (r[12] >> 5) & 1;
    mic_gain = r[14] <= 64 ? r[14] : 0;
#define EXT_GET(var, at, n) if (r[at] >= 1 && r[at] <= (n)) var = (u8)(r[at] - 1)
    EXT_GET(scan_mode, 15, 3); EXT_GET(ch_disp, 16, 3); EXT_GET(roger_mode, 17, 5); EXT_GET(rptrl, 18, 11);
    EXT_GET(light_mode, 19, 6); EXT_GET(autolock, 20, 4); EXT_GET(kf_side_sh, 21, KF_COUNT); EXT_GET(kf_side_lg, 22, KF_COUNT);
    EXT_GET(kf_ptta, 23, KF_COUNT); EXT_GET(kf_band_sh, 24, KF_COUNT); EXT_GET(kf_band_lg, 25, KF_COUNT); EXT_GET(flock, 26, FLOCK_COUNT);
#undef EXT_GET
    if (r[27] >= 1 && r[27] <= 2) boot_logo = r[27] == 2;
    if (r[28] >= 1 && r[28] <= 2) boot_snd = r[28] == 2;
}
/* The newest valid MS2 record in the settings sector (settings_scan only looks at MS1). */
static bool ext_scan(u8 *out)
{
    u8 r[SET_SLOT]; unsigned i; bool found = false;
    if (set_locked) return false;
    for (i = 0; i < SET_SLOTS; ++i) {
        if (!nor_read(SET_SECTOR + i * SET_SLOT, r, SET_SLOT) || all_ff(r, SET_SLOT)) break;
        if (!memcmp(r, ext_magic, 4) && crc16_ccitt(r, 30) == (r[30] | (r[31] << 8))) { memcpy(out, r, SET_SLOT); found = true; }
    }
    return found;
}
/* As settings_tick: after ~3 s unchanged, or now. Slot 0 must stay an MS1 record (settings_scan's ownership check), so
 * when the sector is full the MS1 record is written first (that erases), then this one. */
static void ext_tick(bool now)
{
    static u8 pending[SET_SLOT];
    static unsigned age;
    u8 r[SET_SLOT];
    if (set_locked) return;
    ext_build(r);
    if (ext_have && !memcmp(r, ext_saved, SET_SLOT)) { age = 0; return; }
    if (memcmp(r, pending, SET_SLOT)) { memcpy(pending, r, SET_SLOT); age = 0; }
    if (!now && ++age < 12) return;
    age = 0;
    if (set_next >= SET_SLOTS || set_next == 0) {
        u8 m[SET_SLOT];
        settings_build(m);
        if (!settings_write(m)) { ext_have = false; return; }
        memcpy(set_saved, m, SET_SLOT); set_have = true;
    }
    if (settings_write(r)) { memcpy(ext_saved, r, SET_SLOT); ext_have = true; }
    else ext_have = false;
}

/* ---- channel edits: the loaded channel `ch` changes live; MENU writes it back to its slot ---- */
static void put_lbcd(u8 *b, u32 hz)
{
    u32 v = hz / 10; unsigned i;
    for (i = 0; i < 4; ++i) { u32 two = v % 100; b[i] = (u8)(((two / 10) << 4) | (two % 10)); v /= 100; }
}
static bool chan_save_current(void)
{
    u8 r[32]; u16 code;
    if (!mr_mode || !nor_read(mr_index * 32u, r, 32)) return false;
    put_lbcd(r, ch.rx_hz);
    if (ch.tx_hz) put_lbcd(r + 4, ch.tx_hz);
    code = ch.tone.kind == MINI_TONE_NONE ? 0 : api_subaudio_code_of_index((int)tone_index(ch.tone) - 1);
    r[8] = (u8)code; r[9] = (u8)(code >> 8);
    code = ch.txtone.kind == MINI_TONE_NONE ? 0 : api_subaudio_code_of_index((int)tone_index(ch.txtone) - 1);
    r[10] = (u8)code; r[11] = (u8)(code >> 8);
    r[14] = ch.power;
    r[15] = (u8)((r[15] & ~0x4c) | (ch.narrow ? 0x40 : 0) | (ch.busy ? 0x08 : 0) | (ch.scan_add ? 0x04 : 0));
    return api_chan_write((u16)mr_index, r, 32);
}

/* ---- items ---- */
enum { SG_CHAN, SG_ALL, SG_AUDIO, SG_DISP, SG_KEYS, SG_SYS, SG_COUNT };
static const char *const sg_name[SG_COUNT] = {"CURRENT CHANNEL", "ALL CHANNELS", "AUDIO", "DISPLAY", "KEYS", "SYSTEM"};
enum {
    IT_WN, IT_PWR, IT_RXT, IT_TXT, IT_SHIFT, IT_OFFSET, IT_BUSY, IT_SCANADD, IT_SAVECH, IT_DELCH,
    IT_SQL, IT_MIC, IT_STEP, IT_TOT, IT_TXINH, IT_SCANMD, IT_CHDISP,
    IT_BEEP, IT_ROGER, IT_STE, IT_RPTRL,
    IT_LIGHT, IT_AUTOLK,
    IT_SIDE_SH, IT_SIDE_LG, IT_PTTA, IT_BAND_SH, IT_BAND_LG,
    IT_FLOCK, IT_BOOT, IT_BOOTSND, IT_FERR, IT_CHIP, IT_VER, IT_RESET, IT_COUNT
};
static const char *const it_label[IT_COUNT] = {
    "W/N", "PWR", "RX TONE", "TX TONE", "SHIFT", "OFFSET", "BUSY LOCK", "SCAN ADD", "SAVE AS CH", "DELETE CH",
    "SQL", "MIC GAIN", "STEP", "TOT", "TX INHIB", "SCAN MD", "CH DISP",
    "BEEP", "ROGER", "STE", "RPT TAIL",
    "LIGHT", "AUTO LOCK",
    "SIDE-SH", "SIDE-LG", "PTT-A", "[-]-SH", "[-]-LG",
    "TX LOCK", "BOOT SCR", "BOOT SND", "FREQ ERR", "RF CHIP", "VERSION", "RESET",
};
static const u8 sg_first[SG_COUNT + 1] = {IT_WN, IT_SQL, IT_BEEP, IT_LIGHT, IT_SIDE_SH, IT_FLOCK, IT_COUNT};
static bool chan_item(unsigned it) { return it <= IT_SCANADD; }
static unsigned sg_sel, it_sel;          /* group, item (absolute IT_*) */
static bool in_items, it_edit, reset_armed;
/* N27 SAVE AS CH (VFO mode): stage 1 picks an empty channel, stage 2 types its name (multi-tap, as TEXT). */
static u8 save_stage;
static unsigned save_ch;                 /* 0-based slot */
static u8 save_num[4]; static unsigned save_num_len;
static char save_name[13]; static unsigned save_name_len;
static u8 tap_key = 0xff, tap_idx; static u32 tap_ms;
static const char *const name_taps[10] = {" 0", "-/.1", "ABC2", "DEF3", "GHI4", "JKL5", "MNO6", "PQRS7", "TUV8", "WXYZ9"};
static int it_snap;
static u8 off_digits[5]; static unsigned off_len;   /* OFFSET typed as DD.DDD MHz */
static bool beep_demo2;
/* Current value of an item as an int (tones: tone_at index). */
static int it_get(unsigned it)
{
    switch (it) {
    case IT_WN: return cur_narrow();
    case IT_PWR: return mr_mode ? ch.power != 0 : !tx_high;
    case IT_RXT: return mr_mode ? (int)tone_index(ch.tone) : (int)vfo_tone[vfo_sel];
    case IT_TXT: return mr_mode ? (int)tone_index(ch.txtone) : (int)vfo_txtone[vfo_sel];
    case IT_SHIFT:
        if (mr_mode) return !ch.tx_hz || ch.tx_hz == ch.rx_hz ? 0 : ch.tx_hz > ch.rx_hz ? 1 : 2;
        return vfo_shift[vfo_sel];
    case IT_OFFSET:
        if (mr_mode) return ch.tx_hz ? (int)((ch.tx_hz > ch.rx_hz ? ch.tx_hz - ch.rx_hz : ch.rx_hz - ch.tx_hz) / 1000u) : 0;
        return vfo_offset_khz[vfo_sel];
    case IT_BUSY: return cur_busy();
    case IT_SCANADD: return mr_mode && ch.scan_add;
    case IT_SQL: return radio.squelch;
    case IT_MIC: return mic_gain;
    case IT_STEP: return step_idx;
    case IT_TOT: return tot_level;
    case IT_TXINH: return tx_inhibit;
    case IT_SCANMD: return scan_mode;
    case IT_CHDISP: return ch_disp;
    case IT_BEEP: return beep_mode;
    case IT_ROGER: return roger_mode;
    case IT_STE: return ste_on;
    case IT_RPTRL: return rptrl;
    case IT_LIGHT: return light_mode;
    case IT_AUTOLK: return autolock;
    case IT_SIDE_SH: return kf_side_sh;
    case IT_SIDE_LG: return kf_side_lg;
    case IT_PTTA: return kf_ptta;
    case IT_BAND_SH: return kf_band_sh;
    case IT_BAND_LG: return kf_band_lg;
    case IT_FLOCK: return flock;
    case IT_BOOT: return boot_logo;
    case IT_BOOTSND: return boot_snd;
    default: return 0;
    }
}
static void retune_current(void) { if (mr_mode) tune_channel(); else tune_vfo(vfo_hz[vfo_sel]); }
/* Set an item (live). Channel-mode changes go to `ch`; the slot is written on MENU. */
static void it_set(unsigned it, int v)
{
    switch (it) {
    case IT_WN: if (mr_mode) ch.narrow = v; else vfo_narrow[vfo_sel] = v; retune_current(); break;
    case IT_PWR: if (mr_mode) ch.power = (u8)v; else tx_high = !v; break;
    case IT_RXT:
        if (mr_mode) ch.tone = tone_at((unsigned)v); else vfo_tone[vfo_sel] = (unsigned)v;
        retune_current(); break;
    case IT_TXT: if (mr_mode) ch.txtone = tone_at((unsigned)v); else vfo_txtone[vfo_sel] = (u16)v; break;
    case IT_SHIFT:
        if (mr_mode) {
            u32 off = ch.tx_hz ? (ch.tx_hz > ch.rx_hz ? ch.tx_hz - ch.rx_hz : ch.rx_hz - ch.tx_hz) : 0;
            if (!ch.tx_hz) break;                                /* receive-only channels stay receive-only */
            if (!off) off = ch.rx_hz >= 400000000u ? 5000000u : 600000u;
            ch.tx_hz = v == 1 ? ch.rx_hz + off : v == 2 ? ch.rx_hz - off : ch.rx_hz;
        } else vfo_shift[vfo_sel] = (u8)v;
        break;
    case IT_OFFSET:
        if (mr_mode) {
            if (!ch.tx_hz || ch.tx_hz == ch.rx_hz) break;
            ch.tx_hz = ch.tx_hz > ch.rx_hz ? ch.rx_hz + (u32)v * 1000u : ch.rx_hz - (u32)v * 1000u;
        } else vfo_offset_khz[vfo_sel] = (u16)v;
        break;
    case IT_BUSY: if (mr_mode) ch.busy = v; else vfo_busy[vfo_sel] = v; break;
    case IT_SCANADD: if (mr_mode) ch.scan_add = v; break;
    case IT_SQL: mini_rf_set_squelch(&radio, (u8)v); break;
    case IT_MIC: mic_gain = (u8)v; break;
    case IT_STEP: step_idx = (u8)v; break;
    case IT_TOT: tot_level = (u8)v; break;
    case IT_TXINH: tx_inhibit = v; break;
    case IT_SCANMD: scan_mode = (u8)v; break;
    case IT_CHDISP: ch_disp = (u8)v; break;
    case IT_BEEP: beep_mode = (u8)v; radio.beep_style = beep_mode; beep_demo2 = beep_mode != 0; break;
    case IT_ROGER: roger_mode = (u8)v; break;
    case IT_STE: ste_on = v; break;
    case IT_RPTRL: rptrl = (u8)v; break;
    case IT_LIGHT: light_mode = (u8)v; break;
    case IT_AUTOLK: autolock = (u8)v; break;
    case IT_SIDE_SH: kf_side_sh = (u8)v; break;
    case IT_SIDE_LG: kf_side_lg = (u8)v; break;
    case IT_PTTA: kf_ptta = (u8)v; break;
    case IT_BAND_SH: kf_band_sh = (u8)v; break;
    case IT_BAND_LG: kf_band_lg = (u8)v; break;
    case IT_FLOCK: flock = (u8)v; break;
    case IT_BOOT: boot_logo = v; break;
    case IT_BOOTSND: boot_snd = v; break;
    }
}
/* lo, hi, wrap */
static void it_range(unsigned it, int *lo, int *hi, bool *wrap)
{
    *lo = 0; *wrap = false;
    switch (it) {
    case IT_RXT: case IT_TXT: *hi = TONES - 1; *wrap = true; break;
    case IT_SHIFT: *hi = 2; break;
    case IT_OFFSET: *hi = 65000; break;
    case IT_SQL: *hi = 9; break;
    case IT_MIC: *hi = 64; break;
    case IT_STEP: *hi = 7; break;
    case IT_TOT: *hi = 40; break;
    case IT_SCANMD: case IT_CHDISP: *hi = 2; break;
    case IT_BEEP: case IT_ROGER: *hi = 4; break;
    case IT_RPTRL: *hi = 10; break;
    case IT_LIGHT: *hi = 5; break;
    case IT_AUTOLK: *hi = 3; break;
    case IT_SIDE_SH: case IT_SIDE_LG: case IT_PTTA: case IT_BAND_SH: case IT_BAND_LG: *hi = KF_COUNT - 1; *wrap = true; break;
    case IT_FLOCK: *hi = FLOCK_COUNT - 1; *wrap = true; break;
    default: *hi = 1; *wrap = true; break;
    }
}
static bool it_readonly(unsigned it)
{
    if (it == IT_CHIP || it == IT_VER) return true;
    if (it == IT_SCANADD && !mr_mode) return true;
    if (it == IT_SAVECH && mr_mode) return true;
    if (it == IT_DELCH && !mr_mode) return true;
    if ((it == IT_SHIFT || it == IT_OFFSET) && mr_mode && !ch.tx_hz) return true;
    if (it == IT_OFFSET && mr_mode && ch.tx_hz == ch.rx_hz) return true;
    return false;
}
static void it_value(unsigned it, char *b, unsigned n)
{
    int v = it_get(it);
    static const char *const shift_s[3] = {"OFF", "+", "-"}, *const scanmd_s[3] = {"TIME", "CARRIER", "STOP"};
    static const char *const chdisp_s[3] = {"FREQ", "NAME", "NAME+F"}, *const roger_s[5] = {"OFF", "ROGER", "COIN", "MORSE K", "CHIRP"};
    switch (it) {
    case IT_WN: snprintf(b, n, "%s", v ? "NARROW" : "WIDE"); break;
    case IT_PWR: snprintf(b, n, "%s", v ? "LOW" : "HIGH"); break;
    case IT_RXT: case IT_TXT: tone_value(tone_at((unsigned)v), b, n); break;
    case IT_SHIFT: snprintf(b, n, "%s", mr_mode && !ch.tx_hz ? "RX ONLY" : shift_s[v]); break;
    case IT_OFFSET:
        if (it_edit && it_sel == IT_OFFSET && off_len) {
            char d[8]; unsigned i, k = 0;
            for (i = 0; i < 5; ++i) { if (i == 2) d[k++] = '.'; d[k++] = i < off_len ? (char)('0' + off_digits[i]) : '-'; }
            d[k] = 0; snprintf(b, n, "%s", d);
        } else snprintf(b, n, "%u.%03u", (unsigned)v / 1000u, (unsigned)v % 1000u);
        break;
    case IT_SCANADD: snprintf(b, n, "%s", !mr_mode ? "N/A" : v ? "ON" : "OFF"); break;
    case IT_BUSY: case IT_TXINH: case IT_STE: snprintf(b, n, "%s", v ? "ON" : "OFF"); break;
    case IT_SQL: snprintf(b, n, "%d", v); break;
    case IT_MIC: if (v) snprintf(b, n, "%d", v - 1); else snprintf(b, n, "CAL %u", mic_cal & 0x3f); break;
    case IT_STEP: snprintf(b, n, "%u.%02uK", (unsigned)(steps_hz[v] / 1000), (unsigned)(steps_hz[v] % 1000 / 10)); break;
    case IT_TOT: if (v) snprintf(b, n, "%dS", v * 15); else snprintf(b, n, "OFF"); break;
    case IT_SCANMD: snprintf(b, n, "%s", scanmd_s[v]); break;
    case IT_CHDISP: snprintf(b, n, "%s", chdisp_s[v]); break;
    case IT_BEEP: if (v) snprintf(b, n, "%d", v); else snprintf(b, n, "OFF"); break;
    case IT_ROGER: snprintf(b, n, "%s", roger_s[v]); break;
    case IT_RPTRL: if (v) snprintf(b, n, "%dMS", v * 100); else snprintf(b, n, "OFF"); break;
    case IT_LIGHT: if (v) snprintf(b, n, "%uS", light_s[v]); else snprintf(b, n, "ALWAYS"); break;
    case IT_AUTOLK: if (v) snprintf(b, n, "%uS", autolock_s[v]); else snprintf(b, n, "OFF"); break;
    case IT_SIDE_SH: case IT_SIDE_LG: case IT_PTTA: case IT_BAND_SH: case IT_BAND_LG: snprintf(b, n, "%s", kf_name[v]); break;
    case IT_FLOCK: snprintf(b, n, "%s", flock_name[v]); break;
    case IT_BOOT: snprintf(b, n, "%s", v ? "LOGO" : "NONE"); break;
    case IT_BOOTSND: snprintf(b, n, "%s", v ? "ON" : "OFF"); break;
    case IT_SAVECH: snprintf(b, n, "%s", mr_mode ? "N/A" : "MENU"); break;
    case IT_DELCH: snprintf(b, n, "%s", !mr_mode ? "N/A" : reset_armed ? "SURE? MENU" : "MENU"); break;
    case IT_FERR:
        if (fe_state == 1) snprintf(b, n, "%c%dHZ", fe_hz < 0 ? '-' : '+', fe_hz < 0 ? -fe_hz : fe_hz);
        else snprintf(b, n, "%s", fe_state == 2 ? "NO SIGNAL" : fe_state == 3 ? "LISTENING" : "MENU=TEST");
        break;
    case IT_CHIP: snprintf(b, n, "0x%04X", radio.chip_id); break;
    case IT_VER: snprintf(b, n, "N29"); break;
    case IT_RESET: snprintf(b, n, "%s", reset_armed ? "SURE? MENU" : ""); break;
    default: b[0] = 0;
    }
}
/* ---- N27: SAVE AS CH / DELETE CH ---- */
static bool slot_empty(unsigned i) { u8 r[32]; return i < CHANNELS && !api_chan_read((u16)i, r, 32); }
static unsigned next_empty(unsigned from, int dir)
{
    unsigned k;
    for (k = 1; k <= CHANNELS; ++k) { unsigned i = (from + CHANNELS + (unsigned)(dir * (int)k)) % CHANNELS; if (slot_empty(i)) return i; }
    return CHANNELS;
}
/* The tapped letter is final after 1 s or when another key is pressed. */
static void name_tap_expire(void) { if (tap_key != 0xff && jiffies_msec() - tap_ms >= 1000) tap_key = 0xff; }
static void save_start(void)
{
    save_ch = slot_empty(0) ? 0 : next_empty(0, 1);
    if (save_ch >= CHANNELS) { set_notice("NO EMPTY CHANNEL"); return; }
    save_stage = 1; save_num_len = 0; save_name_len = 0; save_name[0] = 0; tap_key = 0xff;
}
static bool save_commit(void)
{
    u8 r[32]; u16 code; u32 tx = vfo_tx_hz(vfo_sel);
    memset(r, 0, sizeof(r));
    put_lbcd(r, vfo_hz[vfo_sel]);
    put_lbcd(r + 4, tx ? tx : vfo_hz[vfo_sel]);
    code = vfo_tone[vfo_sel] ? api_subaudio_code_of_index((int)vfo_tone[vfo_sel] - 1) : 0;
    r[8] = (u8)code; r[9] = (u8)(code >> 8);
    code = vfo_txtone[vfo_sel] ? api_subaudio_code_of_index((int)vfo_txtone[vfo_sel] - 1) : 0;
    r[10] = (u8)code; r[11] = (u8)(code >> 8);
    r[14] = tx_high ? 0 : 1;
    r[15] = 0x04 | (vfo_narrow[vfo_sel] ? 0x40 : 0) | (vfo_busy[vfo_sel] ? 0x08 : 0);   /* scan add on */
    memset(r + 16, 0xff, 4);                                    /* no scrambler; in every MODE band */
    memset(r + 20, 0xff, 12);
    memcpy(r + 20, save_name, save_name_len);
    return api_chan_write((u16)save_ch, r, 32);
}
static void save_key(u8 key)
{
    int d = key_digit(key);
    char note[24];
    if (save_stage == 1) {
        if (d >= 0) {
            unsigned i, n = 0;
            if (save_num_len >= 4) save_num_len = 0;
            save_num[save_num_len++] = (u8)d;
            for (i = 0; i < save_num_len; ++i) n = n * 10 + save_num[i];
            if (n >= 1 && n <= CHANNELS) save_ch = n - 1;
        } else if (key == K_UP || key == K_DOWN) {
            unsigned i = next_empty(save_ch, key == K_UP ? 1 : -1);
            if (i < CHANNELS) save_ch = i;
            save_num_len = 0;
        } else if (key == K_MENU) {
            if (!slot_empty(save_ch)) set_notice("CHANNEL IN USE");
            else { save_stage = 2; save_num_len = 0; }
        } else if (key == K_EXIT) save_stage = 0;
        return;
    }
    name_tap_expire();
    if (d >= 0) {
        const char *set = name_taps[d];
        if (tap_key == (u8)d && save_name_len) {
            tap_idx = (u8)((tap_idx + 1) % strlen(set));
            save_name[save_name_len - 1] = set[tap_idx];
        } else if (save_name_len < 12) {
            tap_idx = 0;
            save_name[save_name_len++] = set[0];
        }
        save_name[save_name_len] = 0;
        tap_key = (u8)d; tap_ms = jiffies_msec();
    } else if (key == K_EXIT) {
        tap_key = 0xff;
        if (save_name_len) save_name[--save_name_len] = 0; else save_stage = 1;
    } else if (key == K_MENU) {
        while (save_name_len && save_name[save_name_len - 1] == ' ') save_name[--save_name_len] = 0;
        if (save_commit()) { snprintf(note, sizeof(note), "SAVED AS CH %03u", save_ch + 1u); set_notice(note); }
        else set_notice("SAVE FAILED");
        save_stage = 0;
        in_items = false;
        scr = SCR_STANDBY;                     /* the notice shows on the main screen */
        screen_full();
    }
}
static void delete_current(void)
{
    u8 blank[32]; unsigned gone = mr_index;
    char note[24];
    memset(blank, 0xff, sizeof(blank));
    if (!mr_mode || !api_chan_write((u16)gone, blank, 32)) { set_notice("DELETE FAILED"); return; }
    if (side_mr[vfo_sel ^ 1] && side_ch[vfo_sel ^ 1] == gone) side_mr[vfo_sel ^ 1] = false;
    if (!channel_step(1, false)) { mr_mode = false; side_mr[vfo_sel] = false; tune_vfo(vfo_hz[vfo_sel]); }
    else side_ch[vfo_sel] = (u16)mr_index;
    snprintf(note, sizeof(note), "CH %03u DELETED", gone + 1u);
    set_notice(note);
    in_items = false;
    scr = SCR_STANDBY;
    screen_full();
}
static void draw_save(void)
{
    char b[32];
    bool empty;
    g_backdrop(0, 128);
    g_fill(0, 0, 128, 14, C_BLUE);
    g_text_center(&F6, F6.base + 2, "SAVE CHANNEL", C_WHITE);
    snprintf(b, sizeof(b), "%u.%04u", (unsigned)(vfo_hz[vfo_sel] / 1000000u), (unsigned)(vfo_hz[vfo_sel] / 100u % 10000u));
    g_text(&F6, 4, 18 + F6.base, "FREQ", C_DIM);
    g_text(&F6, 40, 18 + F6.base, b, C_WHITE);
    g_text(&F6, 4, 36 + F6.base, "CH", C_DIM);
    if (save_stage == 1 && save_num_len) {
        unsigned i;
        for (i = 0; i < save_num_len; ++i) b[i] = (char)('0' + save_num[i]);
        b[i] = 0;
        g_text(&F9, 40, 32 + F9.base, b, C_AMBER);
    } else {
        snprintf(b, sizeof(b), "%03u", save_ch + 1u);
        g_text(&F9, 40, 32 + F9.base, b, save_stage == 1 ? C_AMBER : C_WHITE);
    }
    if (save_stage == 1) {
        empty = slot_empty(save_ch);
        g_text(&F5, 72, 38 + F5.base, empty ? "EMPTY" : "IN USE", empty ? C_GREEN : C_ALERT);
    }
    g_text(&F6, 4, 58 + F6.base, "NAME", C_DIM);
    g_fill(4, 70, 120, 20, save_stage == 2 ? 0x0210 : C_BLACK);
    if (save_stage == 2) {
        name_tap_expire();
        snprintf(b, sizeof(b), "%s%s", save_name, tap_key == 0xff && save_name_len < 12 ? "_" : "");
        g_text(&F9, 6, 71 + F9.base, b, C_WHITE);
    } else g_text(&F6, 6, 75 + F6.base, "(NEXT STEP)", C_DIM);
    g_fill(0, 100, 128, 28, C_BLACK);
    if (save_stage == 1) {
        g_text(&F5, 4, 104 + F5.base, "UP/DN EMPTY  0-9 NUMBER", C_DIM);
        g_text(&F5, 4, 116 + F5.base, "MENU NEXT  EXIT BACK", C_DIM);
    } else {
        g_text(&F5, 4, 104 + F5.base, "KEYS TYPE  EXIT DELETE", C_DIM);
        g_text(&F5, 4, 116 + F5.base, "MENU SAVE (NAME OPTIONAL)", C_DIM);
    }
}
static void sg_label(unsigned i, char *b, unsigned n) { snprintf(b, n, "%s", sg_name[i]); }
static void it_label_at(unsigned i, char *b, unsigned n) { snprintf(b, n, "%s", it_label[sg_first[sg_sel] + i]); }
static void it_value_at(unsigned i, char *b, unsigned n) { it_value(sg_first[sg_sel] + i, b, n); }
static void draw_settings(void)
{
    if (save_stage) { draw_save(); return; }
    if (!in_items) { draw_list("SETTINGS", SG_COUNT, sg_sel, false, sg_label, 0); return; }
    draw_list(sg_sel == SG_CHAN ? (mr_mode ? "THIS CHANNEL" : (vfo_sel ? "VFO B" : "VFO A")) : sg_name[sg_sel],
              (unsigned)(sg_first[sg_sel + 1] - sg_first[sg_sel]), it_sel - sg_first[sg_sel], it_edit, it_label_at, it_value_at);
}
static void settings_open(void) { in_items = false; it_edit = false; reset_armed = false; save_stage = 0; scr = SCR_MENU; }
static void settings_ui_reset(void) { it_edit = false; reset_armed = false; save_stage = 0; }
static void settings_defaults(void);
/* Returns false when EXIT leaves the settings (back to the launcher). */
static bool settings_key(u8 key)
{
    int d = key_digit(key);
    if (save_stage) { save_key(key); return true; }
    if (!in_items) {
        if (key == K_UP) sg_sel = (sg_sel + SG_COUNT - 1) % SG_COUNT;
        else if (key == K_DOWN) sg_sel = (sg_sel + 1) % SG_COUNT;
        else if (key == K_MENU) { in_items = true; it_sel = sg_first[sg_sel]; it_edit = false; }
        else if (key == K_EXIT) return false;
        return true;
    }
    if (!it_edit) {
        unsigned first = sg_first[sg_sel], last = sg_first[sg_sel + 1] - 1;
        reset_armed = false;
        if (key == K_UP) it_sel = it_sel == first ? last : it_sel - 1;
        else if (key == K_DOWN) it_sel = it_sel == last ? first : it_sel + 1;
        else if (key == K_EXIT) in_items = false;
        else if (key == K_MENU) {
            if (it_sel == IT_FERR) freq_error_measure();
            else if (it_sel == IT_RESET || (it_sel == IT_DELCH && mr_mode)) reset_armed = it_edit = true;
            else if (it_sel == IT_SAVECH) { if (!mr_mode) save_start(); }
            else if (!it_readonly(it_sel)) { it_snap = it_get(it_sel); it_edit = true; off_len = 0; }
        }
        return true;
    }
    /* editing */
    if (it_sel == IT_RESET || it_sel == IT_DELCH) {
        it_edit = reset_armed = false;
        if (key == K_MENU) {
            if (it_sel == IT_RESET) { settings_defaults(); set_notice("SETTINGS RESET"); }
            else delete_current();
        }
        return true;
    }
    if (it_sel == IT_OFFSET && d >= 0) {                 /* typed MHz: DD.DDD */
        if (off_len < 5) off_digits[off_len++] = (u8)d;
        if (off_len == 5) {
            unsigned i; u32 khz = 0;
            for (i = 0; i < 5; ++i) khz = khz * 10 + off_digits[i];
            it_set(IT_OFFSET, (int)khz); off_len = 0;
        }
        return true;
    }
    if (key == K_UP || key == K_DOWN) {
        int lo, hi, v = it_get(it_sel), step = it_sel == IT_OFFSET ? 100 : 1; bool wrap;
        it_range(it_sel, &lo, &hi, &wrap);
        v += key == K_UP ? step : -step;
        if (v > hi) v = wrap ? lo : hi;
        if (v < lo) v = wrap ? hi : lo;
        it_set(it_sel, v);
    } else if (key == K_MENU) {
        if (it_sel == IT_OFFSET && off_len) {
            unsigned i; u32 khz = 0;
            for (i = 0; i < 5; ++i) khz = khz * 10 + (i < off_len ? off_digits[i] : 0);
            it_set(IT_OFFSET, (int)khz); off_len = 0;
        }
        it_edit = false;
        if (chan_item(it_sel) && mr_mode) set_notice(chan_save_current() ? "CHANNEL SAVED" : "SAVE FAILED");
    } else if (key == K_EXIT) {
        if (it_sel == IT_OFFSET && off_len) { off_len = 0; return true; }
        it_set(it_sel, it_snap);
        it_edit = false;
    }
    if (beep_demo2) { static const u16 hz = 600, ms = 60; beep_demo2 = false; screen_full(); mini_rf_beep(&radio, &hz, &ms, 1); }
    return true;
}
