/* FD6818B/BK4829 port: receive, and (N12) stock-sequence transmit.
 * Bus, frequency word, status and mute behavior follow the project's 5RM driver.
 * Initialization constants and external band control follow Mini V0.04:
 *   01e470d0..01e47250 init; 01e3f680 bands; 01e41350 RF GPIO control;
 *   01e4d90a frequency; 01e3fc62 states; 01e40a9e audio.
 * Calibration (squelch tables, crystal trim) is supplied by the caller via mini_rf_set_cal; no flash access here.
 * TX: stock 0x01E4F7E2 key-up, 0x01E5018C release; band switch fn_01e41350; PA 0x01E4DC72; REG 0x30 0x01E3FC62.
 */
#include "mini_rf.h"
#include <string.h>

struct reg_value { uint8_t reg; uint16_t value; };
static const struct reg_value initial[] = {
    {0x37,0x9d1f}, {0x36,0x0022},
    {0x10,0x0318},{0x11,0x033a},{0x12,0x03db},{0x13,0x03de},
    {0x14,0x0210},{0x49,0x2a32},{0x7b,0x73dc},{0x17,0x7829},
    {0x1c,0x0422},{0x1d,0x2aab},{0x1e,0x4c58},{0x1f,0x1657},
    {0x3e,0x94c6},{0x73,0x6681},{0x77,0x88ef},{0x28,0x0b40},
    {0x29,0xaa00},{0x2a,0x6600},{0x2c,0x0022},{0x2f,0x9890},
    {0x53,0x2028},{0x7e,0x303e},{0x19,0x1041},{0x3a,0x9a7c},
    {0x46,0x6050},{0x4d,0xa015},{0x4e,0x5c10},{0x4f,0x2a28},
    {0x4c,0xe520},{0x25,0x6dba},{0x7d,0xe912},{0x48,0xb3ff},
};

static void audio(struct mini_rf *r, bool on)
{
    if (!on) r->io->speaker(false);
    /* AM adds the 5RM's AM_AF_OUT_BIT 0x0600 to the receive-audio state. */
    r->io->write(0x47, on ? (r->am ? 0x6742 : 0x6142) : 0x6042);
    if (on) r->io->speaker(true);
    r->audible = on;
}
/* Bits 3 and 4 control TX paths on this board. Keep both driven low. */
static void path(struct mini_rf *r, bool rx)
{
    r->gpio = (r->io->read(0x33) & (uint16_t)~0x3838) | (rx ? 0x20 : 0);
    r->io->write(0x33, r->gpio);
}
bool mini_rf_init(struct mini_rf *r, const struct mini_rf_io *io)
{
    unsigned i;
    memset(r, 0, sizeof(*r));
    r->io = io;
    io->speaker(false);
    r->chip_id = io->read(0);
    if (r->chip_id != 0x4829) return false;
    io->write(0, 0x8000); io->write(0, 0);
    for (i = 0; i < sizeof(initial) / sizeof(initial[0]); ++i)
        io->write(initial[i].reg, initial[i].value);
    /* Stock 0x01E47224: the one read-modify-write in its init, TX deviation 1290 (N15; missing before). */
    io->write(0x40, (io->read(0x40) & 0xf000) | 1290);
    io->write(0x30, 0);
    path(r, false);
    audio(r, false);
    /* Normal FM, no tone/code qualification and nominal oscillator trim.
     * Per-device calibration decoding is a separate unfinished port step. */
    io->write(0x31, io->read(0x31) & (uint16_t)~1);
    io->write(0x42, 0x6b5a); io->write(0x2a, 0x7400);
    io->write(0x2b, 0); io->write(0x51, 0);
    io->write(0x3f, 0); io->write(0x78, 0x5957);
    r->squelch = 3;
    r->ready = true;
    return true;
}
/* Stock V0.04 0x01e4d90a: word +/- word * table[xtal] / 1e7 (byte table at 0x01e656d0+1148). */
static const uint8_t xtal_table[17] = {40,35,30,25,20,15,10,5,0,5,10,15,20,25,30,35,40};
uint32_t mini_rf_word(const struct mini_rf *r, uint32_t hz)
{
    uint32_t word = hz / 10, adjust;
    unsigned idx = (r->cal_valid && r->cal.xtal <= 16) ? r->cal.xtal : 8;
    adjust = word * xtal_table[idx] / 10000000u;
    return idx > 8 ? word + adjust : word - adjust;
}
/* UV-25 driver (same calibration layout, stock-derived): offset by band, then (in << 8) | out. */
uint16_t mini_rf_squelch_value(const struct mini_rf *r, uint32_t hz)
{
    const struct mini_rf_cal *c = &r->cal;
    unsigned level = r->squelch > 9 ? 9 : r->squelch, mhz = hz / 1000000u, idx;
    uint8_t in = c->th_in[level], out = c->th_out[level], off;
    if (hz >= 400000000u) { idx = (mhz - 400) / 10; if (idx > 15) idx = 15; off = c->off_u400[idx]; }
    else { idx = mhz >= 130 ? (mhz - 130) / 5 : 0; if (idx > 9) idx = 9; off = c->off_v136[idx]; }
    if (in > off) { in -= off; out = out > off ? out - off : 0; }
    return (uint16_t)((in << 8) | out);
}
static void apply_squelch(struct mini_rf *r)
{
    if (!r->cal_valid || !r->frequency_hz) return;
    r->squelch_reg = mini_rf_squelch_value(r, r->frequency_hz);
    r->io->write(0x78, r->squelch_reg);
}
void mini_rf_set_cal(struct mini_rf *r, const struct mini_rf_cal *cal)
{
    r->cal = *cal;
    r->cal_valid = true;
}
void mini_rf_set_squelch(struct mini_rf *r, uint8_t level)
{
    r->squelch = level > 9 ? 9 : level;
    if (r->ready) { apply_squelch(r); mini_rf_poll(r); }
}
/* CHIRP DTCS_CODES + 645, sorted (octal). */
static const uint16_t dcs_codes[105] = {
    023, 025, 026, 031, 032, 036, 043, 047, 051, 053, 054, 065, 071, 072, 073, 074, 0114, 0115, 0116, 0122,
    0125, 0131, 0132, 0134, 0143, 0145, 0152, 0155, 0156, 0162, 0165, 0172, 0174, 0205, 0212, 0223, 0225,
    0226, 0243, 0244, 0245, 0246, 0251, 0252, 0255, 0261, 0263, 0265, 0266, 0271, 0274, 0306, 0311, 0315,
    0325, 0331, 0332, 0343, 0346, 0351, 0356, 0364, 0365, 0371, 0411, 0412, 0413, 0423, 0431, 0432, 0445,
    0446, 0452, 0454, 0455, 0462, 0464, 0465, 0466, 0503, 0506, 0516, 0523, 0526, 0532, 0546, 0565, 0606,
    0612, 0624, 0627, 0631, 0632, 0645, 0654, 0662, 0664, 0703, 0712, 0723, 0731, 0732, 0734, 0743, 0754
};
bool mini_rf_tone_from_channel(uint16_t raw, struct mini_rf_tone *out)
{
    out->kind = MINI_TONE_NONE; out->value = 0;
    if (raw == 0 || raw == 0xffff) return true;
    if (raw >= 0x258) {
        if (raw > 3000) return false;                    /* > 300.0 Hz: not a CTCSS tone */
        out->kind = MINI_TONE_CTCSS; out->value = raw; return true;
    }
    if (raw <= 0x69) {
        if (raw - 1 >= 105) return false;
        out->kind = MINI_TONE_DCS_N; out->value = dcs_codes[raw - 1]; return true;
    }
    if (raw - 0x6a >= 105) return false;
    out->kind = MINI_TONE_DCS_I; out->value = dcs_codes[raw - 0x6a]; return true;
}
/* Golay(23,12) as the UV-25 driver (hardware-tested): source = code | 0x800, poly 0x0C75. */
uint32_t mini_rf_dcs_codeword(uint16_t code)
{
    uint16_t source = code | 0x0800, remainder = 0, bit = 1, feedback;
    unsigned i;
    for (i = 0; i < 12; ++i) {
        feedback = (!!(source & bit) == !!(remainder & 2)) ? 0 : 0x0c75;
        bit <<= 1; remainder >>= 1; remainder ^= feedback;
    }
    return source | ((uint32_t)remainder << 11);
}
/* UV-25 enable_rx_subaudio: REG 0x51 = enable|mode|gain, REG 0x07 tone word (tenths*206489/100000) then the
 * tail word, REG 0x52 threshold; DCS: 134.4 bit/s word 0x0AD7 and the codeword in two halves on REG 0x08. */
/* N26: the same registers program the transmitted tone (5RM fd6818b set_subaudio_tx), so one routine serves both. */
static void apply_tone(struct mini_rf *r, struct mini_rf_tone t)
{
    uint8_t cg = r->cal_valid ? (r->cal.cts_gain & 0x7f) : 0, dg = r->cal_valid ? (r->cal.dcs_gain & 0x7f) : 0;
    uint32_t word;
    switch (t.kind) {
    case MINI_TONE_CTCSS:
        r->io->write(0x51, 0x9000 | cg);
        r->io->write(0x07, (uint16_t)(((uint32_t)t.value * 206489u / 100000u) & 0x1fff));
        r->io->write(0x07, 0x01cd | 0x2000);
        r->io->write(0x52, 0x0292);
        break;
    case MINI_TONE_DCS_N:
    case MINI_TONE_DCS_I:
        r->io->write(0x51, (t.kind == MINI_TONE_DCS_I ? 0xa000 : 0x8000) | dg);
        r->io->write(0x07, 0x0ad7);
        word = mini_rf_dcs_codeword(t.value);
        r->io->write(0x08, word & 0x0fff);
        r->io->write(0x08, ((word >> 12) & 0x0fff) | 0x8000);
        break;
    default:
        r->io->write(0x51, 0);
        break;
    }
}
static void apply_rx_tone(struct mini_rf *r)
{
    r->tone_matched = false;
    apply_tone(r, r->rx_tone);
}
void mini_rf_set_rx_tone(struct mini_rf *r, struct mini_rf_tone t)
{
    if (t.kind > MINI_TONE_DCS_I) t.kind = MINI_TONE_NONE;
    r->rx_tone = t;
    if (r->ready && r->frequency_hz) { apply_rx_tone(r); mini_rf_poll(r); }
}
bool mini_rf_rx_range(const struct mini_rf *r, uint32_t hz)
{
    if (r->wide_range) return hz >= 18000000u && hz <= 660000000u;
    return (hz >= 136000000 && hz <= 174000000) || (hz >= 400000000 && hz <= 520000000);
}
/* AM (5RM apply_modulation): AFC off (REG 0x73 bit 4) and the 8.46 kHz IF (REG 0x3D 0x2AAB); the Mini's own values
 * (stock 0x6681 / 0) are saved first and put back when AM ends. */
static void apply_modulation(struct mini_rf *r)
{
    if (r->am && !r->am_applied) {
        r->saved_73 = r->io->read(0x73);
        r->saved_3d = r->io->read(0x3d);
        r->io->write(0x73, r->saved_73 | 0x0010);
        r->io->write(0x3d, 0x2aab);
        r->am_applied = true;
    } else if (!r->am && r->am_applied) {
        r->io->write(0x73, r->saved_73);
        r->io->write(0x3d, r->saved_3d);
        r->am_applied = false;
    }
}
bool mini_rf_tune(struct mini_rf *r, uint32_t hz)
{
    uint32_t word;
    if (r->transmitting) mini_rf_tx_off(r);
    if (!r->ready || !mini_rf_rx_range(r, hz)) return false;
    audio(r, false);
    r->io->write(0x30, 0);
    path(r, false);
    r->io->band(hz >= 300000000);   /* N22: 300 MHz split = N21's 400 for 136-174/400-520 (350 band on UHF) */
    r->io->write(0x36, 0x0022);
    r->io->write(0x37, 0x9d1f);
    word = mini_rf_word(r, hz);
    r->word = word;
    r->io->write(0x38, word);
    r->io->write(0x39, word >> 16);
    r->io->write(0x43, r->narrow ? 0x4048 : 0x3028); /* stock wide FM; 5RM BANDWIDTH_NARROW */
    apply_modulation(r);
    apply_rx_tone(r);
    path(r, true);
    r->io->write(0x30, 0x0200);
    r->io->write(0x30, 0xbff1);
    r->io->settle(30);
    r->frequency_hz = hz;
    apply_squelch(r);
    r->frequency_readback = r->io->read(0x38) | ((uint32_t)r->io->read(0x39) << 16);
    mini_rf_poll(r);
    return r->frequency_readback == word && r->state == 0xbff1;
}
bool mini_rf_retune(struct mini_rf *r, uint32_t hz, int band_path)
{
    uint32_t word;
    if (!r->ready || r->transmitting || !mini_rf_rx_range(r, hz)) return false;
    r->io->band(band_path < 0 ? hz >= 300000000 : band_path == 1);
    word = mini_rf_word(r, hz);
    r->word = word;
    r->io->write(0x38, word);
    r->io->write(0x39, word >> 16);
    r->io->write(0x43, r->narrow ? 0x4048 : 0x3028);
    r->io->write(0x30, 0x0200);
    r->io->write(0x30, 0xbff1);
    r->frequency_hz = hz;
    return true;
}
/* N25: a scan hop with no audible switching: unlike mini_rf_tune, REG 0x30 never goes to 0 (that powers the AF DAC down
 * and pops), there is no settle wait, and the GPIO path is only cycled when the band changes. The receive tone set in
 * r->rx_tone and the squelch for the new frequency are applied. */
bool mini_rf_hop(struct mini_rf *r, uint32_t hz)
{
    uint32_t word;
    bool uhf = hz >= 300000000;
    if (!r->ready || r->transmitting || !r->frequency_hz || !mini_rf_rx_range(r, hz)) return false;
    if (r->audible) audio(r, false);
    if (uhf != (r->frequency_hz >= 300000000)) { path(r, false); r->io->band(uhf); path(r, true); }
    word = mini_rf_word(r, hz);
    r->word = word;
    r->io->write(0x38, word);
    r->io->write(0x39, word >> 16);
    r->io->write(0x43, r->narrow ? 0x4048 : 0x3028);
    apply_rx_tone(r);
    r->io->write(0x30, 0x0200);
    r->io->write(0x30, 0xbff1);
    r->frequency_hz = hz;
    apply_squelch(r);
    r->frequency_readback = r->io->read(0x38) | ((uint32_t)r->io->read(0x39) << 16);
    return r->frequency_readback == word;
}
void mini_rf_poll(struct mini_rf *r)
{
    bool open, tone_ok;
    if (!r->ready || !r->frequency_hz) return;
    if (r->transmitting) { r->state = r->io->read(0x30); return; }   /* speaker stays off while keyed */
    r->rssi = r->io->read(0x67) & 0x1ff;
    r->state = r->io->read(0x30);
    r->status = r->io->read(0x0c);
    switch (r->rx_tone.kind) {
    case MINI_TONE_CTCSS: r->tone_matched = !!(r->status & 0x0400); break;
    case MINI_TONE_DCS_N: r->tone_matched = !!(r->status & 0x4000); break;
    case MINI_TONE_DCS_I: r->tone_matched = !!(r->status & 0x8000); break;
    default: r->tone_matched = false; break;
    }
    tone_ok = r->rx_tone.kind == MINI_TONE_NONE || r->tone_matched;
    /* Level 0 = always open (as stock): REG 0x78 = 0 removes only the RSSI condition, not noise/glitch. */
    open = r->monitor || r->squelch == 0 || ((r->status & 2) && tone_ok);
    if (r->hold_mute) open = false;
    if (open != r->audible) audio(r, open);
}
void mini_rf_monitor(struct mini_rf *r, bool on)
{
    r->monitor = on;
    mini_rf_poll(r);
}
void mini_rf_stop(struct mini_rf *r)
{
    if (!r->ready) return;
    if (r->transmitting) mini_rf_tx_off(r);
    audio(r, false);
    r->io->write(0x36, 0x0022);
    r->io->write(0x30, 0);
    path(r, false);
    r->frequency_hz = 0;
    r->state = 0;
}

/* Stock fn_01e41350 mode 0 (all TX switches off, GPIO5 on, PC7 high) and mode 2 (GPIO3 VHF / GPIO4 UHF). */
static void tx_switch(struct mini_rf *r, int mode, bool uhf)
{
    r->gpio = (r->io->read(0x33) & (uint16_t)~0x3838) | 0x20 | (mode == 2 ? (uhf ? 0x10 : 0x08) : 0);
    r->io->write(0x33, r->gpio);
}
bool mini_rf_tx_on(struct mini_rf *r, uint32_t hz, uint8_t pa_target, uint8_t pa_gain, uint8_t mic)
{
    bool uhf = hz >= 400000000u;
    uint32_t word;
    if (!r->ready || !r->cal_valid || r->transmitting) return false;
    if (pa_target == 0 || pa_target == 0xff || (pa_gain != 0xff && pa_gain != 0xd7)) return false;
    if (!((hz >= 136000000u && hz <= 174000000u) || (hz >= 400000000u && hz <= 480000000u))) return false;
    /* receive off, speaker muted */
    audio(r, false);
    r->io->write(0x36, 0x007f);                 /* stock PA off value */
    r->io->write(0x30, 0);
    r->frequency_hz = 0;                        /* receive must be re-tuned afterwards */
    /* 0x01E4F800: band switch mode 0 (TX switches off, PC7 high) */
    tx_switch(r, 0, uhf);
    r->io->band(false);                         /* board hook: false = PC7 high */
    /* 0x01E410F4 via 0x01E4D59E(0): tone generator off */
    r->io->write(0x24, r->io->read(0x24) & (uint16_t)~0x20);
    r->io->write(0x70, 0);
    r->io->write(0x37, 0x9d1f);
    r->io->write(0x30, 0);
    word = mini_rf_word(r, hz);
    r->word = word;
    r->io->write(0x38, word);
    r->io->write(0x39, word >> 16);
    r->io->write(0x30, r->io->read(0x30) | 4);  /* mic ADC */
    /* 0x01E4D994(0) + 0x01E507C2: no sub-audio; N26: or the transmitted CTCSS/DCS in r->tx_tone */
    if (r->tx_tone.kind == MINI_TONE_CTCSS || r->tx_tone.kind == MINI_TONE_DCS_N || r->tx_tone.kind == MINI_TONE_DCS_I) {
        apply_tone(r, r->tx_tone);
    } else {
        r->io->write(0x52, r->io->read(0x52) & 0x1fff);
        r->io->write(0x51, 0);
    }
    /* 0x01E507DC: mic gain from calibration 0xF210 */
    r->io->write(0x7d, (r->io->read(0x7d) & 0xffc0) | (mic & 0x3f));
    r->transmitting = true;
    /* 0x01E3FC62(2) */
    r->io->write(0x30, 0x0200);
    r->io->write(0x30, 0xc1fe);
    r->io->settle(10);
    /* 0x01E4DC72: PA = target << 8 | gain; then TX band switch (mode 2) */
    r->pa_reg = (uint16_t)((pa_target << 8) | pa_gain);
    r->io->write(0x36, r->pa_reg);
    tx_switch(r, 2, uhf);
    if (r->io->tx_aux) r->io->tx_aux(true);     /* 0x01E508F6: PR0 after PA and switch */
    r->state = r->io->read(0x30);
    if (r->state != 0xc1fe || (r->io->read(0x38) | ((uint32_t)r->io->read(0x39) << 16)) != word) {
        mini_rf_tx_off(r);
        return false;
    }
    return true;
}
/* N26: end-of-transmission extras while still keyed (5RM radio.rs end_tx): a roger tone sequence through the tone
 * generator in its transmit state (REG 0x70 gain, REG 0x30 0xC3FA, REG 0x71 word; hz 0 = a rest), then the squelch tail
 * (STE: 55 Hz on the sub-audio generator, REG 0x07 0x0471, 300 ms) and a hold of extra carrier. Nothing if not keyed. */
void mini_rf_tx_tail(struct mini_rf *r, const uint16_t *hz, const uint16_t *ms, unsigned n, bool ste, unsigned hold_ms)
{
    unsigned i;
    if (!r->ready || !r->transmitting) return;
    if (n) {
        r->io->write(0x70, (uint16_t)((r->io->read(0x70) & 0x70ff) | 0x8000 | (60 << 8)));
        r->io->write(0x30, 0xc3fa);
        for (i = 0; i < n; ++i) {
            r->io->write(0x71, (uint16_t)((uint32_t)hz[i] * 1032444u / 100000u));
            r->io->settle(ms[i]);
        }
        r->io->write(0x71, 0);
        r->io->write(0x70, 0);
        r->io->write(0x30, 0xc1fe);
    }
    if (ste) {
        uint8_t cg = r->cal_valid ? (r->cal.cts_gain & 0x7f) : 0;
        r->io->write(0x51, 0x9000 | cg);
        r->io->write(0x07, 0x0471);
        r->io->settle(300);
        r->io->write(0x51, 0);
    }
    while (hold_ms) { unsigned k = hold_ms > 100 ? 100 : hold_ms; r->io->settle(k); hold_ms -= k; }
}
/* Stock release 0x01E5018C: PA off (0x7F), REG 0x30 off, band switch mode 0; PR0 low. */
void mini_rf_tx_off(struct mini_rf *r)
{
    if (!r->ready) return;
    r->io->write(0x36, 0x007f);
    r->io->write(0x30, 0);
    path(r, false);                             /* GPIO3/4/5 low */
    if (r->io->tx_aux) r->io->tx_aux(false);
    r->transmitting = false;
    r->frequency_hz = 0;
    r->state = 0;
}

/* Local beep through the RF chip's tone generator: REG70 tone-1 on, REG30 0 then 0x0302 (tone state), tone word
 * hz*1032444/100000, beep AF 0x6342 (stock table 0x01E5E158 entry 3). The speaker amp handling is selectable while the
 * crackle the user hears is tracked down (N17 listening test, r->beep_style):
 *   1 (default) amp switched only while the AF output is muted (N16);
 *   2 amp on 10 ms after the tone starts and off before it stops (N15, custom firmware play_tone_sequence);
 *   3 amp on before any chip change and off only after receive is restored;
 *   4 as 1 with the 5RM custom firmware's lower beep level (tone gain 30, volume 0xB8A2). */
static void beep_tone_setup(struct mini_rf *r, uint16_t gain, uint16_t hz, bool quiet, uint16_t af)
{
    r->io->write(0x47, af);
    r->io->write(0x70, quiet ? (uint16_t)(0x8000 | (30 << 8)) : (uint16_t)((gain | 0xbc00) & 0xfcff));
    r->io->write(0x30, 0x0302);
    r->io->write(0x48, quiet ? 0xb8a2 : 0xb0a3);
    r->io->write(0x71, (uint16_t)((uint32_t)hz * 1032444u / 100000u));
}
static void beep_restore(struct mini_rf *r, uint16_t gain, uint16_t vol)
{
    r->io->write(0x71, 0);
    r->io->write(0x70, gain);
    r->io->write(0x48, vol);
    r->io->write(0x30, 0x0200);
    r->io->write(0x30, 0xbff1);
}
/* N28: start-up "harp": n strums of 4 notes each. A strum is a rising run of plucked notes (pick_ms each, N29: each one
 * fading a little over its first 4 steps so the notes are heard separately); its top note then rings and fades
 * (ring_ms[c]) through all 16 steps of the tone generator's gain (REG 0x70 bits 8-14). Speaker and
 * AF handling as beep style 1: amp switched only while the AF output is muted; receive restored afterwards. */
void mini_rf_harp(struct mini_rf *r, const uint16_t (*chords)[4], unsigned n, unsigned pick_ms, const uint16_t *ring_ms)
{
    static const uint8_t fade[16] = {60, 52, 45, 39, 34, 29, 25, 21, 18, 15, 12, 10, 8, 6, 4, 2};
    uint16_t gain, vol, low;
    unsigned c, i, k;
    if (!r->ready || r->transmitting || !r->frequency_hz || !n) return;
    vol = r->io->read(0x48);
    gain = r->io->read(0x70);
    low = gain & 0x00ff;
    r->io->speaker(false);
    beep_tone_setup(r, gain, chords[0][0], false, 0x6042);   /* tone running, output muted */
    r->io->settle(20);
    r->io->speaker(true);
    r->io->settle(30);
    r->io->write(0x47, 0x6342);
    for (c = 0; c < n; ++c) {
        for (i = 0; i < 4; ++i) {
            unsigned steps = i < 3 ? 4 : 16, step_ms = i < 3 ? pick_ms / 4 : ring_ms[c] / 16;
            r->io->write(0x71, (uint16_t)((uint32_t)chords[c][i] * 1032444u / 100000u));
            for (k = 0; k < steps; ++k) {
                r->io->write(0x70, (uint16_t)(low | 0x8000 | (fade[k] << 8)));
                r->io->settle(step_ms);
            }
        }
    }
    r->io->write(0x47, 0x6042);
    r->io->settle(20);
    r->io->speaker(false);
    r->io->settle(20);
    beep_restore(r, gain, vol);
    r->audible = false;
    mini_rf_poll(r);
}
void mini_rf_beep(struct mini_rf *r, const uint16_t *hz, const uint16_t *ms, unsigned n)
{
    uint16_t gain, vol;
    unsigned i, style = r->beep_style >= 1 && r->beep_style <= 4 ? r->beep_style : 1;
    if (!r->ready || r->transmitting || !r->frequency_hz || !n) return;
    vol = r->io->read(0x48);
    gain = r->io->read(0x70);
    if (style == 3) {
        r->io->speaker(true);
        r->io->settle(20);
        beep_tone_setup(r, gain, hz[0], false, 0x6342);
        for (i = 0; i < n; ++i) {
            if (i) r->io->write(0x71, (uint16_t)((uint32_t)hz[i] * 1032444u / 100000u));
            r->io->settle(ms[i]);
        }
        r->io->write(0x47, 0x6042);
        beep_restore(r, gain, vol);
        r->io->settle(20);
        r->io->speaker(false);
    } else if (style == 2) {
        r->io->speaker(false);
        beep_tone_setup(r, gain, hz[0], false, 0x6342);
        r->io->settle(10);
        r->io->speaker(true);
        for (i = 0; i < n; ++i) {
            if (i) r->io->write(0x71, (uint16_t)((uint32_t)hz[i] * 1032444u / 100000u));
            r->io->settle(ms[i]);
        }
        r->io->speaker(false);
        r->io->write(0x47, 0x6042);
        beep_restore(r, gain, vol);
    } else {
        r->io->speaker(false);
        beep_tone_setup(r, gain, hz[0], style == 4, 0x6042);   /* tone running, output muted */
        r->io->settle(20);
        r->io->speaker(true);
        r->io->settle(30);
        r->io->write(0x47, 0x6342);
        for (i = 0; i < n; ++i) {
            if (i) r->io->write(0x71, (uint16_t)((uint32_t)hz[i] * 1032444u / 100000u));
            r->io->settle(ms[i]);
        }
        r->io->write(0x47, 0x6042);
        r->io->settle(20);
        r->io->speaker(false);
        r->io->settle(20);
        beep_restore(r, gain, vol);
    }
    r->audible = false;
    mini_rf_poll(r);
}
