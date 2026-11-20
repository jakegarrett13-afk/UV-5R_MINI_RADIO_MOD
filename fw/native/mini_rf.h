#ifndef MINI_RF_H
#define MINI_RF_H
#include <stdint.h>
#include <stdbool.h>

/* Board-independent RF portion of the native Mini port. TX (N12) follows stock V0.04 0x01E4F7E2 exactly; the
 * caller decides whether TX is allowed (frequency lock, timeout, watchdog). */
struct mini_rf_io {
    uint16_t (*read)(uint8_t reg);
    void (*write)(uint8_t reg, uint16_t value);
    void (*band)(bool uhf);
    void (*speaker)(bool on);
    void (*settle)(unsigned ms);
    void (*tx_aux)(bool on);      /* stock 0x01E418AE: PR0 high only while transmitting */
};
/* Per-radio calibration from the settings NOR (same layout as stock V0.04 loads and the UV-25 driver uses):
 * th_in/th_out 0xF0C0/0xF0D0 (squelch levels 0-9, REG 0x78 units), off_u400 0xF0E0 ((MHz-400)/10),
 * off_v136 0xF0F0 ((MHz-130)/5), xtal = 0xF210+6 (0..16, 8 = no correction). */
struct mini_rf_cal {
    uint8_t th_in[10], th_out[10], off_u400[16], off_v136[16];
    uint8_t xtal;
    uint8_t cts_gain, dcs_gain;   /* 0xF210+1 / +2: sub-audio gains (REG 0x51 bits 6:0) */
};
/* Receive sub-audio qualification. CTCSS value = tenths of Hz; DCS value = binary code (e.g. 023 octal). */
enum { MINI_TONE_NONE, MINI_TONE_CTCSS, MINI_TONE_DCS_N, MINI_TONE_DCS_I };
struct mini_rf_tone { uint8_t kind; uint16_t value; };
struct mini_rf {
    const struct mini_rf_io *io;
    uint32_t frequency_hz, frequency_readback, word;
    uint16_t chip_id, state, gpio, rssi, squelch_reg, status;
    uint8_t squelch;
    bool ready, monitor, audible, cal_valid;
    struct mini_rf_cal cal;
    struct mini_rf_tone rx_tone;
    bool tone_matched;
    bool transmitting;
    uint8_t beep_style;           /* 1-4, see mini_rf_beep (N17 listening test) */
    bool hold_mute;               /* an app keeps the speaker shut (TEXT modem listening) */
    uint16_t pa_reg;              /* last REG 0x36 written for TX */
    /* N22 (apps): receive options, all false for the standby radio (then tuning is exactly as N21). */
    bool wide_range;              /* accept 18-660 MHz receive (apps: spectrum, sweeps); UHF path from 300 MHz */
    bool narrow;                  /* REG 0x43 narrow (0x4048) instead of wide (0x3028) */
    bool am;                      /* AM demodulation (5RM fd6818b apply_modulation / set_af_out) */
    bool am_applied;
    uint16_t saved_73, saved_3d;  /* the Mini's own AFC / IF values, restored when AM ends */
    struct mini_rf_tone tx_tone;  /* N26: CTCSS/DCS sent by the next mini_rf_tx_on (NONE = none) */
};
bool mini_rf_init(struct mini_rf *, const struct mini_rf_io *);
bool mini_rf_tune(struct mini_rf *, uint32_t hz);
/* N22: frequency-only retune that stays in receive (5RM retune_rx / tune_search_candidate): no settle, squelch, tone
 * or audio change. path: -1 by frequency, 0 VHF, 1 UHF. False outside the receive range or while transmitting. */
bool mini_rf_retune(struct mini_rf *, uint32_t hz, int path);
bool mini_rf_rx_range(const struct mini_rf *, uint32_t hz);
/* N25: scan hop (receive already running): new frequency, r->rx_tone and squelch without REG 0x30 off or settle. */
bool mini_rf_hop(struct mini_rf *, uint32_t hz);
void mini_rf_poll(struct mini_rf *);
void mini_rf_monitor(struct mini_rf *, bool on);
void mini_rf_stop(struct mini_rf *);
/* Calibration and squelch (receive only). Without a calibration the driver keeps its nominal behaviour:
 * uncorrected frequency word and the init REG 0x78 value. */
void mini_rf_set_cal(struct mini_rf *, const struct mini_rf_cal *);
void mini_rf_set_squelch(struct mini_rf *, uint8_t level);
uint32_t mini_rf_word(const struct mini_rf *, uint32_t hz);
uint16_t mini_rf_squelch_value(const struct mini_rf *, uint32_t hz);
/* Applied at the next tune (and immediately if tuned). NONE = carrier squelch only. */
void mini_rf_set_rx_tone(struct mini_rf *, struct mini_rf_tone);
/* CHIRP UV5RMini channel tone field: 0/0xFFFF none, >= 0x258 CTCSS tenths Hz, 1..0x69 DCS normal (index+1),
 * 0x6A..0x257 DCS inverted (index+0x6A); index into the 105-code DCS list. False if out of range. */
bool mini_rf_tone_from_channel(uint16_t raw, struct mini_rf_tone *out);
uint32_t mini_rf_dcs_codeword(uint16_t code);
/* Transmit plain FM (no sub-audio) on hz. pa_target = this radio's calibration byte for the band/frequency/power
 * (NOR 0xF000 HIGH / 0xF080 LOW), pa_gain = stock driver gain (0xFF HIGH, 0xD7 LOW), mic = NOR 0xF210 low 6 bits.
 * Refuses (false, nothing keyed) for an invalid target, frequency outside 136-174/400-480 MHz, or a failed state
 * readback (then also keys off). mini_rf_tx_off returns to idle; the caller re-tunes receive. */
bool mini_rf_tx_on(struct mini_rf *, uint32_t hz, uint8_t pa_target, uint8_t pa_gain, uint8_t mic);
void mini_rf_tx_off(struct mini_rf *);
/* N26: while keyed, before mini_rf_tx_off: roger tones (hz/ms, hz 0 = rest), squelch tail (STE) and extra carrier. */
void mini_rf_tx_tail(struct mini_rf *, const uint16_t *hz, const uint16_t *ms, unsigned n, bool ste, unsigned hold_ms);
/* Local speaker beep (receive only): n tones, hz / ms each. */
void mini_rf_beep(struct mini_rf *, const uint16_t *hz, const uint16_t *ms, unsigned n);
/* N28: n strums of 4 rising notes (pick_ms each), the top note ringing and fading over ring_ms[strum]. Receive only. */
void mini_rf_harp(struct mini_rf *, const uint16_t (*chords)[4], unsigned n, unsigned pick_ms, const uint16_t *ring_ms);
#endif
