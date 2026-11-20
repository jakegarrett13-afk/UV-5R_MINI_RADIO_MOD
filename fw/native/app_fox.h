/* N22: FOX HUNT, ported from firmware-source/apps/foxhunt/src/logic.rs (same logic and constants; 128 px layout).
 * A large signal meter that says whether a transmitter is getting stronger, plus a rough distance range once
 * calibrated at a known distance. Receive-only. */
#define FX_WINDOW_100US 2000u
#define FX_SEGMENT_100US 50000u
#define FX_FLOOR_CAP 75
#define FX_FLOOR_RISE_100US 100000u
/* N23 (Mini): the noise floor is measured on four nearby frequencies (+/-100, +/-200 kHz; the quietest wins) when
 * tuning and every 30 s, instead of being taken from the first reading on the frequency itself. A transmitter that
 * never unkeys (NOAA weather) was otherwise taken as the noise floor and never shown. The floor no longer creeps up
 * while a signal is present. */
#define FX_REMEASURE_100US 300000u
#define FX_SIGNAL_ABOVE_FLOOR 8
#define FX_LOST_ABOVE_FLOOR 4
#define FX_TREND_DB 3
#define FX_STRONG_RAW 140
#define FX_BAR_RAW_MIN 40u
#define FX_BAR_RAW_SPAN 100u
#define FX_AM_LOW_HZ 108000000u
#define FX_AM_HIGH_HZ 136000000u
#define FX_RX_MIN_HZ 18000000u
#define FX_RX_MAX_HZ 660000000u
#define FX_FREQ_DIGITS 6
#define FX_DIST_DIGITS 4
#define FX_DIST_MAX_TENTHS 99999u
static const u16 FX_DB_TO_RATIO[20] = {
    1000, 1122, 1259, 1413, 1585, 1778, 1995, 2239, 2512, 2818, 3162, 3548, 3981, 4467, 5012, 5623, 6310, 7079, 7943, 8913,
};
enum { FXV_METER, FXV_FREQ, FXV_DIST };
static struct fox_state {
    u8 view; u32 hz; bool wide, audio;
    u32 window_100us, window_sum, window_n;
    u16 floor; bool floor_known; u32 floor_timer_100us, remeasure_100us;
    bool present, has_reading; u16 level, peak;
    u32 seg_sum, seg_n, seg_100us; bool skip_window; u16 reference; bool has_reference;
    u16 cal_level; u32 cal_tenths; bool has_cal;
    u8 input[FX_FREQ_DIGITS]; unsigned input_len;
    u8 notice;          /* 0 none, 1 needs a signal, 2 bad value */
    bool dirty, full;
} fx;

static void fx_reset_readings(void)
{
    fx.window_100us = fx.window_sum = fx.window_n = 0;
    fx.floor = FX_FLOOR_CAP; fx.floor_known = false; fx.floor_timer_100us = 0;
    fx.present = fx.has_reading = false; fx.level = fx.peak = 0;
    fx.seg_sum = fx.seg_n = fx.seg_100us = 0;
    fx.has_reference = false; fx.has_cal = false;
}
static void fx_mute(void) { api_set_holds_rx(false); api_set_af_out(false); }
static void fx_measure_floor(void)
{
    static const int off_khz[4] = {-200, -100, 100, 200};
    u16 best = 0xffff; unsigned i, k;
    for (i = 0; i < 4; ++i) {
        u32 sum = 0;
        api_retune_rx((u32)((int)fx.hz + off_khz[i] * 1000), fx.wide);
        api_delay_ms(6);
        for (k = 0; k < 4; ++k) { sum += api_read_rssi(); api_delay_ms(1); }
        if (sum / 4 < best) best = (u16)(sum / 4);
    }
    api_retune_rx(fx.hz, fx.wide);
    fx.floor = best < FX_FLOOR_CAP ? best : FX_FLOOR_CAP;
    fx.floor_known = true; fx.floor_timer_100us = 0; fx.remeasure_100us = 0;
    fx.window_100us = fx.window_sum = fx.window_n = 0;          /* readings taken off frequency don't count */
}
static void fx_tune(void)
{
    bool am = fx.hz >= FX_AM_LOW_HZ && fx.hz <= FX_AM_HIGH_HZ;
    api_set_modulation(am ? 1 : 0);
    api_set_rx_freq(fx.hz);
    api_set_bandwidth(fx.wide);
    api_set_subaudio_rx(0);
    api_set_sql_level((u8)api_settings_get(0));
    api_enter_rx();
    if (!fx.audio) fx_mute();
    fx_measure_floor();
}
static void fox_enter(void)
{
    fx.view = FXV_METER;
    fx.hz = api_get_master_freq();
    fx.wide = api_get_master_wide();
    fx.audio = true;
    fx.input_len = 0; fx.notice = 0;
    fx.dirty = fx.full = true;
    fx_reset_readings();
    api_set_agc_fix(-1);
    api_set_backlight_hold(true);
    fx_tune();
}
static void fox_leave(void) { api_set_af_out(false); api_set_backlight_hold(false); }
static void fx_close_segment(void)
{
    if (fx.seg_n > 0) { fx.reference = (u16)(fx.seg_sum / fx.seg_n); fx.has_reference = true; }
    fx.seg_sum = fx.seg_n = fx.seg_100us = 0;
}
static void fx_update_floor(u16 avg)
{
    if (!fx.floor_known || avg < fx.floor) {
        fx.floor = avg < FX_FLOOR_CAP ? avg : FX_FLOOR_CAP;
        fx.floor_known = true; fx.floor_timer_100us = 0;
    } else if (!fx.present && fx.floor < FX_FLOOR_CAP && fx.floor_timer_100us >= FX_FLOOR_RISE_100US) {
        ++fx.floor; fx.floor_timer_100us = 0;
    }
}
static void fx_finish_window(void)
{
    u16 avg = (u16)(fx.window_sum / (fx.window_n ? fx.window_n : 1));
    u32 elapsed = fx.window_100us;
    bool on;
    fx.window_100us = fx.window_sum = fx.window_n = 0;
    fx.floor_timer_100us += elapsed;
    fx_update_floor(avg);
    on = fx.present ? avg >= fx.floor + FX_LOST_ABOVE_FLOOR : avg >= fx.floor + FX_SIGNAL_ABOVE_FLOOR;
    if (on != fx.present) {
        fx.present = on; fx.dirty = true;
        if (on) fx.skip_window = true; else fx_close_segment();
    }
    if (!on) return;
    fx.dirty = true;
    fx.level = avg; fx.has_reading = true;
    if (avg > fx.peak) fx.peak = avg;
    if (fx.skip_window) { fx.skip_window = false; return; }
    fx.seg_sum += avg; ++fx.seg_n; fx.seg_100us += elapsed;
    if (fx.seg_100us >= FX_SEGMENT_100US) fx_close_segment();
}
static void fox_tick(u32 dt)
{
    if (fx.view != FXV_METER) return;
    fx.remeasure_100us += dt;
    if (fx.remeasure_100us >= FX_REMEASURE_100US) { fx_measure_floor(); return; }
    fx.window_sum += api_read_rssi();
    ++fx.window_n;
    fx.window_100us += dt;
    if (fx.window_100us >= FX_WINDOW_100US) fx_finish_window();
}
static u16 fx_current_level(void)
{
    if (fx.seg_n > 0) return (u16)(fx.seg_sum / fx.seg_n);
    if (fx.has_reference) return fx.reference;
    return fx.level;
}
static void fx_commit_freq(void)
{
    u32 khz = 0, hz; unsigned i; bool typed = fx.input_len > 0;
    for (i = 0; i < FX_FREQ_DIGITS; ++i) khz = khz * 10 + (i < fx.input_len ? fx.input[i] : 0);
    fx.input_len = 0; fx.view = FXV_METER;
    if (!typed) return;
    hz = khz * 1000;
    if (hz >= FX_RX_MIN_HZ && hz <= FX_RX_MAX_HZ) {
        fx.hz = hz; fx.notice = 0;
        fx_reset_readings(); fx.full = true;
        fx_tune();
    } else fx.notice = 2;
}
static void fx_commit_dist(void)
{
    u32 tenths = 0; unsigned i;
    for (i = 0; i < fx.input_len; ++i) tenths = tenths * 10 + fx.input[i];
    fx.input_len = 0; fx.view = FXV_METER;
    if (!tenths) { fx.notice = 2; return; }
    fx.notice = 0; fx.cal_tenths = tenths; fx.cal_level = fx_current_level(); fx.has_cal = true;
}
static int fox_key(u8 id, u8 kind)
{
    if (kind != KIND_SINGLE) return APP_CONTINUE;
    fx.dirty = true;
    if (fx.view != FXV_METER) {
        unsigned cap = fx.view == FXV_FREQ ? FX_FREQ_DIGITS : FX_DIST_DIGITS;
        if (id <= 9) {
            if (fx.input_len < cap) fx.input[fx.input_len++] = id;
            if (fx.view == FXV_FREQ && fx.input_len == cap) fx_commit_freq();
        } else if (id == AK_MENU) { if (fx.view == FXV_FREQ) fx_commit_freq(); else fx_commit_dist(); }
        else if (id == AK_EXIT) { fx.input_len = 0; fx.view = FXV_METER; }
        return APP_CONTINUE;
    }
    switch (id) {
    case AK_EXIT: return APP_EXIT;
    case AK_MENU:
        if (fx.has_reading) { fx.notice = 0; fx.input_len = 0; fx.view = FXV_DIST; } else fx.notice = 1;
        break;
    case 1: fx.audio = !fx.audio; if (fx.audio) fx_tune(); else fx_mute(); break;
    case 5: fx.notice = 0; fx.input_len = 0; fx.view = FXV_FREQ; break;
    case 0: fx.has_cal = false; fx.peak = fx.level; fx.has_reference = false; fx.notice = 0; break;
    }
    return APP_CONTINUE;
}
/* distance */
static u64 fx_ratio_milli(u32 db, u32 divisor)
{
    u32 scaled = db * 20 / divisor, i;
    u64 out = FX_DB_TO_RATIO[scaled % 20];
    for (i = 0; i < scaled / 20; ++i) out = out > 0xFFFFFFFFFFFFull / 10 ? 0xFFFFFFFFFFFFull : out * 10;
    return out;
}
static u64 fx_scale_tenths(u32 tenths, int db_weaker, u32 divisor)
{
    u64 r = fx_ratio_milli((u32)(db_weaker < 0 ? -db_weaker : db_weaker), divisor), t = tenths;
    return db_weaker >= 0 ? t * r / 1000 : t * 1000 / r;
}
static void fx_distance_range(u64 *lo, u64 *hi)
{
    int db_weaker = (int)fx.cal_level - (int)fx_current_level();
    u64 open = fx_scale_tenths(fx.cal_tenths, db_weaker, 20), clut = fx_scale_tenths(fx.cal_tenths, db_weaker, 40);
    *lo = open < clut ? open : clut; *hi = open > clut ? open : clut;
}
static void fx_tenths(struct sbuf *b, u64 t)
{
    u32 v = (u32)(t > FX_DIST_MAX_TENTHS ? FX_DIST_MAX_TENTHS : t);
    ab_u(b, v / 10); ab_ch(b, '.'); ab_u(b, v % 10);
}
/* ui */
static u16 fx_raw_to_x(u16 raw)
{
    u32 v = raw > FX_BAR_RAW_MIN ? raw - FX_BAR_RAW_MIN : 0;
    if (v > FX_BAR_RAW_SPAN) v = FX_BAR_RAW_SPAN;
    return (u16)(v * (A_W - 2) / FX_BAR_RAW_SPAN);
}
static void fx_draw_bar(void)
{
    int y = 38, h = 12, inner = A_W - 2, w;
    api_fill_rect(0, y, A_W, 1, A_GRID);
    api_fill_rect(0, y + h - 1, A_W, 1, A_GRID);
    api_fill_rect(0, y, 1, h, A_GRID);
    api_fill_rect(A_W - 1, y, 1, h, A_GRID);
    w = fx.has_reading ? fx_raw_to_x(fx.level) : 0;
    if (w > inner) w = inner;
    api_fill_rect(1, y + 1, w, h - 2, fx.present ? A_TITLE : A_GRAY);
    api_fill_rect(1 + w, y + 1, inner - w, h - 2, A_BLACK);
    if (fx.has_reading) {
        int p = fx_raw_to_x(fx.peak);
        if (p > inner - 1) p = inner - 1;
        api_fill_rect(1 + p, y + 1, 1, h - 2, A_WHITE);
    }
}
static void fx_draw_level(void)
{
    struct sbuf b; ab_init(&b);
    if (fx.present) { ab_i(&b, (int)fx.level - RSSI_DBM_BASE); ab_s(&b, " DBM"); a_line_tail(16, 20, b.b, A_WHITE, A_F9); }
    else a_line_tail(16, 20, "NO SIGNAL", A_GRAY, A_F9);
}
static void fx_draw_trend(void)
{
    struct sbuf b; u16 color = A_WHITE; ab_init(&b);
    if (fx.level >= FX_STRONG_RAW && fx.present) { ab_s(&b, "TOO STRONG"); color = A_YELLOW; }
    else if (fx.present && fx.has_reference) {
        int diff = (int)fx_current_level() - (int)fx.reference;
        if (diff >= FX_TREND_DB) { ab_s(&b, "CLOSER +"); ab_u(&b, (u32)diff); color = A_GREEN; }
        else if (diff <= -FX_TREND_DB) { ab_s(&b, "FARTHER -"); ab_u(&b, (u32)-diff); color = A_RED; }
        else ab_s(&b, "SAME");
    } else if (fx.present) ab_s(&b, "MEASURING");
    else if (fx.has_reading) { ab_s(&b, "LAST "); ab_i(&b, (int)fx.level - RSSI_DBM_BASE); ab_s(&b, " DBM"); color = A_GRAY; }
    a_line_tail(52, 20, b.b, color, A_F9);
}
static void fx_draw_distance(void)
{
    struct sbuf b, p; ab_init(&b); ab_init(&p);
    if (fx.has_cal && fx.has_reading) {
        u64 lo, hi; fx_distance_range(&lo, &hi);
        if (hi == 0) ab_s(&b, "UNDER 0.1 MI");
        else { ab_s(&b, "ABOUT "); fx_tenths(&b, lo); ab_ch(&b, '-'); fx_tenths(&b, hi); ab_s(&b, " MI"); }
        a_line_tail(74, 12, b.b, A_YELLOW, A_F6);
    } else if (fx.has_reading) a_line_tail(74, 12, "MENU: SET DISTANCE", A_GRAY, A_F6);
    else a_line_tail(74, 12, "WAITING FOR SIGNAL", A_GRAY, A_F6);
    if (fx.has_reading) { ab_s(&p, "PEAK "); ab_i(&p, (int)fx.peak - RSSI_DBM_BASE); }
    if (!fx.audio) { if (fx.has_reading) ab_s(&p, "  "); ab_s(&p, "MUTED"); }
    a_line_tail(87, 12, p.b, A_GRAY, A_F6);
}
static void fx_draw_input(void)
{
    struct sbuf b; unsigned i; ab_init(&b);
    if (fx.view == FXV_FREQ) {
        for (i = 0; i < FX_FREQ_DIGITS; ++i) { if (i == 3) ab_ch(&b, '.'); ab_ch(&b, i < fx.input_len ? (char)('0' + fx.input[i]) : '_'); }
        a_line_tail(52, 20, b.b, A_YELLOW, A_F9);
        a_line_tail(74, 12, "TYPE FREQUENCY, MHZ", A_GRAY, A_F6);
    } else {
        u32 t = 0;
        for (i = 0; i < fx.input_len; ++i) t = t * 10 + fx.input[i];
        fx_tenths(&b, t); ab_s(&b, " MI");
        a_line_tail(52, 20, b.b, A_YELLOW, A_F9);
        a_line_tail(74, 12, "TYPE 1 0 FOR 1.0 MI", A_GRAY, A_F6);
    }
    api_fill_rect(0, 87, A_W, 12, A_BLACK);
    api_fill_rect(0, 102, A_W, 9, A_BLACK);
    a_line_tail(110, 9, "MENU OK  EXIT BACK", A_GRAY, A_F5);
    api_fill_rect(0, 119, A_W, 9, A_BLACK);
}
static void fox_draw(void)
{
    if (!fx.dirty && !fx.full) return;
    if (fx.full) {
        struct sbuf f; ab_init(&f); ab_freq(&f, fx.hz);
        api_fill_rect(0, 0, A_W, A_H, A_BLACK);
        a_title("FOX HUNT", f.b);
        fx.full = false;
    }
    fx_draw_level();
    fx_draw_bar();
    if (fx.view != FXV_METER) fx_draw_input();
    else {
        fx_draw_trend();
        fx_draw_distance();
        if (fx.notice == 1) a_line_tail(101, 9, "NEEDS A SIGNAL FIRST", A_YELLOW, A_F5);
        else if (fx.notice == 2) a_line_tail(101, 9, "NOT A VALID VALUE", A_YELLOW, A_F5);
        else api_fill_rect(0, 101, A_W, 9, A_BLACK);
        a_line_tail(110, 9, "1 AUDIO  5 FREQ  0 RESET", A_GRAY, A_F5);
        a_line_tail(119, 9, "MENU CALIBRATE  EXIT", A_GRAY, A_F5);
    }
    fx.dirty = false;
}
