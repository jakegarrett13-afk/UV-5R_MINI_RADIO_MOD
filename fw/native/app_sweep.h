/* N23: BAND SWEEP, from firmware-source/apps/bandsweep/src/logic.rs (same measurement, hit merging and floor logic),
 * reworked for the Mini at the user's request: nothing is drawn while it measures. Opening it (or changing band, step
 * or range, or pressing *) runs SW_PASSES passes behind a "SCANNING..." progress screen; then a blue graph of the band
 * (strongest reading per column) and the list of heard frequencies, strongest first, to choose from.
 * Results keys: UP/DOWN select, MENU listen, MENU hold = put it in the VFO and leave, * scan again, AB next band,
 * 1 step, 5 custom range, side key = ignore the selected signal, EXIT. While scanning, EXIT leaves. */
#define SW_COLS 128
#define SW_HEADER_Y 0
#define SW_GRAPH_TOP 9
#define SW_GRAPH_BOTTOM 52
#define SW_LIST_Y 56
#define SW_ROW_H 9
#define SW_LIST_ROWS 6
#define SW_STATUS_Y 111
#define SW_HINT_Y 120
#define SW_BAR 0x049f          /* the Mini's electric blue */
#define SW_BAR_DIM 0x0210
#define SW_NBANDS 6
#define SW_CUSTOM 5
#define SW_PASSES 2
static const char *const SW_BAND_NAME[SW_NBANDS] = {"VHF", "UHF", "AIR", "220", "350", "CUSTOM"};
static const u32 SW_BAND_LO[SW_NBANDS] = {136000000, 400000000, 118000000, 200000000, 300000000, 144000000};
static const u32 SW_BAND_HI[SW_NBANDS] = {174000000, 520000000, 137000000, 260000000, 390000000, 148000000};
static const u32 SW_STEPS_HZ[2] = {25000, 12500};
#define SW_RX_MIN_HZ 18000000u
#define SW_RX_MAX_HZ 660000000u
#define SW_AM_LOW_HZ 108000000u
#define SW_AM_HIGH_HZ 136000000u
#define SW_GRAPH_BELOW_FLOOR 4
#define SW_GRAPH_RAW_SPAN 70u
#define SW_DWELL 30
#define SW_CONFIRM 10
#define SW_TICK_BUDGET 500u
#define SW_MARGIN 16
#define SW_INITIAL_FLOOR 70
#define SW_HIST 128
#define SW_FLOOR_EVERY 256
#define SW_MAX_HITS 24
#define SW_MAX_IGNORED 8
#define SW_INPUT_DIGITS 6
struct sw_hit { u32 hz; u8 peak, count; u16 last_pass; };
enum { SWV_SCAN, SWV_RESULTS, SWV_LISTEN, SWV_INPUT0, SWV_INPUT1 };
static struct sweep_state {
    u8 view; unsigned band; u32 custom_lo, custom_hi; unsigned step_index;
    u32 lo_hz, total, pos; u16 pass; u32 scan_start, scan_tenths;
    u16 hist[SW_HIST]; u16 samples; u8 floor, graph_base;
    u8 hold[SW_COLS];
    struct sw_hit hits[SW_MAX_HITS]; unsigned n_hits, selected, scroll;
    u32 ignored[SW_MAX_IGNORED]; unsigned n_ignored;
    u32 run_hz; u8 run_peak; u32 run_pos;
    u32 listen_hz; u8 input[SW_INPUT_DIGITS]; unsigned input_len;
    unsigned shown_pct;
    bool list_dirty, status_dirty, full, inited;
} sw;

static u32 sw_step_hz(void) { return SW_STEPS_HZ[sw.step_index]; }
static void sw_band_range(u32 *lo, u32 *hi)
{
    if (sw.band == SW_CUSTOM) { *lo = sw.custom_lo; *hi = sw.custom_hi; }
    else { *lo = SW_BAND_LO[sw.band]; *hi = SW_BAND_HI[sw.band]; }
}
static u16 sw_trigger(void) { return (u16)(sw.floor + SW_MARGIN); }
static void sw_start_rx(void)
{
    api_set_modulation(0);
    api_set_bandwidth(true);
    api_enter_rx();
    api_set_holds_rx(false);
    api_set_af_out(false);
    api_set_agc_fix(0);
}
/* Start a fresh scan of the current band and step. */
static void sw_restart(void)
{
    u32 lo, hi;
    sw_band_range(&lo, &hi);
    sw.lo_hz = lo;
    sw.total = (hi - lo) / sw_step_hz() + 1;
    sw.pos = 0; sw.pass = 0; sw.scan_tenths = 0; sw.scan_start = api_uptime_100us();
    memset(sw.hist, 0, sizeof(sw.hist)); sw.samples = 0;
    sw.floor = SW_INITIAL_FLOOR; sw.graph_base = SW_INITIAL_FLOOR - SW_GRAPH_BELOW_FLOOR;
    memset(sw.hold, 0, sizeof(sw.hold));
    memset(sw.hits, 0, sizeof(sw.hits)); sw.n_hits = 0; sw.selected = 0; sw.scroll = 0;
    sw.run_peak = 0;
    sw.shown_pct = 101;
    sw.view = SWV_SCAN;
    sw.full = true;
    api_set_rx_freq(lo);
    sw_start_rx();
}
static void sw_update_floor(void)
{
    u32 target = sw.samples / 2u, acc = 0; unsigned bucket;
    if (!target) target = 1;
    for (bucket = 0; bucket < SW_HIST; ++bucket) {
        acc += sw.hist[bucket];
        if (acc >= target) { sw.floor = (u8)(bucket * 2 > 255 ? 255 : bucket * 2); return; }
    }
}
static bool sw_is_ignored(u32 hz)
{
    unsigned i;
    for (i = 0; i < sw.n_ignored; ++i) if (a_absdiff(sw.ignored[i], hz) <= sw_step_hz()) return true;
    return false;
}
static void sw_sort_hits(void)
{
    unsigned i, j;
    for (i = 1; i < sw.n_hits; ++i)
        for (j = i; j > 0 && sw.hits[j].peak > sw.hits[j - 1].peak; --j) {
            struct sw_hit t = sw.hits[j]; sw.hits[j] = sw.hits[j - 1]; sw.hits[j - 1] = t;
        }
}
static void sw_commit_hit(u32 hz, u8 peak)
{
    unsigned i;
    if (sw_is_ignored(hz)) return;
    for (i = 0; i < sw.n_hits; ++i) if (a_absdiff(sw.hits[i].hz, hz) <= sw_step_hz()) break;
    if (i < sw.n_hits) {
        struct sw_hit *h = &sw.hits[i];
        if (peak > h->peak) { h->peak = peak; h->hz = hz; }
        if (h->last_pass != sw.pass) { h->last_pass = sw.pass; if (h->count < 255) ++h->count; }
    } else {
        struct sw_hit n; n.hz = hz; n.peak = peak; n.count = 1; n.last_pass = sw.pass;
        if (sw.n_hits < SW_MAX_HITS) sw.hits[sw.n_hits++] = n;
        else if (peak > sw.hits[SW_MAX_HITS - 1].peak) sw.hits[SW_MAX_HITS - 1] = n;
        else return;
    }
    sw_sort_hits();
}
static void sw_end_run(void)
{
    if (sw.run_peak > 0) { u8 p = sw.run_peak; sw.run_peak = 0; sw_commit_hit(sw.run_hz, p); }
}
static void sw_remove_selected(void)
{
    unsigned i;
    if (!sw.n_hits) return;
    if (sw.n_ignored < SW_MAX_IGNORED) sw.ignored[sw.n_ignored++] = sw.hits[sw.selected].hz;
    for (i = sw.selected; i + 1 < sw.n_hits; ++i) sw.hits[i] = sw.hits[i + 1];
    --sw.n_hits;
    if (sw.selected >= sw.n_hits) sw.selected = sw.n_hits ? sw.n_hits - 1 : 0;
    sw.list_dirty = true;
}
static void sweep_enter(void)
{
    u32 hz = api_get_master_freq(); unsigned b;
    if (!sw.inited) { sw.custom_lo = SW_BAND_LO[SW_CUSTOM]; sw.custom_hi = SW_BAND_HI[SW_CUSTOM]; sw.inited = true; }
    sw.input_len = 0; sw.n_ignored = 0;
    sw.band = 0;
    for (b = 0; b < SW_CUSTOM; ++b) if (hz >= SW_BAND_LO[b] && hz <= SW_BAND_HI[b]) { sw.band = b; break; }
    sw_restart();
    api_set_backlight_hold(true);
}
static void sweep_leave(void) { api_set_af_out(false); api_set_agc_fix(-1); api_set_backlight_hold(false); }
static void sw_dwell(u16 units) { u32 t0 = api_uptime_100us(); while (api_uptime_100us() - t0 < units) {} }
static void sw_finish_pass(void)
{
    u8 base;
    sw_end_run(); sw_update_floor();
    base = sw.floor > SW_GRAPH_BELOW_FLOOR ? sw.floor - SW_GRAPH_BELOW_FLOOR : 0;
    sw.graph_base = base;
    ++sw.pass; sw.pos = 0;
    memset(sw.hist, 0, sizeof(sw.hist)); sw.samples = 0;
    if (sw.pass >= SW_PASSES) {                    /* done: show the results */
        sw.scan_tenths = (api_uptime_100us() - sw.scan_start) / 1000;
        sw.view = SWV_RESULTS; sw.selected = 0; sw.scroll = 0; sw.full = true;
    }
}
static void sw_step(void)
{
    u32 pos = sw.pos, hz = sw.lo_hz + pos * sw_step_hz(); u16 rssi; u8 raw; unsigned col, bucket;
    api_retune_rx(hz, true);
    sw_dwell(SW_DWELL);
    rssi = api_read_rssi();
    if (rssi >= sw_trigger()) { u16 r2; sw_dwell(SW_CONFIRM); r2 = api_read_rssi(); if (r2 < rssi) rssi = r2; }
    raw = (u8)(rssi > 255 ? 255 : rssi);
    col = (unsigned)((u64)pos * SW_COLS / sw.total);
    if (raw > sw.hold[col]) sw.hold[col] = raw;
    bucket = raw / 2u; if (bucket > SW_HIST - 1) bucket = SW_HIST - 1;
    if (sw.hist[bucket] < 0xffff) ++sw.hist[bucket];
    if (sw.samples < 0xffff) ++sw.samples;
    if (sw.samples % SW_FLOOR_EVERY == 0) sw_update_floor();
    if (rssi >= sw_trigger()) {
        if (sw.run_peak > 0 && sw.run_pos + 1 != pos) sw_end_run();
        if (raw > sw.run_peak) { sw.run_peak = raw; sw.run_hz = hz; }
        sw.run_pos = pos;
    } else sw_end_run();
    if (++sw.pos >= sw.total) sw_finish_pass();
}
static void sweep_tick(u32 dt)
{
    (void)dt;
    if (sw.view == SWV_SCAN) {
        u32 t0 = api_uptime_100us();
        do sw_step(); while (sw.view == SWV_SCAN && api_uptime_100us() - t0 < SW_TICK_BUDGET);
    } else if (sw.view == SWV_LISTEN) sw.status_dirty = true;
}
static void sw_start_listen(void)
{
    bool am;
    if (!sw.n_hits) return;
    sw.listen_hz = sw.hits[sw.selected].hz;
    sw.view = SWV_LISTEN; sw.status_dirty = true;
    am = sw.listen_hz >= SW_AM_LOW_HZ && sw.listen_hz <= SW_AM_HIGH_HZ;
    api_set_agc_fix(-1);
    api_set_modulation(am ? 1 : 0);
    api_set_rx_freq(sw.listen_hz);
    api_set_bandwidth(true);
    api_set_subaudio_rx(0);
    api_set_sql_level((u8)api_settings_get(0));
    api_enter_rx();
}
static void sw_stop_listen(void)
{
    sw.view = SWV_RESULTS; sw.status_dirty = sw.list_dirty = true;
    api_set_holds_rx(false);
    api_set_af_out(false);
}
static void sw_commit_input(unsigned stage)
{
    u32 khz = 0; unsigned i; bool typed = sw.input_len > 0;
    for (i = 0; i < SW_INPUT_DIGITS; ++i) khz = khz * 10 + (i < sw.input_len ? sw.input[i] : 0);
    sw.input_len = 0;
    if (typed) { if (stage == 0) sw.custom_lo = khz * 1000; else sw.custom_hi = khz * 1000; }
    if (stage == 0) sw.view = SWV_INPUT1;
    else {
        if (sw.custom_lo < sw.custom_hi && sw.custom_lo >= SW_RX_MIN_HZ && sw.custom_hi <= SW_RX_MAX_HZ) {
            sw.band = SW_CUSTOM; sw_restart(); return;
        }
        sw.view = SWV_INPUT0;
    }
    sw.status_dirty = true;
}
static void sw_input_key(u8 id, unsigned stage)
{
    if (id <= 9) {
        if (sw.input_len < SW_INPUT_DIGITS) sw.input[sw.input_len++] = id;
        if (sw.input_len == SW_INPUT_DIGITS) sw_commit_input(stage);
    } else if (id == AK_MENU) sw_commit_input(stage);
    else if (id == AK_EXIT) { if (sw.input_len) sw.input_len = 0; else { sw.view = SWV_RESULTS; sw.full = true; } }
    sw.status_dirty = true;
}
static void sw_move_selection(bool down)
{
    if (!sw.n_hits) return;
    sw.selected = down ? (sw.selected + 1) % sw.n_hits : (sw.selected + sw.n_hits - 1) % sw.n_hits;
    sw.list_dirty = true;
}
static int sweep_key(u8 id, u8 kind)
{
    if (sw.view == SWV_SCAN) return kind == KIND_SINGLE && id == AK_EXIT ? APP_EXIT : APP_CONTINUE;
    if (sw.view == SWV_INPUT0 || sw.view == SWV_INPUT1) {
        if (kind == KIND_SINGLE) sw_input_key(id, sw.view == SWV_INPUT0 ? 0 : 1);
        return APP_CONTINUE;
    }
    if (sw.view == SWV_LISTEN) {
        if (kind == KIND_SINGLE && (id == AK_MENU || id == AK_EXIT)) sw_stop_listen();
        return APP_CONTINUE;
    }
    if (kind == KIND_LONG && id == AK_MENU) {
        if (sw.n_hits) { api_save_master_vfo(sw.hits[sw.selected].hz, 0); return APP_EXIT; }
        return APP_CONTINUE;
    }
    if (kind == KIND_REPEAT && (id == AK_UP || id == AK_DOWN)) { sw_move_selection(id == AK_DOWN); return APP_CONTINUE; }
    if (kind != KIND_SINGLE) return APP_CONTINUE;
    switch (id) {
    case AK_EXIT: return APP_EXIT;
    case AK_UP: case AK_DOWN: sw_move_selection(id == AK_DOWN); break;
    case AK_MENU: sw_start_listen(); break;
    case AK_STAR: sw_restart(); break;
    case AK_AB: sw.band = (sw.band + 1) % SW_CUSTOM; sw_restart(); break;
    case AK_SIDE1: sw_remove_selected(); break;
    case 1: sw.step_index = (sw.step_index + 1) % 2; sw_restart(); break;
    case 5: sw.input_len = 0; sw.view = SWV_INPUT0; sw.status_dirty = true; break;
    }
    return APP_CONTINUE;
}
/* ui */
static void sw_line(int y, const char *s, u16 fg) { api_fill_rect(0, y, SW_COLS, 8, A_BLACK); a_text(1, y, s, fg, A_BLACK, A_F5); }
static int sw_raw_to_y(u8 raw, u8 base)
{
    u32 v = raw > base ? (u32)(raw - base) : 0, h = SW_GRAPH_BOTTOM - SW_GRAPH_TOP;
    if (v > SW_GRAPH_RAW_SPAN) v = SW_GRAPH_RAW_SPAN;
    return SW_GRAPH_BOTTOM - (int)(v * h / SW_GRAPH_RAW_SPAN);
}
static void sw_draw_header(void)
{
    u32 lo, hi; struct sbuf l, r;
    sw_band_range(&lo, &hi);
    api_fill_rect(0, SW_HEADER_Y, SW_COLS, 8, A_BLACK);
    ab_init(&l); ab_freq(&l, lo); a_text(0, SW_HEADER_Y, l.b, A_WHITE, A_BLACK, A_F5);
    ab_init(&r); ab_freq(&r, hi); a_text(SW_COLS - (int)r.n * 5, SW_HEADER_Y, r.b, A_WHITE, A_BLACK, A_F5);
}
/* The band graph, drawn once when the scan finishes: a column per screen pixel, the selected signal marked. */
static void sw_draw_graph(void)
{
    unsigned x; int sel_x = -1;
    if (sw.n_hits) sel_x = (int)((u64)((sw.hits[sw.selected].hz - sw.lo_hz) / sw_step_hz()) * SW_COLS / sw.total);
    for (x = 0; x < SW_COLS; ++x) {
        int top = sw_raw_to_y(sw.hold[x], sw.graph_base);
        api_fill_rect((int)x, SW_GRAPH_TOP, 1, SW_GRAPH_BOTTOM - SW_GRAPH_TOP + 1, (int)x == sel_x ? 0x2104 : A_BLACK);
        if (sw.hold[x] > sw.graph_base) api_fill_rect((int)x, top, 1, SW_GRAPH_BOTTOM - top + 1, (int)x == sel_x ? A_WHITE : SW_BAR);
    }
    api_fill_rect(0, SW_GRAPH_BOTTOM + 1, SW_COLS, 1, SW_BAR_DIM);
}
static void sw_draw_list(void)
{
    unsigned slot;
    if (sw.selected < sw.scroll) sw.scroll = sw.selected;
    else if (sw.selected >= sw.scroll + SW_LIST_ROWS) sw.scroll = sw.selected + 1 - SW_LIST_ROWS;
    for (slot = 0; slot < SW_LIST_ROWS; ++slot) {
        int y = SW_LIST_Y + (int)slot * SW_ROW_H; unsigned i = sw.scroll + slot;
        struct sbuf b; u16 bg;
        if (i >= sw.n_hits) {
            api_fill_rect(0, y, SW_COLS, SW_ROW_H, A_BLACK);
            if (slot == 0) a_text(1, y, "NOTHING HEARD", A_GRAY, A_BLACK, A_F5);
            continue;
        }
        bg = i == sw.selected ? A_SELECT : A_BLACK;
        ab_init(&b); ab_freq(&b, sw.hits[i].hz); ab_s(&b, "  "); ab_i(&b, (int)sw.hits[i].peak - RSSI_DBM_BASE);
        ab_s(&b, "DBM X"); ab_u(&b, sw.hits[i].count);
        api_fill_rect(0, y, SW_COLS, SW_ROW_H, bg);
        a_text(1, y, b.b, A_WHITE, bg, A_F5);
    }
}
static void sw_draw_status(void)
{
    struct sbuf a; unsigned i; ab_init(&a);
    if (sw.view == SWV_INPUT0 || sw.view == SWV_INPUT1) {
        ab_s(&a, sw.view == SWV_INPUT0 ? "START " : "STOP ");
        for (i = 0; i < SW_INPUT_DIGITS; ++i) { if (i == 3) ab_ch(&a, '.'); ab_ch(&a, i < sw.input_len ? (char)('0' + sw.input[i]) : '-'); }
        ab_s(&a, " MHZ");
        sw_line(SW_STATUS_Y, a.b, A_YELLOW);
        sw_line(SW_HINT_Y, "MENU NEXT  EXIT BACK", A_GRAY);
        return;
    }
    if (sw.view == SWV_LISTEN) {
        ab_s(&a, "RX "); ab_freq(&a, sw.listen_hz); ab_ch(&a, ' '); ab_i(&a, (int)api_read_rssi() - RSSI_DBM_BASE); ab_s(&a, "DBM");
        sw_line(SW_STATUS_Y, a.b, A_YELLOW);
        sw_line(SW_HINT_Y, "MENU BACK TO LIST", A_GRAY);
        return;
    }
    ab_s(&a, sw.band == SW_CUSTOM ? "CUSTOM" : SW_BAND_NAME[sw.band]);
    ab_s(&a, sw.step_index == 0 ? " 25K " : " 12.5K ");
    ab_u(&a, sw.n_hits); ab_s(&a, " HEARD ");
    ab_u(&a, sw.scan_tenths / 10); ab_ch(&a, 'S');
    sw_line(SW_STATUS_Y, a.b, A_GRAY);
    sw_line(SW_HINT_Y, "MENU RX * RESCAN AB BAND", A_GRAY);
}
static void sw_draw_scanning(unsigned pct)
{
    struct sbuf a; int w = (int)((A_W - 20) * pct / 100);
    if (sw.full) {
        api_fill_rect(0, 0, A_W, A_H, A_BLACK);
        a_title("BAND SWEEP", 0);
        a_text(16, 40, "SCANNING...", A_WHITE, A_BLACK, A_F9);
        ab_init(&a); ab_s(&a, sw.band == SW_CUSTOM ? "CUSTOM" : SW_BAND_NAME[sw.band]); ab_ch(&a, ' ');
        ab_freq(&a, sw.lo_hz); ab_ch(&a, '-');
        { u32 lo, hi; sw_band_range(&lo, &hi); ab_u(&a, hi / 1000000); }
        a_text(4, 22, a.b, A_GRAY, A_BLACK, A_F6);
        api_fill_rect(9, 69, A_W - 18, 12, SW_BAR_DIM);
        api_fill_rect(10, 70, A_W - 20, 10, A_BLACK);
        a_text(4, 119, "EXIT LEAVE", A_GRAY, A_BLACK, A_F5);
        sw.full = false;
    }
    api_fill_rect(10, 70, w, 10, SW_BAR);
    ab_init(&a); ab_u(&a, pct); ab_s(&a, "%  "); ab_u(&a, sw.n_hits); ab_s(&a, " SIGNALS");
    a_line_tail(90, 10, a.b, A_WHITE, A_F6);
}
static void sweep_draw(void)
{
    if (sw.view == SWV_SCAN) {
        unsigned pct = (unsigned)(((u64)sw.pass * sw.total + sw.pos) * 100 / ((u64)SW_PASSES * sw.total));
        if (pct != sw.shown_pct || sw.full) { sw.shown_pct = pct; sw_draw_scanning(pct); }
        return;
    }
    if (sw.full) {
        api_fill_rect(0, 0, SW_COLS, A_H, A_BLACK);
        sw_draw_header();
        sw.list_dirty = sw.status_dirty = true;
        sw.full = false;
    }
    if (sw.list_dirty) { sw_draw_graph(); sw_draw_list(); sw.list_dirty = false; }
    if (sw.status_dirty) { sw_draw_status(); sw.status_dirty = false; }
}
