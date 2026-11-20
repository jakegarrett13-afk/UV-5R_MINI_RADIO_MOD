/* N22: SEARCH, SCANQT and SEARCH/QT SCAN, ported from firmware-source/apps/search2 and qtscan2 (logic.rs; search and
 * scanqt are the same code without the "1" key that switches between the two, so one port serves all three).
 * SEARCH: the chip's frequency counter finds a nearby transmitter, then reads its tone; MENU puts both in the VFO.
 * QT SCAN: on the current frequency, reads the tone of whoever is talking; MENU saves it as the receive tone. */
static bool sq_combined;     /* launched as SEARCH/QT SCAN: "1" switches segments */
#define SQ_SEG_QT 0
#define SQ_SEG_SEARCH 1
#define SQ_TITLE_Y 4
#define SQ_FREQ_Y 30
#define SQ_STATUS_Y 61
#define SQ_TONE_X 64
#define SQ_HINT_Y 111
#define SQ_SWITCH_Y 99
/* ---------------- search2 ---------------- */
#define SE_TONE_TIMEOUT 5000u
#define SE_AGREEMENT 25u
enum { SEB_UHF, SEB_VHF, SEB_200, SEB_350 };
static const char *const SE_BAND_LABEL[4] = {"UHF", "VHF", "220M", "350M"};
enum { SEP_SETUP, SEP_HUNT, SEP_CHECK, SEP_TONESETUP, SEP_TONEWAIT, SEP_FOUND };
static struct search_state {
    u8 band, phase; bool have_sample; u32 sample, found_word, timeout; int tone_index;
    bool drawn; u8 last_band, last_phase; u32 last_word; int last_tone;
} se;
static bool se_uhf_path(u8 b) { return b == SEB_UHF || b == SEB_350; }
static bool se_band_fold(u8 band, u32 *f)
{
    u32 freq = *f;
    switch (band) {
    case SEB_UHF: if (freq > 52000000) freq /= 2; if (freq < 40000000) return false; break;
    case SEB_VHF:
        if (freq >= 36000000) freq /= 3; else if (freq > 25000000) freq /= 2; else if (freq < 10000000) return false;
        if (freq > 20000000) return false;
        break;
    case SEB_200: if (freq >= 40000000) freq /= 2; else if (freq < 20000000) return false; if (freq > 30000000) return false; break;
    default: if (freq >= 70000000) freq /= 2; else if (freq < 30000000) return false; if (freq > 40000000) return false; break;
    }
    *f = freq;
    return true;
}
static u32 se_clamp(u8 band, u32 f)
{
    switch (band) {
    case SEB_VHF: return f < 13600000 ? 13600000 : f > 17400000 ? 17400000 : f;
    case SEB_200: return f < 20000000 ? 20000000 : f > 26000000 ? 26000000 : f;
    case SEB_350: return f > 39000000 ? 39000000 : f < 30000000 ? 35000000 : f;
    default: return f < 40000000 ? 40000000 : f > 52000000 ? 52000000 : f;
    }
}
static void se_master_rx(void)
{
    api_set_modulation(0);
    api_set_rx_freq(api_get_master_freq());
    api_set_bandwidth(api_get_master_wide());
    api_set_subaudio_rx(0);
    api_set_sql_level((u8)api_settings_get(0));
    api_enter_rx();
}
static void search_enter(void)
{
    se_master_rx();
    memset(&se, 0, sizeof(se));
    se.band = SEB_UHF; se.phase = SEP_SETUP; se.tone_index = -1;
}
static void search_leave(void) { api_freq_scan_disable(); api_set_subaudio_scan_filter(false); }
static void se_restart(void) { se.have_sample = false; se.tone_index = -1; se.phase = SEP_SETUP; }
static void search_tick(u32 dt)
{
    u32 raw;
    switch (se.phase) {
    case SEP_SETUP: api_freq_scan_enable(); se.have_sample = false; se.phase = SEP_HUNT; break;
    case SEP_HUNT:
        raw = api_check_freq_scan();
        if (raw == 0xFFFFFFFFu) break;
        api_freq_scan_disable();
        if (!se.have_sample) { se.sample = raw; se.have_sample = true; api_freq_scan_enable(); break; }
        se.have_sample = false;
        if (a_absdiff(se.sample, raw) < SE_AGREEMENT) {
            u32 avg = (se.sample + raw) / 2;
            if (se_band_fold(se.band, &avg)) { se.found_word = avg; se.phase = SEP_CHECK; }
            else se.phase = SEP_SETUP;
        } else api_freq_scan_enable();
        break;
    case SEP_CHECK: {
        u32 corrected = api_correct_measured_freq_word(se.found_word), rounded = (corrected + 13) / 25 * 25;
        se.found_word = se_clamp(se.band, rounded);
        api_tune_search_candidate(se.found_word * 10, se_uhf_path(se.band));
        api_set_subaudio_scan_filter(true);
        se.phase = SEP_TONESETUP;
        break;
    }
    case SEP_TONESETUP: se.timeout = SE_TONE_TIMEOUT; se.tone_index = -1; se.phase = SEP_TONEWAIT; break;
    case SEP_TONEWAIT: {
        int idx = api_detect_subaudio();
        if (idx == -1) { se.timeout = se.timeout > dt ? se.timeout - dt : 0; if (!se.timeout) se.phase = SEP_FOUND; }
        else { se.tone_index = idx <= -2 ? -1 : idx; se.phase = SEP_FOUND; }
        break;
    }
    }
}
static int search_key(u8 id, u8 kind)
{
    if (kind != KIND_SINGLE && kind != KIND_LONG) return APP_CONTINUE;
    if (id == AK_AB && kind == KIND_SINGLE) { se.band = (se.band + 1) % 4; se_restart(); return APP_CONTINUE; }
    if ((id == AK_UP || id == AK_DOWN) && kind == KIND_SINGLE && se.phase == SEP_FOUND) { se_restart(); return APP_CONTINUE; }
    if (id == AK_MENU && kind == KIND_SINGLE && se.phase == SEP_FOUND) {
        api_save_master_vfo(se.found_word * 10, se.tone_index >= 0 ? api_subaudio_code_of_index(se.tone_index) : 0);
        search_leave();
        return APP_EXIT;
    }
    if (id == AK_EXIT) { search_leave(); return APP_EXIT; }
    if (id == 1 && kind == KIND_SINGLE && sq_combined) { search_leave(); return APP_CHAIN(SQ_SEG_QT); }
    return APP_CONTINUE;
}
static void se_draw_freq(void)
{
    u8 b[16]; unsigned n = api_fmt_freq(se.found_word * 10, b);
    api_draw_text(4, SQ_FREQ_Y, b, n, A_WHITE, A_BLACK, A_F9);
}
static void se_draw_header(void)
{
    char t[20];
    api_fill_rect(0, 0, A_W, 13, A_TITLE);
    snprintf(t, sizeof(t), "SEARCH %s", SE_BAND_LABEL[se.band]);
    a_text(3, 2, t, A_WHITE, A_TITLE, A_F6);
}
static void se_draw_content(void)
{
    api_fill_rect(0, 26, A_W, 22, A_BLACK);
    api_fill_rect(0, 58, A_W, 14, A_BLACK);
    if (se.phase <= SEP_CHECK) a_text(4, SQ_STATUS_Y, "HUNTING...", A_WHITE, A_BLACK, A_F6);
    else if (se.phase != SEP_FOUND) { se_draw_freq(); a_text(4, SQ_STATUS_Y, "LISTEN", A_WHITE, A_BLACK, A_F6); }
    else {
        struct sbuf b; ab_init(&b); ab_tone(&b, se.tone_index);
        se_draw_freq();
        a_text(4, SQ_STATUS_Y, "FOUND", A_WHITE, A_BLACK, A_F6);
        a_text(SQ_TONE_X, SQ_STATUS_Y, b.b, A_GREEN, A_BLACK, A_F6);
    }
}
static void search_draw(void)
{
    if (!se.drawn) {
        api_fill_rect(0, 0, A_W, A_H, A_BLACK);
        se_draw_header(); se_draw_content();
        if (sq_combined) a_text(4, SQ_SWITCH_Y, "1 QT SCAN", A_GRAY, A_BLACK, A_F6);
        a_text(4, SQ_HINT_Y, "AB BAND  MENU SAVE", A_GRAY, A_BLACK, A_F6);
    } else {
        if (se.last_band != se.band) se_draw_header();
        if (se.last_phase != se.phase || se.last_word != se.found_word || se.last_tone != se.tone_index) se_draw_content();
    }
    se.drawn = true; se.last_band = se.band; se.last_phase = se.phase; se.last_word = se.found_word; se.last_tone = se.tone_index;
}
/* ---------------- qtscan2 ---------------- */
#define QT_CTCSS_HITS 2
#define QT_DEBOUNCE 1000u
enum { QTP_WAIT, QTP_DETECT, QTP_FOUND };
static struct qt_state { u8 phase; u32 debounce; int candidate; u8 hits; int tone_index; bool drawn; u8 last_phase; int last_tone; } qt;
static void qtscan_enter(void)
{
    se_master_rx();
    api_set_subaudio_scan_filter(false);
    memset(&qt, 0, sizeof(qt));
    qt.phase = QTP_WAIT; qt.candidate = -1; qt.tone_index = -1;
}
static void qtscan_leave(void) { api_set_subaudio_scan_filter(false); }
static void qtscan_tick(u32 dt)
{
    bool open = api_squelch_open();
    int idx;
    switch (qt.phase) {
    case QTP_WAIT:
        if (open) {
            qt.debounce += dt;
            if (qt.debounce >= QT_DEBOUNCE) { qt.debounce = 0; qt.hits = 0; qt.candidate = -1; qt.phase = QTP_DETECT; api_set_subaudio_scan_filter(true); }
        } else qt.debounce = 0;
        break;
    case QTP_DETECT:
        if (!open) {
            qt.debounce += dt;
            if (qt.debounce >= QT_DEBOUNCE) { qt.debounce = 0; qt.phase = QTP_WAIT; api_set_subaudio_scan_filter(false); }
            break;
        }
        qt.debounce = 0;
        idx = api_detect_subaudio();
        if (idx < 0) { qt.candidate = -1; qt.hits = 0; }
        else if (idx < CTCSS_COUNT) {
            if (qt.candidate == idx) { if (++qt.hits >= QT_CTCSS_HITS) { qt.tone_index = idx; qt.phase = QTP_FOUND; } }
            else { qt.candidate = idx; qt.hits = 1; }
        } else { qt.tone_index = idx; qt.phase = QTP_FOUND; }
        break;
    }
}
static int qtscan_key(u8 id, u8 kind)
{
    if (kind != KIND_SINGLE) return APP_CONTINUE;
    if ((id == AK_UP || id == AK_DOWN) && qt.phase == QTP_FOUND) {
        qt.tone_index = -1; qt.candidate = -1; qt.hits = 0; qt.debounce = 0; qt.phase = QTP_WAIT;
        api_set_subaudio_scan_filter(false);
        return APP_CONTINUE;
    }
    if (id == AK_MENU && qt.phase == QTP_FOUND) {
        if (qt.tone_index >= 0) api_save_master_subaudio(api_subaudio_code_of_index(qt.tone_index), qt.tone_index < CTCSS_COUNT);
        qtscan_leave();
        return APP_EXIT;
    }
    if (id == AK_EXIT) { qtscan_leave(); return APP_EXIT; }
    if (id == 1 && sq_combined) { api_set_subaudio_scan_filter(false); return APP_CHAIN(SQ_SEG_SEARCH); }
    return APP_CONTINUE;
}
static void qt_draw_status(void)
{
    static const char *const label[3] = {"WAITING...", "DETECTING...", "FOUND"};
    a_text(4, SQ_STATUS_Y, label[qt.phase], A_WHITE, A_BLACK, A_F6);
    if (qt.phase == QTP_FOUND) {
        struct sbuf b; int twin = dcs_twin(qt.tone_index);
        ab_init(&b); ab_tone(&b, qt.tone_index);
        a_text(SQ_TONE_X, SQ_STATUS_Y, b.b, A_GREEN, A_BLACK, A_F6);
        if (twin >= 0) {
            ab_init(&b); ab_tone(&b, twin);
            a_text(4, SQ_STATUS_Y + 13, "SAME AS", A_GRAY, A_BLACK, A_F6);
            a_text(SQ_TONE_X, SQ_STATUS_Y + 13, b.b, A_GREEN, A_BLACK, A_F6);
        }
    }
}
static void qtscan_draw(void)
{
    if (!qt.drawn) {
        u8 b[16]; unsigned n = api_fmt_freq(api_get_master_freq(), b);
        api_fill_rect(0, 0, A_W, A_H, A_BLACK);
        api_fill_rect(0, 0, A_W, 13, A_TITLE);
        a_text(3, 2, "QT SCAN", A_WHITE, A_TITLE, A_F6);
        api_draw_text(4, SQ_FREQ_Y, b, n, A_WHITE, A_BLACK, A_F9);
        qt_draw_status();
        if (sq_combined) a_text(4, SQ_SWITCH_Y, "1 SEARCH", A_GRAY, A_BLACK, A_F6);
        a_text(4, SQ_HINT_Y, "MENU SAVE  EXIT", A_GRAY, A_BLACK, A_F6);
    } else if (qt.last_phase != qt.phase || qt.last_tone != qt.tone_index) {
        api_fill_rect(0, 58, A_W, 28, A_BLACK);
        qt_draw_status();
    }
    qt.drawn = true; qt.last_phase = qt.phase; qt.last_tone = qt.tone_index;
}
