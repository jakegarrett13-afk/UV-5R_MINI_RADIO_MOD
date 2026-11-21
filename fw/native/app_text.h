/* TEXT app: C port of the UV-5RM custom firmware's textapp (source-changes/apps_new/textapp/src/logic.rs).
 * Same over-the-air protocol (BK4829 FSK modem, 450 baud, 4-byte sync, scrambled 32-byte packets with checksum,
 * call and response with up to 8 tries), so it talks to the 5RM radios. Mini changes: 128 px layout (21 columns,
 * 7 message rows, no T9 key reference, 3 compose rows), electric-blue header/highlight, message log at NOR 0xF8000-0xFBFFF and quick
 * messages at 0xF7000 (both verified blank on this radio; the 5RM used 0x1F0000 / 0x194000).
 * Included by mini_app.c after the app API (api_*) and rf_read/rf_write. */
#define TK_STAR 10
#define TK_POUND 11
#define TK_MENU 12
#define TK_EXIT 13
#define TK_UP 14
#define TK_DOWN 15
#define TK_VM 16
#define TK_AB 17
#define TK_BAND 18
#define T_NO_KEY 0xff
#define KIND_SINGLE 2
#define KIND_LONG 3

#define T_BLACK 0x0000
#define T_WHITE 0xffff
#define T_GRAY 0x8410
#define T_GREEN 0x07e0
#define T_HEADER C_BLUE
#define T_HILITE C_BLUE
#define T_AMBER 0xfd20
#define T_F5 0
#define T_F6 1
#define T_W 128
#define T_COLS 21
#define T_TITLE_Y 0
#define T_KEYS_Y 13
#define T_MSG_Y 14          /* N15: T9 key reference removed (user request); messages start under the title */
#define T_MSG_ROWS 7
#define T_COMP_Y 87
#define T_HINT_Y 119

#define R_INT 0x02
#define R_STATUS 0x0C
#define R_FILTER 0x2B
#define R_STATE 0x30
#define R_IRQ_EN 0x3F
#define R_DEV 0x40
#define R_AF 0x47
#define R_VOLUME 0x48
#define R_FSK 0x58
#define R_CTRL 0x59
#define R_SYNC1 0x5A
#define R_SYNC2 0x5B
#define R_CRC 0x5C
#define R_SIZE 0x5D
#define R_THRESH 0x5E
#define R_FIFO 0x5F
#define R_GAIN 0x70
#define R_TONE 0x71
#define R_BAUD 0x72
#define IRQ_TX_DONE (1u << 15)
#define IRQ_RX_DONE (1u << 13)
#define IRQ_WORD (1u << 12)
#define IRQ_SYNC (1u << 1)
#define CTRL_CLEAR ((1u << 15) | (1u << 14))
#define CTRL_RX_ON (1u << 12)
#define CTRL_TX_ON (1u << 11)
#define CTRL_RX (1u << 3)
#define CTRL_TX ((15u << 4) | CTRL_RX)
static const u8 t_saved_regs[10] = {R_FSK, R_CTRL, R_SYNC1, R_SYNC2, R_CRC, R_SIZE, R_THRESH, R_GAIN, R_BAUD, R_FILTER};
#define T_FSK 0x0301
#define T_BAUD 4646
#define LEAD_MS 30
#define NEXT_PACKET_LEAD_MS 20
#define TAIL_MS 20         /* N38: was 100. The 5RM schedules its reply from the end of the packet and needs 150 ms
                            * of quiet first, so every ms of tail is a ms less for the Mini to come back to receive */
#define PACKET 32
#define RX_SIZE 34
#define CHARS 25
#define CHECK_AT (4 + CHARS)
#define T_MAGIC 0xB5
#define MAX_LEN (2 * CHARS)
#define TAP_100US 10000u
#define NOTICE_100US 25000u
#define RX_WAIT_STEPS 350
#define MAX_TRIES 8
#define PACKET_100US 10000u
#define REPLY_DELAY_100US 2500u
#define QUIET_100US 1500u
#define BUSY_RAW 72
#define REPLY_GIVE_UP_100US 30000u
#define RETRY_GIVE_UP_100US 50000u
#define REPLY_WAIT_100US 17000u
#define REPLY_JITTER_MASK 0x0FFFu
#define JOIN_100US 400000u
#define STATUS_RECEIVED 0
#define STATUS_SENDING 3
#define STATUS_DELIVERED 1
#define STATUS_FAILED 2
#define LOG_ADDR 0xF8000u
#define LOG_SLOTS 256
#define LOG_PER_SECTOR 64
#define LOG_SECTORS 4
#define SECTOR_BYTES 4096u
#define REC_BYTES 64
#define REC_MAGIC 0xA7
#define REC_HEAD 6
#define STATE_TONE 0x0302
#define AF_BEEP 0x6340
#define VOLUME_BEEP 0xB8A2
#define TONE_GAIN (0x8000 | (60 << 8))
#define TONE_MS 110
#define CHIME_NEW 1
#define CHIME_DELIVERED 2
#define QUICK_ADDR 0xF7000u
#define QUICK_BYTES 64
#define QUICK_MAGIC 'Q'
#define QUICK_COUNT 10
static const char *const quick_defaults[QUICK_COUNT] = {
    "COPY THAT", "YES", "NO", "ARE YOU OK?", "ALL OK HERE", "ON MY WAY", "ARRIVED SAFE", "WHERE ARE YOU?",
    "STAY PUT, I'M COMING TO YOU", "EMERGENCY! NEED HELP NOW",
};
#define LIST_ROW 7          /* 10 rows end at 83, above the compose box at 87 (5RM: 8 px rows, box at 97) */
#define LIST_CHARS 23
static const char T_LOCKED[] = "LOCKED: HOLD * TO UNLOCK";
static const char *const taps[10] = {" 0", ".,?!-/1", "ABC2", "DEF3", "GHI4", "JKL5", "MNO6", "PQRS7", "TUV8", "WXYZ9"};
enum { TX_SENT, TX_BLOCKED, TX_STUCK };

static struct text_state {
    bool redraw_all, redraw_msgs, redraw_comp, redraw_hint, armed, high_power, locked, picking;
    u8 pick; u32 hz; bool wide; u8 me, next_id; u16 pings;
    u16 saved[10];
    u8 comp[MAX_LEN], comp_len, tap_key, tap_index; u32 tap_left;
    u16 log_head, log_count, log_seq, scroll, out_slot;
    u8 chime_due; bool erase_armed;
    u8 rx_buf[PACKET], rx_i; bool rx_active;
    u8 join[2 * CHARS], join_have, join_total, join_from, join_id; u32 join_age; bool join_active;
    u8 last_from, last_id; bool has_last;
    u8 out[MAX_LEN], out_len, out_id; bool pending; u8 tries; u32 retry_at, last_busy;
    bool reply_due; u32 reply_at; u8 reply_to, reply_id;
    const char *notice; u32 notice_left;
    u8 n_sync, n_ok, n_bad, n_tx, n_delivered, n_reply_seen;   /* N16 diagnostics */
    int rx_off;                                                  /* N37: receive offset in use, Hz */
    u32 unkey_at; u16 carr_ms, sync_ms; bool window;            /* N38: reply-window timing after our unkey */
} ts;

/* N37: receive-offset hunt. The other radio's carrier sits a few hundred Hz off ours (FREQ ERR: -430 / -470 Hz with
 * a 5RM), enough to put TEXT's narrow 450-baud FSK at the edge of the sync detector, so replies were heard on some days
 * and not others. The transmit frequency never moves (the other radio hears the Mini fine); the receive frequency for
 * the reply to try n is the learned offset plus T_RX_STEP[n]. A good packet makes that offset the learned one; it is
 * kept across TEXT sessions (RAM). */
static const s16 T_RX_STEP[MAX_TRIES] = {0, -400, 400, -800, 800, -200, 200, -600};
/* Start: FREQ ERR with the user's 5RM read -470 Hz cool and -630 Hz during TEXT (transmitting warms the crystals), so
 * the receiver starts 550 Hz low; the steps cover drift either way. */
static int t_rx_learn = -550;             /* Hz */
/* N38: the last 8 reply windows (cable diagnostics), each (ms from our unkey to the reply carrier | ms to sync << 16), 0xFFFF = none */
static u32 t_rec[8];
static unsigned t_rec_n;
#define T_RX_LIMIT 1200
static void t_set_notice(const char *text) { ts.notice = text; ts.notice_left = NOTICE_100US; ts.redraw_hint = true; }

static void modem_off(void)
{
    unsigned i;
    if (!ts.armed) return;
    ts.armed = false;
    rf_write(R_IRQ_EN, 0);
    rf_write(R_CTRL, CTRL_CLEAR);
    for (i = 0; i < 10; ++i) rf_write(t_saved_regs[i], t_saved_regs[i] == R_CTRL ? ts.saved[i] & 0x07FF : ts.saved[i]);
    rf_write(R_INT, 0);
}
static void modem_common(u16 size)
{
    rf_write(R_GAIN, (1 << 7) | 96);
    rf_write(R_BAUD, T_BAUD);
    rf_write(R_FSK, T_FSK);
    rf_write(R_SYNC1, 0x3072);
    rf_write(R_SYNC2, 0x576C);
    rf_write(R_CRC, 0x5625);
    rf_write(R_SIZE, size << 8);
}
static void rx_restart(void)
{
    rf_write(R_CTRL, CTRL_RX | CTRL_CLEAR);
    rf_write(R_CTRL, CTRL_RX);
    rf_write(R_INT, 0);
    rf_write(R_CTRL, CTRL_RX | CTRL_RX_ON);
}
static void modem_rx(void)
{
    /* N18 tried REG 0x2B = 0x0300 here (the 5RM's receive filter): syncs went from 1/6 to 0/8, so N19 leaves the
     * Mini's stock filter alone. */
    modem_common(RX_SIZE);
    rf_write(R_THRESH, (64 << 3) | 1);
    rf_write(R_IRQ_EN, IRQ_RX_DONE | IRQ_WORD | IRQ_SYNC);
    ts.rx_active = false;
    rx_restart();
}
static u8 t_checksum(const u8 *p) { u8 a = 0x5A; unsigned i; for (i = 0; i < CHECK_AT; ++i) a += p[i]; return a; }
static void t_scramble(u8 *p)
{
    u8 lfsr = 0xA5; unsigned i;
    for (i = 0; i < PACKET; ++i) { p[i] ^= lfsr; lfsr = (u8)((lfsr << 1) ^ ((lfsr & 0x80) ? 0x1D : 0)); }
}
static bool tx_packet(const u8 *p, u32 lead_ms)
{
    unsigned i; bool done = false;
    rf_write(R_CTRL, CTRL_TX | CTRL_CLEAR);
    rf_write(R_CTRL, CTRL_TX);
    rf_write(R_INT, 0);
    rf_write(R_IRQ_EN, IRQ_TX_DONE);
    api_delay_ms(lead_ms);
    for (i = 0; i < PACKET; i += 2) rf_write(R_FIFO, (u16)(p[i + 1] << 8 | p[i]));
    rf_write(R_CTRL, CTRL_TX | CTRL_TX_ON);
    for (i = 0; i < 300; ++i) {
        api_delay_ms(5);
        if (rf_read(R_STATUS) & 1) {
            rf_write(R_INT, 0);
            if (rf_read(R_INT) & IRQ_TX_DONE) { done = true; break; }
        }
    }
    rf_write(R_CTRL, CTRL_TX);
    rf_write(R_INT, 0);
    return done;
}
static void t_seal(const u8 *in, u8 *p)
{
    memcpy(p, in, PACKET);
    p[0] = T_MAGIC;
    p[CHECK_AT] = t_checksum(p);
    p[CHECK_AT + 1] = 0x55;
    p[CHECK_AT + 2] = 0x55;
    t_scramble(p);
}
static int t_transmit(u8 packets[][PACKET], unsigned count)
{
    u16 dev, filter; unsigned i; int result = TX_SENT; u8 p[PACKET];
    api_set_tx_freq(ts.hz);
    api_set_power(ts.high_power ? 0 : 1);
    api_set_subaudio_tx(0);
    api_set_tx_enabled(true);
    api_set_ptt(true);
    if (!(api_tx_state() & 1)) {
        api_set_ptt(false);
        api_set_tx_enabled(false);
        modem_rx();
        return TX_BLOCKED;
    }
    dev = rf_read(R_DEV);
    filter = rf_read(R_FILTER);
    rf_write(R_DEV, (dev & 0xF000) | (ts.wide ? 1300 : 1200));
    rf_write(R_FILTER, (1 << 2) | 1);
    modem_common(PACKET);
    for (i = 0; i < count; ++i) {
        t_seal(packets[i], p);
        if (!tx_packet(p, i == 0 ? LEAD_MS : NEXT_PACKET_LEAD_MS)) { result = TX_STUCK; break; }
    }
    if (result == TX_SENT) ++ts.n_tx;
    api_delay_ms(TAIL_MS);
    rf_write(R_DEV, dev);
    rf_write(R_FILTER, filter);
    api_set_ptt(false);
    ts.unkey_at = api_uptime_100us();                         /* N38: open the reply window */
    ts.window = true; ts.carr_ms = ts.sync_ms = 0xFFFF;
    api_set_tx_enabled(false);
    api_set_af_out(false);
    modem_rx();
    return result;
}
static int send_out(void)
{
    unsigned len = ts.out_len, total = (len + CHARS - 1) / CHARS, part;
    u8 packets[2][PACKET];
    {   /* N37: listen for this try's reply at the learned offset plus the try's step */
        unsigned k = ts.pending && ts.tries ? (unsigned)(ts.tries - 1) % MAX_TRIES : 0;
        int off = t_rx_learn + T_RX_STEP[k];
        if (off > T_RX_LIMIT) off = T_RX_LIMIT;
        if (off < -T_RX_LIMIT) off = -T_RX_LIMIT;
        ts.rx_off = off;
        api_set_rx_freq((u32)((int)ts.hz + off));
    }
    if (total < 1) total = 1;
    memset(packets, 0, sizeof(packets));
    for (part = 0; part < total; ++part) {
        unsigned start = part * CHARS, end = len < start + CHARS ? len : start + CHARS;
        packets[part][1] = ts.me;
        packets[part][2] = ts.out_id;
        packets[part][3] = (u8)((part << 4) | total);
        if (end > start) memcpy(packets[part] + 4, ts.out + start, end - start);
    }
    ts.redraw_hint = true;
    return t_transmit(packets, total);
}
static u32 next_retry_at(void) { u32 now = api_uptime_100us(); return now + REPLY_WAIT_100US + (now & REPLY_JITTER_MASK); }
static bool is_due(u32 at) { return (int)(api_uptime_100us() - at) >= 0; }
static u32 log_addr(u16 slot) { return LOG_ADDR + (u32)slot * REC_BYTES; }
static void read_head(u16 slot, u8 *h) { api_nor_read(log_addr(slot), h, REC_HEAD); }
static bool head_valid(const u8 *h) { return h[0] == REC_MAGIC && h[3] <= MAX_LEN; }
static void t_store(const u8 *text, unsigned n, u8 status, u8 id);
static bool start_send(const u8 *text, unsigned n)
{
    int r;
    if (ts.pending) { t_set_notice("WAIT: STILL SENDING"); return false; }
    if (n > MAX_LEN) n = MAX_LEN;
    memcpy(ts.out, text, n);
    ts.out_len = n;
    ts.out_id = ts.next_id++;
    r = send_out();
    if (r == TX_SENT) {
        ts.pending = true;
        ts.tries = 1;
        ts.retry_at = next_retry_at();
        ts.notice = 0; ts.notice_left = 0;
        t_store(text, n, STATUS_SENDING, ts.out_id);
        ts.out_slot = (ts.log_head + LOG_SLOTS - 1) % LOG_SLOTS;
        return true;
    }
    t_set_notice(r == TX_BLOCKED ? "TX BLOCKED: SEE SETTINGS" : "TX FAILED: NO DONE FLAG");
    return false;
}
static void end_send(u8 status, const char *notice)
{
    u8 h[REC_HEAD];
    ts.pending = false;
    if (ts.rx_off != t_rx_learn) {                           /* N37: back to the learned offset to listen */
        ts.rx_off = t_rx_learn;
        api_set_rx_freq((u32)((int)ts.hz + t_rx_learn));
        api_enter_rx();
        api_set_holds_rx(false);
        api_set_af_out(false);
        modem_rx();
    }
    read_head(ts.out_slot, h);
    if (head_valid(h) && h[1] == STATUS_SENDING && h[2] == ts.out_id) api_nor_write(log_addr(ts.out_slot) + 1, &status, 1);
    ts.redraw_msgs = true;
    t_set_notice(notice);
}
static void t_retry(void)
{
    int r;
    if (ts.tries >= MAX_TRIES) { end_send(STATUS_FAILED, "NOT CONFIRMED"); return; }
    ++ts.tries;
    r = send_out();
    if (r == TX_SENT) ts.retry_at = next_retry_at();
    else end_send(STATUS_FAILED, r == TX_BLOCKED ? "TX BLOCKED: SEE SETTINGS" : "TX FAILED: NO DONE FLAG");
}
static void log_open(void)
{
    u16 slot, best = 0; bool have = false; u8 h[REC_HEAD];
    ts.log_count = 0; ts.log_head = 0;
    for (slot = 0; slot < LOG_SLOTS; ++slot) {
        u16 seq;
        read_head(slot, h);
        if (!head_valid(h)) continue;
        ++ts.log_count;
        seq = h[4] | (h[5] << 8);
        if (!have || (short)(seq - best) > 0) { best = seq; have = true; ts.log_head = (slot + 1) % LOG_SLOTS; }
    }
    ts.log_seq = have ? best + 1 : 0;
}
static void log_erase(void)
{
    unsigned sector;
    for (sector = 0; sector < LOG_SECTORS; ++sector) api_nor_erase_sector(LOG_ADDR + sector * SECTOR_BYTES);
    ts.log_head = 0; ts.log_count = 0; ts.scroll = 0; ts.redraw_msgs = true;
}
static void t_store(const u8 *text, unsigned n, u8 status, u8 id)
{
    u8 rec[REC_BYTES], h[REC_HEAD];
    if (ts.log_head % LOG_PER_SECTOR == 0) {
        read_head(ts.log_head, h);
        if (head_valid(h)) ts.log_count = ts.log_count > LOG_PER_SECTOR ? ts.log_count - LOG_PER_SECTOR : 0;
        api_nor_erase_sector(log_addr(ts.log_head));
    }
    if (n > MAX_LEN) n = MAX_LEN;
    memset(rec, 0xFF, sizeof(rec));
    rec[0] = REC_MAGIC; rec[1] = status; rec[2] = id; rec[3] = n;
    rec[4] = ts.log_seq; rec[5] = ts.log_seq >> 8;
    memcpy(rec + REC_HEAD, text, n);
    api_nor_write(log_addr(ts.log_head), rec, REC_BYTES);
    ts.log_head = (ts.log_head + 1) % LOG_SLOTS;
    if (ts.log_count < LOG_SLOTS) ++ts.log_count;
    ++ts.log_seq;
    ts.scroll = 0;
    ts.redraw_msgs = true;
}
/* Mini: the 5RM's tone registers (AF 0x6340) only pop here, so the chime uses the firmware's stock-exact beep. */
static void chime(const u16 *notes, unsigned count)
{
    api_beep(notes, count, TONE_MS);
    api_enter_rx();
    api_set_holds_rx(false);
    api_set_af_out(false);
    modem_rx();
}
static void schedule_reply(u8 to, u8 id, u32 parts_after)
{
    ts.reply_due = true; ts.reply_to = to; ts.reply_id = id;
    ts.reply_at = api_uptime_100us() + REPLY_DELAY_100US + parts_after * PACKET_100US;
}
static void send_reply(void)
{
    u8 p[1][PACKET];
    memset(p, 0, sizeof(p));
    p[0][1] = ts.me; p[0][4] = ts.reply_to; p[0][5] = ts.reply_id;
    if (t_transmit(p, 1) != TX_SENT) t_set_notice("CAN'T REPLY: TX BLOCKED");
}
static void finish_join(void)
{
    u8 out[MAX_LEN]; unsigned n = 0, i;
    ts.join_active = false;
    for (i = 0; i < 2 * CHARS && ts.join[i]; ++i) out[n++] = ts.join[i];
    ts.has_last = true; ts.last_from = ts.join_from; ts.last_id = ts.join_id;
    t_store(out, n, STATUS_RECEIVED, ts.join_id);
    ts.chime_due = CHIME_NEW;
    t_set_notice("NEW MESSAGE");
}
static void take_packet(void)
{
    u8 p[PACKET], total, part, from, id; unsigned i, at;
    memcpy(p, ts.rx_buf, PACKET);
    t_scramble(p);
    if (p[0] != T_MAGIC || p[CHECK_AT] != t_checksum(p)) { ++ts.n_bad; return; }
    ++ts.n_ok;
    t_rx_learn = ts.rx_off;                                   /* N37: this offset works with this radio */
    if (p[3] == 0) {
        ++ts.n_reply_seen;
        if (ts.pending && p[4] == ts.me && p[5] == ts.out_id) {
            ++ts.n_delivered;
            end_send(STATUS_DELIVERED, "DELIVERED"); ts.chime_due = CHIME_DELIVERED;
        }
        return;
    }
    total = p[3] & 0x0F; part = p[3] >> 4;
    if (total == 0 || total > 2 || part >= total) return;
    from = p[1]; id = p[2];
    if (ts.has_last && ts.last_from == from && ts.last_id == id) {
        if (part + 1 == total) schedule_reply(from, id, 0);
        return;
    }
    if (!ts.join_active || ts.join_from != from || ts.join_id != id) {
        ts.join_active = true; ts.join_from = from; ts.join_id = id; ts.join_total = total; ts.join_have = 0;
        memset(ts.join, 0, sizeof(ts.join));
    }
    ts.join_age = 0;
    at = part * CHARS;
    for (i = 0; i < CHARS; ++i) { u8 c = p[4 + i]; ts.join[at + i] = (c == 0 || (c >= 0x20 && c < 0x7F)) ? c : '?'; }
    ts.join_have |= 1 << part;
    if (ts.join_have == (1 << ts.join_total) - 1) {
        finish_join();
        schedule_reply(from, id, total - 1 - part);
    }
}
static void rx_word(void)
{
    u16 word = rf_read(R_FIFO); unsigned i = ts.rx_i;
    if (i < PACKET) ts.rx_buf[i] = word;
    if (i + 1 < PACKET) ts.rx_buf[i + 1] = word >> 8;
    ts.rx_i = i + 2 < PACKET ? i + 2 : PACKET;
}
static void finish_rx(void) { ts.rx_active = false; take_packet(); rx_restart(); }
static void poll_modem(void)
{
    unsigned k; u16 wait = 0;
    for (k = 0; k < 1000; ++k) {
        u16 flags;
        if (!(rf_read(R_STATUS) & 1)) {
            if (!ts.rx_active) return;
            if (wait == 0) { ts.rx_active = false; rx_restart(); return; }
            --wait;
            api_delay_ms(2);
            continue;
        }
        rf_write(R_INT, 0);
        flags = rf_read(R_INT);
        if (flags & IRQ_SYNC) { ++ts.n_sync;
            if (ts.window && ts.sync_ms == 0xFFFF) ts.sync_ms = (u16)((api_uptime_100us() - ts.unkey_at) / 10);
            ts.rx_active = true; ts.rx_i = 0; memset(ts.rx_buf, 0, PACKET); wait = RX_WAIT_STEPS; }
        if ((flags & IRQ_WORD) && ts.rx_active) {
            rx_word();
            wait = RX_WAIT_STEPS;
            if (ts.rx_i >= PACKET) finish_rx();
        }
        if (flags & IRQ_RX_DONE) {
            if (ts.rx_active) { while (ts.rx_i < PACKET) rx_word(); finish_rx(); }
            else rx_restart();
        }
    }
}
static bool countdown(u32 *left, u32 dt) { if (!*left) return false; *left = *left > dt ? *left - dt : 0; return *left == 0; }

static void text_enter(void)
{
    unsigned i; u32 t;
    memset(&ts, 0, sizeof(ts));
    ts.redraw_all = true;
    ts.tap_key = T_NO_KEY;
    ts.hz = api_get_master_freq();
    ts.wide = api_get_master_wide();
    api_set_tx_enabled(false);
    api_set_modulation(0);
    ts.rx_off = t_rx_learn;
    api_set_rx_freq((u32)((int)ts.hz + t_rx_learn));          /* N37: listen at the learned offset */
    api_set_tx_freq(ts.hz);
    api_set_bandwidth(ts.wide);
    api_set_subaudio_rx(0);
    api_set_subaudio_tx(0);
    api_set_sql_level((u8)api_settings_get(0));
    api_enter_rx();
    api_set_holds_rx(false);
    api_set_af_out(false);
    api_set_backlight_hold(true);
    t = api_uptime_100us();
    ts.last_busy = t;
    ts.me = (u8)(t ^ (t >> 8) ^ (t >> 16));
    ts.next_id = (u8)(t >> 3);
    for (i = 0; i < 10; ++i) ts.saved[i] = rf_read(t_saved_regs[i]);
    ts.armed = true;
    modem_rx();
    log_open();
}
static void text_leave(void) { modem_off(); }
/* Packet counters since TEXT opened: sync, good, bad, sent, replies seen, delivered (one byte each, wrapping). */
static u32 text_counters(void)
{
    return ts.n_sync | ((u32)ts.n_ok << 8) | ((u32)ts.n_bad << 16) | ((u32)ts.n_tx << 24);
}
static u32 text_counters2(void)
{
    return ts.n_reply_seen | ((u32)ts.n_delivered << 8) | ((u32)(u8)(s8)(t_rx_learn / 10) << 16) | ((u32)(u8)(s8)(ts.rx_off / 10) << 24);
}
static void text_tick(u32 dt)
{
    bool quiet;
    if (countdown(&ts.tap_left, dt)) { ts.tap_key = T_NO_KEY; ts.redraw_comp = true; }
    if (countdown(&ts.notice_left, dt)) { ts.notice = 0; ts.erase_armed = false; ts.redraw_hint = true; }
    if (ts.join_active) { ts.join_age += dt; if (ts.join_age >= JOIN_100US) ts.join_active = false; }
    if (api_read_rssi() >= BUSY_RAW) {
        ts.last_busy = api_uptime_100us();
        if (ts.window && ts.carr_ms == 0xFFFF) ts.carr_ms = (u16)((ts.last_busy - ts.unkey_at) / 10);
    }
    if (ts.window && api_uptime_100us() - ts.unkey_at > 25000) {   /* N38: 2.5 s later, close and record the window */
        ts.window = false;
        t_rec[t_rec_n++ % 8] = ts.carr_ms | ((u32)ts.sync_ms << 16);   /* kept for the cable diagnostics only */
    }
    poll_modem();
    if (ts.chime_due) {
        static const u16 up[2] = {880, 1320}, down[2] = {1320, 880};
        bool is_new = ts.chime_due == CHIME_NEW;
        ts.chime_due = 0;
        chime(is_new ? up : down, 2);
        return;
    }
    quiet = is_due(ts.last_busy + QUIET_100US);
    if (ts.reply_due) {
        if (is_due(ts.reply_at)) {
            if (quiet) { ts.reply_due = false; send_reply(); }
            else if (is_due(ts.reply_at + REPLY_GIVE_UP_100US)) ts.reply_due = false;
        }
    } else if (ts.pending && is_due(ts.retry_at) && (quiet || is_due(ts.retry_at + RETRY_GIVE_UP_100US))) t_retry();
}
static void type_digit(u8 digit)
{
    const char *set = taps[digit]; unsigned len = strlen(set);
    if (ts.tap_key == digit && ts.comp_len > 0) {
        ts.tap_index = (ts.tap_index + 1) % len;
        ts.comp[ts.comp_len - 1] = set[ts.tap_index];
    } else {
        if (ts.comp_len == MAX_LEN) { t_set_notice("MESSAGE FULL"); return; }
        ts.tap_index = 0;
        ts.comp[ts.comp_len++] = set[0];
    }
    ts.tap_key = digit;
    ts.tap_left = TAP_100US;
    ts.redraw_comp = true;
}
static unsigned quick(u8 i, u8 *out)
{
    u8 r[2 + MAX_LEN]; unsigned n, k; const u8 *text;
    api_nor_read(QUICK_ADDR + (u32)i * QUICK_BYTES, r, sizeof(r));
    if (r[0] == QUICK_MAGIC && r[1] <= MAX_LEN) { text = r + 2; n = r[1]; }
    else { text = (const u8 *)quick_defaults[i]; n = strlen(quick_defaults[i]); }
    for (k = 0; k < n; ++k) out[k] = (text[k] >= 0x20 && text[k] < 0x7F) ? text[k] : '?';
    return n;
}
static void close_list(void) { ts.picking = false; ts.redraw_all = true; }
static void pick_key(u8 id)
{
    u8 text[MAX_LEN]; unsigned n;
    if (id == TK_UP) ts.pick = (ts.pick + QUICK_COUNT - 1) % QUICK_COUNT;
    else if (id == TK_DOWN) ts.pick = (ts.pick + 1) % QUICK_COUNT;
    else if (id <= 9) ts.pick = (id + QUICK_COUNT - 1) % QUICK_COUNT;
    else if (id == TK_MENU) {
        n = quick(ts.pick, text);
        if (n == 0) t_set_notice("THAT ONE IS EMPTY");
        else if (start_send(text, n)) close_list();
        return;
    } else if (id == TK_POUND) {
        ts.comp_len = quick(ts.pick, ts.comp);
        ts.tap_key = T_NO_KEY; ts.tap_left = 0;
        close_list();
        return;
    } else if (id == TK_EXIT || id == TK_VM) { close_list(); return; }
    else return;
    ts.redraw_msgs = true;
    ts.redraw_comp = true;
}
/* Returns true when the app should close. */
static bool text_key(u8 id, u8 kind)
{
    if (id == TK_STAR && kind == KIND_LONG) {
        ts.locked = !ts.locked;
        t_set_notice(ts.locked ? T_LOCKED : "KEYS UNLOCKED");
        return false;
    }
    if (ts.locked) { if (kind == KIND_SINGLE) t_set_notice(T_LOCKED); return false; }
    if (ts.picking && kind == KIND_SINGLE) { pick_key(id); return false; }
    if (kind == KIND_LONG) {
        if (id == TK_EXIT) { modem_off(); return true; }
        if (id == TK_POUND) {
            if (ts.erase_armed) { log_erase(); t_set_notice("MESSAGES ERASED"); }
            else { ts.erase_armed = true; t_set_notice("HOLD # AGAIN TO ERASE ALL"); }
        }
        return false;
    }
    if (kind != KIND_SINGLE) return false;
    ts.erase_armed = false;
    if (id <= 9) type_digit(id);
    else if (id == TK_MENU) {
        if (ts.comp_len == 0) t_set_notice("TYPE A MESSAGE FIRST");
        else {
            u8 text[MAX_LEN]; unsigned n = ts.comp_len;
            memcpy(text, ts.comp, n);
            ts.tap_key = T_NO_KEY; ts.tap_left = 0;
            if (start_send(text, n)) ts.comp_len = 0;
            ts.redraw_comp = true;
        }
    } else if (id == TK_VM) { ts.picking = true; ts.redraw_all = true; }
    else if (id == TK_BAND) {
        char b[16];
        snprintf(b, sizeof(b), "PING %u", ts.pings + 1u);
        if (start_send((const u8 *)b, strlen(b))) ++ts.pings;
    } else if (id == TK_AB) { ts.high_power = !ts.high_power; ts.redraw_all = true; }
    else if (id == TK_UP) { if (ts.scroll + 1 < ts.log_count) { ++ts.scroll; ts.redraw_msgs = true; } }
    else if (id == TK_DOWN) { if (ts.scroll > 0) { --ts.scroll; ts.redraw_msgs = true; } }
    else if (id == TK_EXIT) {
        if (ts.pending) { end_send(STATUS_FAILED, "STOPPED: NOT CONFIRMED"); return false; }
        if (ts.comp_len == 0) { modem_off(); return true; }
        --ts.comp_len;
        ts.tap_key = T_NO_KEY; ts.tap_left = 0;
        ts.redraw_comp = true;
    }
    return false;
}
static void t_text(int x, int y, const void *s, unsigned n, u16 fg, u16 bg, u8 font) { api_draw_text(x, y, s, n, fg, bg, font); }
static void t_texts(int x, int y, const char *s, u16 fg, u16 bg, u8 font) { api_draw_text(x, y, s, strlen(s), fg, bg, font); }
static void draw_title(void)
{
    char b[32], f[16];
    api_fill_rect(0, T_TITLE_Y, T_W, 12, T_HEADER);
    api_fmt_freq(ts.hz, (u8 *)f);
    snprintf(b, sizeof(b), "TEXT %s %s", f, ts.high_power ? "HIGH" : "LOW");   /* N39: power shown again (N38 showed timing) */
    t_texts(4, T_TITLE_Y + 1, b, T_WHITE, T_HEADER, T_F6);
}
static u16 slot_back(u16 back) { return (ts.log_head + LOG_SLOTS - 1 - back) % LOG_SLOTS; }
static unsigned rows_for(u8 len) { return (len + 1 + T_COLS - 1) / T_COLS; }
static void draw_msgs(void)
{
    u16 newest, oldest, back; unsigned rows; int y; u8 h[REC_HEAD];
    api_fill_rect(0, T_MSG_Y, T_W, T_MSG_ROWS * 10, T_BLACK);
    if (ts.log_count == 0) { t_texts(4, T_MSG_Y, "NO MESSAGES YET", T_GRAY, T_BLACK, T_F6); return; }
    newest = ts.scroll; oldest = newest;
    read_head(slot_back(newest), h);
    rows = rows_for(h[3]);
    while (oldest + 1 < ts.log_count) {
        unsigned more;
        read_head(slot_back(oldest + 1), h);
        more = rows_for(h[3]);
        if (rows + more > T_MSG_ROWS) break;
        ++oldest; rows += more;
    }
    y = T_MSG_Y;
    for (back = oldest + 1; back-- > newest;) {
        u16 slot = slot_back(back);
        u8 rec[REC_BYTES], row[T_COLS], mark; u16 color; unsigned col = 1, len, i;
        bool live = ts.pending && slot == ts.out_slot;
        api_nor_read(log_addr(slot), rec, REC_BYTES);
        if (rec[1] == STATUS_RECEIVED) { mark = '<'; color = T_GREEN; }
        else if (rec[1] == STATUS_SENDING && live) { mark = '>'; color = T_GRAY; }
        else if (rec[1] == STATUS_DELIVERED) { mark = '+'; color = T_WHITE; }
        else { mark = '!'; color = T_AMBER; }
        row[0] = mark;
        len = rec[3] < MAX_LEN ? rec[3] : MAX_LEN;
        for (i = 0; i < len; ++i) {
            if (col == T_COLS) { t_text(2, y, row, T_COLS, color, T_BLACK, T_F6); y += 10; col = 0; }
            row[col++] = rec[REC_HEAD + i];
        }
        t_text(2, y, row, col, color, T_BLACK, T_F6);
        y += 10;
    }
}
static void draw_qlist(void)
{
    u8 i;
    api_fill_rect(0, T_KEYS_Y, T_W, QUICK_COUNT * LIST_ROW, T_BLACK);
    for (i = 0; i < QUICK_COUNT; ++i) {
        u8 q[MAX_LEN]; char l[40]; unsigned n = quick(i, q);
        int y = T_KEYS_Y + i * LIST_ROW; u16 bg = i == ts.pick ? T_HILITE : T_BLACK;
        if (n > LIST_CHARS) n = LIST_CHARS;
        snprintf(l, sizeof(l), "%u ", (i + 1u) % QUICK_COUNT);
        memcpy(l + 2, q, n); l[2 + n] = 0;
        api_fill_rect(0, y, T_W, LIST_ROW, bg);
        t_texts(2, y, l, T_WHITE, bg, T_F5);
    }
}
static void draw_comp(void)
{
    u8 q[MAX_LEN]; const u8 *t; unsigned n, row;
    api_fill_rect(0, T_COMP_Y, T_W, T_HINT_Y - T_COMP_Y, T_BLACK);
    api_fill_rect(0, T_COMP_Y, T_W, 1, T_HEADER);
    if (ts.picking) { n = quick(ts.pick, q); t = q; } else { t = ts.comp; n = ts.comp_len; }
    for (row = 0; row * T_COLS < n; ++row) {
        unsigned k = n - row * T_COLS; if (k > T_COLS) k = T_COLS;
        t_text(2, T_COMP_Y + 2 + row * 10, t + row * T_COLS, k, T_WHITE, T_BLACK, T_F6);
    }
    if (ts.picking) return;
    if (ts.tap_key != T_NO_KEY && n > 0)
        t_text(2 + ((n - 1) % T_COLS) * 6, T_COMP_Y + 2 + ((n - 1) / T_COLS) * 10, ts.comp + n - 1, 1, T_WHITE, T_HILITE, T_F6);
    if (n < MAX_LEN) t_texts(2 + (n % T_COLS) * 6, T_COMP_Y + 2 + (n / T_COLS) * 10, "_", T_GRAY, T_BLACK, T_F6);
}
static void draw_hint(void)
{
    api_fill_rect(0, T_HINT_Y, T_W, 9, T_BLACK);
    if (ts.notice) t_texts(2, T_HINT_Y, ts.notice, T_AMBER, T_BLACK, T_F5);
    else if (ts.locked) t_texts(2, T_HINT_Y, T_LOCKED, T_AMBER, T_BLACK, T_F5);
    else if (ts.pending) {
        char l[32];
        snprintf(l, sizeof(l), "TRY %u OF %u  EXIT STOPS", ts.tries, MAX_TRIES);
        t_texts(2, T_HINT_Y, l, T_AMBER, T_BLACK, T_F5);
    } else if (ts.picking) t_texts(2, T_HINT_Y, "MENU SEND #EDIT EXIT BACK", T_GRAY, T_BLACK, T_F5);
    else t_texts(2, T_HINT_Y, "MENU SEND EXIT DEL V/M QK", T_GRAY, T_BLACK, T_F5);
}
static void text_draw(void)
{
    if (ts.redraw_all) {
        api_fill_rect(0, 0, T_W, 128, T_BLACK);
        draw_title();
        ts.redraw_msgs = ts.redraw_comp = ts.redraw_hint = true;
        ts.redraw_all = false;
    }
    if (ts.redraw_msgs) { if (ts.picking) draw_qlist(); else draw_msgs(); ts.redraw_msgs = false; }
    if (ts.redraw_comp) { draw_comp(); ts.redraw_comp = false; }
    if (ts.redraw_hint) { draw_hint(); ts.redraw_hint = false; }
}
