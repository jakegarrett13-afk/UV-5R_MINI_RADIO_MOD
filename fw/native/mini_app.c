/* Native Mini port, milestone N38: TEXT tail 100 -> 20 ms; reply-window timing measured (TEXT title, diag).
 * N37: TEXT hunts the other radio's frequency offset (receive offset per try, the
 * working one remembered).
 * N36: battery gauge (PB3 / stock thresholds from NOR 0xF200), top right.
 * N35: TEXT receive settings final (N21 set + the 5RM RX audio filter 0x2B 0x0300);
 * the profile test UI removed.
 * N34 (TEXT test build, round 3: power-on values of the Mini-only init registers).
 * N33 (TEXT test build, round 2: the 5RM RX filter 0x2B in every profile).
 * N32 (TEXT test build): receive profiles P0-P3 on the side key in TEXT, with replies
 * detected / sent and the reply signal strength in the TEXT title.
 * N31: PB8 held LOW while transmitting (N30 held it high: no mic audio).
 * N30: BAND SWEEP removed (user); PB8 audio power driven as stock (on with the speaker and for the whole
 * transmission; was floating: key-up/release pops); TEXT also uses the 5RM RX AGC values 0x13/0x49 and 0x2A.
 * N29: harp picks slowed to 120 ms, each plucked note fading.
 * N28: start-up chime is a harp (four rising strums, fading top notes); boot screen 3 s.
 * N27: SAVE AS CH (VFO -> a named channel), DELETE CH (with confirm), start-up chime
 * (BOOT SND).
 * N26: SETTINGS groups as the 5RM settings app (mini_settings.h): per-channel W/N,
 * power, RX/TX tone, shift/offset, busy lock, scan add; TX tone, roger, STE, repeater tail, mic gain, TX inhibit and
 * TX lock, scan mode, channel display, light time, auto lock, key functions, boot screen, reset. Manual tuning uses the
 * quiet hop (no pop).
 * N25: quiet, responsive scan (mini_rf_hop, scan list read once, speaker shut while
 * hopping); RPT FINDER and SEARCH/QT SCAN removed (SEARCH and SCANQT stay as separate apps).
 * N24: screen light off after 10 s without a key (apps stay lit).
 * N23: hold # = scan (5RM app/scan.rs: channels in the current MODE, or the VFO band);
 * BAND SWEEP scans first and then lists what it heard; FOX HUNT measures the noise floor off frequency; RPT FINDER
 * returns to the output with a fast retune; SPECTRUM removed (user).
 * N22: the UV-5RM custom firmware apps FOX HUNT, BAND SWEEP, RPT FINDER, SEARCH, SCANQT,
 * SEARCH/QT SCAN and SPECTRUM (generic app layer, app_*.h), next to TEXT.
 * N21: TEXT runs with the 5RM's baseband receive settings (AFC, IF, noise/level registers;
 * Mini values saved and restored on exit). FREQ ERR on this pair measured -430 Hz.
 * N20: an update never hands over while transmitting (TX blocked, waits up to 8 s, else
 * refuses); every app transmission capped at 10 s.
 * N19: FREQ ERR measurement (hardware frequency counter) for the TEXT investigation; PTT
 * sequence back to stock (N18's was louder); TEXT receive filter back to stock (0x0300 made it worse).
 * N18: no AF-DAC power-down on beeps (crackle fixed for beeps).
 * N17: channel name large in the middle with the frequency top right; BEEP styles 1-4
 * (listening test for the speaker crackle).
 * N16: the PC channel app's own programming protocol (UV-5RM custom firmware CPS session at
 * 115200, port alternating with the 9600 updater), click-free beeps, TEXT packet counters in the diagnostics.
 * N15: key beeps + TEXT chime (stock beep path), TX deviation init, UV Channels page link
 * (NOR program/erase/reload for the channel table and quick messages).
 * N14: TEXT app (5RM protocol), app launcher, MODE bands, A/B per side, PTT-A = TEXT,
 * side key click = power / hold = flashlight. N13: custom-firmware UI port (Blue Angel, electric-blue menus), TX on
 * 136-174/400-480 MHz with TOT, LOW/HIGH from this radio's calibration; settings saved to a dedicated NOR sector (0xFE000), VFO receive tones, CTCSS/DCS receive, SQL 0 = open (calibration read-only from NOR).
 * Board signals from stock V0.04; do not substitute the 5RM ARM pin map.
 * The independent SDK uart_update task remains available throughout this loop.
 */
#include "system/includes.h"
#include "asm/gpio.h"
#include "asm/timer.h"
#include "asm/power/p33.h"
#include "asm/spi.h"
#include "asm/adc_api.h"
#include "mini_rf.h"
#include <string.h>

#define LCD_CLK IO_PORTB_00
#define LCD_MOSI IO_PORTB_01
#define LCD_CS IO_PORTB_05
#define LCD_DC IO_PORT_PR_01
#define LCD_RST IO_PORTB_04
#define LCD_LIGHT IO_PORTB_07
#define RF_CS IO_PORTC_01
#define RF_CLK IO_PORTA_10
#define RF_DATA IO_PORTC_00
#define WHITE 0xffff
#define CYAN 0x07ff
#define DARK 0x0843
#define GOLD 0xfd20

/* Board SDK SPI1 declaration is disabled; this app owns LCD SPI1 port A. */
const struct spi_platform_data spi1_p_data = {
    .port = 'A', .mode = SPI_MODE_BIDIR_1BIT,
    .clk = 4000000, .role = SPI_ROLE_MASTER,
};
static volatile u32 native_ticks, native_chip, native_key = 0xff, spi_errors;
static struct mini_rf radio;
static volatile u32 rx_snapshot[8];
static bool tuned;
static volatile u32 nor_id, nor_errors, nor_request_pending, nor_request_addr, nor_request_len;
static u32 set_saves_status(void);
static volatile bool update_pending;   /* N20: an update is handing over: no transmitting */
static u32 tx_status_word(void);
static u32 text_diag(void);
static u32 link_diag(void);
static u32 fe_diag(void);
static u32 text_regs_diag(void);
static u32 text_diag2(void);
static u32 text_window(unsigned i);
static u8 bat_level, bat_th[6];           /* N36 battery (defined with the battery code) */
static u16 bat_raw, bat_avg16;

/* Read-only telemetry, sent only on a diagnostic request at idle UART speed. */
void mini_diag_reply(void)
{
    extern bool uart_update_cmd(u8, u8 *, u32);
    u32 d[45];
    unsigned i;
    d[0] = 0x334e494d; d[1] = 38; d[2] = native_ticks; d[3] = native_chip;
    d[4] = spi_errors;
    d[5] = p33_rx_1byte(R3_PR_OUT) | (p33_rx_1byte(R3_PR_DIR) << 8)
           | (p33_rx_1byte(R3_PR_DIE) << 16);
    d[6] = JL_PORTB->OUT; d[7] = JL_PORTB->DIR; d[8] = JL_PORTB->DIE;
    d[9] = JL_SPI1->CON; d[10] = JL_IOMAP->CON1;
    d[11] = native_key | (!gpio_read(IO_PORTA_04) << 8) | (!gpio_read(IO_PORTA_12) << 9);
    for (i = 0; i < 8; ++i) d[12+i] = rx_snapshot[i];
    d[20] = nor_id; d[21] = nor_errors;
    d[22] = radio.squelch_reg | ((u32)radio.squelch << 16) | ((u32)radio.cal_valid << 24);
    d[23] = radio.word | ((u32)radio.cal.xtal << 26);
    /* N10: kind bits 0-1, matched bit 2, value bits 3-15 (CTCSS tenths up to 254.1 Hz need 12 bits). */
    d[24] = (radio.rx_tone.kind & 3) | ((u32)radio.tone_matched << 2) | (((u32)radio.rx_tone.value & 0x1fff) << 3)
            | ((u32)radio.status << 16);
    d[25] = set_saves_status();
    d[26] = tx_status_word();
    d[27] = text_diag();
    d[28] = text_diag2();
    d[29] = link_diag();
    d[30] = fe_diag();
    d[31] = text_regs_diag();
    /* N34: the RF chip's power-on values of 0x17 0x25 0x3A 0x42 0x4C 0x4D 0x4E (two per word, first in the low half) */
    for (i = 0; i < 4; ++i)
        d[32 + i] = radio.reset_vals[2 * i] | (i < 3 ? (u32)radio.reset_vals[2 * i + 1] << 16 : 0);
    d[36] = bat_raw | ((u32)(bat_avg16 / 16u) << 8) | ((u32)bat_level << 16) | ((u32)bat_th[0] << 24);   /* N36 battery */
    for (i = 0; i < 8; ++i) d[37 + i] = text_window(i);                   /* N38 TEXT reply windows, oldest first */
    uart_update_cmd(0x70, (u8 *)d, sizeof(d));
}

/* BR23 gpio_set_output_value() silently ignores PR0..PR4 (IO 56..60).
 * Stock LCD D/C uses RTC/PMU R3_PR_OUT and R3_PR_DIR, not normal GPIO.
 * Match stock 0x01e5739e: change only PR1, preserving every other pin. */
static void lcd_dc(int value)
{
    u8 out = p33_rx_1byte(R3_PR_OUT) & ~BIT(1);
    p33_tx_1byte(R3_PR_OUT, out | (value ? BIT(1) : 0));
    p33_and_1byte(R3_PR_DIR, (u8)~BIT(1));
}
static void output(unsigned p, int v)
{
    if (p == LCD_DC) { lcd_dc(v); return; }
    gpio_set_pull_up(p, 0);
    gpio_set_pull_down(p, 0);
    gpio_set_die(p, 1);
    gpio_direction_output(p, v);
}
static void input(unsigned p)
{
    gpio_set_direction(p, 1);
    gpio_set_pull_down(p, 0);
    gpio_set_pull_up(p, 1);
    gpio_set_die(p, 1);
}
/* SDK delay() counts cycles, not microseconds. Slow bus edges tolerate
 * interruptions; 100 iterations conservatively exceed the RF minimum. */
static void bus_wait(void) { delay(100); }
static void put(unsigned p, int v)
{
    if (p == LCD_DC) lcd_dc(v);
    else gpio_set_output_value(p, v);
}

/* Stock uses SPI1 on PB0/PB1. Use the same hardware peripheral and mux rather
 * than relying on reset-state pin ownership for software-generated clocks. */
static void lcd_byte(u8 b)
{
    if (spi_send_byte(SPI1, b)) ++spi_errors;
}
/* External settings NOR shares LCD SPI1; only main-loop code owns this bus.
 * Supported operations: JEDEC identification (9F), data read (03), and - only inside the one 4 KB sector at
 * SET_SECTOR (blank on this radio, outside channels/settings/calibration/font) - write enable (06), page program
 * (02), sector erase (20) and status read (05). No status-register write, block/chip erase, or other address. */
static void nor_identify(void)
{
    unsigned i; int err;
    put(LCD_CS, 1); put(IO_PORTA_03, 0);
    if (spi_send_byte(SPI1, 0x9f)) ++nor_errors;
    for (i = 0; i < 3; ++i) {
        u8 b = spi_recv_byte(SPI1, &err);
        if (err) ++nor_errors;
        nor_id = (nor_id << 8) | b;
    }
    put(IO_PORTA_03, 1);
}
/* Called by UART decoder: copy a bounded request only, never use SPI here. */
void mini_nor_request(const u8 *p)
{
    u32 addr = p[0] | ((u32)p[1]<<8) | ((u32)p[2]<<16) | ((u32)p[3]<<24);
    unsigned n = p[4] | ((unsigned)p[5]<<8);
    if (nor_request_pending || !n || n > 512 || addr > 0x100000 - n) return;
    nor_request_addr = addr; nor_request_len = n;
    __asm__ volatile("" ::: "memory");
    nor_request_pending = 1;
}
/* Flash READ (03) only, main loop only (SPI1 is shared with the LCD). True if no SPI error. */
static bool nor_read(u32 addr, u8 *buf, unsigned n)
{
    u32 previous_errors = nor_errors;
    unsigned i; int err;
    put(LCD_CS, 1); put(IO_PORTA_03, 0);
    if (spi_send_byte(SPI1, 3)) ++nor_errors;
    if (spi_send_byte(SPI1, addr >> 16)) ++nor_errors;
    if (spi_send_byte(SPI1, addr >> 8)) ++nor_errors;
    if (spi_send_byte(SPI1, addr)) ++nor_errors;
    for (i = 0; i < n; ++i) {
        buf[i] = spi_recv_byte(SPI1, &err);
        if (err) ++nor_errors;
    }
    put(IO_PORTA_03, 1);
    return nor_errors == previous_errors;
}

/* ---- Native settings store: append-only 32-byte records in one dedicated sector. ---- */
#define SET_SECTOR 0xFE000u
#define SET_SLOT 32u
#define SET_SLOTS (0x1000u / SET_SLOT)
static const u8 set_magic[4] = {'M', 'S', '1', 0};
static unsigned set_next = SET_SLOTS;   /* first blank slot; SET_SLOTS = full (erase before next write) */
static bool set_locked;                 /* sector holds foreign data or the store failed: never write */
static u32 set_saves, set_fail;
static u32 set_saves_status(void)
{
    return (set_saves & 0xffff) | ((set_fail & 0xff) << 16) | ((u32)(set_next > 127 ? 127 : set_next) << 24)
           | ((u32)set_locked << 31);
}
static bool set_in_sector(u32 addr, unsigned n)
{
    return addr >= SET_SECTOR && n <= 0x1000 && addr + n <= SET_SECTOR + 0x1000;
}
/* App data (N14): TEXT quick messages 0xF7000, message log 0xF8000-0xFBFFF. Read blank on this radio before use. */
#define APP_DATA_LO 0xF7000u
#define APP_DATA_HI 0xFC000u
static bool app_in_region(u32 addr, unsigned n)
{
    return addr >= APP_DATA_LO && n <= APP_DATA_HI - APP_DATA_LO && addr + n <= APP_DATA_HI;
}
/* UV Channels page (N15): the channel table + band records (0x0000-0x7FFF, CPS address = NOR address) and the
 * TEXT quick messages (0xF7000-0xF7FFF). Nothing else is writable from the cable. */
static bool cps_in_region(u32 addr, unsigned n)
{
    return (n <= 0x8000 && addr + n <= 0x8000) || (addr >= 0xF7000 && n <= 0x1000 && addr + n <= 0xF8000);
}
static void nor_select(void) { put(LCD_CS, 1); put(IO_PORTA_03, 0); }
static void nor_deselect(void) { put(IO_PORTA_03, 1); }
static u8 nor_status(void)
{
    int err; u8 sr;
    nor_select();
    if (spi_send_byte(SPI1, 0x05)) ++nor_errors;
    sr = spi_recv_byte(SPI1, &err);
    if (err) ++nor_errors;
    nor_deselect();
    return sr;
}
/* Write enable, confirmed by WEL (status bit 1). */
static bool nor_wren(void)
{
    nor_select();
    if (spi_send_byte(SPI1, 0x06)) ++nor_errors;
    nor_deselect();
    return (nor_status() & 3) == 2;
}
/* Wait for WIP (bit 0) to clear; ticks of 10 ms. */
static bool nor_wait(unsigned ticks)
{
    while (ticks--) {
        if (!(nor_status() & 1)) return true;
        os_time_dly(1);
    }
    return !(nor_status() & 1);
}
static void nor_addr_cmd(u8 cmd, u32 addr)
{
    nor_select();
    if (spi_send_byte(SPI1, cmd)) ++nor_errors;
    if (spi_send_byte(SPI1, addr >> 16)) ++nor_errors;
    if (spi_send_byte(SPI1, addr >> 8)) ++nor_errors;
    if (spi_send_byte(SPI1, addr)) ++nor_errors;
}
static bool nor_erase_settings(void)
{
    if (!nor_wren()) return false;
    nor_addr_cmd(0x20, SET_SECTOR);
    nor_deselect();
    return nor_wait(100);              /* 4 KB erase: typ 50 ms, max 400 ms */
}
static bool nor_program(u32 addr, const u8 *b, unsigned n)
{
    if (!(set_in_sector(addr, n) || app_in_region(addr, n) || cps_in_region(addr, n)) || (addr & 0xff) + n > 256) return false;
    if (!nor_wren()) return false;
    nor_addr_cmd(0x02, addr);
    while (n--) if (spi_send_byte(SPI1, *b++)) ++nor_errors;
    nor_deselect();
    return nor_wait(10);
}
/* Sector erase inside the app data region or the CPS regions only. */
static bool nor_erase_app(u32 addr)
{
    if ((addr & 0xfff) || !(app_in_region(addr, 0x1000) || cps_in_region(addr, 0x1000))) return false;
    if (!nor_wren()) return false;
    nor_addr_cmd(0x20, addr);
    nor_deselect();
    return nor_wait(100);
}
static bool all_ff(const u8 *b, unsigned n) { while (n--) if (*b++ != 0xff) return false; return true; }
static u16 crc16_ccitt(const u8 *b, unsigned n)
{
    u16 c = 0; unsigned i;
    while (n--) { c ^= (u16)*b++ << 8; for (i = 0; i < 8; ++i) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1; }
    return c;
}
static bool set_record_ok(const u8 *r)
{
    return !memcmp(r, set_magic, 4) && crc16_ccitt(r, 30) == (r[30] | (r[31] << 8));
}
/* Scan the sector: copies the newest valid record to out (returns false if none) and finds the first blank slot. */
static bool settings_scan(u8 *out)
{
    u8 r[SET_SLOT]; unsigned i; bool found = false;
    set_next = SET_SLOTS;
    if (!nor_read(SET_SECTOR, r, SET_SLOT)) { set_locked = true; return false; }
    if (!all_ff(r, SET_SLOT) && memcmp(r, set_magic, 4)) { set_locked = true; return false; }   /* not ours */
    for (i = 0; i < SET_SLOTS; ++i) {
        if (!nor_read(SET_SECTOR + i * SET_SLOT, r, SET_SLOT)) { set_locked = true; return found; }
        if (all_ff(r, SET_SLOT)) { set_next = i; break; }
        if (set_record_ok(r)) { memcpy(out, r, SET_SLOT); found = true; }
    }
    return found;
}
static bool settings_write(const u8 *rec)
{
    u8 r[SET_SLOT]; unsigned i;
    if (set_locked) return false;
    if (set_next >= SET_SLOTS) {
        /* Full: erase only if slot 0 still carries our magic, then confirm the whole sector is blank. */
        if (!nor_read(SET_SECTOR, r, SET_SLOT) || memcmp(r, set_magic, 4)) { set_locked = true; return false; }
        if (!nor_erase_settings()) { ++set_fail; set_locked = true; return false; }
        for (i = 0; i < SET_SLOTS; ++i)
            if (!nor_read(SET_SECTOR + i * SET_SLOT, r, SET_SLOT) || !all_ff(r, SET_SLOT)) {
                ++set_fail; set_locked = true; return false;
            }
        set_next = 0;
    }
    i = set_next++;
    if (nor_program(SET_SECTOR + i * SET_SLOT, rec, SET_SLOT) && nor_read(SET_SECTOR + i * SET_SLOT, r, SET_SLOT)
        && !memcmp(r, rec, SET_SLOT)) { ++set_saves; return true; }
    if (++set_fail >= 8) set_locked = true;
    return false;
}

static void nor_service(void)
{
    extern bool uart_update_cmd(u8, u8 *, u32);
    static u8 response[519];
    u32 addr, previous_errors;
    unsigned n;
    if (!nor_request_pending) return;
    addr = nor_request_addr; n = nor_request_len; previous_errors = nor_errors;
    nor_read(addr, response + 7, n);
    response[0] = addr; response[1] = addr >> 8;
    response[2] = addr >> 16; response[3] = addr >> 24;
    response[4] = n; response[5] = n >> 8;
    response[6] = nor_errors != previous_errors;
    uart_update_cmd(0x71, response, n + 7);
    nor_request_pending = 0;
}
static void command(u8 c, const u8 *d, unsigned n)
{
    put(LCD_CS, 0); put(LCD_DC, 0); lcd_byte(c);
    put(LCD_DC, 1);
    while (n--) lcd_byte(*d++);
    put(LCD_CS, 1);
}
static void window(unsigned x, unsigned y, unsigned w, unsigned h)
{
    u8 a[4] = {0, x + 32, 0, x + w + 31};
    u8 b[4] = {0, y, 0, y + h - 1};
    command(0x2a, a, 4); command(0x2b, b, 4);
    command(0x2c, 0, 0); put(LCD_CS, 0);
}
static void rect(unsigned x, unsigned y, unsigned w, unsigned h, u16 color)
{
    unsigned row, col;
    if (!w || !h || x + w > 128 || y + h > 128) return;
    /* Row-sized transactions let a higher-priority downloader preempt us. */
    for (row = 0; row < h; ++row) {
        window(x, y + row, w, 1);
        for (col = 0; col < w; ++col) { lcd_byte(color >> 8); lcd_byte(color); }
        put(LCD_CS, 1);
        if ((row & 15) == 15) os_time_dly(1);
    }
}

static void lcd_init(void)
{
    static const u8 sequence[] = {
      0xb1,3,5,0x3a,0x3a, 0xb2,3,5,0x3a,0x3a,
      0xb3,6,5,0x3a,0x3a,5,0x3a,0x3a, 0xb4,1,3,
      0xc0,3,0x62,2,4, 0xc1,1,0xc0, 0xc2,2,0x0d,0,
      0xc3,2,0x8d,0x6a, 0xc4,2,0x8d,0xee, 0xc5,1,0x12,
      0xe0,16,3,0x1b,0x12,0x11,0x3f,0x3a,0x32,0x34,0x2f,0x2b,0x30,0x3a,0,1,2,5,
      0xe1,16,3,0x1b,0x12,0x11,0x32,0x2f,0x2a,0x2f,0x2e,0x2c,0x35,0x3f,0,0,1,5,
      0xfc,1,0x8c, 0x36,1,0xa8, 0x3a,1,5
    };
    unsigned i = 0;
    /* Stock disables the RTC low-frequency oscillator before using PR0/PR1
     * as pads (0x01e48a7a writes zero to 0x80a5). Release that pad function. */
    p33_tx_1byte(R3_OSL_CON, 0);
    output(LCD_LIGHT, 0); output(LCD_CLK, 0); output(LCD_MOSI, 0);
    output(LCD_CS, 1); output(LCD_DC, 1); output(LCD_RST, 1);
    /* Stock explicitly enables PR1's digital pad before writing OUT/DIR. */
    p33_or_1byte(R3_PR_DIE, BIT(1));
    spi_open(SPI1);
    os_time_dly(2); put(LCD_RST, 0); delay_2ms(1);
    put(LCD_RST, 1); os_time_dly(12);
    command(0x11, 0, 0); os_time_dly(24);
    while (i < sizeof(sequence)) {
        u8 c = sequence[i++], n = sequence[i++];
        command(c, sequence + i, n); i += n;
    }
    rect(0, 0, 128, 128, 0);
    command(0x29, 0, 0); put(LCD_LIGHT, 1);
}
static void rf_byte(u8 b)
{
    unsigned i;
    for (i = 0; i < 8; ++i) {
        put(RF_CLK, 0); put(RF_DATA, !!(b & 0x80)); bus_wait();
        put(RF_CLK, 1); bus_wait(); b <<= 1;
    }
}
static u16 rf_read(u8 reg)
{
    unsigned i; u16 v = 0;
    put(RF_CS, 0); bus_wait(); rf_byte(reg | 0x80);
    put(RF_CLK, 0); input(RF_DATA); bus_wait();
    for (i = 0; i < 16; ++i) {
        put(RF_CLK, 1); bus_wait();
        v = (v << 1) | !!gpio_read(RF_DATA);
        put(RF_CLK, 0); bus_wait();
    }
    put(RF_CS, 1); output(RF_DATA, 0); return v;
}
static void rf_write(u8 reg, u16 value)
{
    put(RF_CS, 0); bus_wait(); rf_byte(reg & 0x7f);
    rf_byte(value >> 8); rf_byte(value);
    put(RF_CS, 1); bus_wait(); put(RF_CLK, 0);
}
/* Stock fn_01e41350: PC7 high for VHF, low for UHF in RX mode. */
static void rf_band(bool uhf) { put(IO_PORTC_07, !uhf); }
/* N30: PB8 is the audio power, shared like stock fn_01e400d4 (high while a sound source needs it); PB9 enables the
 * speaker. N29 and earlier never drove PB8 (left floating), so the RF burst at key-up and release popped the speaker.
 * On: PB8 then PB9; off: PB9 then PB8.
 * N31: during a transmission PB8 is held LOW. N30 held it high (misreading stock mode 9/10, which belongs to the DTMF /
 * PTT-ID sequencer fn_01e4dcc0, not to plain TX), and that silenced the microphone (roger tones, made inside the RF
 * chip, still went out). Stock key-up first turns receive audio off (mode 6), so PB8 is low while keyed. */
static u8 amp_users;
#define AMP_SPEAKER 1
static void amp_power(void) { gpio_set_output_value(IO_PORTB_08, amp_users != 0); }
static void rf_speaker(bool on)
{
    if (on) { amp_users |= AMP_SPEAKER; amp_power(); put(IO_PORTB_09, 1); }
    else { put(IO_PORTB_09, 0); amp_users &= (u8)~AMP_SPEAKER; amp_power(); }
}
static void rf_tx_audio(bool on)
{
    if (on) { put(IO_PORTB_09, 0); amp_users = 0; amp_power(); }   /* keyed: speaker off, audio power driven low */
}
static void rf_settle(unsigned ms) { os_time_dly((ms + 9) / 10); }
/* Stock 0x01E418AE -> 0x01E5739E(0, on): PR0 output, high only while transmitting. Other PR bits preserved. */
static void rf_tx_aux(bool on)
{
    u8 out = p33_rx_1byte(R3_PR_OUT) & ~BIT(0);
    p33_tx_1byte(R3_PR_OUT, out | (on ? BIT(0) : 0));
    p33_and_1byte(R3_PR_DIR, (u8)~BIT(0));
}
static const struct mini_rf_io radio_io = {rf_read, rf_write, rf_band, rf_speaker, rf_settle, rf_tx_aux, rf_tx_audio};

static void rx_status(void)
{
    mini_rf_poll(&radio);
    rx_snapshot[0] = radio.frequency_hz;
    rx_snapshot[1] = radio.frequency_readback;
    rx_snapshot[2] = radio.state;
    rx_snapshot[3] = radio.gpio;
    rx_snapshot[4] = radio.rssi;
    rx_snapshot[5] = radio.monitor;
    rx_snapshot[6] = radio.audible;
    rx_snapshot[7] = tuned;
}
static struct mini_rf_tone want_tone;   /* receive tone for the next tune: channel tone, or none in VFO */
static bool cur_narrow(void);
static bool in_app(void);
/* N26: while receive is already running, a re-tune is the quiet scan hop (no REG 0x30 off, no pop); the full
 * mini_rf_tune runs after TX, at power-up, after an app (frequency_hz is 0 then) and for AM / wide-range apps. */
static bool tune(u32 hz)
{
    if (!in_app()) radio.narrow = cur_narrow();
    if (radio.frequency_hz && !radio.transmitting && !radio.am_applied && !radio.wide_range && mini_rf_rx_range(&radio, hz)) {
        struct mini_rf_tone t = want_tone;
        if (t.kind > MINI_TONE_DCS_I) t.kind = MINI_TONE_NONE;
        radio.rx_tone = t;
        tuned = mini_rf_hop(&radio, hz);
        if (tuned) { rx_status(); return true; }
    }
    mini_rf_set_rx_tone(&radio, want_tone);
    tuned = mini_rf_tune(&radio, hz);
    rx_status();
    return tuned;
}

static const char *notice = "";
static bool scanning, scan_up = true;   /* N23: hold # scan running, direction */
static u8 scan_phase;
static u8 last_key = 0xff;
static u32 vfo_hz[2] = {146520000, 446000000};   /* VFO A / B, RAM only */
static unsigned vfo_sel;
/* Three octal digits: the SDK's snprintf has no %o (N10 showed only "D"). */
static void oct3(u16 v, char *o) { o[0] = '0' + ((v >> 6) & 7); o[1] = '0' + ((v >> 3) & 7); o[2] = '0' + (v & 7); o[3] = 0; }
static unsigned vfo_tone[2];         /* per VFO: index into the receive tone list (tone_at) */
static char entry[8];           /* direct entry: 6 digits (VFO, XXX.XXX MHz) or 3 digits (channel) */
static unsigned entry_len;
static char note_buf[24];      /* notices that carry a number */

/* Stored channels: CHIRP UV5RMini layout, CPS address == NOR address (checked on this radio).
 * 999 x 32 bytes at 0x0000: lbcd rx[4], lbcd tx[4], ul16 rxtone, ul16 txtone, scode, pttid, power,
 * flags (wide, sqmode, bcl, scan, fhss), 4 unknown, char name[12]. Read-only. */
#define CHANNELS 1000          /* N15: as the UV Channels page (slot 1000 at 0x7CE0 was blank) */
struct channel { u32 rx_hz, tx_hz; u16 rxtone; struct mini_rf_tone tone, txtone; bool tone_bad, scan_add, narrow, busy; u8 power; u16 bands; char name[13]; };
static bool mr_mode;
static unsigned mr_index;
static struct channel ch;
static struct channel ch_other;        /* the inactive side's channel, for display */
static bool side_mr[2];                /* per side: channel mode */
static u16 side_ch[2];
static u8 chan_mode;                   /* MODE: 0 = ALL, 1..16 = band (custom firmware app/modes.rs) */
static bool lbcd_hz(const u8 *b, u32 *hz)
{
    u32 v = 0; int i;
    for (i = 3; i >= 0; --i) {
        if ((b[i] >> 4) > 9 || (b[i] & 15) > 9) return false;
        v = v * 100 + (b[i] >> 4) * 10 + (b[i] & 15);
    }
    if (!v) return false;
    *hz = v * 10;
    return true;
}
static bool rx_allowed(u32 hz)
{
    return (hz >= 136000000 && hz <= 174000000) || (hz >= 400000000 && hz <= 520000000);
}
static bool channel_load(unsigned n, struct channel *c)
{
    u8 r[32]; unsigned i;
    if (n >= CHANNELS || !nor_read(n * 32, r, 32)) return false;
    if (!lbcd_hz(r, &c->rx_hz) || !rx_allowed(c->rx_hz)) return false;
    if (!lbcd_hz(r + 4, &c->tx_hz)) c->tx_hz = 0;
    c->rxtone = r[8] | (r[9] << 8);
    c->power = r[14] ? 1 : 0;         /* CHIRP lowpower: 0 HIGH, else LOW */
    c->txtone.kind = MINI_TONE_NONE; c->txtone.value = 0;
    c->bands = r[17] | (r[18] << 8);  /* custom firmware: band bits in bytes 17-18 */
    c->scan_add = (r[15] & 0x04) != 0; /* N23: flash_map.rs scan_add */
    c->narrow = (r[15] & 0x40) != 0;   /* N26: flash_map.rs narrow, busy lock; TX tone at bytes 10-11 */
    c->busy = (r[15] & 0x08) != 0;
    if (!mini_rf_tone_from_channel(r[10] | (r[11] << 8), &c->txtone)) c->txtone.kind = MINI_TONE_NONE;
    c->tone_bad = !mini_rf_tone_from_channel(c->rxtone, &c->tone);   /* bad field -> carrier squelch */
    for (i = 0; i < 12; ++i) {
        char k = r[20 + i];
        if (k >= 'a' && k <= 'z') k -= 32;
        c->name[i] = (k >= 32 && k < 127) ? k : ' ';
    }
    c->name[12] = 0;
    for (i = 12; i && c->name[i - 1] == ' '; --i) c->name[i - 1] = 0;
    return true;
}
/* Move to the next/previous stored channel (dir +1/-1); from_start begins the search at the top. */
static bool tune(u32 hz);
static void set_notice(const char *s);
/* Tune the loaded channel with its receive tone. */
static void tune_channel(void)
{
    want_tone = ch.tone;
    tune(ch.rx_hz);
    if (ch.tone_bad) set_notice("BAD CH TONE");
}
static bool in_mode(const struct channel *c) { return !chan_mode || (c->bands & (1u << (chan_mode - 1))); }
static bool channel_step(int dir, bool from_start)
{
    unsigned k, idx;
    struct channel c;
    for (k = from_start ? 0 : 1; k <= CHANNELS; ++k) {
        idx = from_start ? (k % CHANNELS) : (mr_index + CHANNELS + dir * (int)k) % CHANNELS;
        if (channel_load(idx, &c) && in_mode(&c)) {
            ch = c;
            mr_index = idx;
            tune_channel();
            return true;
        }
    }
    return false;
}
/* ---- TX (N13) ----
 * Plain FM on 136-174 / 400-480 MHz while PTT is held; time-out from the TOT setting (custom firmware: level x 15 s,
 * default 120 s, 0 = off). Power: VFO = menu PWR, channel = its own power byte. LOW = stock index 1 (NOR 0xF080 table,
 * gain 0xD7), HIGH = index 0 (0xF000, gain 0xFF). RF chip ID and REG 0x30 are checked every loop while keyed. */
extern unsigned long jiffies_msec(void);
static u8 pa_low_uhf[16], pa_high_uhf[16], pa_low_vhf[16], pa_high_vhf[16], mic_cal;
static bool pa_loaded, tx_high, tx_hold, tx_high_now;    /* tx_hold: refused or timed out; wait for PTT release */
static u32 tx_start_ms, tx_rx_hz, tx_count, tx_aborts;
static u8 tx_last_target;
static const char *tx_notice;
static u32 tx_status_word(void)
{
    return radio.transmitting | ((u32)tx_high << 1) | ((u32)pa_loaded << 2) | ((u32)tx_hold << 3)
           | ((tx_count & 0xff) << 4) | ((tx_aborts & 0xf) << 12) | ((u32)radio.pa_reg << 16);
}
/* ==== UI (N13): port of the UV-5RM custom firmware look (ui/list.rs, ui/standby.rs) to the Mini's 128x128 screen.
 * Electric-blue theme, Blue Angel boot logo and dimmed menu backdrop (as backdrop.rs did with the skull), no
 * start-up static/glitch effect. Rendering: the active screen is drawn into a 16-row strip buffer that is sent to the
 * LCD by SPI DMA; only regions whose content changed are redrawn. */
#include "ui_assets.h"
#define C_BLACK 0x0000
#define C_WHITE 0xffff
#define C_BLUE  0x049f      /* electric blue: menu title, selection, accents */
#define C_DIM   0x6b4d      /* Rgb565(13,26,13) as standby.rs */
#define C_HAIR  0x3186      /* (6,12,6) */
#define C_SLAVE 0xa514      /* (20,40,20) */
#define C_AMBER 0xfe40      /* (31,50,0) */
#define C_ALERT 0xf904      /* (31,8,4): transmit */
#define C_GREEN 0x06e6      /* (0,55,6): receive */
struct font { const u8 *bits; u8 w, h, base, stride; };
static const struct font F5 = {font_5x8_bits, 5, 8, 6, 1};
static const struct font F6 = {font_6x10_bits, 6, 10, 7, 1};
static const struct font F9 = {font_9x18_bits, 9, 18, 14, 2};
#define STRIP_H 16
static u8 strip[128 * STRIP_H * 2] __attribute__((aligned(4)));
static int sy0;
static void g_fill(int x, int y, int w, int h, u16 c)
{
    int x0 = x < 0 ? 0 : x, x1 = x + w > 128 ? 128 : x + w;
    int y0 = y < sy0 ? sy0 : y, y1 = y + h > sy0 + STRIP_H ? sy0 + STRIP_H : y + h;
    int yy, xx;
    for (yy = y0; yy < y1; ++yy) {
        u8 *p = strip + ((yy - sy0) * 128 + x0) * 2;
        for (xx = x0; xx < x1; ++xx) { *p++ = c >> 8; *p++ = c; }
    }
}
static int text_w(const struct font *f, const char *s) { return (int)strlen(s) * f->w; }
/* Transparent text; returns the x after the last glyph. Non-ASCII draws '?', as embedded-graphics does. */
static int g_text(const struct font *f, int x, int baseline, const char *s, u16 fg)
{
    int top = baseline - f->base, r0 = sy0 - top, r1 = sy0 + STRIP_H - top, row, col;
    if (r0 < 0) r0 = 0;
    if (r1 > f->h) r1 = f->h;
    for (; *s; ++s, x += f->w) {
        unsigned c = (u8)*s, idx = (c < 0x20 || c > 0x7f) ? '?' - 0x20 : c - 0x20;
        const u8 *g = f->bits + idx * f->h * f->stride;
        if (r0 >= r1 || x >= 128 || x + f->w <= 0) continue;
        for (row = r0; row < r1; ++row) {
            const u8 *b = g + row * f->stride;
            for (col = 0; col < f->w; ++col) {
                int px = x + col;
                if (px < 0 || px >= 128 || !(b[col >> 3] & (0x80 >> (col & 7)))) continue;
                strip[((top + row - sy0) * 128 + px) * 2] = fg >> 8;
                strip[((top + row - sy0) * 128 + px) * 2 + 1] = fg;
            }
        }
    }
    return x;
}
static void g_text_right(const struct font *f, int right, int baseline, const char *s, u16 fg)
{
    g_text(f, right - text_w(f, s), baseline, s, fg);
}
static void g_text_center(const struct font *f, int baseline, const char *s, u16 fg)
{
    g_text(f, (128 - text_w(f, s)) / 2, baseline, s, fg);
}
/* The angel behind menus at half brightness (backdrop.rs dim()). */
static void g_backdrop(int y, int h)
{
    int y0 = y < sy0 ? sy0 : y, y1 = y + h > sy0 + STRIP_H ? sy0 + STRIP_H : y + h, yy, xx;
    for (yy = y0; yy < y1; ++yy) {
        u8 *p = strip + (yy - sy0) * 256;
        const unsigned short *src = angel_logo + yy * 128;
        for (xx = 0; xx < 128; ++xx) { u16 v = (src[xx] >> 1) & 0x7bef; *p++ = v >> 8; *p++ = v; }
    }
}
static void g_logo(void)
{
    int yy, xx;
    for (yy = sy0; yy < sy0 + STRIP_H; ++yy) {
        u8 *p = strip + (yy - sy0) * 256;
        for (xx = 0; xx < 128; ++xx) { u16 v = angel_logo[yy * 128 + xx]; *p++ = v >> 8; *p++ = v; }
    }
}
static void strip_send(void)
{
    unsigned i;
    window(0, sy0, 128, STRIP_H);
    if (spi_dma_send(SPI1, strip, sizeof(strip)) < 0) {
        ++spi_errors;
        for (i = 0; i < sizeof(strip); ++i) lcd_byte(strip[i]);
    }
    put(LCD_CS, 1);
}
enum { SCR_BOOT, SCR_STANDBY, SCR_MENU, SCR_LAUNCH, SCR_MODE, SCR_APP, SCR_PROG };
static u8 scr = SCR_BOOT;
static bool in_app(void) { return scr == SCR_APP; }
static void draw_screen(void);
/* Redraw rows [y0, y1). */
static void render(int y0, int y1)
{
    int s;
    for (s = y0 & ~(STRIP_H - 1); s < y1 && s < 128; s += STRIP_H) {
        sy0 = s;
        draw_screen();
        strip_send();
    }
}

/* ---- notices: centred box over the bands for 1.5 s (standby.rs draw_overlay) ---- */
static u32 notice_ms;
static void set_notice(const char *s) { notice = s; notice_ms = jiffies_msec(); }
static bool notice_on(void) { return notice[0] && jiffies_msec() - notice_ms < 1500; }

/* ---- tone list: OFF, 50 CTCSS, 105 DCS normal, 105 DCS inverted ---- */
static const u16 ctcss50[50] = {
    670,693,719,744,770,797,825,854,885,915,948,974,1000,1035,1072,1109,1148,1188,1230,1273,1318,1365,1413,
    1462,1514,1567,1598,1622,1655,1679,1713,1738,1773,1799,1835,1862,1899,1928,1966,1995,2035,2065,2107,2181,
    2257,2291,2336,2418,2503,2541
};
#define TONES 261
static struct mini_rf_tone tone_at(unsigned i)
{
    struct mini_rf_tone t = {MINI_TONE_NONE, 0};
    if (i >= 1 && i <= 50) { t.kind = MINI_TONE_CTCSS; t.value = ctcss50[i - 1]; }
    else if (i >= 51 && i <= 155) mini_rf_tone_from_channel(i - 50, &t);
    else if (i >= 156 && i < TONES) mini_rf_tone_from_channel(0x6a + i - 156, &t);
    return t;
}
static unsigned tone_index(struct mini_rf_tone t)
{
    unsigned i;
    for (i = 1; i < TONES; ++i) { struct mini_rf_tone k = tone_at(i); if (k.kind == t.kind && k.value == t.value) return i; }
    return 0;
}
/* "100.0Hz", "D023N", "OFF" */
static void tone_value(struct mini_rf_tone t, char *buf, unsigned n)
{
    char o[4];
    if (t.kind == MINI_TONE_CTCSS) snprintf(buf, n, "%u.%uHz", t.value / 10, t.value % 10);
    else if (t.kind == MINI_TONE_DCS_N || t.kind == MINI_TONE_DCS_I) {
        oct3(t.value, o);
        snprintf(buf, n, "D%s%c", o, t.kind == MINI_TONE_DCS_I ? 'I' : 'N');
    } else snprintf(buf, n, "OFF");
}

/* Key codes, confirmed on this radio (N06 KEY display). */
enum {
    K_SIDE_EQ = 0x00, K_VFO_MR = 0x02, K_AB = 0x03, K_MINUS = 0x04, K_MENU = 0x05, K_UP = 0x06,
    K_DOWN = 0x07, K_EXIT = 0x08, K_STAR = 0x0c, K_HASH = 0x14, K_0 = 0x10
};
/* Digit value of a key, or -1: 1-3 = 09-0B, 4-6 = 0D-0F, 7-9 = 11-13, 0 = 10. */
static int key_digit(u8 k)
{
    if (k >= 0x09 && k <= 0x0b) return k - 0x08;
    if (k >= 0x0d && k <= 0x0f) return k - 0x09;
    if (k >= 0x11 && k <= 0x13) return k - 0x0a;
    if (k == K_0) return 0;
    return -1;
}
/* ---- settings menu (list.rs look) ---- */
static const u32 steps_hz[8] = {2500, 5000, 6250, 10000, 12500, 20000, 25000, 50000};
static u8 step_idx = 1, tot_level = 8;      /* 5 kHz; TOT 8 x 15 s = 120 s (custom firmware default) */
static int fe_hz;                       /* last FREQ ERR result: other radio's carrier minus ours, Hz */
static u8 fe_state;                     /* 0 none, 1 measured, 2 no carrier found, 3 measuring */
static u32 fe_diag(void) { return ((u32)fe_hz & 0xffffff) | ((u32)fe_state << 24); }
static u8 beep_mode = 1;                /* 0 off, 1-4 = mini_rf_beep style (N17 listening test) */
static void apply_vfo_tone(void)
{
    want_tone = tone_at(vfo_tone[vfo_sel]);
    mini_rf_set_rx_tone(&radio, want_tone);
    rx_status();
}
static bool beep_demo;                 /* play the newly chosen beep style once */
/* FREQ ERR: the chip's hardware frequency counter (REG 0x32 enable, 0x0D/0x0E reading in the tuning word's 10 Hz units
 * and crystal) measures the other radio's carrier while the user keys it. As the custom firmware's Search app
 * (search2/logic.rs): a reading only counts when a second one, taken after re-arming the counter, agrees within 25
 * (250 Hz); on UHF a reading above 52,000,000 is the doubled frequency and is halved (correct_band_fold). N20: N19
 * took one unconfirmed reading (+661834 Hz, meaningless). Results more than 50 kHz from the tuned word are ignored.
 * Result = measured word - our tuned word, in Hz. REG 0x32 is restored and the receiver re-tuned afterwards. */
static void screen_full(void);
static u32 fe_read(u32 t_end)
{
    while (jiffies_msec() < t_end) {
        u16 hi = rf_read(0x0D);
        if (!(hi & 0x8000)) {
            u32 raw = ((u32)(hi & 0x07ff) << 16) | rf_read(0x0E);
            if (radio.frequency_hz >= 400000000u && raw > 52000000u) raw /= 2;
            return raw;
        }
        os_time_dly(2);
    }
    return 0;
}
static void freq_error_measure(void)
{
    u32 t_end, first = 0, raw;
    u16 saved32;
    bool got = false;
    if (!radio.frequency_hz || radio.transmitting) return;
    fe_state = 3;                                   /* the row shows LISTENING while it measures */
    screen_full();
    saved32 = rf_read(0x32);
    t_end = jiffies_msec() + 8000;
    while (!got && jiffies_msec() < t_end) {
        rf_write(0x32, 0x0244);
        rf_write(0x32, 0x0244 | 1);                 /* re-arm the counter */
        raw = fe_read(t_end);
        if (!raw) break;
        if ((int)(raw - radio.word) > 5000 || (int)(radio.word - raw) > 5000) { first = 0; continue; }
        if (first && (first > raw ? first - raw : raw - first) < 25) {
            fe_hz = ((int)((first + raw) / 2) - (int)radio.word) * 10;
            got = true;
        } else first = raw;
    }
    rf_write(0x32, saved32);
    tune(radio.frequency_hz);
    fe_state = got ? 1 : 2;
}
#define MENU_ROW 14
/* list.rs: title bar, 8 rows of 14 px, selected row filled, the others over the dimmed logo. */
static void draw_list(const char *title, unsigned total, unsigned sel, bool edit,
                      void (*label)(unsigned, char *, unsigned), void (*value)(unsigned, char *, unsigned))
{
    unsigned slot, top = 0;
    char l[24], v[24];
    g_fill(0, 0, 128, MENU_ROW, C_BLUE);
    g_text_center(&F6, F6.base + 2, title, C_WHITE);
    if (total > 8 && sel > 4) top = sel - 4 > total - 8 ? total - 8 : sel - 4;
    for (slot = 0; slot < 8; ++slot) {
        unsigned i = top + slot;
        int y = MENU_ROW + slot * MENU_ROW, base = y + F6.base + 2;
        bool is_sel = i == sel && i < total;
        if (is_sel) g_fill(0, y, 128, MENU_ROW, C_BLUE);
        else g_backdrop(y, MENU_ROW);
        if (i >= total) continue;
        label(i, l, sizeof(l));
        g_text(&F6, 4, base, l, C_WHITE);
        if (!value) continue;
        value(i, v, sizeof(v));
        if (is_sel && edit) {
            char e[28];
            snprintf(e, sizeof(e), "< %s >", v);
            g_text_right(&F6, 124, base, e, C_WHITE);
        } else g_text_right(&F6, 124, base, v, C_WHITE);
    }
    g_backdrop(MENU_ROW * 9, 128 - MENU_ROW * 9);
}
/* N26: SETTINGS groups (mini_settings.h) */
static void settings_build(u8 *r);
static u8 set_saved[SET_SLOT];
static bool set_have;
static u16 api_subaudio_code_of_index(int i);
#include "mini_settings.h"
/* App launcher (the custom firmware's MENU list, in its order). Apps appear here as they are ported.
 * Each entry names up to two app segments (bf5rh_abi Chain): SEARCH/QT SCAN starts on QT SCAN and "1" switches. */
enum { AP_TEXT, AP_FOX, AP_SEARCH, AP_QT, AP_COUNT };   /* N23: SPECTRUM, N25: RPT FINDER and SEARCH/QT SCAN, N30: BAND SWEEP removed (user) */
struct launch_entry { const char *name; u8 seg[2]; bool combined; };
static const struct launch_entry launch_list[] = {
    {"SETTINGS", {0xff, 0xff}, false},
    {"SEARCH", {AP_SEARCH, 0xff}, false},
    {"SCANQT", {AP_QT, 0xff}, false},
    {"FOX HUNT", {AP_FOX, 0xff}, false},
    {"TEXT", {AP_TEXT, 0xff}, false},
};
#define LA_SETTINGS 0
#define LA_COUNT (sizeof(launch_list) / sizeof(launch_list[0]))
static u8 app_cur;                           /* the running app (AP_*) while scr == SCR_APP */
static bool cps_reload;
static unsigned launch_sel;
static void launch_label(unsigned i, char *b, unsigned n) { snprintf(b, n, "%s", launch_list[i].name); }
static void draw_launch(void) { draw_list("MENU", LA_COUNT, launch_sel, false, launch_label, 0); }
/* MODE: ALL plus the bands the PC page wrote (channel slots 1000-1015: name 0-15, short 16-20, 'B' at 31). */
#define MAX_BANDS 16
static u8 band_count;
static char band_name[MAX_BANDS][17], band_short[MAX_BANDS][6];
static unsigned mode_sel;
static void mode_label(unsigned i, char *b, unsigned n) { snprintf(b, n, "%s", i == 0 ? "ALL" : band_name[i - 1]); }
static void mode_value(unsigned i, char *b, unsigned n) { snprintf(b, n, "%s", i == chan_mode ? "*" : ""); }
static void draw_mode(void) { draw_list("MODE", band_count + 1u, mode_sel, false, mode_label, mode_value); }
/* PC programming session (the custom firmware's ui::cps programming screen). */
static void draw_prog(void)
{
    g_backdrop(0, 128);
    g_fill(0, 0, 128, MENU_ROW, C_BLUE);
    g_text_center(&F6, F6.base + 2, "PROGRAMMING", C_WHITE);
    g_fill(14, 52, 100, 24, C_BLUE);
    g_fill(15, 53, 98, 22, C_BLACK);
    g_text_center(&F6, 61 + F6.base, "PC LINK", C_WHITE);
}
static const char *mode_short(void)
{
    if (chan_mode && chan_mode <= band_count) return band_short[chan_mode - 1][0] ? band_short[chan_mode - 1] : "BAND";
    return mr_mode ? "MR" : "VFO";
}

/* ---- standby (standby.rs layout, 128 px wide) ---- */
#define ST_STATUS_H 12
#define BAND_H 46
#define BAND_TOP0 13
#define BAND_TOP1 (BAND_TOP0 + BAND_H + 1)
#define METER_RULE (BAND_TOP1 + BAND_H)
#define METER_TOP (METER_RULE + 2)
#define MARGIN 6
#define LABEL_X (MARGIN + 22)
static u32 tx_elapsed_s(void);
static int rssi_dbm(void) { return (int)(radio.rssi / 2) - 160; }
static void draw_band(unsigned i, int top)
{
    bool master = i == vfo_sel, tx_here = master && radio.transmitting, chan = master ? mr_mode : side_mr[i];
    const struct channel *cp = master ? &ch : &ch_other;
    u32 hz = chan ? (tx_here ? cp->tx_hz : cp->rx_hz) : (tx_here ? vfo_tx_hz(i) : vfo_hz[i]);
    bool named = chan && cp->name[0] && ch_disp != 0;   /* N26 CH DISP: FREQ shows no name */
    char b[24], o[16];
    u16 fg = master ? C_WHITE : C_SLAVE;
    int base1 = top + F6.base + 2, base2 = top + 12 + F9.base + 3, base3 = top + 34 + F5.base + 2;
    g_fill(0, top, 3, BAND_H, tx_here ? C_ALERT : (master ? C_BLUE : C_HAIR));
    if (master && radio.transmitting) g_text(&F6, MARGIN, base1, "TX", C_ALERT);
    else if (master && (radio.audible || radio.monitor)) g_text(&F6, MARGIN, base1, "RX", C_GREEN);
    /* row 1: label / channel entry */
    if (master && mr_mode && entry_len) {
        snprintf(b, sizeof(b), "M%s%s", entry, &"---"[entry_len]);
        g_text(&F6, LABEL_X, base1, b, C_AMBER);
    } else {
        if (chan) snprintf(b, sizeof(b), "M%03u", (master ? mr_index : side_ch[i]) + 1u); else snprintf(b, sizeof(b), "VFO %c", i ? 'B' : 'A');
        g_text(&F6, LABEL_X, base1, b, master ? C_BLUE : C_DIM);
        if (named && ch_disp == 2) {                /* N17: frequency small, top right (name goes in the middle) */
            snprintf(o, sizeof(o), "%u.%04u", (unsigned)(hz / 1000000u), (unsigned)(hz / 100u % 10000u));
            g_text_right(&F6, 122, base1, o, master ? C_WHITE : C_DIM);
        }
    }
    /* row 2: frequency (9x18 head + 6x10 tail, right aligned) or entry */
    if (master && !mr_mode && entry_len) {
        unsigned k, j = 0;
        for (k = 0; k < 6; ++k) { if (k == 3) b[j++] = '.'; b[j++] = k < entry_len ? entry[k] : '-'; }
        b[j] = 0;
        g_text_right(&F9, 122, base2, b, C_AMBER);
    } else if (named) {                         /* N17: channel name large, centred */
        g_text(&F9, 3 + (125 - text_w(&F9, cp->name)) / 2, base2, cp->name, fg);
    } else {
        unsigned mhz = hz / 1000000, frac = (hz % 1000000) / 10;
        int x;
        snprintf(b, sizeof(b), "%3u.%03u", mhz, frac / 100);
        snprintf(o, sizeof(o), "%02u", frac % 100);
        x = 122 - text_w(&F6, o) - text_w(&F9, b);
        g_text(&F9, x, base2, b, fg);
        g_text(&F6, x + text_w(&F9, b), base2 - F9.base + F6.base, o, fg);
    }
    /* row 3: mode line FM / power / shift / tones / WIDE or NAR (N26: TX tone and shift) */
    {
        struct mini_rf_tone t = chan ? cp->tone : tone_at(vfo_tone[i]), tt = chan ? cp->txtone : tone_at(vfo_txtone[i]);
        bool high = chan ? cp->power == 0 : tx_high, narrow = chan ? cp->narrow : vfo_narrow[i];
        u32 rx = chan ? cp->rx_hz : vfo_hz[i], tx = chan ? cp->tx_hz : vfo_tx_hz(i);
        bool dcs = t.kind == MINI_TONE_DCS_N || t.kind == MINI_TONE_DCS_I || tt.kind == MINI_TONE_DCS_N || tt.kind == MINI_TONE_DCS_I;
        int x = MARGIN;
        x = g_text(&F5, x, base3, "FM", C_DIM) + F5.w;
        x = g_text(&F5, x, base3, high ? "HIGH" : "LOW", C_WHITE) + F5.w;
        if (tx && tx != rx) x = g_text(&F5, x, base3, tx > rx ? "+" : "-", C_WHITE) + F5.w;
        if (t.kind != MINI_TONE_NONE || tt.kind != MINI_TONE_NONE)
            x = g_text(&F5, x, base3, t.kind != MINI_TONE_NONE && tt.kind != MINI_TONE_NONE ? (dcs ? "DTCS" : "TSQL")
                                      : t.kind != MINI_TONE_NONE ? (dcs ? "DTCS-R" : "TSQL-R") : (dcs ? "DCS-T" : "TONE"), C_BLUE) + F5.w;
        if (chan && !cp->tx_hz) x = g_text(&F5, x, base3, "RXO", C_AMBER) + F5.w;
        g_text_right(&F5, 122, base3, narrow ? "NAR" : "WIDE", narrow ? C_AMBER : C_DIM);
        (void)x;
    }
}
/* ---- battery (N36): as stock V0.04 ----
 * PB3 = the battery divider, ADC channel AD_CH_PB3 (stock fn_01e4695a sets PB3 analog; fn_01e46a00 reads it >> 2).
 * Stock fn_01e46dfa compares that 8-bit value with six thresholds from the calibration at NOR 0xF200 (fn at 0x01E4750E;
 * defaults if 0 or 0xFF). This radio's: 140 145 155 165 183 205. Here: 5 bars above the highest down to 0 bars at the
 * lowest; below it = critical. Sampled every 0.5 s except while transmitting and 1.5 s after (the voltage sags under
 * load), smoothed, and a new bar count has to hold for 4 samples. */
static u8 bat_th[6] = {140, 145, 155, 165, 183, 205};
static u16 bat_raw, bat_avg16;           /* last 8-bit sample, smoothed x16 */
static u8 bat_level = 0xff, bat_pending = 0xff, bat_pending_n; /* 0..5 bars, 6 = critical; 0xff = not measured */
static u32 bat_next_ms, bat_note_ms, bat_tx_ms;
static bool bat_blink;
static void battery_init(void)
{
    u8 t[6];
    gpio_set_direction(IO_PORTB_03, 1);
    gpio_set_die(IO_PORTB_03, 0);
    gpio_set_pull_up(IO_PORTB_03, 0);
    gpio_set_pull_down(IO_PORTB_03, 0);
    adc_add_sample_ch(AD_CH_PB3);
    if (nor_read(0xF200, t, 6) && t[0] != 0 && t[0] != 0xff) {
        unsigned i; bool ok = true;
        for (i = 1; i < 6; ++i) if (t[i] <= t[i - 1] || t[i] == 0xff) ok = false;
        if (ok) memcpy(bat_th, t, 6);
    }
}
static u8 battery_bars(unsigned v)
{
    int k;
    for (k = 5; k >= 1; --k) if (v > bat_th[k]) return (u8)k;
    return v >= bat_th[0] ? 0 : 6;          /* 6 = below the lowest threshold: critical */
}
static void battery_service(void)
{
    u32 now = jiffies_msec();
    u8 lv;
    if (radio.transmitting) { bat_tx_ms = now; return; }
    if ((int)(now - bat_next_ms) < 0 || now - bat_tx_ms < 1500) return;
    bat_next_ms = now + 500;
    bat_raw = (u16)(adc_get_value(AD_CH_PB3) >> 2);
    if (bat_raw == 0) return;                /* not sampled yet */
    if (!bat_avg16) bat_avg16 = (u16)(bat_raw * 16);
    else bat_avg16 = (u16)(bat_avg16 + ((int)(bat_raw * 16) - (int)bat_avg16) / 8);
    lv = battery_bars(bat_avg16 / 16u);
    if (bat_level == 0xff) bat_level = lv;
    else if (lv != bat_level) {
        if (lv == bat_pending) { if (++bat_pending_n >= 4) { bat_level = lv; bat_pending = 0xff; } }
        else { bat_pending = lv; bat_pending_n = 1; }
    } else bat_pending = 0xff;
    bat_blink = !bat_blink;
    if (bat_level == 6 && now - bat_note_ms > 30000) { bat_note_ms = now; set_notice("CHARGE BATTERY"); }
    else if (bat_level == 1 && !bat_note_ms) { bat_note_ms = now; set_notice("BATTERY LOW"); }
    if (bat_level >= 2 && bat_level <= 5) bat_note_ms = 0;
}
/* 5-segment icon, top right of the status bar (x 104-125). */
static void draw_battery(void)
{
    int x = 104, y = 2, w = 19, h = 8, k;
    bool crit = bat_level == 6;
    u16 frame = crit ? (bat_blink ? C_ALERT : C_HAIR) : C_DIM;
    u16 fill = bat_level >= 3 && bat_level <= 5 ? C_GREEN : bat_level == 2 ? C_AMBER : C_ALERT;
    g_fill(x, y, w, 1, frame); g_fill(x, y + h - 1, w, 1, frame);
    g_fill(x, y, 1, h, frame); g_fill(x + w - 1, y, 1, h, frame);
    g_fill(x + w, y + 2, 2, h - 4, frame);                     /* the terminal */
    if (bat_level > 5) return;                                  /* unknown or critical: empty */
    for (k = 0; k < bat_level; ++k) g_fill(x + 2 + k * 3, y + 2, 2, h - 4, fill);
}
static void draw_standby(void)
{
    char b[24];
    g_fill(0, 0, 128, 128, C_BLACK);
    /* status bar */
    if (radio.transmitting) {
        u32 s = tx_elapsed_s();
        snprintf(b, sizeof(b), "%02u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
        g_text(&F6, 4, F6.base + 2, b, C_ALERT);
    } else {
        snprintf(b, sizeof(b), "SQ%u", radio.squelch);
        g_text(&F5, 4, F5.base + 2, b, C_DIM);
        if (scanning) g_text(&F5, 4 + 4 * F5.w, F5.base + 2, scan_phase ? "SCAN||" : (scan_up ? "SCAN>" : "SCAN<"), C_AMBER);
        else if (radio.monitor) g_text(&F5, 4 + 4 * F5.w, F5.base + 2, "MON", C_AMBER);
        if (tot_level == 0 && !scanning) g_text(&F5, 4 + 8 * F5.w, F5.base + 2, "NOTOT", C_DIM);
    }
    g_text_right(&F5, 100, F5.base + 2, mode_short(), C_BLUE);   /* N36: left of the battery */
    draw_battery();
    g_fill(0, ST_STATUS_H, 128, 1, C_BLUE);
    draw_band(0, BAND_TOP0);
    g_fill(0, BAND_TOP1 - 1, 128, 1, C_HAIR);
    draw_band(1, BAND_TOP1);
    g_fill(MARGIN, METER_RULE, 128 - 2 * MARGIN, 1, C_HAIR);
    /* meter: S-meter (13 segments) or TX power */
    {
        int base_y = METER_TOP + 14 - 2, baseline = METER_TOP + F5.base + 3, k, x0 = 128 - 6 - 13 * 5 + 1;
        if (radio.transmitting) {
            g_text(&F5, MARGIN, baseline, tx_high_now ? "TX HIGH" : "TX LOW", C_ALERT);
            for (k = 0; k < 13; ++k) { int h = k + 1 > 10 ? 10 : k + 1; g_fill(x0 + k * 5, base_y - h + 1, 4, h, C_ALERT); }
        } else {
            int dbm = rssi_dbm(), s = (dbm + 127) / 6, level;
            if (s < 0) s = 0;
            if (s > 9) s = 9;
            level = s;
            if (dbm > -73) { level = 9 + (dbm + 73) / 10 + 1; if (level > 13) level = 13; }
            if (radio.audible) {
                if (dbm > -73) snprintf(b, sizeof(b), "%ddBm +%d", dbm, dbm + 73);
                else snprintf(b, sizeof(b), "%ddBm S%d", dbm, s);
            } else snprintf(b, sizeof(b), "%ddBm", dbm);
            g_text(&F5, MARGIN, baseline, b, radio.audible ? C_WHITE : C_DIM);
            for (k = 0; k < 13; ++k) {
                int h = k + 1 > 10 ? 10 : k + 1;
                u16 c = !radio.audible || k >= level ? C_HAIR : (k < 9 ? C_GREEN : (k < 12 ? C_AMBER : C_ALERT));
                g_fill(x0 + k * 5, base_y - h + 1, 4, h, c);
            }
        }
    }
    /* notice overlay */
    if (notice_on()) {
        int w = text_w(&F6, notice) + 12, x = (128 - w) / 2, y = 64 - 11;
        g_fill(x, y, w, 22, C_BLUE);
        g_fill(x + 1, y + 1, w - 2, 20, C_BLACK);
        g_text(&F6, x + 6, y + F6.base + 6, notice, C_WHITE);
    }
}
static void draw_screen(void)
{
    if (scr == SCR_BOOT) g_logo();
    else if (scr == SCR_MENU) draw_settings();
    else if (scr == SCR_LAUNCH) draw_launch();
    else if (scr == SCR_MODE) draw_mode();
    else if (scr == SCR_PROG) draw_prog();
    else draw_standby();
}
/* Region signatures: redraw only what changed. */
static char sig_old[4][200];
static bool sig_valid;
static void screen_sig(unsigned r, char *s, unsigned n)
{
    bool ov = notice_on();
    unsigned n2;
    switch (r) {
    case 0: snprintf(s, n, "%u%u%u%u%u%u%u%lu", scr, radio.transmitting, radio.squelch, radio.monitor, mr_mode * 2 + !tot_level, chan_mode,
                     scanning * 4 + scan_phase * 2 + scan_up,
                     radio.transmitting ? (unsigned long)tx_elapsed_s() : (unsigned long)(bat_level * 2u + (bat_level == 6 && bat_blink)) + 1000ul); break;
    case 1: case 2: {
        unsigned i = r - 1;
        snprintf(s, n, "%u%u%u%u%lu%lu%u%u%u%s|%s|%u%u%u%u%s", scr, vfo_sel, mr_mode, mr_index, (unsigned long)vfo_hz[i],
                 (unsigned long)(mr_mode ? ch.rx_hz : 0), vfo_tone[i], tx_high, radio.transmitting * 2 + (radio.audible || radio.monitor),
                 entry, ov ? notice : "", ch.tone.kind, ch.power, ch.tx_hz != 0, entry_len, ch.name);
        n2 = strlen(s);
        snprintf(s + n2, n - n2, "|%u%u%u%u%u%s", side_mr[i], side_ch[i], (unsigned)ch_other.rx_hz, ch_other.power,
                 ch_other.tone.kind, ch_other.name);
        n2 = strlen(s);
        snprintf(s + n2, n - n2, "|%u%u%u%u%u%u%lu%u%u%u%u", vfo_shift[i], vfo_offset_khz[i], vfo_txtone[i], vfo_narrow[i],
                 ch.txtone.kind * 16 + ch.txtone.value % 16, ch.narrow, (unsigned long)ch.tx_hz, ch_other.narrow, ch_other.txtone.kind, ch_disp,
                 (unsigned)ch_other.tx_hz % 1000u);
        break;
    }
    default: snprintf(s, n, "%u%u%u%u%u", scr, radio.transmitting, tx_high_now, radio.audible,
                      radio.audible ? radio.rssi / 12 : radio.rssi / 24); break;
    }
}
static const u8 region_y[5] = {0, BAND_TOP0, BAND_TOP1 - 1, METER_RULE, 128};
static void screen(void)
{
    char s[200];
    unsigned r;
    if (scr == SCR_APP) return;                       /* the app draws itself */
    if (scr != SCR_STANDBY) { render(0, 128); sig_valid = false; return; }
    for (r = 0; r < 4; ++r) {
        screen_sig(r, s, sizeof(s));
        if (!sig_valid || strcmp(s, sig_old[r])) { render(region_y[r], region_y[r + 1]); strcpy(sig_old[r], s); }
    }
    sig_valid = true;
}
static void screen_full(void) { sig_valid = false; screen(); }

static bool tune_vfo(u32 hz)
{
    want_tone = tone_at(vfo_tone[vfo_sel] % TONES);      /* VFO: selected receive tone, none by default */
    if (!tune(hz)) return false;
    vfo_hz[vfo_sel] = hz;
    return true;
}
static void enter_mr(void)
{
    /* The last channel used, else the first stored one. */
    if (channel_load(mr_index, &ch)) {
        mr_mode = true;
        tune_channel();
    } else if (channel_step(1, true)) {
        mr_mode = true;
    } else {
        set_notice("NO CHANNELS");
    }
}
static void entry_digit(int d)
{
    unsigned want = mr_mode ? 3 : 6;
    entry[entry_len++] = '0' + d;
    entry[entry_len] = 0;
    if (entry_len < want) return;
    if (mr_mode) {
        unsigned n = (entry[0] - '0') * 100 + (entry[1] - '0') * 10 + (entry[2] - '0');
        if (n >= 1 && n <= CHANNELS && channel_load(n - 1, &ch)) {
            mr_index = n - 1;
            tune_channel();
        } else set_notice("EMPTY CHANNEL");
    } else {
        u32 khz = 0; unsigned i;
        for (i = 0; i < 6; ++i) khz = khz * 10 + (entry[i] - '0');
        if (!tune_vfo(khz * 1000)) set_notice("OUT OF BAND");
    }
    entry_len = 0; entry[0] = 0;
}
static void app_open_entry(unsigned launch);
static void app_key(u8 key, u8 kind);
#define LA_TEXT (LA_COUNT - 1)
static bool flash_on;
/* Stock 0x01E418D4: PB6 drives the flashlight (0 off, 1 on). */
static void flashlight(bool on) { flash_on = on; output(IO_PORTB_06, on); }
/* A/B: each side keeps its own VFO or channel (custom firmware sides[]). */
static void side_switch(void)
{
    struct channel t;
    side_mr[vfo_sel] = mr_mode; side_ch[vfo_sel] = mr_index;
    t = ch; ch = ch_other; ch_other = t;
    vfo_sel ^= 1;
    mr_mode = side_mr[vfo_sel]; mr_index = side_ch[vfo_sel];
    if (mr_mode && channel_load(mr_index, &ch)) tune_channel();
    else { mr_mode = false; tune_vfo(vfo_hz[vfo_sel]); }
    side_mr[vfo_sel] = mr_mode;
}
static void mode_select(unsigned m)
{
    chan_mode = m;
    if (!m) { set_notice("ALL CHANNELS"); return; }
    mr_index = 0;
    if (channel_step(1, true)) { mr_mode = true; side_mr[vfo_sel] = true; }
    else set_notice("NO CHANNELS IN BAND");
}
/* ---- scan (N23, reworked N25): hold # in standby; the 5RM's app/scan.rs in its carrier mode ----
 * Channels: the stored channels in the current MODE that have scan-add (byte 15 bit 2); if none has it, all of them.
 * The list is read once when the scan starts (a bitmap), so a hop is one channel read. VFO: the VFO's band (136-174 or
 * 400-520 MHz) in STEP steps. Hops use mini_rf_hop (no REG 0x30 off, no settle: no pop) and the speaker is held shut
 * while hopping; it opens only when the scan stops on a signal. Each hop listens 200 ms (400 ms at the start); a
 * signal (carrier and, on a toned channel, the tone) for 5 polls stops it there until it has been gone 5 s.
 * UP/DOWN change direction and hop; # (click or hold), EXIT, any other key or PTT end the scan.
 * N25: N23 did a full tune per hop (REG 0x30 off = pop, 30 ms settle) and searched all 1000 slots per hop, which
 * slowed the main loop so much that keys stopped registering. */
#define SCAN_STEP_MS 200u
#define SCAN_START_MS 400u
#define SCAN_LOST_MS 5000u
static u8 scan_deb;                           /* scan_phase: 0 hopping, 1 holding on a signal */
static u32 scan_until;
static u8 scan_map[(CHANNELS + 7) / 8];
static unsigned scan_count;
static void scan_build(bool all)
{
    struct channel c; unsigned i;
    memset(scan_map, 0, sizeof(scan_map)); scan_count = 0;
    for (i = 0; i < CHANNELS; ++i)
        if (channel_load(i, &c) && in_mode(&c) && (all || c.scan_add)) { scan_map[i >> 3] |= (u8)(1u << (i & 7)); ++scan_count; }
}
static bool scan_next_channel(int dir, unsigned *out, struct channel *c)
{
    unsigned k, idx;
    for (k = 1; k <= CHANNELS; ++k) {
        idx = (mr_index + CHANNELS + dir * (int)k) % CHANNELS;
        if ((scan_map[idx >> 3] & (1u << (idx & 7))) && channel_load(idx, c)) { *out = idx; return true; }
    }
    return false;
}
static void scan_stop(const char *why)
{
    scanning = false;
    radio.hold_mute = false;
    side_ch[vfo_sel] = mr_index;
    if (why) set_notice(why);
    mini_rf_poll(&radio);
}
static void scan_hop(void)
{
    radio.hold_mute = true;                   /* nothing from the speaker while hopping */
    if (mr_mode) {
        struct channel c; unsigned idx;
        if (scan_next_channel(scan_up ? 1 : -1, &idx, &c)) {
            ch = c; mr_index = idx; side_ch[vfo_sel] = idx;
            want_tone = ch.tone; radio.rx_tone = ch.tone;
            if (!mini_rf_hop(&radio, ch.rx_hz)) tune_channel();
        }
    } else {
        u32 hz = vfo_hz[vfo_sel], lo = hz < 300000000u ? 136000000u : 400000000u, hi = lo == 136000000u ? 174000000u : 520000000u;
        u32 step = steps_hz[step_idx];
        if (hz < lo || hz > hi) hz = lo;
        if (scan_up) hz = hz + step > hi ? lo : hz + step; else hz = hz < lo + step ? hi : hz - step;
        vfo_hz[vfo_sel] = hz;
        want_tone = tone_at(vfo_tone[vfo_sel] % TONES); radio.rx_tone = want_tone;
        if (!mini_rf_hop(&radio, hz)) tune_vfo(hz);
    }
    scan_phase = 0; scan_deb = 0; scan_until = jiffies_msec() + SCAN_STEP_MS;
    screen();
}
static void scan_start(void)
{
    if (mr_mode) {
        scan_build(false);
        if (!scan_count) scan_build(true);
        if (!scan_count) { set_notice("NO CHANNELS TO SCAN"); return; }
    }
    if (radio.monitor) mini_rf_monitor(&radio, false);
    scanning = true; scan_up = true; scan_phase = 0; scan_deb = 0;
    radio.hold_mute = true;
    scan_until = jiffies_msec() + SCAN_START_MS;
    set_notice(mr_mode ? "SCAN CHANNELS" : "SCAN VFO");
}
static void scan_service(void)
{
    u32 now = jiffies_msec(); bool open;
    if (!scanning) return;
    if (scr != SCR_STANDBY) { scanning = false; return; }   /* an app (PTT-A) owns the audio now */
    if (radio.transmitting) { scan_stop(0); return; }
    mini_rf_poll(&radio);
    open = (radio.status & 2) && (radio.rx_tone.kind == MINI_TONE_NONE || radio.tone_matched);
    if (scan_phase == 0) {
        if ((int)(now - scan_until) < 0) return;
        if (open) {
            if (++scan_deb < 5) return;
            if (scan_mode == 2) { scan_stop("SIGNAL FOUND"); screen(); return; }   /* N26 SCAN MD STOP */
            scan_phase = 1; scan_until = now + SCAN_LOST_MS;
            radio.hold_mute = false;          /* stopped on a signal: let it be heard */
            mini_rf_poll(&radio);
            screen();
            return;
        }
        scan_hop();
    } else if (open && scan_mode == 1) scan_until = now + SCAN_LOST_MS;   /* CARRIER: wait for it to go; TIME: 5 s */
    else if ((int)(now - scan_until) >= 0) scan_hop();
}
/* Keys that also have a hold action report a click on release; the others act at once. */
/* N22: inside an app MENU also has a hold (Band Sweep: hold MENU = put the signal in the VFO). Decided at press time. */
static bool has_long(u8 key)
{
    return key == K_STAR || key == K_HASH || key == K_EXIT || key == K_SIDE_EQ || (key == K_MENU && scr == SCR_APP)
        || (key == K_MINUS && scr == SCR_STANDBY && kf_band_lg != KF_NONE);
}
/* N26: programmable key functions (5RM keyfn.rs subset): side key click / hold, PTT-A, [-] click / hold. */
static bool keys_locked;
static void keyfn_run(u8 f)
{
    switch (f) {
    case KF_WN:
        if (mr_mode) ch.narrow = !ch.narrow; else vfo_narrow[vfo_sel] = !vfo_narrow[vfo_sel];
        if (mr_mode) tune_channel(); else tune_vfo(vfo_hz[vfo_sel]);
        set_notice(cur_narrow() ? "NARROW" : "WIDE");
        break;
    case KF_MONITOR: mini_rf_monitor(&radio, !radio.monitor); set_notice(radio.monitor ? "MONITOR ON" : "MONITOR OFF"); break;
    case KF_MODE: scr = SCR_MODE; mode_sel = chan_mode <= band_count ? chan_mode : 0; screen_full(); return;
    case KF_TEXT: app_open_entry(LA_TEXT); return;
    case KF_SCAN: if (scanning) scan_stop("SCAN OFF"); else if (!entry_len) scan_start(); break;
    case KF_POWER: {
        bool high;
        if (mr_mode) { ch.power ^= 1; high = ch.power == 0; } else { tx_high = !tx_high; high = tx_high; }
        set_notice(high ? "HIGH POWER" : "LOW POWER");
        break;
    }
    case KF_LIGHT: flashlight(!flash_on); return;
    default: return;
    }
    screen();
}
static void list_key(u8 key)
{
    if (scr == SCR_MENU) {
        if (!settings_key(key)) scr = SCR_LAUNCH;
    } else if (scr == SCR_LAUNCH) {
        if (key == K_UP) launch_sel = (launch_sel + LA_COUNT - 1) % LA_COUNT;
        else if (key == K_DOWN) launch_sel = (launch_sel + 1) % LA_COUNT;
        else if (key == K_MENU) {
            if (launch_sel == LA_SETTINGS) settings_open();
            else { app_open_entry(launch_sel); return; }
        } else if (key == K_EXIT) { scr = SCR_STANDBY; screen_full(); return; }
    } else if (scr == SCR_MODE) {
        unsigned total = band_count + 1u;
        if (key == K_UP) mode_sel = (mode_sel + total - 1) % total;
        else if (key == K_DOWN) mode_sel = (mode_sel + 1) % total;
        else if (key == K_MENU) { scr = SCR_STANDBY; mode_select(mode_sel); screen_full(); return; }
        else if (key == K_EXIT || key == K_MINUS) { scr = SCR_STANDBY; screen_full(); return; }
    }
    screen_full();
}
/* A click (or a press, for keys without a hold action). */
static void key_action(u8 key)
{
    int d = key_digit(key);
    if (scr == SCR_APP) { app_key(key, 2); return; }
    if (scr == SCR_MENU || scr == SCR_LAUNCH || scr == SCR_MODE) { list_key(key); return; }
    if (scanning) {                                  /* N23: UP/DOWN steer the scan, anything else ends it */
        if (key == K_UP || key == K_DOWN) { scan_up = key == K_UP; scan_hop(); }
        else scan_stop("SCAN OFF");
        screen();
        return;
    }
    if (keys_locked) { set_notice("LOCKED: HOLD *"); screen(); return; }   /* N26 auto lock */
    if (key == K_SIDE_EQ) { keyfn_run(kf_side_sh); return; }            /* N26: SIDE-SH (default POWER) */
    if (key == K_MINUS && !entry_len) { keyfn_run(kf_band_sh); if (scr != SCR_STANDBY) screen_full(); return; }   /* [-]-SH */
    if (entry_len) {
        if (d >= 0) entry_digit(d);
        else if (key == K_EXIT) { entry_len = 0; entry[0] = 0; }
        screen();
        return;
    }
    if (d >= 0) { entry_digit(d); screen(); return; }
    if (key == K_STAR) {
        static char b[12];
        mini_rf_set_squelch(&radio, (radio.squelch + 1) % 10);
        snprintf(b, sizeof(b), "SQL %u", radio.squelch);
        set_notice(b);
    } else if (key == K_MENU) {
        scr = SCR_LAUNCH;
    } else if (key == K_AB) {
        side_switch();
    } else if (key == K_VFO_MR) {
        if (mr_mode) { mr_mode = false; tune_vfo(vfo_hz[vfo_sel]); }
        else enter_mr();
        side_mr[vfo_sel] = mr_mode;
    } else if (mr_mode) {
        if (key == K_UP || key == K_DOWN) { channel_step(key == K_UP ? 1 : -1, false); side_ch[vfo_sel] = mr_index; }
        else if (key == K_HASH) set_notice("SET BY CHANNEL");
    } else {
        if (key == K_HASH) {
            static char b[20];
            char v[12];
            vfo_tone[vfo_sel] = (vfo_tone[vfo_sel] + 1) % TONES;
            apply_vfo_tone();
            tone_value(want_tone, v, sizeof(v));
            snprintf(b, sizeof(b), "R-CTC %s", v);
            set_notice(b);
        }
        if (key == K_UP || key == K_DOWN) {
            u32 step = steps_hz[step_idx], hz = radio.frequency_hz;
            hz = key == K_UP ? hz + step : hz - step;
            if (radio.frequency_hz < 200000000) {
                if (hz > 174000000) hz = 136000000;
                if (hz < 136000000) hz = 174000000;
            } else {
                if (hz > 520000000) hz = 400000000;
                if (hz < 400000000) hz = 520000000;
            }
            tune_vfo(hz);
        }
    }
    if (scr != SCR_STANDBY) screen_full(); else screen();
}
/* A hold (0.8 s). */
static void key_long(u8 key)
{
    if (key == K_SIDE_EQ) {                          /* SIDE-LG (default LIGHT: everywhere, as a torch should be) */
        if (kf_side_lg == KF_LIGHT) flashlight(!flash_on);
        else if (scr == SCR_STANDBY && !keys_locked) keyfn_run(kf_side_lg);
        return;
    }
    if (scr == SCR_APP) { app_key(key, 3); return; }
    if (scr != SCR_STANDBY) return;
    if (keys_locked) {
        if (key == K_STAR) { keys_locked = false; set_notice("UNLOCKED"); } else set_notice("LOCKED: HOLD *");
        screen();
        return;
    }
    if (key == K_MINUS) { keyfn_run(kf_band_lg); return; }
    if (key == K_STAR) { if (scanning) scan_stop(0); mini_rf_monitor(&radio, !radio.monitor); set_notice(radio.monitor ? "MONITOR ON" : "MONITOR OFF"); screen(); }
    if (key == K_HASH) {                             /* N23: hold # = scan on / off */
        if (scanning) scan_stop("SCAN OFF"); else if (!entry_len) scan_start();
        screen();
    }
}
/* Return stock matrix codes. Label mapping is verified on the target before
 * assigning VFO/menu actions. Direct PTT inputs are separately displayed. */
static u8 keys(void)
{
    static const u8 map[5][4] = {
        {0,3,4,2}, {8,7,6,5}, {0x14,0x13,0x12,0x11},
        {0x10,0x0f,0x0e,0x0d}, {0x0c,0x0b,0x0a,9}
    };
    unsigned r, c, k; u8 found = 0xff;
    for (c = 0; c < 4; ++c) put(IO_PORTA_06 + c, 1);
    delay(500);
    for (r = 0; r < 4; ++r) if (!gpio_read(IO_PORTC_02+r)) return map[0][r];
    for (c = 0; c < 4; ++c) {
        put(IO_PORTA_06+c, 0); delay(500);
        for (r = 0; r < 4; ++r) if (!gpio_read(IO_PORTC_02+r)) {
            k = map[c+1][r];
            if (found != 0xff) { found = 0xfe; break; }
            found = k;
        }
        put(IO_PORTA_06+c, 1);
        if (found == 0xfe) break;
    }
    return found;
}
/* This radio's calibration and squelch setting, read-only from the settings NOR.
 * Tables as stock V0.04 loads them (0x01e49016..): 0xF0C0/0xF0D0 squelch thresholds, 0xF0E0 UHF and 0xF0F0 VHF
 * offsets, 0xF210+6 crystal trim. Settings record at 0x9010 (header at 0x9000), byte 0 = squelch level.
 * Implausible tables are rejected and the driver keeps its nominal values. */
static void load_calibration(void)
{
    struct mini_rf_cal c;
    u8 level = 3;
    unsigned i;
    bool ok = nor_read(0xF0C0, c.th_in, 10) && nor_read(0xF0D0, c.th_out, 10)
           && nor_read(0xF0E0, c.off_u400, 16) && nor_read(0xF0F0, c.off_v136, 16)
           && nor_read(0xF216, &c.xtal, 1) && nor_read(0xF211, &c.cts_gain, 1) && nor_read(0xF212, &c.dcs_gain, 1);
    for (i = 1; ok && i < 10; ++i)
        if (c.th_in[i] == 0xff || c.th_in[i] <= c.th_in[i - 1] || c.th_out[i] > c.th_in[i]) ok = false;
    if (ok) {
        if (c.xtal > 16) c.xtal = 8;
        mini_rf_set_cal(&radio, &c);
    }
    /* TX power targets as the stock loader 0x01E48FB2 places them: 0xF000 HIGH UHF, 0xF010 HIGH VHF,
     * 0xF080 LOW UHF, 0xF090 LOW VHF; 0xF210 mic gain (low 6 bits). */
    pa_loaded = ok && nor_read(0xF000, pa_high_uhf, 16) && nor_read(0xF010, pa_high_vhf, 16)
             && nor_read(0xF080, pa_low_uhf, 16) && nor_read(0xF090, pa_low_vhf, 16) && nor_read(0xF210, &mic_cal, 1);
    if (nor_read(0x9010, &level, 1) && level > 9) level = 3;
    mini_rf_set_squelch(&radio, level);
}
/* Record: magic[4], vfo_hz A/B (u32 LE), tone kind A/B, tone value A/B (u16), squelch, vfo_sel, mr_mode, 0,
 * mr_index (u16), power (N13), TOT+1, step+1, 3 reserved zero bytes, CRC16-CCITT of bytes 0..29. */
static u8 set_saved[SET_SLOT];
static bool set_have;
static void put32(u8 *p, u32 v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static u32 get32(const u8 *p) { return p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24); }
static void settings_build(u8 *r)
{
    unsigned i; u16 c;
    memset(r, 0, SET_SLOT);
    memcpy(r, set_magic, 4);
    put32(r + 4, vfo_hz[0]); put32(r + 8, vfo_hz[1]);
    for (i = 0; i < 2; ++i) {
        struct mini_rf_tone t = tone_at(vfo_tone[i] % TONES);
        r[12 + i] = t.kind; r[14 + 2 * i] = t.value; r[15 + 2 * i] = t.value >> 8;
    }
    r[18] = radio.squelch; r[19] = vfo_sel; r[20] = mr_mode; r[22] = mr_index; r[23] = mr_index >> 8;
    r[24] = tx_high | ((beep_mode == 0 ? 1 : beep_mode == 1 ? 0 : beep_mode) << 1);   /* bits 1-3: 0 style 1, 1 off, 2-4 style */ r[25] = tot_level + 1; r[26] = step_idx + 1;     /* N13; 0 in older records = default */
    r[27] = chan_mode;                                                 /* N14: MODE, other side (mr << 15 | channel) */
    {
        unsigned o = vfo_sel ^ 1; u16 v = (u16)((side_mr[o] ? 0x8000 : 0) | side_ch[o]);
        r[28] = v; r[29] = v >> 8;
    }
    c = crc16_ccitt(r, 30); r[30] = c; r[31] = c >> 8;
}
/* Apply a loaded record; each field is range-checked and falls back to the current default. */
static void settings_apply(const u8 *r)
{
    unsigned i, k;
    for (i = 0; i < 2; ++i) {
        u32 hz = get32(r + 4 + 4 * i);
        u16 v = r[14 + 2 * i] | (r[15 + 2 * i] << 8);
        struct mini_rf_tone t;
        if (rx_allowed(hz)) vfo_hz[i] = hz;
        t.kind = r[12 + i]; t.value = v;
        vfo_tone[i] = t.kind == MINI_TONE_NONE ? 0 : tone_index(t);
        (void)k;
    }
    tx_high = r[24] & 1;
    { u8 f = (r[24] >> 1) & 7; beep_mode = f == 0 ? 1 : f == 1 ? 0 : (f <= 4 ? f : 1); radio.beep_style = beep_mode; }
    if (r[25] >= 1 && r[25] <= 41) tot_level = r[25] - 1;
    if (r[26] >= 1 && r[26] <= 8) step_idx = r[26] - 1;
    if (r[18] <= 9) mini_rf_set_squelch(&radio, r[18]);
    vfo_sel = r[19] & 1;
    mr_index = (r[22] | (r[23] << 8)) % CHANNELS;
    mr_mode = r[20] == 1;
    if (r[27] <= MAX_BANDS) chan_mode = r[27];
    {
        unsigned o = vfo_sel ^ 1; u16 v = r[28] | (r[29] << 8);
        side_mr[o] = (v & 0x8000) != 0; side_ch[o] = (v & 0x7fff) % CHANNELS;
    }
}
/* N26 RESET: every setting back to its default (stored channels untouched), saved at once. */
static void settings_defaults(void)
{
    unsigned i;
    vfo_hz[0] = 146520000; vfo_hz[1] = 446000000; vfo_sel = 0; mr_mode = false; chan_mode = 0;
    for (i = 0; i < 2; ++i) {
        vfo_tone[i] = 0; vfo_txtone[i] = 0; vfo_shift[i] = 0; vfo_narrow[i] = false; vfo_busy[i] = false; side_mr[i] = false;
    }
    vfo_offset_khz[0] = 600; vfo_offset_khz[1] = 5000;
    tx_high = false; beep_mode = 1; radio.beep_style = 1; tot_level = 8; step_idx = 1;
    mic_gain = 0; tx_inhibit = false; ste_on = false; scan_mode = 1; ch_disp = 2; roger_mode = 0; rptrl = 0;
    light_mode = 2; autolock = 0; kf_side_sh = KF_POWER; kf_side_lg = KF_LIGHT; kf_ptta = KF_TEXT; kf_band_sh = KF_MODE;
    kf_band_lg = KF_NONE; flock = 0; boot_logo = true; boot_snd = true;
    mini_rf_set_squelch(&radio, 3);
    tune_vfo(vfo_hz[0]);
    settings_tick(true);
    ext_tick(true);
}
/* Save when the state differs from the last saved record and has been stable for ~3 s, or right away (now). */
static void settings_tick(bool now)
{
    static u8 pending[SET_SLOT];
    static unsigned age;
    u8 r[SET_SLOT];
    if (set_locked) return;
    settings_build(r);
    if (set_have && !memcmp(r, set_saved, SET_SLOT)) { age = 0; return; }
    if (memcmp(r, pending, SET_SLOT)) { memcpy(pending, r, SET_SLOT); age = 0; }
    if (!now && ++age < 12) return;
    age = 0;
    if (settings_write(r)) { memcpy(set_saved, r, SET_SLOT); set_have = true; if (set_next == 1) ext_have = false; }
    else set_have = false;
}
/* UHF index (MHz-400)/10 as stock 0x01E5080C; VHF (MHz-130)/3, above 14 -> 15, as 0x01E508A4. */
static u8 tx_target(u32 hz, bool high)
{
    unsigned mhz = hz / 1000000u, idx;
    if (hz >= 400000000u) { idx = (mhz - 400) / 10; if (idx > 15) idx = 15; return (high ? pa_high_uhf : pa_low_uhf)[idx]; }
    idx = mhz >= 130 ? (mhz - 130) / 3 : 0; if (idx > 14) idx = 15;
    return (high ? pa_high_vhf : pa_low_vhf)[idx];
}
static u32 tx_elapsed_s(void) { return radio.transmitting ? (jiffies_msec() - tx_start_ms) / 1000 : 0; }
/* N26 ROGER (5RM radio.rs, hz / 10 ms): ROGER, COIN, MORSE K, CHIRP. */
static const u16 rg_roger_hz[2] = {1000, 850}, rg_roger_ms[2] = {80, 80};
static const u16 rg_coin_hz[2] = {990, 1320}, rg_coin_ms[2] = {80, 350};
static const u16 rg_morse_hz[5] = {800, 0, 800, 0, 800}, rg_morse_ms[5] = {150, 50, 50, 50, 150};
static const u16 rg_chirp_hz[3] = {900, 1350, 1800}, rg_chirp_ms[3] = {40, 40, 60};
static void tx_end(const char *why)
{
    if (!why) {                                   /* a normal release: roger, STE, repeater tail (N26) */
        const u16 *hz = 0, *ms = 0; unsigned n = 0;
        if (roger_mode == 1) { hz = rg_roger_hz; ms = rg_roger_ms; n = 2; }
        else if (roger_mode == 2) { hz = rg_coin_hz; ms = rg_coin_ms; n = 2; }
        else if (roger_mode == 3) { hz = rg_morse_hz; ms = rg_morse_ms; n = 5; }
        else if (roger_mode == 4) { hz = rg_chirp_hz; ms = rg_chirp_ms; n = 3; }
        if (n || ste_on || rptrl) mini_rf_tx_tail(&radio, hz, ms, n, ste_on, rptrl * 100u);
    }
    mini_rf_tx_off(&radio);
    tune(tx_rx_hz);
    if (why) { tx_notice = why; set_notice(why); tx_hold = true; }
    screen();
}
static void ptt_service(void)
{
    static u8 ptt_count;
    bool ptt = !gpio_read(IO_PORTA_12);           /* PTT-B transmits; PTT-A opens TEXT (N14) */
    if (scr == SCR_APP) return;                   /* an app keys the transmitter itself */
    ptt_count = ptt ? (ptt_count < 3 ? ptt_count + 1 : 3) : 0;
    if (radio.transmitting) {
        if (!ptt || update_pending) { tx_end(update_pending ? "UPDATING" : 0); return; }
        if (tot_level && jiffies_msec() - tx_start_ms >= tot_level * 15000u) { tx_end("TIME OUT"); return; }
        if (rf_read(0) != 0x4829 || rf_read(0x30) != 0xc1fe) { ++tx_aborts; tx_end("TX ABORT RF CHECK"); return; }
        return;
    }
    if (!ptt) {
        tx_hold = false;
        tx_notice = 0;
        return;
    }
    if (scanning) scan_stop(0);                   /* N23: PTT ends a scan and talks on the current frequency */
    if (ptt_count < 2 || tx_hold || entry_len || update_pending) return;
    if (scr != SCR_STANDBY) { scr = SCR_STANDBY; settings_ui_reset(); screen_full(); }
    {
        u32 hz = mr_mode ? ch.tx_hz : vfo_tx_hz(vfo_sel);
        bool high = mr_mode ? ch.power == 0 : tx_high;
        u8 target;
        tx_hold = true;                                   /* until released, unless keyed below */
        if (tx_inhibit) { tx_notice = "TX INHIBITED"; set_notice(tx_notice); screen(); return; }
        if (!hz) { tx_notice = "RX ONLY"; set_notice(tx_notice); screen(); return; }
        if (!((hz >= 136000000u && hz <= 174000000u) || (hz >= 400000000u && hz <= 480000000u)) || !flock_allows(hz)) {
            tx_notice = "TX NOT ALLOWED"; set_notice(tx_notice); screen(); return;
        }
        if (cur_busy() && (radio.status & 2)) { tx_notice = "CHANNEL BUSY"; set_notice(tx_notice); screen(); return; }
        if (!pa_loaded || !radio.cal_valid) { tx_notice = "TX OFF: NO CAL"; set_notice(tx_notice); screen(); return; }
        target = tx_target(hz, high);
        tx_last_target = target;
        tx_rx_hz = radio.frequency_hz;
        if (radio.monitor) mini_rf_monitor(&radio, false);
        radio.tx_tone = mr_mode ? ch.txtone : tone_at(vfo_txtone[vfo_sel]);
        if (!mini_rf_tx_on(&radio, hz, target, high ? 0xff : 0xd7, mic_gain ? (u8)(mic_gain - 1) : mic_cal)) {
            ++tx_aborts;
            tune(tx_rx_hz);
            tx_notice = "TX REFUSED"; set_notice(tx_notice);
            screen();
            return;
        }
        tx_hold = false;
        tx_high_now = high;
        tx_start_ms = jiffies_msec();
        ++tx_count;
        screen();
    }
}
/* ==== App API (N14): the parts of bf5rh_abi::Api the ported apps use, on the Mini ==== */
extern unsigned long jiffies_half_msec(void);
static struct { u32 rx_hz, tx_hz; bool wide, high, tx_enabled, refused; u8 sql, mod; bool holds_rx; } ac;
static u8 abuf[512] __attribute__((aligned(4)));
static u32 api_uptime_100us(void) { return (u32)jiffies_half_msec() * 5; }
static void api_set_ptt(bool on);
static void api_delay_ms(u32 ms)
{
    unsigned long t0 = jiffies_half_msec(), want = ms * 2, el;
    /* N20 backstop inside an app's blocking transmission: 10 s at most, and never once an update is pending. */
    if (radio.transmitting && (update_pending || jiffies_msec() - tx_start_ms > 10000)) api_set_ptt(false);
    while ((el = jiffies_half_msec() - t0) < want) if (want - el >= 24) os_time_dly(1);
}
static void lcd_send(const u8 *b, unsigned n)
{
    unsigned i;
    if (spi_dma_send(SPI1, b, n) < 0) { ++spi_errors; for (i = 0; i < n; ++i) lcd_byte(b[i]); }
}
static void api_fill_rect(int x, int y, int w, int h, u16 c)
{
    unsigned n, i;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > 128) w = 128 - x;
    if (y + h > 128) h = 128 - y;
    if (w <= 0 || h <= 0) return;
    for (i = 0; i < sizeof(abuf); i += 2) { abuf[i] = c >> 8; abuf[i + 1] = c; }
    window(x, y, w, h);
    for (n = (unsigned)(w * h) * 2; n; ) { unsigned k = n > sizeof(abuf) ? sizeof(abuf) : n; lcd_send(abuf, k); n -= k; }
    put(LCD_CS, 1);
}
/* Text with background, top-left at (x, y), font 0 = 5x8, 1 = 6x10, 2 = 9x18; returns the next x. */
static int api_draw_text(int x, int y, const void *t, unsigned len, u16 fg, u16 bg, u8 font)
{
    const struct font *f = font == 2 ? &F9 : (font == 1 ? &F6 : &F5);
    const u8 *s = t;
    unsigned i;
    for (i = 0; i < len; ++i, x += f->w) {
        unsigned c = s[i], idx = (c < 0x20 || c > 0x7f) ? '?' - 0x20 : c - 0x20, row, col, k = 0;
        const u8 *g = f->bits + idx * f->h * f->stride;
        if (x < 0 || x + f->w > 128 || y < 0 || y + f->h > 128) continue;
        for (row = 0; row < f->h; ++row)
            for (col = 0; col < f->w; ++col) {
                u16 v = (g[row * f->stride + (col >> 3)] & (0x80 >> (col & 7))) ? fg : bg;
                abuf[k++] = v >> 8; abuf[k++] = v;
            }
        window(x, y, f->w, f->h);
        lcd_send(abuf, k);
        put(LCD_CS, 1);
    }
    return x;
}
static u16 api_fmt_freq(u32 hz, u8 *out)
{
    return (u16)sprintf((char *)out, "%u.%04u", (unsigned)(hz / 1000000u), (unsigned)(hz / 100u % 10000u));
}
static void api_set_rx_freq(u32 hz) { ac.rx_hz = hz; }
static void api_set_tx_freq(u32 hz) { ac.tx_hz = hz; }
static void api_set_power(u8 level) { ac.high = level == 0; }      /* 0 High; Mid and Low are LOW on the Mini */
static void api_set_subaudio_tx(u16 code) { (void)code; }          /* TX sub-audio: not ported yet (apps pass 0) */
static void api_set_subaudio_rx(u16 code) { (void)code; }
static void api_set_bandwidth(bool wide) { ac.wide = wide; }
static void api_set_modulation(u8 mode) { ac.mod = mode; }        /* 0 FM, 1 AM (N22); USB is received as FM */
static void api_set_sql_level(u8 level) { ac.sql = level > 9 ? 9 : level; }
/* Everything set so far reaches the chip. N22: apps may tune 18-660 MHz, narrow and AM; the firmware then runs the
 * squelch and speaker for the app (5RM APP_HOLDS_RX) until the app takes the audio with set_holds_rx(false). */
static void api_enter_rx(void)
{
    struct mini_rf_tone none = {MINI_TONE_NONE, 0};
    want_tone = none;
    if (ac.sql != radio.squelch) mini_rf_set_squelch(&radio, ac.sql);
    radio.wide_range = true;
    radio.narrow = !ac.wide;
    radio.am = ac.mod == 1;
    radio.hold_mute = false;
    radio.monitor = false;
    ac.holds_rx = true;
    tune(ac.rx_hz);
}
/* N23: handing RX back to the firmware also hands back the speaker (the main loop's squelch decides). */
static void api_set_holds_rx(bool on) { ac.holds_rx = on; if (on) { radio.hold_mute = false; radio.monitor = false; } }
static void api_set_af_out(bool on) { radio.hold_mute = !on; radio.monitor = on; mini_rf_poll(&radio); }
static void api_retune_rx(u32 hz, bool wide) { radio.narrow = !wide; mini_rf_retune(&radio, hz, -1); }
static void api_tune_search_candidate(u32 hz, bool uhf) { radio.narrow = false; mini_rf_retune(&radio, hz, uhf ? 1 : 0); }
/* REG 0x7E AGC (fd6818b set_agc_fix): rank 0-7 fixes the gain index, negative = automatic (0b011). */
static void api_set_agc_fix(int rank)
{
    static const u16 by_rank[8] = {3, 2, 1, 0, 7, 6, 5, 4};
    u16 base = rf_read(0x7e) & 0x0fff;
    rf_write(0x7e, rank < 0 ? (u16)((3u << 12) | base) : (u16)(0x8000 | (by_rank[rank > 7 ? 7 : rank] << 12) | base));
}
static bool api_squelch_open(void) { return (rf_read(0x0c) & 2) != 0; }
/* REG 0x51 0x0300: the wide sub-audio filter for detecting an unknown tone; off re-applies the configured tone. */
static void api_set_subaudio_scan_filter(bool on)
{
    rf_write(0x51, on ? 0x0300 : 0);
    if (!on && radio.rx_tone.kind != MINI_TONE_NONE) mini_rf_set_rx_tone(&radio, radio.rx_tone);
}
static void api_freq_scan_enable(void) { rf_write(0x32, 0x0244 | 1); }
static void api_freq_scan_disable(void) { rf_write(0x32, 0x0244); }
static u32 api_check_freq_scan(void)
{
    u16 hi = rf_read(0x0d);
    if (hi & 0x8000) return 0xFFFFFFFFu;
    return ((u32)(hi & 0x07ff) << 16) | rf_read(0x0e);
}
/* Inverse of the crystal correction mini_rf_word applies (fd6818b correct_measured_freq_word). */
static u32 api_correct_measured_freq_word(u32 raw)
{
    static const u8 xtal_table[17] = {40,35,30,25,20,15,10,5,0,5,10,15,20,25,30,35,40};
    unsigned idx = (radio.cal_valid && radio.cal.xtal <= 16) ? radio.cal.xtal : 8;
    u32 adjust = raw * xtal_table[idx] / 10000000u;
    return idx > 8 ? raw - adjust : raw + adjust;
}
/* Sub-audio index (5RM convert.rs): -1 none, 0-49 CTCSS, 50-154 DCS normal, 155-259 DCS inverted = tone_at(i + 1).
 * Code = the channel tone field (CTCSS tenths, DCS index + 1, inverted + 106). */
static u16 api_subaudio_code_of_index(int i)
{
    if (i < 0) return 0;
    if (i < 50) return ctcss50[i];
    if (i < 155) return (u16)(i - 50 + 1);
    if (i > 259) i = 259;
    return (u16)(i - 155 + 106);
}
static int api_subaudio_index_of_code(u16 code)
{
    struct mini_rf_tone t;
    if (!mini_rf_tone_from_channel(code, &t) || t.kind == MINI_TONE_NONE) return -1;
    return (int)tone_index(t) - 1;
}
static u16 api_subaudio_format(int i, u8 *out, u16 cap)
{
    char b[16]; unsigned n;
    tone_value(tone_at(i < 0 ? 0 : (unsigned)i + 1), b, sizeof(b));
    n = strlen(b); if (n > cap) n = cap;
    memcpy(out, b, n);
    return (u16)n;
}
/* The chip's tone decoder (fd6818b detect_subaudio_raw + find_standard_*): index, -1 nothing, -2 not a standard one. */
static int api_detect_subaudio(void)
{
    u16 dcs_hi = rf_read(0x69), cts;
    if (!(dcs_hi & 0x8000)) {
        u32 raw = (((u32)(dcs_hi & 0x0fff)) << 12) | (rf_read(0x6a) & 0x0fff);
        u8 by[5]; unsigned i, ff = 0, zz = 0, k, r;
        by[0] = raw; by[1] = raw >> 4; by[2] = raw >> 8; by[3] = raw >> 12; by[4] = raw >> 16;
        for (i = 0; i < 5; ++i) { ff += by[i] == 0xff; zz += by[i] == 0; }
        if (ff >= 4 || zz >= 4) return -1;
        raw &= 0x7fffff;
        for (k = 1; k <= 105; ++k) {
            struct mini_rf_tone t; u32 w;
            mini_rf_tone_from_channel((u16)k, &t);
            w = mini_rf_dcs_codeword(t.value) & 0x7fffff;
            for (r = 0; r < 23; ++r) { w = ((w << 1) | (w >> 22)) & 0x7fffff; if (w == raw) return 50 + (int)k - 1; }
        }
        return -2;
    }
    cts = rf_read(0x68);
    if (!(cts & 0x8000)) {
        u16 tenths = (u16)((u32)(cts & 0x1fff) * 100000u / 206489u); unsigned i;
        for (i = 0; i < 50; ++i) if ((tenths > ctcss50[i] ? tenths - ctcss50[i] : ctcss50[i] - tenths) < 10) return (int)i;
        return -2;
    }
    return -1;
}
static u16 api_fmt_u32(u32 v, u8 *out) { return (u16)sprintf((char *)out, "%lu", (unsigned long)v); }
/* RGB565 pixels to the LCD (big-endian on the wire), through the 512-byte DMA buffer. */
static void api_blit(int x, int y, int w, int h, const u16 *px)
{
    unsigned n, k = 0;
    if (x < 0 || y < 0 || w <= 0 || h <= 0 || x + w > 128 || y + h > 128) return;
    window(x, y, w, h);
    for (n = (unsigned)(w * h); n; --n) {
        abuf[k++] = *px >> 8; abuf[k++] = (u8)*px++;
        if (k == sizeof(abuf)) { lcd_send(abuf, k); k = 0; }
    }
    if (k) lcd_send(abuf, k);
    put(LCD_CS, 1);
}
static void api_set_speaker(bool on) { rf_speaker(on); }
static void api_set_backlight_hold(bool on) { (void)on; }
static void api_set_tx_enabled(bool on) { ac.tx_enabled = on; }
static bool tx_hz_allowed(u32 hz) { return (hz >= 136000000u && hz <= 174000000u) || (hz >= 400000000u && hz <= 480000000u); }
static void api_set_ptt(bool on)
{
    if (on && !radio.transmitting) {
        ac.refused = true;
        if (update_pending || !ac.tx_enabled || !tx_hz_allowed(ac.tx_hz) || !pa_loaded || !radio.cal_valid || ac.mod != 0) return;
        tx_rx_hz = ac.rx_hz;
        radio.tx_tone.kind = MINI_TONE_NONE;      /* apps send no sub-audio */
        if (!mini_rf_tx_on(&radio, ac.tx_hz, tx_target(ac.tx_hz, ac.high), ac.high ? 0xff : 0xd7, mic_cal)) {
            ++tx_aborts;
            tune(ac.rx_hz);
            return;
        }
        ac.refused = false;
        tx_high_now = ac.high;
        tx_start_ms = jiffies_msec();
        ++tx_count;
    } else if (!on && radio.transmitting) {
        mini_rf_tx_off(&radio);                    /* end_tx -> enter_rx */
        tune(ac.rx_hz);
    }
}
static u32 api_tx_state(void) { return (u32)radio.transmitting | ((u32)ac.refused << 1); }
static u16 api_read_rssi(void) { return (rf_read(0x67) & 0x1ff) >> 1; }
static u32 api_get_master_freq(void) { return mr_mode ? ch.rx_hz : vfo_hz[vfo_sel]; }
static bool api_get_master_wide(void) { return true; }
static u32 api_settings_get(u16 id) { return id == 0 ? radio.squelch : (id == 1 ? tot_level : 0); }
static bool api_nor_read(u32 addr, void *buf, u32 len) { return nor_read(addr, buf, len); }
static bool api_nor_write(u32 addr, const void *buf, u32 len)
{
    const u8 *b = buf;
    if (!app_in_region(addr, len)) return false;
    while (len) {
        u32 k = 256 - (addr & 0xff);
        if (k > len) k = len;
        if (!nor_program(addr, b, k)) return false;
        addr += k; b += k; len -= k;
    }
    return true;
}
static bool api_nor_erase_sector(u32 addr) { return nor_erase_app(addr); }
/* N22: the channel the apps hand back (5RM save_master_vfo / save_master_subaudio). The standby radio only receives
 * 136-174 / 400-520 MHz, so anything else is refused with a notice rather than left untunable. */
static const char *app_exit_notice;
static void api_save_master_vfo(u32 hz, u16 code)
{
    struct mini_rf_tone t;
    if (!rx_allowed(hz)) { app_exit_notice = "OUT OF BAND"; return; }
    vfo_hz[vfo_sel] = hz;
    vfo_tone[vfo_sel] = mini_rf_tone_from_channel(code, &t) ? tone_index(t) : 0;
    mr_mode = false; side_mr[vfo_sel] = false;
    app_exit_notice = "SAVED TO VFO";
}
static void api_save_master_subaudio(u16 code, bool also_tx)
{
    struct mini_rf_tone t;
    (void)also_tx;                                  /* no TX sub-audio on the Mini yet */
    if (!mini_rf_tone_from_channel(code, &t)) return;
    if (mr_mode) ch.tone = t;                       /* this session only, as the 5RM side cfg */
    else vfo_tone[vfo_sel] = tone_index(t);
    app_exit_notice = "TONE SAVED";
}
/* Channel slots (32 bytes at n * 32, the layout the UV Channels page writes). Empty = no valid receive frequency. */
static bool chan_changed;
static bool api_chan_read(u16 num, u8 *out, u16 len)
{
    u32 hz;
    if (num >= CHANNELS || len < 32 || !nor_read((u32)num * 32u, out, 32)) return false;
    return lbcd_hz(out, &hz);
}
/* Read-modify-write of the slot's 4 KB sector; a blank slot is programmed in place (no erase). Verified. */
static u8 chan_sector[4096];
static bool api_chan_write(u16 num, const u8 *buf, u16 len)
{
    u32 addr = (u32)num * 32u, sec = addr & ~0xfffu, i;
    u8 back[32];
    if (num >= CHANNELS || len < 32 || radio.transmitting) return false;
    if (!nor_read(addr, back, 32)) return false;
    chan_changed = true;
    if (all_ff(back, 32)) {
        if (!nor_program(addr, buf, 32)) return false;
    } else {
        if (!nor_read(sec, chan_sector, sizeof(chan_sector))) return false;
        memcpy(chan_sector + (addr - sec), buf, 32);
        if (!nor_erase_app(sec)) return false;
        for (i = 0; i < sizeof(chan_sector); i += 256)
            if (!all_ff(chan_sector + i, 256) && !nor_program(sec + i, chan_sector + i, 256)) return false;
    }
    return nor_read(addr, back, 32) && !memcmp(back, buf, 32);
}
/* Local beep (custom firmware BEEP_TONES 600 Hz / 60 ms for keys); TEXT chimes use the same path. */
static void api_beep(const u16 *hz, unsigned n, u32 ms)
{
    u16 len[4]; unsigned i;
    if (n > 4) n = 4;
    for (i = 0; i < n; ++i) len[i] = (u16)ms;
    mini_rf_beep(&radio, hz, len, n);
}
static void modem_rx(void);
static void key_beep(void)
{
    static const u16 hz = 600, ms = 60;
    if (!beep_mode || radio.audible || radio.transmitting) return;
    mini_rf_beep(&radio, &hz, &ms, 1);
    if (scr == SCR_APP && app_cur == AP_TEXT) modem_rx();   /* the beep cycles REG 0x30: put TEXT's modem back to listening */
}
#include "app_text.h"
#include "app_shared.h"
#include "app_fox.h"
#include "app_search.h"

static u32 text_diag(void) { return text_counters(); }
static u32 text_diag2(void) { return text_counters2(); }
static u32 text_window(unsigned i)
{
    unsigned n = t_rec_n < 8 ? t_rec_n : 8;
    if (i >= n) return 0;
    return t_rec[(t_rec_n - n + i) % 8];
}
/* N21: TEXT was written and tuned on the UV-5RM, whose driver (fd6818b.rs init/apply_modulation for FM) sets these
 * baseband receive registers differently from the Mini's stock init. Same chip (BK4829); none of them is a PLL/VCO or
 * board-specific RF setting. While TEXT is open the Mini uses the 5RM values; its own are saved first and restored on
 * exit, so voice keeps the stock configuration.
 *   0x73 AFC 0x4681 (Mini 0x6681: bit 13 differs)     0x3D IF select 0x2AAB (8.46 kHz; the Mini never writes it)
 *   0x46 0x600A (Mini 0x6050)   0x4F 0x2F2E (0x2A28)   0x26 0x13A0   0x4A 0x5430   0x2C 0x1822 (0x0022) */
/* TEXT receive settings: the UV-5RM driver's baseband receive values (TEXT was written and tuned on the 5RM), the
 * Mini's own values saved on open and restored on exit. Never PLL/VCO registers (0x1C/0x1D/0x1F stay the Mini's).
 *   0x73 AFC 0x4681   0x3D IF 0x2AAB   0x46 0x600A   0x4F 0x2F2E   0x26 0x13A0   0x4A 0x5430   0x2C 0x1822
 *   0x2B RX audio filter 0x0300 (5RM rx_on; N32: 0/5 -> 4/5 confirmed)
 * N32-N34 profile tests: the 5RM AGC values (0x13/0x49/0x2A), its volume (0x48) and the chip's power-on values of the
 * Mini-only init registers gave no better result. After a flat battery was replaced: 5/5 confirmed with this set (N34 P0)
 * -- a nearly flat battery had been skewing the earlier rounds. */
#define TEXT_RX_REGS 8
static const struct { u8 reg; u16 val; } text_rx_regs[TEXT_RX_REGS] = {
    {0x73, 0x4681}, {0x3D, 0x2AAB}, {0x46, 0x600A}, {0x4F, 0x2F2E}, {0x26, 0x13A0}, {0x4A, 0x5430}, {0x2C, 0x1822},
    {0x2B, 0x0300},
};
static u16 text_rx_saved[TEXT_RX_REGS];
static bool text_rx_applied;
static u16 orig_3d, orig_73;
static u32 text_regs_diag(void) { return ((u32)orig_3d << 16) | orig_73; }
static void text_rx_apply(void)
{
    unsigned i;
    if (!text_rx_applied) {
        for (i = 0; i < TEXT_RX_REGS; ++i) text_rx_saved[i] = rf_read(text_rx_regs[i].reg);
        orig_73 = text_rx_saved[0]; orig_3d = text_rx_saved[1];
    }
    for (i = 0; i < TEXT_RX_REGS; ++i) rf_write(text_rx_regs[i].reg, text_rx_regs[i].val);
    text_rx_applied = true;
}
static void text_rx_restore(void)
{
    unsigned i;
    if (!text_rx_applied) return;
    for (i = 0; i < TEXT_RX_REGS; ++i) rf_write(text_rx_regs[i].reg, text_rx_saved[i]);
    text_rx_applied = false;
}
/* ---- app framework (N22): one app at a time owns the screen and the radio ---- */
static void text_app_enter(void) { text_enter(); text_rx_apply(); }
static int text_app_key(u8 id, u8 kind) { return text_key(id, kind) ? APP_EXIT : APP_CONTINUE; }
static void text_app_leave(void) { text_leave(); text_rx_restore(); }
struct app_def { void (*enter)(void); int (*key)(u8, u8); void (*tick)(u32); void (*draw)(void); void (*leave)(void); };
static const struct app_def apps[AP_COUNT] = {
    [AP_TEXT] = {text_app_enter, text_app_key, text_tick, text_draw, text_app_leave},
    [AP_FOX] = {fox_enter, fox_key, fox_tick, fox_draw, fox_leave},
    [AP_SEARCH] = {search_enter, search_key, search_tick, search_draw, search_leave},
    [AP_QT] = {qtscan_enter, qtscan_key, qtscan_tick, qtscan_draw, qtscan_leave},
};
static u32 app_last_100us;
static unsigned app_launch;                  /* launcher entry that is open (segments) */
/* Registers an app may change that a re-tune does not rewrite: the frequency counter and the AGC. */
static u16 app_saved_32, app_saved_7e;
static void app_start(unsigned id)
{
    app_cur = (u8)id;
    apps[id].enter();
    app_last_100us = api_uptime_100us();
    apps[id].draw();
}
static void app_open_entry(unsigned launch)
{
    if (radio.transmitting || launch >= LA_COUNT || launch_list[launch].seg[0] >= AP_COUNT) return;
    if (radio.monitor) mini_rf_monitor(&radio, false);
    memset(&ac, 0, sizeof(ac));
    ac.sql = radio.squelch;
    ac.rx_hz = ac.tx_hz = api_get_master_freq();
    ac.wide = true;
    app_launch = launch;
    sq_combined = launch_list[launch].combined;
    app_exit_notice = 0;
    chan_changed = false;
    app_saved_32 = rf_read(0x32);
    app_saved_7e = rf_read(0x7e);
    scr = SCR_APP;
    app_start(launch_list[launch].seg[0]);
}
/* Leave the running app and give the radio back (also used at power-off). */
static void app_leave_current(void)
{
    apps[app_cur].leave();
    if (radio.transmitting) mini_rf_tx_off(&radio);
    rf_write(0x51, 0);
    rf_write(0x32, app_saved_32);
    rf_write(0x7e, app_saved_7e);
    radio.hold_mute = false;
    radio.monitor = false;
    radio.wide_range = false;
    radio.narrow = false;
    radio.am = false;
    ac.holds_rx = false;
}
static void app_close(void)
{
    app_leave_current();
    scr = SCR_STANDBY;
    radio.frequency_hz = 0;                  /* N26: a full tune after an app, not the quiet hop */
    if (mr_mode) tune_channel(); else if (!tune_vfo(vfo_hz[vfo_sel])) tune(146520000);
    if (chan_changed) { chan_changed = false; cps_reload = true; }   /* reload channels / bands in the main loop */
    if (app_exit_notice) set_notice(app_exit_notice);
    screen_full();
}
static void app_result(int r)
{
    if (r == APP_EXIT) { app_close(); return; }
    if (r >= APP_CHAIN(0)) {
        unsigned seg = (unsigned)r & 0x0f, next = seg < 2 ? launch_list[app_launch].seg[seg] : 0xff;
        if (next >= AP_COUNT) return;
        apps[app_cur].leave();
        app_start(next);
    }
}
/* Mini keys to the custom firmware's app key ids; the side key is SIDE1. */
static void app_key(u8 key, u8 kind)
{
    int d = key_digit(key);
    u8 id = d >= 0 ? (u8)d : key == K_STAR ? AK_STAR : key == K_HASH ? AK_POUND : key == K_MENU ? AK_MENU :
            key == K_EXIT ? AK_EXIT : key == K_UP ? AK_UP : key == K_DOWN ? AK_DOWN : key == K_VFO_MR ? AK_VM :
            key == K_AB ? AK_AB : key == K_MINUS ? AK_BAND : key == K_SIDE_EQ ? AK_SIDE1 : T_NO_KEY;
    int r;
    if (id == T_NO_KEY) return;
    r = apps[app_cur].key(id, kind);
    if (r != APP_CONTINUE) { app_result(r); return; }
    apps[app_cur].draw();
}
static void app_service(void)
{
    static unsigned polls;
    u32 now = api_uptime_100us();
    /* N20 backstop: an app transmission never lasts more than 10 s, nor continues once an update is pending. */
    if (radio.transmitting && (update_pending || jiffies_msec() - tx_start_ms > 10000)) api_set_ptt(false);
    apps[app_cur].tick(now - app_last_100us);
    app_last_100us = now;
    if (scr != SCR_APP) return;
    /* The firmware runs squelch and speaker while the app lets it (enter_rx until set_holds_rx(false)). */
    if (ac.holds_rx && !radio.transmitting && !(++polls % 4)) mini_rf_poll(&radio);
    apps[app_cur].draw();
}
/* N24: backlight (PB7) off after 10 s without a key, except while an app is open (apps stay lit), while
 * transmitting, receiving a signal or in a PC programming session. A key pressed on a dark screen only lights it. */
static bool light_on = true;
static u32 light_ms, key_ms;
static void light_wake(void)
{
    light_ms = jiffies_msec();
    if (!light_on) { put(LCD_LIGHT, 1); light_on = true; }
}
static void light_service(void)
{
    if (scr == SCR_APP || scr == SCR_PROG || scr == SCR_BOOT || radio.transmitting || radio.audible || !gpio_read(IO_PORTA_12)) light_wake();
    else if (light_on && light_mode && jiffies_msec() - light_ms >= light_s[light_mode] * 1000u) { put(LCD_LIGHT, 0); light_on = false; }
    /* N26 AUTO LOCK: the keypad locks after the set time without a key on the main screen; hold * unlocks. */
    if (autolock && !keys_locked && scr == SCR_STANDBY && !scanning && jiffies_msec() - key_ms >= autolock_s[autolock] * 1000u) {
        keys_locked = true; set_notice("KEYS LOCKED"); screen();
    }
}
static void bands_load(void)
{
    u8 r[32]; unsigned i, k;
    band_count = 0;
    for (i = 0; i < MAX_BANDS; ++i) {
        if (!nor_read((1000u + i) * 32u, r, 32) || r[31] != 'B') break;
        for (k = 0; k < 16 && r[k] && r[k] != 0xff; ++k) band_name[i][k] = (r[k] >= 32 && r[k] < 127) ? r[k] : '?';
        band_name[i][k] = 0;
        for (k = 0; k < 5 && r[16 + k] && r[16 + k] != 0xff; ++k) band_short[i][k] = (r[16 + k] >= 32 && r[16 + k] < 127) ? r[16 + k] : '?';
        band_short[i][k] = 0;
        ++band_count;
    }
    if (chan_mode > band_count) chan_mode = 0;
}
/* ==== UV Channels page link (N15) ====
 * 0x72 program {addr u32, len u16, data<=256}, 0x73 erase {addr u32}, 0x74 reload {}; replies {addr, [len,] status}.
 * status: 0 ok, 1 refused (address), 2 busy (transmitting or an app is open), 3 failed (read-back mismatch).
 * The UART task only copies a request; the NOR (shared with the LCD) is touched in the main loop. */
static volatile u32 cps_pending;
static u8 cps_cmd, cps_buf[256];
static u32 cps_addr;
static unsigned cps_len;
static bool cps_reload;
void mini_nor_write_request(u8 cmd, const u8 *p, unsigned n)
{
    u32 addr;
    if (cps_pending) return;
    addr = n >= 4 ? (p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24)) : 0;
    if (cmd == 0x72) {
        unsigned len;
        if (n < 6) return;
        len = p[4] | (p[5] << 8);
        if (!len || len > 256 || n != 6 + len) return;
        memcpy(cps_buf, p + 6, len);
        cps_len = len;
    } else if (cmd == 0x73) { if (n != 4) return; }
    else if (cmd == 0x74) { if (n != 0) return; }
    else return;
    cps_cmd = cmd; cps_addr = addr;
    __asm__ volatile("" ::: "memory");
    cps_pending = 1;
}
static void cps_service(void)
{
    extern bool uart_update_cmd(u8, u8 *, u32);
    u8 r[8], back[256], status = 0;
    unsigned k = 0, i;
    if (!cps_pending) return;
    if (radio.transmitting || scr == SCR_APP) status = 2;
    else if (cps_cmd == 0x72) {
        if (!cps_in_region(cps_addr, cps_len) || (cps_addr & 0xff) + cps_len > 256) status = 1;
        else if (!nor_program(cps_addr, cps_buf, cps_len) || !nor_read(cps_addr, back, cps_len) || memcmp(back, cps_buf, cps_len)) status = 3;
    } else if (cps_cmd == 0x73) {
        if ((cps_addr & 0xfff) || !cps_in_region(cps_addr, 0x1000)) status = 1;
        else if (!nor_erase_app(cps_addr)) status = 3;
        else for (i = 0; i < 0x1000 && !status; i += 256)
            if (!nor_read(cps_addr + i, back, 256) || !all_ff(back, 256)) status = 3;
    } else cps_reload = true;
    if (cps_cmd != 0x74) { r[0] = cps_addr; r[1] = cps_addr >> 8; r[2] = cps_addr >> 16; r[3] = cps_addr >> 24; k = 4; }
    if (cps_cmd == 0x72) { r[k++] = cps_len; r[k++] = cps_len >> 8; }
    r[k++] = status;
    uart_update_cmd(cps_cmd, r, k);
    cps_pending = 0;
}
/* ==== PC channel app link (N16): the UV-5RM custom firmware's programming session, unchanged on the wire ====
 * Handshake "PROGRAMBF-5RH" -> 0x06, then frames A6 cmd addr(BE16) len(BE16) data CRC16-XMODEM(BE) over cmd..data;
 * error reply cmd 0xEE {code}. Commands as cps/mod.rs: 0x52 read channel slot 0-1015 (32 B at slot*32, same NOR
 * layout), 0x57 write a slot (the first write in a session to a 4 KB sector erases it), 0x44 raw read 64 B at
 * addr*64 (the 5RM quick-message area 0x194000 maps to the Mini's 0xF7000), 0x53 quick messages (512 B chunks,
 * offset 0 erases first; the Mini keeps them in one sector), 0x45 end. Idle 2 s ends the session.
 * The page talks at 115200 while the updater lives at 9600, so the idle port alternates 450 ms at 9600 and 330 ms at
 * 115200; any valid updater frame holds 9600 for 10 s, and a handshake at 115200 holds it for the session.
 * Holding the side key while switching on keeps the port at 9600 (no alternation) as a fallback. */
static const char cps_hs[] = "PROGRAMBF-5RH";
static volatile u8 cps_state;              /* 0 idle, 1 handshake seen, 2 session */
static volatile u32 hop_hold_until;
static volatile u32 hop_baud = 9600;
static u32 hop_last, cps_sessions, cps_frames;
static bool hop_disabled;
static volatile u8 hs_match, cf_ready, cf_crcerr;
static u8 cf_cmd; static u16 cf_addr, cf_len, cf_err_addr; static u8 cf_data[512];
static u8 fp_state, fp_crchi; static u16 fp_idx, fp_need; static u8 fp_buf[5 + 512];
extern void mini_uart_raw_send(const u8 *buf, u32 len);
extern void mini_uart_hw_baud(u32 baud);
extern u32 mini_uart_update_baud(void);
u32 mini_uart_hw_baud_now(void) { return hop_baud; }
/* UART receive context: only flags and copies. */
void mini_uart_frame_seen(void) { hop_hold_until = jiffies_msec() + 10000; }
int mini_cps_feed(const u8 *b, u16 n)
{
    u16 i;
    if (cps_state == 0) {
        if (hop_baud != 115200) return 0;
        for (i = 0; i < n; ++i) {
            if (b[i] == (u8)cps_hs[hs_match]) {
                if (++hs_match == sizeof(cps_hs) - 1) { hs_match = 0; fp_state = 0; cps_state = 1; return 1; }
            } else hs_match = b[i] == (u8)cps_hs[0];
        }
        return 0;
    }
    for (i = 0; i < n; ++i) {
        u8 c = b[i];
        switch (fp_state) {
        case 0: if (c == 0xA6) { fp_state = 1; fp_idx = 0; } break;
        case 1:
            fp_buf[fp_idx++] = c;
            if (fp_idx == 5) {
                u16 len = (u16)(fp_buf[3] << 8 | fp_buf[4]);
                if (len > 512) { fp_state = 0; break; }
                fp_need = 5 + len;
                fp_state = len ? 2 : 3;
            }
            break;
        case 2: fp_buf[fp_idx++] = c; if (fp_idx == fp_need) fp_state = 3; break;
        case 3: fp_crchi = c; fp_state = 4; break;
        default:
            fp_state = 0;
            if (cf_ready) break;                       /* the PC waits for each reply, so this doesn't happen */
            if ((u16)(fp_crchi << 8 | c) == crc16_ccitt(fp_buf, fp_need)) {
                cf_cmd = fp_buf[0];
                cf_addr = (u16)(fp_buf[1] << 8 | fp_buf[2]);
                cf_len = fp_need - 5;
                memcpy(cf_data, fp_buf + 5, cf_len);
                cf_ready = 1;
            } else { cf_err_addr = (u16)(fp_buf[1] << 8 | fp_buf[2]); cf_crcerr = 1; }
            break;
        }
    }
    return 1;
}
static void hop_service(void)
{
    u32 now = jiffies_msec();
    if (cps_state) return;
    if (hop_disabled || mini_uart_update_baud() != 9600 || (int)(hop_hold_until - now) > 0) {
        if (hop_baud != 9600) { hop_baud = 9600; mini_uart_hw_baud(9600); }
        hop_last = now;
        return;
    }
    if (now - hop_last >= (hop_baud == 9600 ? 450u : 330u)) {
        hs_match = 0;
        hop_baud = hop_baud == 9600 ? 115200 : 9600;
        mini_uart_hw_baud(hop_baud);
        hop_last = now;
    }
}
static u32 link_diag(void)
{
    return (hop_baud == 115200) | ((u32)hop_disabled << 1) | ((u32)cps_state << 2) | ((cps_sessions & 0xff) << 8)
           | ((cps_frames & 0xffff) << 16);
}
static void cps_reply(u8 cmd, u16 addr, const u8 *d, u16 n)
{
    static u8 out[8 + 512];
    u16 c;
    out[0] = 0xA6; out[1] = cmd; out[2] = addr >> 8; out[3] = addr; out[4] = n >> 8; out[5] = n;
    if (n) memcpy(out + 6, d, n);
    c = crc16_ccitt(out + 1, 5 + n);
    out[6 + n] = c >> 8; out[7 + n] = c;
    mini_uart_raw_send(out, 8 + n);
}
static void cps_err(u16 addr, u8 code) { cps_reply(0xEE, addr, &code, 1); }
#define CPS_SLOTS 1016
#define CPS_QUICK_5RM 0x194000u
#define CPS_QUICK_MINI 0xF7000u
static void cps_session(void)
{
    u32 last;
    u8 erased = 0, buf[64];
    bool wrote = false, done = false;
    if (radio.transmitting) { cps_state = 0; return; }
    if (scr == SCR_APP) app_close();
    cps_state = 2;
    ++cps_sessions;
    radio.hold_mute = true;
    mini_rf_poll(&radio);
    scr = SCR_PROG;
    screen_full();
    { static const u8 ack = 0x06; mini_uart_raw_send(&ack, 1); }
    last = jiffies_msec();
    while (!done) {
        if (cf_crcerr) { cf_crcerr = 0; cps_err(cf_err_addr, 0x01); last = jiffies_msec(); }
        if (!cf_ready) {
            if (jiffies_msec() - last > 2000 || gpio_read(IO_PORTB_11)) break;
            os_time_dly(1);
            continue;
        }
        ++cps_frames;
        switch (cf_cmd) {
        case 0x52:                                       /* read a channel slot */
            if (cf_addr % 32 || cf_addr / 32 >= CPS_SLOTS || !nor_read(cf_addr, buf, 32)) cps_err(cf_addr, 0x02);
            else cps_reply(0x52, cf_addr, buf, 32);
            break;
        case 0x57: {                                     /* write a channel slot */
            unsigned sector = cf_addr / 4096;
            if (cf_addr % 32 || cf_addr / 32 >= CPS_SLOTS || cf_len != 32) { cps_err(cf_addr, 0x02); break; }
            if (!(erased & (1u << sector))) {
                if (!nor_erase_app(sector * 4096u)) { cps_err(cf_addr, 0x02); break; }
                erased |= 1u << sector;
            }
            if (!nor_program(cf_addr, cf_data, 32)) { cps_err(cf_addr, 0x02); break; }
            wrote = true;
            cps_reply(0x57, cf_addr, 0, 0);
            break;
        }
        case 0x44: {                                     /* raw read, 64 B units */
            u32 a = (u32)cf_addr * 64u;
            if (a >= CPS_QUICK_5RM && a < CPS_QUICK_5RM + 0x1000u) a = CPS_QUICK_MINI + (a - CPS_QUICK_5RM);
            if (a + 64 > 0x200000u || !nor_read(a, buf, 64)) cps_err(cf_addr, 0x02);
            else cps_reply(0x44, cf_addr, buf, 64);
            break;
        }
        case 0x53: {                                     /* quick messages */
            u32 off = cf_addr;
            if (off % 512 || cf_len != 512 || off + 512 > 0x1000u) { cps_err(cf_addr, 0x02); break; }
            if (off == 0 && !nor_erase_app(CPS_QUICK_MINI)) { cps_err(cf_addr, 0x02); break; }
            if (!nor_program(CPS_QUICK_MINI + off, cf_data, 256) || !nor_program(CPS_QUICK_MINI + off + 256, cf_data + 256, 256)) {
                cps_err(cf_addr, 0x02); break;
            }
            cps_reply(0x53, cf_addr, 0, 0);
            break;
        }
        case 0x45:                                       /* end */
            cps_reply(0x45, cf_addr, 0, 0);
            done = true;
            break;
        default:
            cps_err(cf_addr, 0x03);
            break;
        }
        cf_ready = 0;
        last = jiffies_msec();
    }
    os_time_dly(2);                                      /* let the last reply leave before the port changes */
    hs_match = 0;
    hop_baud = 9600;
    mini_uart_hw_baud(9600);
    hop_last = jiffies_msec();
    cps_state = 0;
    radio.hold_mute = false;
    scr = SCR_STANDBY;
    if (wrote) cps_reload = true;
    mini_rf_poll(&radio);
    screen_full();
}
/* N20: called by the SDK updater right before it hands the radio to the loader. The loader runs none of this firmware,
 * so a transmitter still keyed at that moment would stay keyed for the whole update. Block new transmissions, wait up
 * to 8 s for one in progress to end (TEXT's longest is about 3.5 s; PTT-B release or TOT ends voice), and refuse the
 * hand-over (the PC tool then times out and can be run again) if the radio is still keyed. */
int mini_update_prepare(void)
{
    u32 t0 = jiffies_msec();
    update_pending = true;
    while (radio.transmitting && jiffies_msec() - t0 < 8000) os_time_dly(1);
    if (radio.transmitting) { update_pending = false; return 0; }
    return 1;
}
void mini_app_main(void)
{
    unsigned i, ticks = 0, stable = 0; u8 prev = 0xff, shown = 0xfe; u32 boot_ms, down_ms = 0, ptta_ms = 0;
    bool long_done = false, ptta_prev = true, shown_long = false;
    u32 rep_ms = 0;
    /* Keep the power latch alive and external flash deselected. */
    output(IO_PORTB_10, 1); output(IO_PORTA_03, 1);
    for (i = 0; i < 4; ++i) { output(IO_PORTA_06+i, 1); input(IO_PORTC_02+i); }
    input(IO_PORTA_04); input(IO_PORTA_12); input(IO_PORTB_11);
    output(RF_CS, 1); output(RF_CLK, 0); output(RF_DATA, 0);
    output(IO_PORTB_08, 0); output(IO_PORTB_09, 0); output(IO_PORTC_07, 1);   /* N30: PB8 audio power driven low */
    flashlight(false);
    hop_disabled = keys() == K_SIDE_EQ;    /* side key held at power-on: PC port stays at 9600 */
    lcd_init();
    scr = SCR_BOOT;
    render(0, 128);                        /* Blue Angel, no static effect */
    boot_ms = jiffies_msec();
    nor_identify();
    mini_rf_init(&radio, &radio_io);
    native_chip = radio.chip_id;
    load_calibration();
    battery_init();
    set_have = settings_scan(set_saved);
    if (set_have) settings_apply(set_saved);
    ext_have = ext_scan(ext_saved);          /* N26 */
    if (ext_have) ext_apply(ext_saved);
    bands_load();
    if (side_mr[vfo_sel ^ 1] && !channel_load(side_ch[vfo_sel ^ 1], &ch_other)) side_mr[vfo_sel ^ 1] = false;
    if (mr_mode && channel_load(mr_index, &ch)) tune_channel();
    else {
        mr_mode = false;
        if (!tune_vfo(vfo_hz[vfo_sel])) tune(146520000);
    }
    side_mr[vfo_sel] = mr_mode; side_ch[vfo_sel] = mr_index;
    if (boot_snd) {                          /* N28: the angel harp: four strums, each higher (C, F, G, high C) */
        static const u16 strums[4][4] = {
            {523, 659, 784, 1047}, {698, 880, 1047, 1397}, {784, 988, 1175, 1568}, {1047, 1319, 1568, 2093},
        };
        static const u16 ring[4] = {320, 320, 320, 800};   /* N29: slower picks (120 ms), rings trimmed: about 3.2 s */
        mini_rf_harp(&radio, strums, 4, 120, ring);
    }
    while (boot_logo && jiffies_msec() - boot_ms < 3000) os_time_dly(5);   /* N28: boot screen 3 s (was 1.5 s) */
    scr = SCR_STANDBY;
    light_wake(); key_ms = jiffies_msec();
    if (set_locked) set_notice("SAVE OFF");
    else if (hop_disabled) set_notice("PC LINK 9600 ONLY");
    screen_full();
    for (;;) {
        nor_service();
        cps_service();
        hop_service();
        if (cps_state == 1) cps_session();
        if (cps_reload && scr != SCR_APP && !radio.transmitting) {   /* the page wrote channels / bands */
            cps_reload = false;
            bands_load();
            if (side_mr[vfo_sel ^ 1] && !channel_load(side_ch[vfo_sel ^ 1], &ch_other)) side_mr[vfo_sel ^ 1] = false;
            if (mr_mode) { if (channel_load(mr_index, &ch)) tune_channel(); else if (!channel_step(1, true)) { mr_mode = false; tune_vfo(vfo_hz[vfo_sel]); } }
            side_mr[vfo_sel] = mr_mode; side_ch[vfo_sel] = mr_index;
            set_notice("CHANNELS UPDATED");
            if (scr == SCR_STANDBY) screen_full(); else screen();
        }
        u8 key = keys();
        native_key = key;
        native_ticks = ticks;
        if (key == prev) { if (stable < 3) ++stable; }
        else { stable = 0; prev = key; }
        if (stable == 3 && key != shown) {
            /* release of a key with a hold action, not held: that was a click */
            if (shown < 0xfe && shown_long && !long_done && !radio.transmitting) { key_beep(); key_action(shown); }
            shown = key; down_ms = jiffies_msec(); long_done = false; rep_ms = down_ms + 500;
            shown_long = key < 0xfe && has_long(key);        /* N22: decided at press (MENU differs in apps) */
            if (key < 0xfe && !light_on) {                   /* N24: a key on a dark screen only lights it */
                light_wake(); long_done = true; shown_long = true;
            } else if (key < 0xfe && !radio.transmitting) { last_key = key; if (!shown_long) { key_beep(); key_action(key); } }
            if (key < 0xfe) { light_wake(); key_ms = jiffies_msec(); }
        }
        if (shown < 0xfe) light_wake();                     /* a held key keeps the light on */
        if (shown < 0xfe && shown_long && !long_done && jiffies_msec() - down_ms >= 800) {
            long_done = true;
            if (!radio.transmitting) { key_beep(); key_long(shown); }
        }
        /* N22: UP/DOWN held in an app repeat (5RM KIND_REPEAT) after 0.5 s, every 120 ms. */
        if (scr == SCR_APP && (shown == K_UP || shown == K_DOWN) && !long_done && !radio.transmitting && (int)(jiffies_msec() - rep_ms) >= 0) {
            rep_ms = jiffies_msec() + 120;
            app_key(shown, KIND_REPEAT);
        }
        {   /* PTT-A opens / closes TEXT (the 5RM's CALL key) */
            bool a = !gpio_read(IO_PORTA_04);
            if (a && !ptta_prev && !radio.transmitting && jiffies_msec() - ptta_ms > 300) {
                ptta_ms = jiffies_msec();
                light_wake();
                if (scr == SCR_APP) app_close();
                else if (scr == SCR_STANDBY && !keys_locked) keyfn_run(kf_ptta);   /* N26 PTT-A function (default TEXT) */
            }
            ptta_prev = a;
        }
        scan_service();
        ptt_service();
        light_service();
        battery_service();
        if (scr == SCR_APP) app_service();
        else if (!(ticks++ % 10)) {
            rx_status();
            if (!radio.transmitting) { settings_tick(false); ext_tick(false); }
            screen();
        }
        if (gpio_read(IO_PORTB_11)) {
            if (scr == SCR_APP) app_leave_current();
            flashlight(false);
            settings_tick(true);           /* power latch still held: flush any unsaved change */
            ext_tick(true);
            mini_rf_stop(&radio);
            put(LCD_LIGHT, 0); put(IO_PORTB_10, 0);
            for (;;) os_time_dly(10);
        }
        /* Always yield: updater and system tasks must run without key input. */
        os_time_dly(2);
    }
}
