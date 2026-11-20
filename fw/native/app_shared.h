/* N22: shared pieces for the apps ported from the UV-5RM custom firmware (firmware-source/apps/...).
 * Included into mini_app.c (and the host tests) after the api_* functions. Each app is a set of static functions
 * X_enter / X_key / X_tick / X_draw / X_leave, as the 5RM AppEvent Enter / Key / Tick / Draw / Leave.
 * The 5RM screen is 160 x 128; the Mini's is 128 x 128, so every layout below is re-fitted to 128 px
 * (F5 = 25 columns, F6 = 21, F9 = 14). The red 5RM accents are the Mini's electric blue, as in TEXT. */
#ifndef APP_SHARED_H
#define APP_SHARED_H

#define APP_CONTINUE 0
#define APP_EXIT 1
#define APP_CHAIN(n) (0x10 | (n))

/* Key ids (bf5rh_abi / app/overlay.rs) and kinds. */
#ifndef KIND_SINGLE
#define KIND_SINGLE 2
#endif
#ifndef KIND_LONG
#define KIND_LONG 3
#endif
#define KIND_REPEAT 4
#define AK_STAR 10
#define AK_POUND 11
#define AK_MENU 12
#define AK_EXIT 13
#define AK_UP 14
#define AK_DOWN 15
#define AK_VM 16
#define AK_AB 17
#define AK_BAND 18
#define AK_SIDE1 19
#define AK_SIDE2 20

#define A_F5 0
#define A_F6 1
#define A_F9 2
#define A_W 128
#define A_H 128

#define A_BLACK 0x0000
#define A_WHITE 0xFFFF
#define A_GRAY 0x8410
#define A_GRID 0x2124
#define A_YELLOW 0xFFE0
#define A_GREEN 0x07E0
#define A_RED 0xF800
#define A_CYAN 0x07FF
#define A_TITLE 0x049f      /* 5RM TITLE_BG 0x5800 -> the Mini's electric blue */
#define A_SELECT 0x049f     /* 5RM SELECT_BG 0x8000 */
#define A_DARKSEL 0x0230    /* a quieter selection for lists drawn over graphs */

#define RSSI_DBM_BASE 170   /* the apps' dBm = raw - 170 (5RM fd6818b get_rssi units) */

/* The 5RM apps' Buf: a bounded text buffer. */
struct sbuf { char b[40]; unsigned n; };
static void ab_init(struct sbuf *b) { b->n = 0; b->b[0] = 0; }
static void ab_ch(struct sbuf *b, char c) { if (b->n < sizeof(b->b) - 1) { b->b[b->n++] = c; b->b[b->n] = 0; } }
static void ab_s(struct sbuf *b, const char *s) { while (*s) ab_ch(b, *s++); }
static void ab_u(struct sbuf *b, u32 v)
{
    char t[12]; int i = 0;
    do { t[i++] = (char)('0' + v % 10); v /= 10; } while (v && i < 11);
    while (i) ab_ch(b, t[--i]);
}
static void ab_i(struct sbuf *b, int v) { if (v < 0) { ab_ch(b, '-'); ab_u(b, (u32)-v); } else ab_u(b, (u32)v); }
static void ab_freq(struct sbuf *b, u32 hz)
{
    u8 t[16]; unsigned n = api_fmt_freq(hz, t), i;
    for (i = 0; i < n && i < sizeof(t); ++i) ab_ch(b, (char)t[i]);
}
static void ab_tone(struct sbuf *b, int idx)
{
    u8 t[16]; unsigned n = api_subaudio_format(idx, t, sizeof(t)), i;
    for (i = 0; i < n; ++i) ab_ch(b, (char)t[i]);
}
static void a_text(int x, int y, const char *s, u16 fg, u16 bg, u8 font) { api_draw_text(x, y, s, strlen(s), fg, bg, font); }
/* A full-width line: background, then text at x 4 (5RM line()). */
static void a_line(int y, int h, const char *s, u16 fg, u16 bg, u8 font)
{
    api_fill_rect(0, y, A_W, h, bg);
    a_text(4, y + (h > 10 && font != A_F9 ? 1 : 0), s, fg, bg, font);
}
/* Text with its line cleared to the right of it (fox hunt line()). */
static void a_line_tail(int y, int h, const char *s, u16 fg, u8 font)
{
    int end;
    api_fill_rect(0, y, 4, h, A_BLACK);
    end = api_draw_text(4, y, s, strlen(s), fg, A_BLACK, font);
    if (end < A_W) api_fill_rect(end, y, A_W - end, h, A_BLACK);
}
static void a_title(const char *title, const char *right)
{
    api_fill_rect(0, 0, A_W, 13, A_TITLE);
    a_text(3, 2, title, A_WHITE, A_TITLE, A_F6);
    if (right && *right) a_text(A_W - 2 - 6 * (int)strlen(right), 2, right, A_WHITE, A_TITLE, A_F6);
}
static u32 a_absdiff(u32 a, u32 b) { return a > b ? a - b : b - a; }

/* On air every inverted DCS code is the bitstream of some normal code (qtscan2 / rptfind DCS_TWIN). */
static const u8 DCS_TWIN[105] = {
    7, 39, 76, 90, 8, 31, 70, 0, 4, 72, 66, 49, 51, 40, 80, 32, 98, 26, 104,
    36, 62, 61, 85, 35, 65, 50, 17, 100, 47, 79, 42, 5, 15, 46, 60, 23, 19, 64,
    59, 1, 13, 82, 30, 75, 71, 101, 33, 28, 73, 11, 25, 12, 96, 67, 83, 77, 74,
    84, 88, 38, 34, 21, 20, 102, 37, 24, 10, 53, 99, 81, 6, 44, 9, 48, 56, 43,
    2, 55, 95, 29, 14, 69, 41, 54, 57, 22, 97, 91, 58, 92, 3, 87, 89, 255, 103,
    78, 52, 86, 16, 68, 27, 45, 63, 94, 18,
};
#define CTCSS_COUNT 50
#define DCS_COUNT 105
static int dcs_twin(int idx)
{
    int i = idx - CTCSS_COUNT;
    if (i < 0 || i >= DCS_COUNT || DCS_TWIN[i] == 255) return -1;
    return CTCSS_COUNT + DCS_COUNT + DCS_TWIN[i];
}
#endif
