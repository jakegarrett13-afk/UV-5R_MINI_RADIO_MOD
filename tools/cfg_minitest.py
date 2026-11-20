"""Turn the SDK 3.1.0 working copy (fw/sdk) into the UV-5R Mini minimal test app: BLE "walkie-talkie" + UART update.

Usage: py -I cfg_minitest.py SDK_DIR
Every change is an exact text replacement that must match once, so a mismatch stops the script (it never half-edits
silently). It is safe to run again: already-applied changes are detected and skipped.

What it does (see BOARD_UV5RMINI.md for why):
  board_ac695x_demo_cfg.h:
    - apps off: music, line-in, FM, PC, RTC, record (BT only)
    - UART0 debug off; HW SPI1/SPI2 off (SPI port A = the Mini's LCD on PB0/PB1); SD0 off; USB disk / USB device off
    - AD key off; PWM LED off; UI (LED7) off
    - DAC: DACVDD_LDO_2_60V, DAC_OUTPUT_MONO_R (as V0.04)
    - BLE on (TCFG_USER_BLE_ENABLE 1) + TRANS_DATA_EN 1 (the AE30 service V0.04 has)
  app_config.h: USER_UART_UPDATE_ENABLE 1 (role stays SLAVE)
  board_ac695x_demo.c: uart_update_cfg = DM rx / DP tx, INPUT_CH0, CH1_UT1_TX (V0.04's constant at 0x01E65788)
Kept as in V0.04: power (LDO15, demo levels), mic ADC config, wake-up port, LVD, charge off.
"""
import os
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sdk = sys.argv[1]
B = os.path.join(sdk, "apps/soundbox/board/br23/board_ac695x_demo")
EDITS = [
    # SDK 3.1.0 update.a implements the legacy downloader entry point as a
    # "not supported" stub. Use the active-update API, as SDK 3.1.2 does.
    (os.path.join(sdk, "apps/common/update/uart_update.c"), [
        ("static void uart_update_callback(void *priv, u8 type, u8 cmd)",
         """/* UV-5R Mini: use the update library's supported v2 entry point. */
static void mini_uart_update_private_fill(UPDATA_PARM *p)
{
    UPDATA_UART param = {
        .control_baud = update_baudrate,
        .control_io_tx = update_cfg.tx,
        .control_io_rx = update_cfg.rx,
    };
    memcpy(p->parm_priv, &param, sizeof(param));
}

static void mini_uart_update_jump(int type)
{
    cpu_reset();
}

static void mini_uart_update_state(int type, u32 state, void *priv)
{
    update_ret_code_t *result = (update_ret_code_t *)priv;
    if (state != UPDATE_CH_EXIT) {
        return;
    }
    if (result && result->stu == 0 && result->err_code == 0) {
        update_mode_api_v2(UART_UPDATA, mini_uart_update_private_fill,
                           mini_uart_update_jump);
    } else {
        rx_cnt = 0;
        uart_file_offset = 0;
        update_baudrate = 9600;
        uart_update_set_baud(update_baudrate);
    }
}

static void uart_update_callback(void *priv, u8 type, u8 cmd)"""),
        ("""            app_update_loader_downloader_init(              //设置完波特率后开始升级
                UART_UPDATA,
                uart_update_callback,
                NULL,
                &uart_ch_update_op);""",
         """            update_mode_info_t info = {
                .type = UART_UPDATA,
                .state_cbk = mini_uart_update_state,
                .p_op_api = &uart_ch_update_op,
                .task_en = 1,
            };
            app_active_update_task_init(&info);"""),
    ]),
    (os.path.join(B, "board_ac695x_demo_cfg.h"), [
        ("#define TCFG_APP_MUSIC_EN			        1", "#define TCFG_APP_MUSIC_EN			        0"),
        ("#define TCFG_APP_LINEIN_EN					1", "#define TCFG_APP_LINEIN_EN					0"),
        ("#define TCFG_APP_FM_EN					    1", "#define TCFG_APP_FM_EN					    0"),
        ("#define TCFG_APP_PC_EN					    1", "#define TCFG_APP_PC_EN					    0"),
        ("#define TCFG_APP_RTC_EN					    1", "#define TCFG_APP_RTC_EN					    0"),
        ("#define TCFG_APP_RECORD_EN				    1", "#define TCFG_APP_RECORD_EN				    0"),
        ("#define TCFG_UART0_ENABLE					ENABLE_THIS_MOUDLE", "#define TCFG_UART0_ENABLE					DISABLE_THIS_MOUDLE"),
        ("#define	TCFG_HW_SPI1_ENABLE		ENABLE_THIS_MOUDLE", "#define	TCFG_HW_SPI1_ENABLE		DISABLE_THIS_MOUDLE"),
        ("#define	TCFG_HW_SPI2_ENABLE		ENABLE_THIS_MOUDLE", "#define	TCFG_HW_SPI2_ENABLE		DISABLE_THIS_MOUDLE"),
        ("#define TCFG_SD0_ENABLE						ENABLE_THIS_MOUDLE", "#define TCFG_SD0_ENABLE						DISABLE_THIS_MOUDLE"),
        ("#define TCFG_UDISK_ENABLE					ENABLE_THIS_MOUDLE//U盘模块使能", "#define TCFG_UDISK_ENABLE					DISABLE_THIS_MOUDLE//U盘模块使能"),
        ("#define TCFG_OTG_USB_DEV_EN                 BIT(0)//USB0 = BIT(0)  USB1 = BIT(1)", "#define TCFG_OTG_USB_DEV_EN                 0//USB0 = BIT(0)  USB1 = BIT(1)"),
        ("#define TCFG_ADKEY_ENABLE                   ENABLE_THIS_MOUDLE//是否使能AD按键", "#define TCFG_ADKEY_ENABLE                   DISABLE_THIS_MOUDLE//是否使能AD按键"),
        ("#define TCFG_PWMLED_ENABLE					ENABLE_THIS_MOUDLE			//是否支持PMW LED推灯模块", "#define TCFG_PWMLED_ENABLE					DISABLE_THIS_MOUDLE			//是否支持PMW LED推灯模块"),
        ("#define TCFG_UI_ENABLE 						ENABLE_THIS_MOUDLE 	//UI总开关", "#define TCFG_UI_ENABLE 						DISABLE_THIS_MOUDLE 	//UI总开关"),
        ("#define TCFG_UI_LED7_ENABLE 			 	ENABLE_THIS_MOUDLE 	//UI使用LED7显示", "#define TCFG_UI_LED7_ENABLE 			 	DISABLE_THIS_MOUDLE 	//UI使用LED7显示"),
        ("#define TCFG_AUDIO_DAC_LDO_VOLT				DACVDD_LDO_2_90V", "#define TCFG_AUDIO_DAC_LDO_VOLT				DACVDD_LDO_2_60V"),
        ("#define TCFG_AUDIO_DAC_CONNECT_MODE   DAC_OUTPUT_LR", "#define TCFG_AUDIO_DAC_CONNECT_MODE   DAC_OUTPUT_MONO_R"),
        ("#define TCFG_USER_BLE_ENABLE                0   //BLE功能使能", "#define TCFG_USER_BLE_ENABLE                1   //BLE功能使能"),
        ("#define TRANS_DATA_EN              0  //not surport", "#define TRANS_DATA_EN              1  //not surport"),
    ]),
    # The SDK moves on from the power-on and BT-mode tones only in the tones' end callbacks (the "tone missing"
    # fallback is commented out). Radio 2's res/tone holds Baofeng's voice files, not power_on.* / bt.*, so the first
    # test build stayed in the power-on task forever (update task alive, no BT). Skip both tones and go on directly.
    (os.path.join(sdk, "apps/soundbox/task_manager/power_on/power_on.c"), [
        ("""    int err =  tone_play_with_callback_by_name(tone_table[IDEX_TONE_POWER_ON], 1, tone_play_end_callback, (void *)IDEX_TONE_POWER_ON);""",
         """    /* UV-5R Mini: no SDK tones in radio 2's res/tone; skip the power-on tone (its end callback is the only way on) */
    power_on_init();"""),
    ]),
    (os.path.join(sdk, "apps/soundbox/task_manager/bt/bt.c"), [
        ("""        tone_play_with_callback_by_name(tone_table[IDEX_TONE_BT_MODE], 1, bt_tone_play_end_callback, (void *)IDEX_TONE_BT_MODE);
        //协议栈初始化在提示音结束进行""",
         """        /* UV-5R Mini: skip the BT-mode tone (missing on radio 2); start the stack directly */
        bt_task_start();"""),
    ]),
    (os.path.join(sdk, "apps/soundbox/include/app_config.h"), [
        ("#define USER_UART_UPDATE_ENABLE           0//", "#define USER_UART_UPDATE_ENABLE           1//"),
    ]),
    (os.path.join(B, "board_ac695x_demo.c"), [
        # start the update listener first thing after power init (it was the last step of board_init): the way back
        # comes up before ADC / cfg parse / dev manager / power mode, so a later hang can't block it
        ("""void board_init()
{
    board_power_init();
""",
         """void board_init()
{
    board_power_init();
#if USER_UART_UPDATE_ENABLE
    {   /* UV-5R Mini: update listener first (moved up from the end of board_init) */
        extern void uart_update_init(void *cfg);
        static const struct { u16 rx; u16 tx; u8 input_channel; u8 output_channel; } mini_update_cfg = {
            IO_PORT_DM, IO_PORT_DP, INPUT_CH0, CH1_UT1_TX     /* = V0.04's constant at 0x01E65788 */
        };
        uart_update_init((void *)&mini_update_cfg);
    }
#endif
"""),
        ("""#if USER_UART_UPDATE_ENABLE
        {
#include "uart_update.h"
            uart_update_cfg  update_cfg = {""",
         """#if 0 /* UV-5R Mini: moved to the start of board_init */
        {
#include "uart_update.h"
            uart_update_cfg  update_cfg = {"""),
        ("""                .rx = IO_PORTA_02,
                .tx = IO_PORTA_03,
                .output_channel = CH1_UT1_TX,
                .input_channel = INPUT_CH0""",
         """                .rx = IO_PORT_DM,             /* UV-5R Mini: programming cable on USB DM/DP (V0.04 0x01E65788) */
                .tx = IO_PORT_DP,
                .output_channel = CH1_UT1_TX,
                .input_channel = INPUT_CH0"""),
    ]),
]
# Size: the demo config builds 724 KB; the res-link slot holds 491 KB (V0.04's code was ~438 KB). Switch off the
# codecs/effects a BLE + update test doesn't use. Matched by name: "#define NAME <ws> ENABLE|1" -> DISABLE|0,
# exactly one active (non-commented) definition each.
import re  # noqa: E402
SIZE_OFF = ["TCFG_DEC_G729_ENABLE", "TCFG_DEC_MP3_ENABLE", "TCFG_DEC_WMA_ENABLE", "TCFG_DEC_WAV_ENABLE",
            "TCFG_DEC_FLAC_ENABLE", "TCFG_DEC_APE_ENABLE", "TCFG_DEC_M4A_ENABLE", "TCFG_DEC_ALAC_ENABLE",
            "TCFG_DEC_AMR_ENABLE", "TCFG_DEC_DTS_ENABLE",
            "TCFG_ENC_G726_ENABLE", "TCFG_ENC_MP3_ENABLE", "TCFG_ENC_ADPCM_ENABLE", "TCFG_ENC_SBC_ENABLE"]
SIZE_ZERO_FIRST = ["TCFG_EQ_ENABLE", "TCFG_DRC_ENABLE"]   # the first (main) definition; the later ones sit under #if MIC_EFFECT
cfgp = os.path.join(B, "board_ac695x_demo_cfg.h")
s = open(cfgp, encoding="utf-8", errors="surrogateescape").read()
for name in SIZE_OFF:
    pat = re.compile(r"^(#define\s+" + name + r"\s+)ENABLE\b", re.M)
    if re.search(r"^#define\s+" + name + r"\s+DISABLE\b", s, re.M) and not pat.search(s):
        print(f"  already: {name} DISABLE")
        continue
    hits = pat.findall(s)
    if len(hits) != 1:
        raise SystemExit(f"{name}: expected 1 active ENABLE definition, found {len(hits)}")
    s = pat.sub(r"\1DISABLE", s)
    print(f"  edited:  {name} -> DISABLE")
for name in SIZE_ZERO_FIRST:
    pat = re.compile(r"^(#define\s+" + name + r"\s+)1(\s)", re.M)
    m = pat.search(s)
    if not m:
        print(f"  already/absent: {name} = 1 not found (check by hand)")
        continue
    s = s[:m.start()] + m.group(1) + "0" + m.group(2) + s[m.end():]
    print(f"  edited:  {name} (first definition, line {s[:m.start()].count(chr(10)) + 1}) -> 0")
open(cfgp, "w", encoding="utf-8", errors="surrogateescape", newline="").write(s)

for path, edits in EDITS:
    s = open(path, encoding="utf-8", errors="surrogateescape").read()
    for old, new in edits:
        if new in s:                      # result already there (also when new starts with old)
            print(f"  already: {os.path.basename(path)}: {new.split(chr(10))[0].strip()[:70]}")
            continue
        n = s.count(old)
        if n != 1:
            raise SystemExit(f"{path}: expected exactly 1 match, found {n}: {old[:80]!r}")
        s = s.replace(old, new)
        print(f"  edited:  {os.path.basename(path)}: {new.split(chr(10))[0].strip()[:70]}")
    open(path, "w", encoding="utf-8", errors="surrogateescape", newline="").write(s)
print("done")
