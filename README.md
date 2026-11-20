# UV-5R Mini custom firmware

Free, open-source replacement firmware for the **Baofeng UV-5R Mini** (the 5 W model with the 1.44" colour
screen). It brings the apps and look of the [UV-5RM custom firmware](https://github.com/sophiel-meow/bf-5rh-firmware-open)
to the Mini, rewritten in C for the Mini's processor.

> **Read this first.** Flashing replaces the factory firmware and **there is no way back to stock**: no factory image
> for this radio is published. It has been run on **one** radio (firmware V0.05, hardware V1.7, model `NRFB`). A
> failed flash can leave the radio unusable. You use this entirely at your own risk. You are responsible for
> transmitting only where you are licensed to.

## What you get

| | |
|---|---|
| **Radio** | VFO A/B and 1000 channels, direct frequency entry, receive CTCSS/DCS, transmit tone, repeater shift/offset, wide/narrow, HIGH/LOW power from the radio's own calibration, time-out timer |
| **Scan** | Hold `#`: scans the stored channels (or the VFO band). Stops on a signal and resumes after it ends |
| **Apps** (`MENU`) | **TEXT** (short messages between radios, with delivery confirmation), **SEARCH** (finds a nearby transmitter's frequency and tone), **SCANQT** (reads the tone on your frequency), **BAND SWEEP** (scans a whole band and lists what it heard), **FOX HUNT** (signal meter and rough distance) |
| **Settings** (`MENU` → `SETTINGS`) | Current channel (incl. save the VFO as a named channel, delete a channel), all channels, audio (roger beep, squelch tail), display (screen time-out, key lock), programmable keys, TX band lock |
| **Look** | Blue Angel theme, harp start-up chime |
| **PC** | `UV_Channels.html` reads and writes channels and bands in Chrome or Edge |

## What you need

- A UV-5R Mini. To check the model, hold `8` while switching it on: the screen should show `5MINI NRFB`.
- The Baofeng programming cable (USB, CH340), pushed **all the way** into the radio's jack.
- A **fully charged** battery. The update takes about 4 minutes and the radio must not lose power.
- Windows with [Python 3](https://www.python.org/) and pyserial: `py -m pip install pyserial`.
  (Other systems work too: use `python3` and your serial port name instead of `COM3`.)

Find the cable's COM port in Device Manager → Ports. The examples use `COM3`. Run every command from this folder.

## First install (from the factory firmware)

1. **Back up the radio (recommended).** This only reads: it saves your channels and the radio's calibration area.
   ```
   py tools\mini_read.py COM3 scan backup
   ```
2. **Put the radio in update mode.** Switch it off, hold **PTT-A** and the **`=`** side key, and switch it on. The
   screen shows the update screen.
3. **Flash.** The last argument is the image's SHA-256 (also in `firmware\SHA256SUMS.txt`). The tool refuses to send
   a file that doesn't match it.
   ```
   py tools\mini_fw_update.py COM3 flash firmware\mini-native-n29.ufw 595954df7eecdf67c7c0dfedd934c52661a75449612c20122ae4fe46885b3a5c
   ```
   You should see `image ok`, a `PONG`, then read counts scrolling for 3½–4 minutes, then
   `RESULT 0x00 in loader: OK`. **Don't touch the radio or the cable** until it says `Done`. The screen may go dark
   while it writes; that is normal.
4. The radio restarts by itself: the Blue Angel logo and chime, then the main screen.

## Updating to a newer build

No key combination is needed: the custom firmware listens for updates all the time.

1. Leave the radio on its main screen, with no app open and not transmitting. Check it answers:
   ```
   py tools\mini_native_diag.py COM3
   ```
   It should show `"build": ...`, `"rf_chip": "0x4829"` and `"tx_keyed": false`. Only flash when `tx_keyed` is false.
2. Flash the new image the same way as step 3 above. Your settings, channels and TEXT messages are kept.

## Programming channels

Open `UV_Channels.html` in Chrome or Edge, connect the cable, and use **Read from radio** / **Write to radio**. It
can also load the built-in Clark County (Nevada) list as a starting point; change it to suit your area.

## Using the radio

| Key | Main screen |
|---|---|
| `0`–`9` | Type a frequency (VFO: 6 digits, MHz) or a channel number (3 digits) |
| `UP` / `DOWN` | Step the frequency, or the next/previous channel |
| `V/M` | Switch between VFO and channels |
| `A/B` | Switch between the two sides |
| `[-]` | Pick a band (MODE); programmable |
| `MENU` | Apps and settings |
| `*` | Squelch level; hold = monitor (hold also unlocks the keys) |
| `#` | VFO: cycle the receive tone; **hold = scan** |
| Side `=` | HIGH/LOW power; hold = flashlight (both programmable) |
| PTT-B | Talk |
| PTT-A | Opens TEXT (programmable) |

In the settings lists: `MENU` edits an item, `UP`/`DOWN` change it, `MENU` keeps the change, `EXIT` puts the old value
back.

## Troubleshooting

| Problem | Fix |
|---|---|
| The tools get no answer | Push the cable fully in, charge the battery, unplug and replug the USB, then try again |
| `image sha256 ... does not match` | Wrong file or hash. Nothing was sent |
| The cable stops answering after transmitting | Transmitting with the cable plugged in can lock up the cable's USB chip. Unplug and replug it. Test transmit with the cable unplugged |
| The flash stopped part way | Run the same flash command again; the radio's loader waits for it |

## Building from source

The image is built from `fw/native` with JieLi's AC695N soundbox SDK 3.1.0 and their pi32v2 toolchain 2.4.6 (both
published by JieLi). `tools/cfg_minitest.py` and `tools/cfg_native.py` patch the SDK, `tools/build_sdk_ref.sh` builds
the ELF, and `tools/make_app_ufw.py` packages it. Packaging needs a base image made from the stock V0.04 firmware,
which isn't included here.

## Credits and licence

- The apps, protocols and UI are ported from the UV-5RM custom firmware, built on
  [bf-5rh-firmware-open](https://github.com/sophiel-meow/bf-5rh-firmware-open).
- The update protocol follows Codeplug Studio's UV-5R Mini updater, as used by the
  [MM9PDY firmware](https://github.com/pskillen/mm9pdy-firmware) project.
- Licensed under the GNU GPL v3 (see `LICENSE`). Free to use, share and change; no warranty.
- The firmware image contains Baofeng's bootloader and stock resources, which are not covered by this licence.
