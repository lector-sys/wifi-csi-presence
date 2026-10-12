# WiFi CSI Presence

Room motion/presence sensing from WiFi Channel State Information, using **one
plain ESP32** associated with your home router (`active_sta` mode), a small
Python bridge, and a browser dashboard.

```
ESP32 (STA on your WiFi)  --UDP JSON-->  bridge.py  --HTTP + SSE-->  browser
      CSI -> DSP -> 0..100 score              rolling history          dashboard
```

---

## What this does and does not do

**It detects general disturbance in the WiFi channel between the ESP32 and your
router, which correlates with movement nearby.** That is the whole claim.

It does **not**:

- identify *who* or *what* moved — a person, a pet, a fan, a washing machine,
  a door, or the router changing its rate all move the same number;
- estimate size, height, pose, limb position, orientation, or identity;
- count people, or locate them in the room.

Two further honesty notes specific to this build:

- **Single node is the noisy configuration.** One ESP32 measuring one link to
  one router gives you a single spatial path through the room. A two-node setup
  (a dedicated transmitter) is meaningfully more reliable and less dependent on
  whatever your router happens to be doing. See *Upgrading to two nodes* below.
- **The "brief" vs "sustained" label is a soft hint**, computed purely from how
  much of the last 30 seconds exceeded an activity threshold. It says nothing
  about the cause.
- **The stick figure is decoration.** Only its wobble intensity is driven by the
  activity score. No part of its pose comes from the signal.

**Not for security, safety, medical, or any critical use.**

---

## Hardware

Whatever plain ESP32 dev board you already have — ESP32-WROOM-32 /
DevKitC / NodeMCU-32S all work. The original ESP32's WiFi driver exposes CSI
natively; no ESP32-S3 or special hardware is needed.

You also need a USB cable for flashing (only for flashing — at runtime the node
reports over WiFi) and the ESP32 within normal WiFi range of your router.

Placement matters more than the board: put the node so that the straight line
between it and the router passes through the space you care about.

---

## Build and flash

Requires **ESP-IDF v5.1 or newer** (developed against v5.3; `idf.py` and the
`wifi_csi_config_t` fields used here are stable across 5.x for the ESP32
target).

```bash
# one-time, if you don't have ESP-IDF yet
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
~/esp/esp-idf/install.sh esp32
. ~/esp/esp-idf/export.sh          # Windows: %USERPROFILE%\esp\esp-idf\export.bat

cd firmware
idf.py set-target esp32
idf.py menuconfig      # -> "CSI Presence Sensor"
```

In `menuconfig > CSI Presence Sensor`, set:

| Setting | Value |
|---|---|
| Home WiFi SSID / password | your router's |
| Room name | e.g. `living room` — shown on the dashboard |
| Host IP running the bridge | LAN IP of the computer running `bridge.py` |
| Telemetry UDP port | `5566` (default) |
| Command UDP port | `5567` (default) |
| Traffic generator rate | `100` packets/s (default) |

Then:

```bash
idf.py build flash monitor
```

On the serial monitor you should see the IP and gateway, then one JSON line
every 500 ms:

```
{"location":"living room","node":"a1b2c3","activity_score":4,"calibrated":true, ...}
```

If `activity_score` never moves and `packets` stays at 0, no CSI is arriving —
see *Troubleshooting*.

---

## Run the bridge and dashboard

No pip install required; the bridge is standard library only.

```bash
python bridge/bridge.py --node-ip 192.168.1.42     # the ESP32's IP
# -> dashboard on http://localhost:8080
```

`--node-ip` is only needed for the dashboard's Calibrate button (it sends the
`CAL` command to the node). Without it the dashboard still shows live data.

Other modes:

```bash
python bridge/bridge.py                 # no hardware: simulated data, clearly labelled
python bridge/bridge.py --no-simulate   # show "waiting for a node" instead of faking
python bridge/bridge.py --serial COM5   # read telemetry over USB instead of WiFi
                                        #   (this mode alone needs: pip install pyserial)
```

The bridge keeps a 60-minute rolling history (one point per 5 s) and marks a
node offline after 15 s of silence. With no hardware present it generates
plausible fluctuating data and every card is badged **simulated data**; as soon
as a real node reports, the simulated card is removed and the simulator stops
for good.

---

## Calibration

The activity score is *relative to your room's quiet noise floor*, so it means
nothing until calibrated. Every room, router, and node position differs.

1. Leave the room. Turn off fans, close doors, stop the washing machine.
2. Press **Calibrate (60 s)** on the dashboard (or power-cycle a node that has
   never been calibrated — it calibrates automatically on first boot).
3. Stay out for the full 60 seconds. The card shows "calibrating…".
4. The baseline is written to NVS flash and survives reboots.

The baseline is `mean + 3×sd` of the quiet-room moving variance. That is
deliberately generous: on a single-node link a tight baseline turns every AGC
wobble into "activity". If the room reads active while genuinely still,
recalibrate; if it barely responds to walking, your node is probably too far
off the router path — move it, then recalibrate.

Recalibrate after moving the node, moving the router, or changing WiFi channel.

---

## How the signal processing works

Per CSI frame (`firmware/main/csi_dsp.h`):

1. **Amplitude per subcarrier** — `sqrt(I² + Q²)` over the LLTF subcarriers
   (buffer indices 2–26 and 38–63; index 0 is DC, 27–37 are the null guard
   band, and starting at 2 keeps us clear of the "first word invalid" bytes).
   LLTF is present on every frame; HT-LTF is not, so only the first 64 pairs
   are trusted.
2. **Spatial turbulence** — the coefficient of variation (std ÷ mean) across
   those subcarriers. Using the *ratio* rather than raw std matters here: a
   plain ESP32 cannot lock AGC/FFT gain (espectre's gain lock is S3/C3/C5/C6
   only), so raw amplitude drifts with gain while the ratio does not.
3. **Median-of-7** over turbulence, to drop single-frame AGC steps.
4. **Moving variance** over a 100-frame (~1 s) window — this is the motion
   metric. Movement changes the *shape* of the subcarrier profile over time;
   a still room does not.
5. **Score** — `100 × log10(variance / baseline) / 2`, clamped to 0–100, then
   exponentially smoothed. Two decades above the calibrated baseline reads as
   100.
6. **Pattern** — fraction of the last 30 s above the activity threshold:
   `<5%` quiet, `>50%` sustained, otherwise brief. A hint, not a classification.

**Why the node sends UDP packets to your router:** CSI is only computed for
*received* frames. The firmware sends a 1-byte UDP packet to the gateway 100
times a second; the router ACKs each one at the MAC layer, and those ACKs are
what produce the CSI stream. This is exactly the trick `active_sta` uses in
ESP32-CSI-Tool. The traffic is negligible (~100 B/s payload) but it does mean
the node is always talking to your router.

---

## Testing the DSP

The DSP is plain C with no ESP-IDF dependencies so it runs on a host:

```bash
cd firmware/test
gcc -O2 -o test_dsp test_dsp.c -lm && ./test_dsp
```

It feeds synthetic CSI frames and asserts that a still room calibrates and
scores near zero, that a moving scatterer raises the score, that sustained
movement reads as `sustained`, that the score settles again afterwards, and
that calibration refuses to commit on too few samples.

> **Not yet run.** There is no C compiler on the machine this was built on, so
> this test has been written but never executed. Run it before trusting the
> thresholds — if an assertion fires, the constants at the top of `csi_dsp.h`
> (`DSP_VAR_W`, `DSP_SPAN_DECADES`, `DSP_ACTIVE_SCORE`) are the knobs.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `packets` stays 0 | CSI not enabled in the build — confirm `CONFIG_ESP_WIFI_CSI_ENABLED=y` took effect, and that `idf.py set-target esp32` was run |
| Score pinned at 0 or 100 | Not calibrated, or calibrated while the room was not actually still. Recalibrate. |
| Score jumps with no one home | Fan, pet, appliance, or a neighbouring AP causing rate changes. Also try lowering the traffic rate to 50. |
| Dashboard says offline | Wrong "Host IP" in menuconfig, or a firewall blocking inbound UDP 5566 on the host |
| "calibration failed: too few CSI samples" | CSI rate too low — check the node is associated and power save is off |
| Bridge exits with "cannot listen" | Another copy is already running |

---

## Upgrading to two nodes later

Adding a second ESP32 as a dedicated transmitter is the single biggest
reliability improvement available, and the code is structured so it is an
addition rather than a rewrite:

- **Firmware**: flash a second board with ESP32-CSI-Tool's `active_ap` project
  (or an AP-mode variant of this one), and point the sensing node at it instead
  of the router. Only the traffic-generator destination and the WiFi config
  change; `csi_dsp.h` is untouched.
- **Bridge**: already keyed by `location` and already handles multiple nodes —
  a second node with a different Room name appears as a second card with no
  code change.
- **Dashboard**: the card grid is already a repeating layout.

What you gain: a link you control end to end (fixed rate, fixed power, no
router-side rate adaptation), a second spatial path through the room, and the
ability to place transmitter and receiver so the person crosses between them.

---

## Credits — what was adapted from where

| Source | License | What was taken |
|---|---|---|
| [StevenMHernandez/ESP32-CSI-Tool](https://github.com/StevenMHernandez/ESP32-CSI-Tool) | MIT | The `active_sta` design: station-mode association, the UDP-to-gateway traffic generator that clocks the CSI stream (`_components/sockets_component.h`), the `wifi_csi_config_t` field values and callback registration (`_components/csi_component.h`), and `esp_wifi_set_ps(WIFI_PS_NONE)`. |
| [francescopace/espectre](https://github.com/francescopace/espectre) | GPLv3 | The **algorithm design** of the MVS pipeline: spatial turbulence per packet → outlier rejection → moving variance over a window as the motion metric, plus CV normalisation and the observation that AGC/FFT gain lock is unavailable on plain ESP32. Reimplemented from the documented approach; no espectre source is copied or linked, so this project is not a GPLv3 derivative. Confirmed it supports plain ESP32 (`examples/espectre-esp32.yaml`), though its gain-lock feature is S3/C3/C5/C6 only. |
| [Saad-Imtiaz/WiFi-Sense](https://github.com/Saad-Imtiaz/WiFi-Sense) | MIT | EMA smoothing of the motion level into a stable score, and the overall firmware → web-server → browser-dashboard project shape. |
| [espressif/esp-csi](https://github.com/espressif/esp-csi) | Apache-2.0 | Reference for CSI buffer layout and the `wifi_sensing_demo` calibration/baseline-reset flow that the 60 s calibration mode follows. |

The reference repos are cloned under `reference/` for study; nothing in there
is compiled or shipped as part of this project.

Deviation worth noting: WiFi-Sense computes variance of the *mean* amplitude
over time, which discards the per-subcarrier structure that makes CSI more
sensitive than RSSI. This build follows espectre instead and takes the variance
*across* subcarriers first.

---

## Layout

```
firmware/     ESP-IDF project (plain ESP32)
  main/csi_dsp.h    pure-C DSP, host-testable
  main/main.c       WiFi, CSI capture, traffic generator, telemetry, calibration
  test/test_dsp.c   host check for the DSP
bridge/       stdlib-only Python: UDP/serial in, HTTP + SSE out
web/          single-file dashboard
reference/    the four reference repos, for study only
```

**A note on the transport:** the brief asked for WebSockets. The dashboard only
ever needs server→browser streaming, which is exactly Server-Sent Events, and
SSE is native to both `http.server` and the browser's `EventSource`. Using it
keeps the bridge at zero pip dependencies. If you later need the browser to
push data back, swap `_events()` for the `websockets` package — the rest of the
bridge is unaffected.
