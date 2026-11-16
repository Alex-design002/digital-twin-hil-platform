# Universal Digital Twin HIL Platform

Multi-model Hardware-in-the-Loop (HIL) terramechanics digital twin dashboard and Raspberry Pi 5 Wi-Fi actuator controller. Built originally around the Traxxas TRX-4 Defender and extensible across custom RC crawlers, short-course buggies, tracked UGVs, and 6x6 expedition haulers.

## Supported Vehicle Fleet & Circuits

- **Traxxas TRX-4 Defender** (`trx4_defender`, 1/10 Scale Trail Crawler)
- **Traxxas Slash 4x4 Desert Buggy** (`slash_4x4_buggy`, 1/10 Short-Course)
- **6-DOF Tracked Inspection UGV** (`ugv_tracked_rover`, Skid-Steer Crawler)
- **1/10 Heavy 6x6 Expedition Hauler** (`hauler_6x6`, TRX-6 Multi-Axle)
- **Custom Vehicle Specs** (live editable mass, wheelbase, CG height, ground clearance, tire radius, aero drag, and motor force limits)

## Dual Execution Modes

### 1. Standalone Website on GitHub Pages
Open `index.html` on GitHub Pages (`Settings -> Pages -> Deploy from branch: main / (root)`). When no local Raspberry Pi bridge is reachable, the embedded JavaScript physics engine computes grade force ($F_g$), rolling/sinkage force ($F_r$), aerodynamic drag ($F_d$), tractive load ratio ($\lambda$), rollover/clearance interlocks, and 50 Hz PWM commands right inside your browser.

### 2. Live Raspberry Pi 5 Wi-Fi & BenchTwin Hardware Mode
Run on a Raspberry Pi 5 connected over Wi-Fi to your LAN or hotspot:

```bash
python3 trx4_hil_controller.py --web
```

Or bridge live to BenchTwin Desktop on Windows (`127.0.0.1:9222`):

```bash
python trx4_hil_controller.py --web --dry-run --benchtwin-cdp http://127.0.0.1:9222
```

Connect the web UI to `http://<pi-ip>:5000` (or `http://127.0.0.1:5000`) via the **Pi Wi-Fi / Bridge Endpoint** bar to stream real-time SSE telemetry and drive physical USB CDC Arduino Nano servos + I2C IMU + Active-LOW alarm hardware.

## Repository Layout

- `index.html` / `trx4_web_ui.html` — Multi-model mission control web UI (hybrid standalone sim + REST/SSE hardware client)
- `trx4_web_server.py` — Zero-dependency Python HTTP/SSE Wi-Fi bridge server
- `trx4_hil_controller.py` — Raspberry Pi 5 GPIO, MPU-6050 / ADXL345 I2C, Serial & BenchTwin CDP controller
- `trx4_benchtwin_project.json` — Breadboard-seated BenchTwin 3D workbench project (13 parts, 40 wires)
- `trx4_wokwi/` — Standalone Wokwi Nano firmware & wiring schematic (`trx4_wokwi.ino`, `diagram.json`)
