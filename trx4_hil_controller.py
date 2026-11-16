#!/usr/bin/env python3
"""Traxxas TRX-4 Hardware-in-the-Loop (HIL) Digital Twin Controller.

Designed for Raspberry Pi 5 (Linux SBC) with automatic Virtual Dry-Run fallback
for non-Pi hosts or disconnected hardware peripherals.

System Architecture & Pinout:
    - Piezo Buzzer (Part 19): Active-LOW on Physical Pin 11 (BCM GPIO 17).
        Driven LOW (0) to sound alarm; driven HIGH (1) to silence.
    - Capacitive Touch Sensor (Part 12): Active-HIGH on Physical Pin 13 (BCM GPIO 27).
        Pulled LOW by default; triggers next scenario on release (HIGH -> LOW).
    - MPU-6050 6-DOF IMU (Part 29): I2C1 (SDA=Pin 3/GPIO 2, SCL=Pin 5/GPIO 3),
        Address 0x68. Provides live pitch angle (theta_pitch) in degrees.
    - Arduino Nano V3.0 Bridge (Part 21): USB CDC Serial (/dev/ttyUSB0, 115200 8N1).
        Packet format: ``$ESC,SHIFT,DIFF_F,DIFF_R\\n`` (1000-2000 us).
    - Excel Terramechanics Engine: Reads ``trx4_hil_simulation_template.xlsx``
        (``Vehicle_Specs`` and ``Track_And_Weather_Scenarios``) via pandas.
"""

from __future__ import annotations

import argparse
import base64
import ctypes
from dataclasses import asdict, dataclass, field
from enum import Enum
import json
import logging
import math
import os
from pathlib import Path
import platform
import queue
import re
import shutil
import signal
import socket
import struct
import sys
import threading
import time
from typing import Any, Callable, Optional, Sequence, Union
import urllib.request
import warnings
import xml.etree.ElementTree as ET
import zipfile

try:
    import pandas as pd  # type: ignore

    HAS_PANDAS = True
except ImportError:
    pd = None  # type: ignore
    HAS_PANDAS = False


# ---------------------------------------------------------------------------
# Constants & Hardware Pinout Definitions
# ---------------------------------------------------------------------------

DEFAULT_EXCEL_FILENAME = "trx4_hil_simulation_template.xlsx"

BUZZER_BCM_PIN = 17       # Physical Pin 11, Active-LOW (0=ON, 1=OFF)
TOUCH_BCM_PIN = 27        # Physical Pin 13, Legacy optional touch input
I2C_BUS_DEFAULT = 1       # SDA=Pin 3 (GPIO 2), SCL=Pin 5 (GPIO 3)
MPU6050_ADDR_DEFAULT = 0x68
ADXL345_ADDR_DEFAULT = 0x53

SERIAL_PORT_DEFAULT = "/dev/ttyUSB0"
SERIAL_BAUD_DEFAULT = 115200
WEB_HOST_DEFAULT = "0.0.0.0"
WEB_PORT_DEFAULT = 5000

PWM_MIN_US = 1000
PWM_MAX_US = 2000
PWM_ESC_NEUTRAL_US = 1500
PWM_ESC_FULL_US = 2000
PWM_HIGH_GEAR_US = 1100
PWM_LOW_GEAR_US = 1900
PWM_DIFF_OPEN_US = 1100
PWM_DIFF_LOCKED_US = 1900

SAFE_SHUTDOWN_PWM = (
    PWM_ESC_NEUTRAL_US,
    PWM_LOW_GEAR_US,
    PWM_DIFF_LOCKED_US,
    PWM_DIFF_LOCKED_US,
)

DEFAULT_SURFACE_PRESETS: tuple[dict[str, Any], ...] = (
    {"surface_type": "Dry Asphalt", "track_name": "Sunny Tarmac Cruise", "friction_coeff": 0.85, "rolling_res": 0.020, "mud_depth_m": 0.000, "slope_deg": 5.0, "wind_speed_kmh": 10.0, "temp_c": 22.0},
    {"surface_type": "Dry Dirt Trail", "track_name": "Green Valley Dirt Road", "friction_coeff": 0.68, "rolling_res": 0.055, "mud_depth_m": 0.005, "slope_deg": 10.0, "wind_speed_kmh": 15.0, "temp_c": 19.0},
    {"surface_type": "Wet Asphalt", "track_name": "Coastal Highway Gale", "friction_coeff": 0.65, "rolling_res": 0.025, "mud_depth_m": 0.000, "slope_deg": 8.0, "wind_speed_kmh": 55.0, "temp_c": 14.0},
    {"surface_type": "Wet Rock", "track_name": "Wet Rock Crawler Line", "friction_coeff": 0.48, "rolling_res": 0.040, "mud_depth_m": 0.000, "slope_deg": 15.0, "wind_speed_kmh": 15.0, "temp_c": 14.0},
    {"surface_type": "Loose Gravel", "track_name": "Quarry Gravel Climb", "friction_coeff": 0.50, "rolling_res": 0.090, "mud_depth_m": 0.015, "slope_deg": 22.0, "wind_speed_kmh": 20.0, "temp_c": 16.0},
    {"surface_type": "Wet Grass", "track_name": "Moist Meadow Slope", "friction_coeff": 0.35, "rolling_res": 0.080, "mud_depth_m": 0.010, "slope_deg": 14.0, "wind_speed_kmh": 25.0, "temp_c": 11.0},
    {"surface_type": "Wet Mud", "track_name": "Carpathian Forest Rain", "friction_coeff": 0.40, "rolling_res": 0.180, "mud_depth_m": 0.040, "slope_deg": 18.0, "wind_speed_kmh": 35.0, "temp_c": 8.0},
    {"surface_type": "Dry Sand Dunes", "track_name": "Desert Dune Ascent", "friction_coeff": 0.42, "rolling_res": 0.220, "mud_depth_m": 0.045, "slope_deg": 16.0, "wind_speed_kmh": 40.0, "temp_c": 34.0},
    {"surface_type": "Packed Snow", "track_name": "Winter Timber Trail", "friction_coeff": 0.28, "rolling_res": 0.120, "mud_depth_m": 0.025, "slope_deg": 12.0, "wind_speed_kmh": 30.0, "temp_c": -4.0},
    {"surface_type": "Deep Mud Bog", "track_name": "Flooded Logging Trench", "friction_coeff": 0.32, "rolling_res": 0.250, "mud_depth_m": 0.085, "slope_deg": 15.0, "wind_speed_kmh": 25.0, "temp_c": 6.0},
    {"surface_type": "Deep Snow / Ice", "track_name": "Alpine Blizzard Peak", "friction_coeff": 0.15, "rolling_res": 0.300, "mud_depth_m": 0.090, "slope_deg": 32.0, "wind_speed_kmh": 70.0, "temp_c": -12.0},
    {"surface_type": "Glare Ice", "track_name": "Glare Ice Sheet", "friction_coeff": 0.10, "rolling_res": 0.015, "mud_depth_m": 0.000, "slope_deg": 5.0, "wind_speed_kmh": 20.0, "temp_c": -8.0},
)


def _calc_theta_max(wheelbase_m: float, h_cg_m: float) -> float:
    return math.degrees(math.atan((max(0.05, wheelbase_m) / 2.0) / max(0.02, h_cg_m)))


MODEL_PROFILES: dict[str, dict[str, Any]] = {
    "trx4_defender": {
        "id": "trx4_defender",
        "name": "Traxxas TRX-4 Defender",
        "category": "1/10 Scale Trail Crawler",
        "circuit": "Pi 5 (Wi-Fi) -> Arduino Nano Bridge + 6.0V BEC + 5.1V Buck + ADXL345/MPU-6050",
        "channels": {
            "d9": "XL-5 HV ESC",
            "d10": "2065A 2-Speed Shift",
            "d11": "Front T-Lock Diff",
            "d12": "Rear T-Lock Diff",
        },
        "specs": {
            "m": 3.75,
            "L": 0.324,
            "h_cg": 0.135,
            "h_clear": 0.080,
            "r_tire": 0.059,
            "A_f": 0.065,
            "C_d": 0.65,
            "F_motor_low": 45.0,
            "F_motor_high": 18.0,
            "g": 9.81,
            "rho_air": 1.225,
            "v_target": 1.5,
            "k_sink": 0.25,
            "theta_max": _calc_theta_max(0.324, 0.135),
        },
    },
    "slash_4x4_buggy": {
        "id": "slash_4x4_buggy",
        "name": "Traxxas Slash 4x4 Desert Buggy",
        "category": "1/10 Short-Course",
        "circuit": "Pi 5 (Wi-Fi) -> Nano/PCA9685 PWM Bridge + 6.0V VXL BEC + IMU",
        "channels": {
            "d9": "VXL-3s ESC",
            "d10": "Center Diff Bias",
            "d11": "Active Front Aero",
            "d12": "Active Rear Wing DRS",
        },
        "specs": {
            "m": 2.45,
            "L": 0.324,
            "h_cg": 0.078,
            "h_clear": 0.045,
            "r_tire": 0.054,
            "A_f": 0.048,
            "C_d": 0.42,
            "F_motor_low": 32.0,
            "F_motor_high": 22.0,
            "g": 9.81,
            "rho_air": 1.225,
            "v_target": 1.5,
            "k_sink": 0.25,
            "theta_max": _calc_theta_max(0.324, 0.078),
        },
    },
    "ugv_tracked_rover": {
        "id": "ugv_tracked_rover",
        "name": "6-DOF Tracked Inspection UGV",
        "category": "Skid-Steer Crawler",
        "circuit": "Pi 5 (Wi-Fi) -> Arduino Nano Bridge + Dual Track ESC + IMU",
        "channels": {
            "d9": "Dual-Track Drive ESC",
            "d10": "Planetary Range Box",
            "d11": "Left Track Brake/Tension",
            "d12": "Right Track Brake/Lock",
        },
        "specs": {
            "m": 5.20,
            "L": 0.380,
            "h_cg": 0.110,
            "h_clear": 0.065,
            "r_tire": 0.048,
            "A_f": 0.072,
            "C_d": 0.75,
            "F_motor_low": 68.0,
            "F_motor_high": 34.0,
            "g": 9.81,
            "rho_air": 1.225,
            "v_target": 1.5,
            "k_sink": 0.25,
            "theta_max": _calc_theta_max(0.380, 0.110),
        },
    },
    "hauler_6x6": {
        "id": "hauler_6x6",
        "name": "1/10 Heavy 6x6 Expedition Hauler",
        "category": "TRX-6 Multi-Axle",
        "circuit": "Pi 5 (Wi-Fi) -> Arduino Nano Bridge + 6V HV BEC + Tandem T-Lock",
        "channels": {
            "d9": "XL-5 HV Crawl ESC",
            "d10": "2-Speed Transfer Case",
            "d11": "Front Axle T-Lock",
            "d12": "Tandem Rear Axles Lock",
        },
        "specs": {
            "m": 4.85,
            "L": 0.454,
            "h_cg": 0.145,
            "h_clear": 0.085,
            "r_tire": 0.059,
            "A_f": 0.078,
            "C_d": 0.72,
            "F_motor_low": 56.0,
            "F_motor_high": 24.0,
            "g": 9.81,
            "rho_air": 1.225,
            "v_target": 1.5,
            "k_sink": 0.25,
            "theta_max": _calc_theta_max(0.454, 0.145),
        },
    },
}


class GovernorState(str, Enum):
    """Environmental governor operational states."""

    FREE = "FREE"
    HARSH = "HARSH"
    STOP = "STOP"


class ConditionMode(str, Enum):
    """Whether active terrain parameters come from Excel scenario presets or Wi-Fi sliders."""

    PRESET = "PRESET"
    CUSTOM = "CUSTOM"


class ControlMode(str, Enum):
    """Governor operating mode selectable from Wi-Fi Web UI."""

    AUTO = "AUTO"
    FORCE_FREE = "FORCE_FREE"
    FORCE_HARSH = "FORCE_HARSH"
    FORCE_STOP = "FORCE_STOP"


@dataclass(frozen=True)
class HardwareConfig:
    """Configuration for Pi 5 GPIO, I2C IMU, Serial bridge, Web UI, and HIL loop."""

    excel_path: Optional[Path] = None
    buzzer_pin: int = BUZZER_BCM_PIN
    touch_pin: int = TOUCH_BCM_PIN
    enable_touch_button: bool = False
    i2c_bus: int = I2C_BUS_DEFAULT
    imu_address: int = MPU6050_ADDR_DEFAULT
    imu_type: str = "auto"  # "auto", "mpu6050", or "adxl345"
    serial_port: str = SERIAL_PORT_DEFAULT
    serial_baud: int = SERIAL_BAUD_DEFAULT
    arduino_boot_delay_s: float = 2.0
    loop_hz: float = 15.0
    buzzer_mode: str = "pulse"  # "pulse" or "continuous"
    buzzer_pulse_hz: float = 2.0
    use_live_imu: bool = False
    imu_invert_pitch: bool = False
    imu_pitch_offset_deg: float = 0.0
    dry_run: bool = False
    initial_scenario: int = 1
    auto_cycle_sec: Optional[float] = None
    web_enabled: bool = False
    web_host: str = WEB_HOST_DEFAULT
    web_port: int = WEB_PORT_DEFAULT
    benchtwin_cdp_url: Optional[str] = None


@dataclass(frozen=True)
class VehicleSpecs:
    """TRX-4 Defender physical & terramechanics parameters from Vehicle_Specs."""

    m: float = 3.75            # Total mass (kg)
    L: float = 0.324           # Wheelbase (m)
    h_cg: float = 0.135        # CG height (m)
    h_clear: float = 0.080     # Portal axle ground clearance (m)
    r_tire: float = 0.059      # Static tire rolling radius (m)
    A_f: float = 0.065         # Projected frontal area (m^2)
    C_d: float = 0.65          # Aerodynamic drag coefficient (-)
    F_motor_low: float = 45.0  # Max wheel tractive force in Low Gear (N)
    F_motor_high: float = 18.0 # Max wheel tractive force in High Gear (N)
    g: float = 9.81            # Gravity (m/s^2)
    rho_air: float = 1.225     # Air density (kg/m^3)
    v_target: float = 1.5      # Target trail speed (m/s)
    k_sink: float = 0.25       # Terrain sinkage penalty factor (-)
    theta_max: float = 50.19442890773481  # Static pitch rollover limit (deg)


@dataclass(frozen=True)
class ScenarioParams:
    """Single track & weather scenario row from Excel."""

    scenario_id: int
    track_name: str
    surface_type: str
    slope_deg: float
    friction_coeff: float
    rolling_res: float
    mud_depth_m: float
    wind_speed_kmh: float
    temp_c: float
    excel_lambda: Optional[float] = None
    excel_status: Optional[str] = None
    excel_esc_pwm: Optional[int] = None


@dataclass(frozen=True)
class PWMCommand:
    """Microsecond servo/ESC command tuple clamped to [1000, 2000] us."""

    esc_us: int
    shift_us: int
    diff_f_us: int
    diff_r_us: int

    def clamped(self) -> "PWMCommand":
        """Return a copy with all channels clamped to [1000, 2000] us."""
        return PWMCommand(
            esc_us=clamp_pwm(self.esc_us),
            shift_us=clamp_pwm(self.shift_us),
            diff_f_us=clamp_pwm(self.diff_f_us),
            diff_r_us=clamp_pwm(self.diff_r_us),
        )

    def to_packet(self) -> str:
        """Format packet as ``$ESC,SHIFT,DIFF_F,DIFF_R\\n``."""
        c = self.clamped()
        return f"${c.esc_us},{c.shift_us},{c.diff_f_us},{c.diff_r_us}\n"


@dataclass
class TelemetryFrame:
    """Complete computed telemetry snapshot for a single HIL cycle."""

    timestamp: float
    scenario: ScenarioParams
    effective_slope_deg: float
    imu_pitch_deg: float
    imu_source: str
    clearance_status: str
    rollover_status: str
    f_g: float
    f_r: float
    f_d: float
    f_req: float
    f_grip: float
    f_avail: float
    load_ratio: float
    state: GovernorState
    drivetrain_label: str
    throttle_limit: float
    pwm: PWMCommand
    buzzer_requested: bool
    buzzer_physical_active: bool = False
    packet_sent: str = ""


def clamp_pwm(pulse_us: int) -> int:
    """Clamp a PWM microsecond command to valid RC servo/ESC bounds."""
    return max(PWM_MIN_US, min(PWM_MAX_US, int(round(pulse_us))))


def is_raspberry_pi() -> bool:
    """Return True if running on Linux Raspberry Pi hardware."""
    if platform.system() != "Linux":
        return False
    model_path = Path("/proc/device-tree/model")
    if model_path.exists():
        try:
            text = model_path.read_text(encoding="utf-8", errors="ignore").lower()
            return "raspberry pi" in text
        except OSError:
            pass
    return False


# ---------------------------------------------------------------------------
# Hardware Abstraction Layer (GPIO, MPU-6050 I2C, Arduino Nano CDC Serial)
# ---------------------------------------------------------------------------

class BaseGPIOInterface:
    """Abstract GPIO interface for Piezo Buzzer (Part 19) & Touch Sensor (Part 12)."""

    backend_name: str = "Base"
    is_mock: bool = True

    def set_buzzer(self, sound_on: bool) -> None:
        raise NotImplementedError

    def read_touch_raw(self) -> bool:
        raise NotImplementedError

    def poll_touch_release(self) -> bool:
        """Return True once per debounced HIGH->LOW release event."""
        raise NotImplementedError

    def cleanup(self) -> None:
        raise NotImplementedError


class MockGPIOInterface(BaseGPIOInterface):
    """Virtual GPIO fallback for dry-run / non-Pi execution."""

    def __init__(self, buzzer_pin: int, touch_pin: int, reason: str) -> None:
        self.backend_name = f"MOCK ({reason})"
        self.is_mock = True
        self.buzzer_pin = buzzer_pin
        self.touch_pin = touch_pin
        self.buzzer_sound_on = False
        self.buzzer_pin_level = 1  # Active-LOW: HIGH = silenced
        self._virtual_touch_queue: queue.SimpleQueue[bool] = queue.SimpleQueue()

    def set_buzzer(self, sound_on: bool) -> None:
        self.buzzer_sound_on = bool(sound_on)
        self.buzzer_pin_level = 0 if self.buzzer_sound_on else 1

    def read_touch_raw(self) -> bool:
        return False

    def trigger_virtual_release(self) -> None:
        """Simulate a capacitive touch release event from software/CLI."""
        self._virtual_touch_queue.put(True)

    def poll_touch_release(self) -> bool:
        triggered = False
        while not self._virtual_touch_queue.empty():
            try:
                self._virtual_touch_queue.get_nowait()
                triggered = True
            except queue.Empty:
                break
        return triggered

    def cleanup(self) -> None:
        self.set_buzzer(False)


class DebouncedReleaseMixin:
    """Software debounce tracker for active-HIGH switch release (HIGH -> LOW)."""

    def _init_debounce(self, hold_s: float = 0.02, cooldown_s: float = 0.20) -> None:
        self._hold_s = hold_s
        self._cooldown_s = cooldown_s
        self._pressed_since: Optional[float] = None
        self._armed = False
        self._last_trigger_ts = 0.0

    def _update_release_detector(self, is_high: bool, now: Optional[float] = None) -> bool:
        ts = time.monotonic() if now is None else now
        if is_high:
            if self._pressed_since is None:
                self._pressed_since = ts
            elif (ts - self._pressed_since) >= self._hold_s:
                self._armed = True
            return False

        # Pin is LOW: fire only if previously held HIGH long enough and outside cooldown.
        triggered = False
        if self._armed and (ts - self._last_trigger_ts) >= self._cooldown_s:
            self._last_trigger_ts = ts
            triggered = True
        self._pressed_since = None
        self._armed = False
        return triggered


class GpioZeroInterface(BaseGPIOInterface):
    """Raspberry Pi 5 compatible GPIO backend using gpiozero (lgpio pin factory)."""

    def __init__(self, buzzer_pin: int, touch_pin: Optional[int]) -> None:
        from gpiozero import Button, DigitalOutputDevice  # type: ignore

        self.backend_name = "gpiozero"
        self.is_mock = False
        self._release_events: queue.SimpleQueue[bool] = queue.SimpleQueue()
        self._last_release_ts = 0.0

        # Active-LOW buzzer: active_high=False -> off() sets pin HIGH (3.3V), on() sets LOW (0V).
        self._buzzer = DigitalOutputDevice(
            buzzer_pin, active_high=False, initial_value=False
        )
        self._touch = None
        if touch_pin is not None:
            self._touch = Button(touch_pin, pull_up=False, bounce_time=0.05)
            self._touch.when_released = self._on_touch_released

    def _on_touch_released(self) -> None:
        now = time.monotonic()
        if (now - self._last_release_ts) >= 0.20:
            self._last_release_ts = now
            self._release_events.put(True)

    def set_buzzer(self, sound_on: bool) -> None:
        if sound_on:
            self._buzzer.on()
        else:
            self._buzzer.off()

    def read_touch_raw(self) -> bool:
        return bool(self._touch.is_pressed) if self._touch is not None else False

    def poll_touch_release(self) -> bool:
        triggered = False
        while not self._release_events.empty():
            try:
                self._release_events.get_nowait()
                triggered = True
            except queue.Empty:
                break
        return triggered

    def cleanup(self) -> None:
        try:
            self._buzzer.off()  # Silence (drive HIGH) before releasing pin
        except Exception:
            pass
        for dev in (getattr(self, "_touch", None), getattr(self, "_buzzer", None)):
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass


class LgpioInterface(BaseGPIOInterface, DebouncedReleaseMixin):
    """Direct RP1 gpiochip backend using lgpio for Raspberry Pi 5."""

    def __init__(self, buzzer_pin: int, touch_pin: Optional[int]) -> None:
        import lgpio  # type: ignore

        self._lgpio = lgpio
        self.buzzer_pin = buzzer_pin
        self.touch_pin = touch_pin
        self.backend_name = "lgpio"
        self.is_mock = False
        self._init_debounce()

        self._chip = None
        last_err: Optional[Exception] = None
        # Pi 5 kernels expose RP1 on gpiochip4 (kernel 6.6) or gpiochip0 (kernel 6.12+).
        for chip_idx in (4, 0):
            try:
                handle = lgpio.gpiochip_open(chip_idx)
                lgpio.gpio_claim_output(handle, buzzer_pin, 1)  # HIGH = silenced
                if touch_pin is not None:
                    pull_down = getattr(lgpio, "SET_PULL_DOWN", 0)
                    lgpio.gpio_claim_input(handle, touch_pin, pull_down)
                self._chip = handle
                self.backend_name = f"lgpio (gpiochip{chip_idx})"
                break
            except Exception as exc:
                last_err = exc
                if self._chip is not None:
                    try:
                        lgpio.gpiochip_close(self._chip)
                    except Exception:
                        pass
                    self._chip = None
        if self._chip is None:
            raise RuntimeError(f"lgpio open failed: {last_err}")

    def set_buzzer(self, sound_on: bool) -> None:
        # Active-LOW: 0 sounds alarm, 1 silences
        self._lgpio.gpio_write(self._chip, self.buzzer_pin, 0 if sound_on else 1)

    def read_touch_raw(self) -> bool:
        if self.touch_pin is None:
            return False
        return bool(self._lgpio.gpio_read(self._chip, self.touch_pin))

    def poll_touch_release(self) -> bool:
        if self.touch_pin is None:
            return False
        return self._update_release_detector(self.read_touch_raw())

    def cleanup(self) -> None:
        if self._chip is not None:
            try:
                self.set_buzzer(False)
            except Exception:
                pass
            try:
                self._lgpio.gpiochip_close(self._chip)
            except Exception:
                pass
            self._chip = None


class RPiGpioInterface(BaseGPIOInterface, DebouncedReleaseMixin):
    """Fallback GPIO backend using RPi.GPIO / rpi-lgpio."""

    def __init__(self, buzzer_pin: int, touch_pin: Optional[int]) -> None:
        import RPi.GPIO as GPIO  # type: ignore

        self._gpio = GPIO
        self.buzzer_pin = buzzer_pin
        self.touch_pin = touch_pin
        self.backend_name = "RPi.GPIO"
        self.is_mock = False
        self._init_debounce()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(buzzer_pin, GPIO.OUT, initial=GPIO.HIGH)
        if touch_pin is not None:
            GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def set_buzzer(self, sound_on: bool) -> None:
        self._gpio.output(
            self.buzzer_pin, self._gpio.LOW if sound_on else self._gpio.HIGH
        )

    def read_touch_raw(self) -> bool:
        if self.touch_pin is None:
            return False
        return bool(self._gpio.input(self.touch_pin))

    def poll_touch_release(self) -> bool:
        if self.touch_pin is None:
            return False
        return self._update_release_detector(self.read_touch_raw())

    def cleanup(self) -> None:
        try:
            self.set_buzzer(False)
        except Exception:
            pass
        pins = [self.buzzer_pin] + ([self.touch_pin] if self.touch_pin is not None else [])
        try:
            self._gpio.cleanup(pins)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# MPU-6050 6-DOF IMU Interface
# ---------------------------------------------------------------------------

class BaseIMUInterface:
    """Abstract IMU interface returning vehicle pitch angle in degrees."""

    backend_name: str = "Base"
    is_mock: bool = True

    def read_pitch_deg(self) -> float:
        raise NotImplementedError

    def close(self) -> None:
        pass


class MockIMUInterface(BaseIMUInterface):
    """Virtual MPU-6050 IMU for dry-run or disconnected I2C."""

    def __init__(self, reason: str, initial_pitch_deg: float = 0.0) -> None:
        self.backend_name = f"MOCK ({reason})"
        self.is_mock = True
        self._pitch_deg = initial_pitch_deg

    def set_simulated_pitch(self, pitch_deg: float) -> None:
        self._pitch_deg = max(-85.0, min(85.0, float(pitch_deg)))

    def adjust_simulated_pitch(self, delta_deg: float) -> float:
        self.set_simulated_pitch(self._pitch_deg + delta_deg)
        return self._pitch_deg

    def read_pitch_deg(self) -> float:
        return self._pitch_deg


class MPU6050Interface(BaseIMUInterface):
    """I2C driver for MPU-6050 6-DOF IMU (Part 29) at 0x68 on I2C1."""

    REG_PWR_MGMT_1 = 0x6B
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_ACCEL_XOUT_H = 0x3B
    REG_WHO_AM_I = 0x75

    ACCEL_SCALE_2G = 16384.0
    GYRO_SCALE_250DPS = 131.0

    def __init__(
        self,
        bus_id: int = I2C_BUS_DEFAULT,
        address: int = MPU6050_ADDR_DEFAULT,
        invert_pitch: bool = False,
        pitch_offset_deg: float = 0.0,
        alpha: float = 0.92,
    ) -> None:
        try:
            import smbus2 as smbus_mod  # type: ignore
        except ImportError:
            import smbus as smbus_mod  # type: ignore

        self.bus_id = bus_id
        self.address = address
        self.invert_pitch = invert_pitch
        self.pitch_offset_deg = pitch_offset_deg
        self.alpha = alpha
        self.is_mock = False
        self._bus = smbus_mod.SMBus(bus_id)
        self._pitch_est_deg: Optional[float] = None
        self._last_ts: Optional[float] = None

        # Wake MPU-6050 and configure +/-2g, +/-250 deg/s, and ~44 Hz DLPF.
        who_am_i = self._bus.read_byte_data(self.address, self.REG_WHO_AM_I)
        self._bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self._bus.write_byte_data(self.address, self.REG_CONFIG, 0x03)
        self._bus.write_byte_data(self.address, self.REG_GYRO_CONFIG, 0x00)
        self._bus.write_byte_data(self.address, self.REG_ACCEL_CONFIG, 0x00)
        self.backend_name = (
            f"MPU-6050 I2C{bus_id} @ 0x{address:02X} (ID 0x{who_am_i:02X})"
        )

    def read_pitch_deg(self) -> float:
        raw = self._bus.read_i2c_block_data(self.address, self.REG_ACCEL_XOUT_H, 14)
        ax_r, ay_r, az_r, _temp_r, _gx_r, gy_r, _gz_r = struct.unpack(
            ">hhhhhhh", bytes(raw)
        )
        ax = ax_r / self.ACCEL_SCALE_2G
        ay = ay_r / self.ACCEL_SCALE_2G
        az = az_r / self.ACCEL_SCALE_2G
        gy_dps = gy_r / self.GYRO_SCALE_250DPS

        accel_pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        now = time.monotonic()

        if self._pitch_est_deg is None or self._last_ts is None:
            self._pitch_est_deg = accel_pitch_deg
        else:
            dt = now - self._last_ts
            if 0.0 < dt <= 0.5:
                gyro_pred = self._pitch_est_deg + gy_dps * dt
                self._pitch_est_deg = (
                    self.alpha * gyro_pred + (1.0 - self.alpha) * accel_pitch_deg
                )
            else:
                self._pitch_est_deg = accel_pitch_deg
        self._last_ts = now

        pitch = -self._pitch_est_deg if self.invert_pitch else self._pitch_est_deg
        return pitch + self.pitch_offset_deg

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass


class ADXL345Interface(BaseIMUInterface):
    """I2C driver for ADXL345 3-Axis Accelerometer (Part 14) at 0x53 on I2C1."""

    REG_DEVID = 0x00
    REG_POWER_CTL = 0x2D
    REG_DATA_FORMAT = 0x31
    REG_DATAX0 = 0x32

    def __init__(
        self,
        bus_id: int = I2C_BUS_DEFAULT,
        address: int = ADXL345_ADDR_DEFAULT,
        invert_pitch: bool = False,
        pitch_offset_deg: float = 0.0,
        alpha: float = 0.25,
    ) -> None:
        try:
            import smbus2 as smbus_mod  # type: ignore
        except ImportError:
            import smbus as smbus_mod  # type: ignore

        self.bus_id = bus_id
        self.address = address
        self.invert_pitch = invert_pitch
        self.pitch_offset_deg = pitch_offset_deg
        self.alpha = alpha
        self.is_mock = False
        self._bus = smbus_mod.SMBus(bus_id)
        self._pitch_est_deg: Optional[float] = None

        dev_id = self._bus.read_byte_data(self.address, self.REG_DEVID)
        if dev_id != 0xE5:
            self._bus.close()
            raise RuntimeError(f"Unexpected ADXL345 DEVID 0x{dev_id:02X}")
        self._bus.write_byte_data(self.address, self.REG_DATA_FORMAT, 0x08)
        self._bus.write_byte_data(self.address, self.REG_POWER_CTL, 0x08)
        time.sleep(0.02)
        self.backend_name = (
            f"ADXL345 I2C{bus_id} @ 0x{address:02X} (ID 0x{dev_id:02X})"
        )

    def read_pitch_deg(self) -> float:
        raw = self._bus.read_i2c_block_data(self.address, self.REG_DATAX0, 6)
        ax_r, ay_r, az_r = struct.unpack("<hhh", bytes(raw))
        ax = ax_r * 0.0039
        ay = ay_r * 0.0039
        az = az_r * 0.0039
        accel_pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        if self._pitch_est_deg is None:
            self._pitch_est_deg = accel_pitch_deg
        else:
            self._pitch_est_deg = (
                self.alpha * accel_pitch_deg + (1.0 - self.alpha) * self._pitch_est_deg
            )
        pitch = -self._pitch_est_deg if self.invert_pitch else self._pitch_est_deg
        return pitch + self.pitch_offset_deg

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Arduino Nano V3.0 Serial Actuator Bridge (Physical CDC & BenchTwin CDP)
# ---------------------------------------------------------------------------

class BaseSerialBridge:
    """Abstract actuator serial bridge."""

    backend_name: str = "Base"
    is_mock: bool = True
    last_rx_line: str = ""

    def send_packet(self, packet: str) -> None:
        raise NotImplementedError

    def send_alarm(self, alarm_on: bool) -> None:
        """Optional hook to forward buzzer/alarm state to simulator bridges."""
        _ = alarm_on

    def close(self) -> None:
        pass


class MockSerialBridge(BaseSerialBridge):
    """Virtual serial bridge recording sent PWM packets."""

    def __init__(self, port: str, baud: int, reason: str) -> None:
        self.backend_name = f"MOCK {port}@{baud} ({reason})"
        self.is_mock = True
        self.port = port
        self.baud = baud
        self.packet_count = 0
        self.last_packet = ""

    def send_packet(self, packet: str) -> None:
        self.last_packet = packet
        self.packet_count += 1


class ArduinoSerialBridge(BaseSerialBridge):
    """USB CDC Serial bridge to Arduino Nano V3.0 (115200 baud, 8N1)."""

    def __init__(
        self,
        port: str = SERIAL_PORT_DEFAULT,
        baud: int = SERIAL_BAUD_DEFAULT,
        boot_delay_s: float = 2.0,
    ) -> None:
        import serial  # type: ignore

        self._serial_mod = serial
        self.is_mock = False
        self._conn = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.5,
        )
        # Nano V3.0 resets on DTR toggle when serial port opens; wait for bootloader.
        if boot_delay_s > 0:
            time.sleep(boot_delay_s)
        try:
            self._conn.reset_input_buffer()
        except Exception:
            pass
        self.backend_name = f"Arduino CDC {port} ({baud} 8N1)"

    def send_packet(self, packet: str) -> None:
        self._conn.write(packet.encode("ascii"))
        self._conn.flush()
        # Drain any incoming telemetry/ACK lines to prevent OS buffer buildup.
        while getattr(self._conn, "in_waiting", 0) > 0:
            line = self._conn.readline().decode("utf-8", errors="replace").strip()
            if line:
                self.last_rx_line = line

    def close(self) -> None:
        try:
            if self._conn.is_open:
                self._conn.close()
        except Exception:
            pass


class _MinimalCDPClient:
    """Zero-dependency RFC 6455 WebSocket client for Chrome DevTools Protocol."""

    def __init__(self, ws_url: str, timeout: float = 2.5) -> None:
        m = re.match(r"^ws://([^:/]+):(\d+)(/.*)$", ws_url)
        if not m:
            raise ValueError(f"Unsupported CDP WebSocket URL: {ws_url}")
        host, port_str, path = m.group(1), m.group(2), m.group(3)
        self._sock = socket.create_connection((host, int(port_str)), timeout=timeout)
        self._sock.settimeout(timeout)
        self._next_id = 1
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host}:{port_str}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self._sock.sendall(req.encode("ascii"))
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("CDP WebSocket handshake closed")
            buf += chunk
        header = buf.split(b"\r\n\r\n", 1)[0].decode("latin1")
        if " 101 " not in header:
            raise ConnectionError(f"CDP handshake failed: {header.splitlines()[0]}")

    def _send_frame(self, opcode: int, payload: bytes) -> None:
        mask = os.urandom(4)
        length = len(payload)
        hdr = bytearray([0x80 | opcode])
        if length < 126:
            hdr.append(0x80 | length)
        elif length < 65536:
            hdr.append(0x80 | 126)
            hdr.extend(struct.pack("!H", length))
        else:
            hdr.append(0x80 | 127)
            hdr.extend(struct.pack("!Q", length))
        hdr.extend(mask)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self._sock.sendall(bytes(hdr) + masked)

    def _recv_exact(self, n: int) -> bytes:
        out = bytearray()
        while len(out) < n:
            chunk = self._sock.recv(n - len(out))
            if not chunk:
                raise ConnectionError("CDP socket closed")
            out.extend(chunk)
        return bytes(out)

    def _recv_frame(self) -> tuple[int, bytes]:
        b1, b2 = self._recv_exact(2)
        opcode = b1 & 0x0F
        masked = bool(b2 & 0x80)
        length = b2 & 0x7F
        if length == 126:
            (length,) = struct.unpack("!H", self._recv_exact(2))
        elif length == 127:
            (length,) = struct.unpack("!Q", self._recv_exact(8))
        mask = self._recv_exact(4) if masked else None
        data = self._recv_exact(length)
        if mask is not None:
            data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        return opcode, data

    def evaluate(self, expression: str) -> Any:
        msg_id = self._next_id
        self._next_id += 1
        payload = json.dumps(
            {
                "id": msg_id,
                "method": "Runtime.evaluate",
                "params": {"expression": expression, "returnByValue": True},
            }
        ).encode("utf-8")
        self._send_frame(0x1, payload)
        while True:
            opcode, data = self._recv_frame()
            if opcode == 0x9:  # Ping -> Pong
                self._send_frame(0xA, data)
                continue
            if opcode == 0x8:
                raise ConnectionError("CDP peer closed connection")
            if opcode != 0x1:
                continue
            resp = json.loads(data.decode("utf-8"))
            if resp.get("id") == msg_id:
                return resp.get("result", {}).get("result", {}).get("value")

    def close(self) -> None:
        try:
            self._send_frame(0x8, b"")
        except Exception:
            pass
        try:
            self._sock.close()
        except Exception:
            pass


class BenchTwinCDPBridge(BaseSerialBridge):
    """Forwards PWM & alarm commands to the live BenchTwin Chrome Serial Monitor via CDP."""

    def __init__(self, cdp_url: str = "http://127.0.0.1:9222") -> None:
        self.cdp_url = cdp_url.rstrip("/")
        self.is_mock = False
        self._lock = threading.Lock()
        self._last_sent_pkt = ""
        self._last_sent_ts = 0.0
        self._last_alarm: Optional[bool] = None
        self._client = self._connect_benchtwin_tab()
        self.backend_name = f"BenchTwin CDP ({self.cdp_url})"
        self._send_serial_line("HOLD")

    def _connect_benchtwin_tab(self) -> _MinimalCDPClient:
        with urllib.request.urlopen(f"{self.cdp_url}/json/list", timeout=2.5) as resp:
            pages = json.loads(resp.read().decode("utf-8"))
        for page in pages:
            if (
                page.get("type") == "page"
                and "benchtwin.com" in page.get("url", "")
                and page.get("webSocketDebuggerUrl")
            ):
                return _MinimalCDPClient(page["webSocketDebuggerUrl"])
        raise RuntimeError(f"No live benchtwin.com page tab found at {self.cdp_url}")

    def _send_serial_line(self, text: str) -> None:
        line = text.strip()
        if not line:
            return
        js = (
            "(() => {"
            "const inp = document.getElementById('mon-text');"
            "if (!inp) return 'NO_INPUT';"
            "const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;"
            f"setter.call(inp, {json.dumps(line)});"
            "inp.dispatchEvent(new Event('input', {bubbles:true}));"
            "const btn = Array.from(inp.parentElement.querySelectorAll('button')).find(b => /send/i.test(b.textContent));"
            "if (btn) { btn.click(); return 'SENT'; }"
            "return 'NO_BTN';"
            "})()"
        )
        res = self._client.evaluate(js)
        self.last_rx_line = f"{line} -> {res}"

    def send_packet(self, packet: str) -> None:
        clean = packet.strip()
        now = time.monotonic()
        with self._lock:
            # Push immediately on PWM change or at gentle keepalive intervals to keep DOM smooth.
            if clean == self._last_sent_pkt and (now - self._last_sent_ts) < 2.0:
                return
            self._send_serial_line(clean)
            self._last_sent_pkt = clean
            self._last_sent_ts = now

    def send_alarm(self, alarm_on: bool) -> None:
        with self._lock:
            if self._last_alarm is alarm_on:
                return
            self._send_serial_line("ALARM:1" if alarm_on else "ALARM:0")
            self._last_alarm = alarm_on

    def close(self) -> None:
        try:
            with self._lock:
                self._send_serial_line("ALARM:0")
        except Exception:
            pass
        self._client.close()


# ---------------------------------------------------------------------------
# Cross-Platform Keyboard Helper & ANSI Terminal Dashboard
# ---------------------------------------------------------------------------

class NonBlockingKeyboard:
    """Single-key non-blocking reader for Linux/Pi (termios) and Windows (msvcrt)."""

    def __init__(self) -> None:
        self._is_windows = platform.system() == "Windows"
        self._termios = None
        self._tty = None
        self._select = None
        self._fd: Optional[int] = None
        self._old_attrs: Any = None
        self._active = False

    def __enter__(self) -> "NonBlockingKeyboard":
        if not sys.stdin or not sys.stdin.isatty():
            return self
        if not self._is_windows:
            try:
                import select
                import termios
                import tty

                self._select = select
                self._termios = termios
                self._tty = tty
                self._fd = sys.stdin.fileno()
                self._old_attrs = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
                self._active = True
            except Exception:
                self._active = False
        else:
            self._active = True
        return self

    def __exit__(self, *_exc: object) -> None:
        if not self._is_windows and self._active and self._old_attrs is not None:
            try:
                self._termios.tcsetattr(
                    self._fd, self._termios.TCSADRAIN, self._old_attrs
                )
            except Exception:
                pass
        self._active = False

    def read_key(self) -> Optional[str]:
        if not self._active:
            return None
        if self._is_windows:
            import msvcrt

            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0") and msvcrt.kbhit():
                ext = msvcrt.getwch()
                return {"H": "UP", "P": "DOWN", "K": "LEFT", "M": "RIGHT"}.get(ext)
            return ch

        ready, _, _ = self._select.select([sys.stdin], [], [], 0)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ready2, _, _ = self._select.select([sys.stdin], [], [], 0.01)
            if ready2 and sys.stdin.read(1) == "[":
                ready3, _, _ = self._select.select([sys.stdin], [], [], 0.01)
                if ready3:
                    code = sys.stdin.read(1)
                    return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(
                        code
                    )
            return "ESC"
        return ch


class TerminalDashboard:
    """Flicker-free ANSI terminal dashboard for TRX-4 HIL telemetry."""

    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    BG_GREEN = "\033[42;30;1m"
    BG_YELLOW = "\033[43;30;1m"
    BG_RED = "\033[41;97;1m"

    def __init__(self) -> None:
        self._vt_enabled = self._enable_vt_if_windows()
        self._cursor_hidden = False
        self._first_draw = True

    @staticmethod
    def _enable_vt_if_windows() -> bool:
        if platform.system() != "Windows":
            return True
        try:
            kernel32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
            handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
            mode = ctypes.c_uint32()
            if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
                # ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
                kernel32.SetConsoleMode(handle, mode.value | 0x0004)
            return True
        except Exception:
            return False

    def enter(self) -> None:
        if sys.stdout.isatty():
            sys.stdout.write("\033[2J\033[H\033[?25l")
            sys.stdout.flush()
            self._cursor_hidden = True
            self._first_draw = True

    def exit(self) -> None:
        if self._cursor_hidden:
            sys.stdout.write("\033[?25h" + self.RESET + "\n")
            sys.stdout.flush()
            self._cursor_hidden = False

    def _state_badge(self, state: GovernorState) -> str:
        if state == GovernorState.FREE:
            return f"{self.BG_GREEN}  STATE 1: FREE   {self.RESET}"
        if state == GovernorState.HARSH:
            return f"{self.BG_YELLOW}  STATE 2: HARSH  {self.RESET}"
        return f"{self.BG_RED}  STATE 3: STOP   {self.RESET}"

    @staticmethod
    def _lambda_bar(load_ratio: float, width: int = 28) -> str:
        capped = max(0.0, min(1.5, load_ratio))
        filled = int(round((capped / 1.5) * width))
        marker_free = int(round((0.45 / 1.5) * width))
        marker_stop = int(round((1.00 / 1.5) * width))
        chars = []
        for idx in range(width):
            if idx == marker_free or idx == marker_stop:
                chars.append("|")
            elif idx < filled:
                chars.append("#")
            else:
                chars.append("-")
        return "[" + "".join(chars) + "]"

    def render(
        self,
        frame: TelemetryFrame,
        scenarios: Sequence[ScenarioParams],
        current_idx: int,
        hw_status: dict[str, str],
        use_live_imu: bool,
        buzzer_mode: str,
        logs: Sequence[str],
        loop_hz: float,
    ) -> None:
        sc = frame.scenario
        w = 88
        sep = "=" * w
        sub = "-" * w

        any_mock = any("MOCK" in v for v in hw_status.values())
        mode_tag = (
            f"{self.YELLOW}[VIRTUAL DRY-RUN]{self.RESET}"
            if any_mock
            else f"{self.GREEN}[RASPBERRY PI 5 LIVE]{self.RESET}"
        )
        imu_tag = (
            f"{self.CYAN}CLOSED-LOOP MPU-6050{self.RESET}"
            if use_live_imu
            else f"{self.DIM}EXCEL PRESET SLOPE{self.RESET}"
        )

        lines: list[str] = [
            f"{self.BOLD}{self.CYAN}{sep}{self.RESET}",
            (
                f"{self.BOLD} TRAXXAS TRX-4 HIL DIGITAL TWIN CONTROLLER{self.RESET}  "
                f"{mode_tag}   Loop: {loop_hz:4.1f} Hz"
            ),
            (
                f" GPIO: {hw_status.get('gpio', '?')} | "
                f"IMU: {hw_status.get('imu', '?')} | "
                f"Bridge: {hw_status.get('serial', '?')}"
            ),
            f"{self.CYAN}{sep}{self.RESET}",
            (
                f" {self.BOLD}SCENARIO [{current_idx + 1}/{len(scenarios)}]:{self.RESET} "
                f"#{sc.scenario_id} {self.WHITE}{sc.track_name}{self.RESET}  "
                f"({sc.surface_type})"
            ),
            (
                f" Slope Mode: {imu_tag} | "
                f"Preset: {sc.slope_deg:5.1f} deg | "
                f"Live IMU: {frame.imu_pitch_deg:+6.2f} deg | "
                f"Active: {self.BOLD}{frame.effective_slope_deg:5.2f} deg{self.RESET}"
            ),
            (
                f" Env Inputs: mu={sc.friction_coeff:.2f}  C_rr={sc.rolling_res:.3f}  "
                f"Depth={sc.mud_depth_m:.3f} m  Wind={sc.wind_speed_kmh:.0f} km/h  "
                f"Temp={sc.temp_c:+.1f} C"
            ),
            (
                f" Safety Checks: Clearance={frame.clearance_status:<13}  "
                f"Rollover={frame.rollover_status:<8}"
            ),
            sub,
            (
                f" {self.BOLD}REAL-TIME TERRAMECHANICS FORCES{self.RESET}                  "
                f"{self.BOLD}GOVERNOR & ACTUATION OUTPUT{self.RESET}"
            ),
            (
                f"   Grade Force   (F_g) : {frame.f_g:7.2f} N               "
                f"{self._state_badge(frame.state)}"
            ),
            (
                f"   Rolling/Sink  (F_r) : {frame.f_r:7.2f} N               "
                f"Drivetrain : {self.BOLD}{frame.drivetrain_label}{self.RESET}"
            ),
            (
                f"   Aero Drag     (F_d) : {frame.f_d:7.2f} N               "
                f"Throttle   : {frame.throttle_limit * 100:5.1f} %"
            ),
            (
                f"   Total Req   (F_req) : {self.BOLD}{frame.f_req:7.2f} N{self.RESET}               "
                f"ESC / Shift: {frame.pwm.esc_us} us / {frame.pwm.shift_us} us"
            ),
            (
                f"   Grip / Avail Force  : {frame.f_grip:6.2f} / {frame.f_avail:5.2f} N        "
                f"Diff F / R : {frame.pwm.diff_f_us} us / {frame.pwm.diff_r_us} us"
            ),
            (
                f"   Load Ratio (lambda) : {self.BOLD}{frame.load_ratio:7.4f}{self.RESET} "
                f"{self._lambda_bar(frame.load_ratio)} (0.45 | 1.00)"
            ),
            (
                f"   Serial Packet (Tx)  : {self.CYAN}{frame.packet_sent.strip():<24}{self.RESET} "
                f"Buzzer(G17): "
                + (
                    f"{self.RED}{self.BOLD}ON ({buzzer_mode.upper()}, LOW=0){self.RESET}"
                    if frame.buzzer_requested
                    else f"{self.GREEN}OFF (HIGH=1){self.RESET}"
                )
                + f"  [Pin={'0' if frame.buzzer_physical_active else '1'}]"
            ),
            sub,
            f" {self.BOLD}SCENARIO MATRIX (Touch Sensor Pin 13 / GPIO 27 Release -> Next):{self.RESET}",
        ]

        # Render two scenarios per row aligned cleanly within 88 columns (44 + 44).
        half = (len(scenarios) + 1) // 2
        for row_i in range(half):
            left_s = scenarios[row_i]
            right_s = scenarios[row_i + half] if (row_i + half) < len(scenarios) else None
            left_mark = ">" if row_i == current_idx else " "
            left_txt = (
                f" {left_mark} [{left_s.scenario_id:2d}] {left_s.track_name[:17]:<17} "
                f"({left_s.slope_deg:4.1f}d,{left_s.surface_type[:9]:<9})"
            )
            if right_s is not None:
                r_idx = row_i + half
                right_mark = ">" if r_idx == current_idx else " "
                right_txt = (
                    f" {right_mark} [{right_s.scenario_id:2d}] {right_s.track_name[:17]:<17} "
                    f"({right_s.slope_deg:4.1f}d,{right_s.surface_type[:9]:<9})"
                )
            else:
                right_txt = ""
            lines.append(f"{left_txt:<44}{right_txt}")

        lines.append(sub)
        lines.append(
            f" {self.DIM}[N] Next(Touch) [P] Prev [1-0] Scen [I] Live-IMU "
            f"[+/-] Pitch [B] Buzzer [R] Load [Q] Quit{self.RESET}"
        )
        term_rows = shutil.get_terminal_size((88, 24)).lines
        avail_log_rows = max(1, min(3, term_rows - len(lines) - 1))
        lines.append(f" {self.BOLD}Event Log:{self.RESET}")
        for entry in list(logs)[-avail_log_rows:]:
            lines.append(f"   {self.DIM}{entry[:w - 4]}{self.RESET}")

        output = ("\033[H" if not self._first_draw else "\033[2J\033[H") + "\n".join(
            f"{line}\033[K" for line in lines
        )
        if not self._first_draw:
            output += "\033[J"
        self._first_draw = False
        sys.stdout.write(output)
        sys.stdout.flush()


# ---------------------------------------------------------------------------
# Main TRX-4 Hardware-in-the-Loop Controller
# ---------------------------------------------------------------------------

class TRX4HILController:
    """Object-oriented HIL Digital Twin Controller for the Traxxas TRX-4."""

    def __init__(
        self,
        config: Optional[HardwareConfig] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.config = config or HardwareConfig()
        self.logger = logger or self._build_default_logger()

        self._lock = threading.RLock()
        self.excel_specs: VehicleSpecs = VehicleSpecs()
        self.specs: VehicleSpecs = VehicleSpecs()
        self.active_model_id: str = "trx4_defender"
        self.scenarios: list[ScenarioParams] = []
        self.surface_presets: list[dict[str, Any]] = [
            dict(p) for p in DEFAULT_SURFACE_PRESETS
        ]
        self.current_scenario_idx: int = 0
        self.condition_mode: ConditionMode = ConditionMode.PRESET
        self.control_mode: ControlMode = ControlMode.AUTO
        self.custom_scenario: Optional[ScenarioParams] = None
        self.use_live_imu: bool = self.config.use_live_imu
        self.buzzer_mode: str = self.config.buzzer_mode

        self.gpio: BaseGPIOInterface = MockGPIOInterface(
            self.config.buzzer_pin, self.config.touch_pin, "uninitialized"
        )
        self.imu: BaseIMUInterface = MockIMUInterface("uninitialized")
        self.serial: BaseSerialBridge = MockSerialBridge(
            self.config.serial_port, self.config.serial_baud, "uninitialized"
        )

        self.hardware_status: dict[str, str] = {}
        self.event_log: list[str] = []
        self.current_pitch_deg: float = 0.0
        self.last_telemetry: Optional[TelemetryFrame] = None
        self.last_packet: str = ""
        self.web_urls: list[str] = []
        self._web_server: Any = None

        self._stop_event = threading.Event()
        self._is_shut_down = False
        self._hw_initialized = False

        self.load_physics(self.config.excel_path)
        self.select_scenario(self.config.initial_scenario, by_id=True)
        self.initialize_hardware()
        self.refresh_telemetry_preview()

    @staticmethod
    def _build_default_logger() -> logging.Logger:
        logger = logging.getLogger("TRX4HILController")
        if not logger.handlers:
            handler = logging.StreamHandler(sys.stdout)
            handler.setFormatter(
                logging.Formatter("[%(levelname)s] %(asctime)s - %(message)s", "%H:%M:%S")
            )
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
        return logger

    def _record_event(self, message: str, level: str = "INFO") -> None:
        stamp = time.strftime("%H:%M:%S")
        with self._lock:
            self.event_log.append(f"[{stamp}] {level}: {message}")
            if len(self.event_log) > 50:
                self.event_log = self.event_log[-50:]

    def _warn_fallback(self, subsystem: str, reason: str) -> None:
        msg = f"{subsystem} fallback to Virtual Dry-Run: {reason}"
        self.logger.warning(msg)
        self._record_event(msg, "WARN")

    @property
    def current_scenario(self) -> ScenarioParams:
        """Return the active terrain/weather scenario (preset or custom Wi-Fi sliders)."""
        with self._lock:
            if not self.scenarios:
                raise RuntimeError("No scenarios loaded. Call load_physics() first.")
            return self.scenarios[self.current_scenario_idx]

    @property
    def active_condition(self) -> ScenarioParams:
        """Return currently effective condition (custom sliders or active preset)."""
        with self._lock:
            if self.condition_mode == ConditionMode.CUSTOM and self.custom_scenario is not None:
                return self.custom_scenario
            return self.current_scenario

    @staticmethod
    def resolve_excel_path(explicit_path: Optional[Union[str, Path]]) -> Path:
        """Locate trx4_hil_simulation_template.xlsx from explicit or standard paths."""
        if explicit_path is not None:
            candidate = Path(explicit_path).expanduser()
            if not candidate.exists():
                raise FileNotFoundError(
                    f"Excel simulation workbook not found at: {candidate}"
                )
            return candidate.resolve()

        candidates = [
            Path.cwd() / DEFAULT_EXCEL_FILENAME,
            Path(__file__).resolve().parent / DEFAULT_EXCEL_FILENAME,
            Path(r"C:\Anul 3\DTMobile") / DEFAULT_EXCEL_FILENAME,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()

        searched = ", ".join(str(p) for p in candidates)
        raise FileNotFoundError(
            f"Could not locate '{DEFAULT_EXCEL_FILENAME}'. Searched: {searched}"
        )

    @staticmethod
    def _col_index_from_ref(cell_ref: str) -> int:
        letters = ""
        for ch in cell_ref:
            if ch.isalpha():
                letters += ch.upper()
            else:
                break
        idx = 0
        for ch in letters:
            idx = idx * 26 + (ord(ch) - ord("A") + 1)
        return max(0, idx - 1)

    @classmethod
    def _read_xlsx_sheets_stdlib(cls, resolved: Path) -> dict[str, list[list[Any]]]:
        """Pure-stdlib OOXML workbook reader preserving column alignment."""
        ns = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"
        rel_ns = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}"
        with zipfile.ZipFile(resolved, "r") as zf:
            shared: list[str] = []
            if "xl/sharedStrings.xml" in zf.namelist():
                ss_root = ET.fromstring(zf.read("xl/sharedStrings.xml"))
                for si in ss_root.findall(f"{ns}si"):
                    parts = [t.text or "" for t in si.iter(f"{ns}t")]
                    shared.append("".join(parts))

            rels: dict[str, str] = {}
            if "xl/_rels/workbook.xml.rels" in zf.namelist():
                rels_root = ET.fromstring(zf.read("xl/_rels/workbook.xml.rels"))
                for rel in rels_root:
                    rid = rel.attrib.get("Id", "")
                    target = rel.attrib.get("Target", "").lstrip("/")
                    if target and not target.startswith("xl/"):
                        target = f"xl/{target}"
                    if rid and target:
                        rels[rid] = target

            wb_root = ET.fromstring(zf.read("xl/workbook.xml"))
            sheets: dict[str, list[list[Any]]] = {}
            for sheet_el in wb_root.iter(f"{ns}sheet"):
                name = sheet_el.attrib.get("name", "")
                rid = sheet_el.attrib.get(f"{rel_ns}id", "")
                target = rels.get(rid, "")
                if not name or not target or target not in zf.namelist():
                    continue
                s_root = ET.fromstring(zf.read(target))
                rows: list[list[Any]] = []
                for r_el in s_root.iter(f"{ns}row"):
                    row_map: dict[int, Any] = {}
                    max_col = -1
                    for c_el in r_el.findall(f"{ns}c"):
                        col_idx = cls._col_index_from_ref(c_el.attrib.get("r", "A1"))
                        max_col = max(max_col, col_idx)
                        c_type = c_el.attrib.get("t", "")
                        val: Any = None
                        if c_type == "inlineStr":
                            is_el = c_el.find(f"{ns}is")
                            if is_el is not None:
                                val = "".join(t.text or "" for t in is_el.iter(f"{ns}t"))
                        else:
                            v_el = c_el.find(f"{ns}v")
                            if v_el is not None and v_el.text is not None:
                                if c_type == "s":
                                    val = shared[int(v_el.text)]
                                elif c_type in ("str", "b", "e"):
                                    val = v_el.text
                                else:
                                    try:
                                        val = float(v_el.text)
                                    except ValueError:
                                        val = v_el.text
                        row_map[col_idx] = val
                    rows.append([row_map.get(i) for i in range(max_col + 1)])
                sheets[name] = rows
            return sheets

    def load_physics(
        self, excel_path: Optional[Union[str, Path]] = None
    ) -> tuple[VehicleSpecs, list[ScenarioParams]]:
        """Load Vehicle_Specs and scenarios from the Excel terramechanics workbook."""
        resolved = self.resolve_excel_path(excel_path or self.config.excel_path)
        sym_map: dict[str, float] = {}
        loaded: list[ScenarioParams] = []
        scenario_sheet = "Track_And_Weather_Scenarios"

        if HAS_PANDAS and pd is not None:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore", category=UserWarning, module=r"openpyxl(\..*)?"
                )
                xl = pd.ExcelFile(resolved, engine="openpyxl")
                sheet_names = xl.sheet_names

                if "Vehicle_Specs" not in sheet_names:
                    raise ValueError(
                        f"Expected sheet 'Vehicle_Specs' in {resolved}. Found: {sheet_names}"
                    )

                specs_df = pd.read_excel(
                    xl, sheet_name="Vehicle_Specs", header=2
                ).dropna(subset=["Symbol", "Value"])

                for _, row in specs_df.iterrows():
                    symbol = str(row["Symbol"]).strip()
                    val = pd.to_numeric(row["Value"], errors="coerce")
                    if symbol and pd.notna(val):
                        sym_map[symbol] = float(val)

                scenario_sheet = ""
                for candidate_sheet in ("Track_And_Weather_Scenarios", "Simulation_Dashboard"):
                    if candidate_sheet in sheet_names:
                        scenario_sheet = candidate_sheet
                        break
                if not scenario_sheet:
                    raise ValueError(
                        f"No scenario sheet found in {resolved}. Available: {sheet_names}"
                    )

                raw_df = pd.read_excel(xl, sheet_name=scenario_sheet, header=None)
                header_row_idx = self._find_header_row(raw_df, ("Scenario_ID", "ID"))
                scen_df = pd.read_excel(xl, sheet_name=scenario_sheet, header=header_row_idx)
                scen_df.columns = [str(c).strip() for c in scen_df.columns]

            id_col = "Scenario_ID" if "Scenario_ID" in scen_df.columns else "ID"
            scen_df["_numeric_id"] = pd.to_numeric(scen_df[id_col], errors="coerce")
            scen_df = scen_df.dropna(subset=["_numeric_id"]).copy()

            for _, row in scen_df.iterrows():
                loaded.append(
                    ScenarioParams(
                        scenario_id=int(row["_numeric_id"]),
                        track_name=str(
                            row.get("Track_Name", row.get("Track Scenario", "Unknown"))
                        ).strip(),
                        surface_type=str(
                            row.get("Surface_Type", row.get("Surface", "Unknown"))
                        ).strip(),
                        slope_deg=float(row.get("Slope_Deg", row.get("Slope (deg)", 0.0))),
                        friction_coeff=float(row.get("Friction_Coeff", 0.5)),
                        rolling_res=float(row.get("Rolling_Res", 0.05)),
                        mud_depth_m=float(row.get("Obstacle_Or_Mud_Depth_m", 0.0)),
                        wind_speed_kmh=float(row.get("Wind_Speed_kmh", 0.0)),
                        temp_c=float(row.get("Temp_C", 20.0)),
                        excel_lambda=self._opt_float(row.get("Load_Ratio_Lambda")),
                        excel_status=self._opt_str(row.get("Sim_Status")),
                        excel_esc_pwm=self._opt_int(row.get("ESC_PWM_Command_us")),
                    )
                )
        else:
            sheets = self._read_xlsx_sheets_stdlib(resolved)
            if "Vehicle_Specs" not in sheets:
                raise ValueError(
                    f"Expected sheet 'Vehicle_Specs' in {resolved}. Found: {list(sheets)}"
                )
            v_rows = sheets["Vehicle_Specs"]
            v_hdr = self._find_header_row_rows(v_rows, ("Symbol", "Value"))
            v_cols = {str(h).strip(): idx for idx, h in enumerate(v_rows[v_hdr]) if h is not None}
            sym_idx = v_cols["Symbol"]
            val_idx = v_cols["Value"]
            for row in v_rows[v_hdr + 1 :]:
                sym = row[sym_idx] if sym_idx < len(row) else None
                val = row[val_idx] if val_idx < len(row) else None
                num = self._opt_float(val)
                if sym is not None and num is not None:
                    sym_map[str(sym).strip()] = num

            scenario_sheet = ""
            for candidate_sheet in ("Track_And_Weather_Scenarios", "Simulation_Dashboard"):
                if candidate_sheet in sheets:
                    scenario_sheet = candidate_sheet
                    break
            if not scenario_sheet:
                raise ValueError(
                    f"No scenario sheet found in {resolved}. Available: {list(sheets)}"
                )
            s_rows = sheets[scenario_sheet]
            s_hdr = self._find_header_row_rows(s_rows, ("Scenario_ID", "ID"))
            s_cols = {str(h).strip(): idx for idx, h in enumerate(s_rows[s_hdr]) if h is not None}
            id_key = "Scenario_ID" if "Scenario_ID" in s_cols else "ID"
            for row in s_rows[s_hdr + 1 :]:
                def _cell(key: str, default: Any = None) -> Any:
                    ci = s_cols.get(key)
                    if ci is None or ci >= len(row):
                        return default
                    return row[ci] if row[ci] is not None else default

                sid = self._opt_int(_cell(id_key))
                if sid is None:
                    continue
                loaded.append(
                    ScenarioParams(
                        scenario_id=sid,
                        track_name=str(_cell("Track_Name", _cell("Track Scenario", "Unknown"))).strip(),
                        surface_type=str(_cell("Surface_Type", _cell("Surface", "Unknown"))).strip(),
                        slope_deg=float(_cell("Slope_Deg", _cell("Slope (deg)", 0.0))),
                        friction_coeff=float(_cell("Friction_Coeff", 0.5)),
                        rolling_res=float(_cell("Rolling_Res", 0.05)),
                        mud_depth_m=float(_cell("Obstacle_Or_Mud_Depth_m", 0.0)),
                        wind_speed_kmh=float(_cell("Wind_Speed_kmh", 0.0)),
                        temp_c=float(_cell("Temp_C", 20.0)),
                        excel_lambda=self._opt_float(_cell("Load_Ratio_Lambda")),
                        excel_status=self._opt_str(_cell("Sim_Status")),
                        excel_esc_pwm=self._opt_int(_cell("ESC_PWM_Command_us")),
                    )
                )

        m = sym_map.get("m", 3.75)
        L = sym_map.get("L", 0.324)
        h_cg = sym_map.get("h_cg", 0.135)
        theta_max = sym_map.get("theta_max")
        if theta_max is None or math.isnan(theta_max):
            theta_max = math.degrees(math.atan((L / 2.0) / h_cg))

        with self._lock:
            loaded_specs = VehicleSpecs(
                m=m,
                L=L,
                h_cg=h_cg,
                h_clear=sym_map.get("h_clear", 0.080),
                r_tire=sym_map.get("r_tire", 0.059),
                A_f=sym_map.get("A_f", 0.065),
                C_d=sym_map.get("C_d", 0.65),
                F_motor_low=sym_map.get("F_motor_low", 45.0),
                F_motor_high=sym_map.get("F_motor_high", 18.0),
                g=sym_map.get("g", 9.81),
                rho_air=sym_map.get("rho_air", 1.225),
                v_target=sym_map.get("v_target", 1.5),
                k_sink=sym_map.get("k_sink", 0.25),
                theta_max=theta_max,
            )
            self.excel_specs = loaded_specs
            MODEL_PROFILES["trx4_defender"]["specs"] = asdict(loaded_specs)
            if self.active_model_id == "trx4_defender":
                self.specs = loaded_specs

            if not loaded:
                raise ValueError(f"Zero valid scenarios parsed from '{scenario_sheet}'.")

            self.scenarios = loaded
            if self.current_scenario_idx >= len(self.scenarios):
                self.current_scenario_idx = 0
            if self.custom_scenario is None:
                self.custom_scenario = self.scenarios[self.current_scenario_idx]

        self._record_event(
            f"Loaded {len(loaded)} scenarios from {resolved.name} ({scenario_sheet})."
        )
        return self.specs, self.scenarios

    @staticmethod
    def _find_header_row(raw_df: Any, markers: tuple[str, ...]) -> int:
        for idx in range(min(15, len(raw_df))):
            row_vals = {
                str(v).strip()
                for v in raw_df.iloc[idx].tolist()
                if v is not None and not (isinstance(v, float) and math.isnan(v))
            }
            if any(m in row_vals for m in markers):
                return idx
        return 2

    @staticmethod
    def _find_header_row_rows(rows: list[list[Any]], markers: tuple[str, ...]) -> int:
        for idx in range(min(15, len(rows))):
            row_vals = {str(v).strip() for v in rows[idx] if v is not None}
            if any(m in row_vals for m in markers):
                return idx
        return 2

    @staticmethod
    def _opt_float(val: Any) -> Optional[float]:
        if val is None or isinstance(val, bool):
            return None
        if isinstance(val, (int, float)):
            return None if math.isnan(float(val)) else float(val)
        try:
            num = float(str(val).strip())
            return None if math.isnan(num) else num
        except ValueError:
            return None

    @staticmethod
    def _opt_int(val: Any) -> Optional[int]:
        num = TRX4HILController._opt_float(val)
        return None if num is None else int(round(num))

    @staticmethod
    def _opt_str(val: Any) -> Optional[str]:
        if val is None or (isinstance(val, float) and math.isnan(val)):
            return None
        text = str(val).strip()
        return text if text else None

    def initialize_hardware(self) -> dict[str, str]:
        """Initialize Pi 5 GPIO, I2C IMU (MPU-6050/ADXL345), and Nano/BenchTwin bridge."""
        if self._hw_initialized:
            return dict(self.hardware_status)

        self._is_shut_down = False
        pi_detected = is_raspberry_pi()

        if self.config.dry_run:
            self._warn_fallback("GPIO", "explicit --dry-run requested")
            self.gpio = MockGPIOInterface(
                self.config.buzzer_pin, self.config.touch_pin, "dry-run"
            )
            self._warn_fallback("IMU", "explicit --dry-run requested")
            self.imu = MockIMUInterface("dry-run", self.active_condition.slope_deg)
            if self.config.benchtwin_cdp_url:
                self.serial = self._init_serial()
            else:
                self._warn_fallback("Serial", "explicit --dry-run requested")
                self.serial = MockSerialBridge(
                    self.config.serial_port, self.config.serial_baud, "dry-run"
                )
        else:
            if not pi_detected:
                self.logger.warning(
                    "Non-Raspberry Pi host detected (%s). Attempting device probes "
                    "with Virtual Dry-Run fallback.",
                    platform.system(),
                )
            self.gpio = self._init_gpio(pi_detected)
            self.imu = self._init_imu()
            self.serial = self._init_serial()

        self.hardware_status = {
            "gpio": self.gpio.backend_name,
            "imu": self.imu.backend_name,
            "serial": self.serial.backend_name,
        }
        self._hw_initialized = True
        return dict(self.hardware_status)

    def _init_gpio(self, pi_detected: bool) -> BaseGPIOInterface:
        if not pi_detected:
            reason = f"non-Pi OS ({platform.system()})"
            self._warn_fallback("GPIO", reason)
            return MockGPIOInterface(self.config.buzzer_pin, self.config.touch_pin, reason)

        touch_arg: Optional[int] = (
            self.config.touch_pin if self.config.enable_touch_button else None
        )
        errors: list[str] = []
        for cls in (GpioZeroInterface, LgpioInterface, RPiGpioInterface):
            try:
                gpio = cls(self.config.buzzer_pin, touch_arg)
                mode_note = "touch+buzzer" if touch_arg is not None else "buzzer-only (Wi-Fi UI)"
                self._record_event(f"GPIO initialized via {gpio.backend_name} ({mode_note}).")
                return gpio
            except Exception as exc:
                errors.append(f"{cls.__name__}: {exc}")

        reason = "; ".join(errors) or "no GPIO library available"
        self._warn_fallback("GPIO", reason)
        return MockGPIOInterface(self.config.buzzer_pin, self.config.touch_pin, "no GPIO")

    def _init_imu(self) -> BaseIMUInterface:
        i2c_dev_str = f"/dev/i2c-{self.config.i2c_bus}"
        if platform.system() != "Linux" or not Path(i2c_dev_str).exists():
            reason = f"{i2c_dev_str} not found"
            self._warn_fallback("IMU", reason)
            return MockIMUInterface(reason, self.active_condition.slope_deg)

        imu_pref = self.config.imu_type.lower()
        attempts: list[tuple[str, Any, int]] = []
        if imu_pref == "mpu6050":
            attempts.append(("MPU-6050", MPU6050Interface, self.config.imu_address))
        elif imu_pref == "adxl345":
            addr = (
                ADXL345_ADDR_DEFAULT
                if self.config.imu_address == MPU6050_ADDR_DEFAULT
                else self.config.imu_address
            )
            attempts.append(("ADXL345", ADXL345Interface, addr))
        else:
            attempts.append(("MPU-6050", MPU6050Interface, self.config.imu_address))
            attempts.append(("ADXL345", ADXL345Interface, ADXL345_ADDR_DEFAULT))

        errors: list[str] = []
        for label, cls, addr in attempts:
            try:
                imu = cls(
                    bus_id=self.config.i2c_bus,
                    address=addr,
                    invert_pitch=self.config.imu_invert_pitch,
                    pitch_offset_deg=self.config.imu_pitch_offset_deg,
                )
                self._record_event(f"IMU initialized via {imu.backend_name}.")
                return imu
            except Exception as exc:
                errors.append(f"{label}@0x{addr:02X}: {exc}")

        reason = "; ".join(errors)
        self._warn_fallback("IMU", reason)
        return MockIMUInterface("I2C unavailable", self.active_condition.slope_deg)

    def _init_serial(self) -> BaseSerialBridge:
        if self.config.benchtwin_cdp_url:
            try:
                bridge = BenchTwinCDPBridge(self.config.benchtwin_cdp_url)
                self._record_event(f"Serial linked to {bridge.backend_name}.")
                return bridge
            except Exception as exc:
                self._warn_fallback("BenchTwin CDP Bridge", str(exc))

        port_path = Path(self.config.serial_port)
        if platform.system() == "Linux" and not port_path.exists():
            reason = f"{self.config.serial_port} not found"
            self._warn_fallback("Arduino Serial", reason)
            return MockSerialBridge(self.config.serial_port, self.config.serial_baud, reason)

        try:
            bridge = ArduinoSerialBridge(
                port=self.config.serial_port,
                baud=self.config.serial_baud,
                boot_delay_s=self.config.arduino_boot_delay_s,
            )
            self._record_event(f"Serial initialized via {bridge.backend_name}.")
            return bridge
        except Exception as exc:
            reason = f"{self.config.serial_port} open failed: {exc}"
            self._warn_fallback("Arduino Serial", reason)
            return MockSerialBridge(
                self.config.serial_port, self.config.serial_baud, "port unavailable"
            )

    def read_imu(self) -> float:
        """Read current vehicle pitch angle (theta_pitch) in degrees from MPU-6050."""
        try:
            self.current_pitch_deg = float(self.imu.read_pitch_deg())
        except Exception as exc:
            self._warn_fallback("IMU runtime read", str(exc))
            self.imu = MockIMUInterface("read error", self.current_pitch_deg)
            self.hardware_status["imu"] = self.imu.backend_name
        return self.current_pitch_deg

    def evaluate_governor(
        self,
        scenario: Optional[ScenarioParams] = None,
        pitch_deg: Optional[float] = None,
    ) -> TelemetryFrame:
        """Compute longitudinal terramechanics forces, Load Ratio, and Governor PWM."""
        with self._lock:
            sc = scenario or self.active_condition
            sp = self.specs
            live_imu = self.use_live_imu
            pitch_now = self.current_pitch_deg
            ctrl_mode = self.control_mode if scenario is None else ControlMode.AUTO

            if pitch_deg is not None:
                effective_slope = float(pitch_deg)
                imu_source = "OVERRIDE"
            elif live_imu:
                effective_slope = pitch_now
                imu_source = "LIVE_IMU"
            else:
                effective_slope = sc.slope_deg
                imu_source = (
                    "WEB_CUSTOM"
                    if (scenario is None and self.condition_mode == ConditionMode.CUSTOM)
                    else "EXCEL_PRESET"
                )

            theta_rad = math.radians(effective_slope)
            cos_theta = math.cos(theta_rad)
            sin_theta = math.sin(theta_rad)

            clearance_status = "HIGH-CENTERED" if sc.mud_depth_m >= sp.h_clear else "PASS"
            rollover_status = "TIP-OVER" if abs(effective_slope) >= sp.theta_max else "PASS"

            # Force balance matching Excel Track_And_Weather_Scenarios formulas:
            f_g = sp.m * sp.g * sin_theta
            f_r = (
                sp.m
                * sp.g
                * cos_theta
                * (sc.rolling_res + sp.k_sink * (sc.mud_depth_m / sp.r_tire))
            )
            v_rel = sp.v_target + (sc.wind_speed_kmh / 3.6)
            f_d = 0.5 * sp.rho_air * sp.C_d * sp.A_f * (v_rel ** 2)
            f_req = f_g + f_r + f_d

            # Cold-weather tire stiffening (<0 C) and LiPo/drivetrain derating (<-5 C):
            temp_grip_factor = 0.85 if sc.temp_c < 0.0 else 1.0
            temp_motor_factor = 0.80 if sc.temp_c < -5.0 else 1.0

            f_grip = (sc.friction_coeff * temp_grip_factor) * sp.m * sp.g * cos_theta
            f_avail = max(0.0, min(f_grip, sp.F_motor_low * temp_motor_factor))
            load_ratio = (f_req / f_avail) if f_avail > 0.0 else 99.9

            # 3-State Governor Logic (or remote Wi-Fi mode override)
            if ctrl_mode == ControlMode.FORCE_STOP:
                state = GovernorState.STOP
                drivetrain_label = "LOW / LOCKED (E-STOP)"
                throttle_limit = 0.0
                pwm = PWMCommand(
                    esc_us=PWM_ESC_NEUTRAL_US,
                    shift_us=PWM_LOW_GEAR_US,
                    diff_f_us=PWM_DIFF_LOCKED_US,
                    diff_r_us=PWM_DIFF_LOCKED_US,
                )
                buzzer_requested = True
            elif ctrl_mode == ControlMode.FORCE_FREE:
                state = GovernorState.FREE
                drivetrain_label = "HIGH / OPEN (FORCED)"
                throttle_limit = 1.0
                pwm = PWMCommand(
                    esc_us=PWM_ESC_FULL_US,
                    shift_us=PWM_HIGH_GEAR_US,
                    diff_f_us=PWM_DIFF_OPEN_US,
                    diff_r_us=PWM_DIFF_OPEN_US,
                )
                buzzer_requested = False
            elif ctrl_mode == ControlMode.FORCE_HARSH:
                state = GovernorState.HARSH
                drivetrain_label = "LOW / LOCKED (FORCED)"
                throttle_limit = 0.25
                pwm = PWMCommand(
                    esc_us=1625,
                    shift_us=PWM_LOW_GEAR_US,
                    diff_f_us=PWM_DIFF_LOCKED_US,
                    diff_r_us=PWM_DIFF_LOCKED_US,
                )
                buzzer_requested = False
            elif clearance_status != "PASS" or rollover_status != "PASS" or load_ratio > 1.0:
                state = GovernorState.STOP
                drivetrain_label = "LOW / LOCKED"
                throttle_limit = 0.0
                pwm = PWMCommand(
                    esc_us=PWM_ESC_NEUTRAL_US,
                    shift_us=PWM_LOW_GEAR_US,
                    diff_f_us=PWM_DIFF_LOCKED_US,
                    diff_r_us=PWM_DIFF_LOCKED_US,
                )
                buzzer_requested = True
            elif load_ratio <= 0.45:
                state = GovernorState.FREE
                drivetrain_label = "HIGH / OPEN"
                throttle_limit = 1.0
                pwm = PWMCommand(
                    esc_us=PWM_ESC_FULL_US,
                    shift_us=PWM_HIGH_GEAR_US,
                    diff_f_us=PWM_DIFF_OPEN_US,
                    diff_r_us=PWM_DIFF_OPEN_US,
                )
                buzzer_requested = False
            else:
                state = GovernorState.HARSH
                drivetrain_label = "LOW / LOCKED"
                throttle_limit = max(0.25, min(1.0, round(1.15 - load_ratio, 2)))
                esc_cmd = int(round(PWM_ESC_NEUTRAL_US + 500.0 * throttle_limit))
                pwm = PWMCommand(
                    esc_us=esc_cmd,
                    shift_us=PWM_LOW_GEAR_US,
                    diff_f_us=PWM_DIFF_LOCKED_US,
                    diff_r_us=PWM_DIFF_LOCKED_US,
                ).clamped()
                buzzer_requested = False

            frame = TelemetryFrame(
                timestamp=time.time(),
                scenario=sc,
                effective_slope_deg=effective_slope,
                imu_pitch_deg=pitch_now,
                imu_source=imu_source,
                clearance_status=clearance_status,
                rollover_status=rollover_status,
                f_g=f_g,
                f_r=f_r,
                f_d=f_d,
                f_req=f_req,
                f_grip=f_grip,
                f_avail=f_avail,
                load_ratio=load_ratio,
                state=state,
                drivetrain_label=drivetrain_label,
                throttle_limit=throttle_limit,
                pwm=pwm.clamped(),
                buzzer_requested=buzzer_requested,
            )
            self.last_telemetry = frame
            return frame

    def refresh_telemetry_preview(self) -> TelemetryFrame:
        """Compute updated telemetry immediately after a Wi-Fi configuration change."""
        frame = self.evaluate_governor()
        with self._lock:
            frame.packet_sent = frame.pwm.to_packet()
        return frame

    def send_actuator_packet(
        self, esc_us: int, shift_us: int, diff_f_us: int, diff_r_us: int
    ) -> str:
        """Format and transmit ``$ESC,SHIFT,DIFF_F,DIFF_R\\n`` to Arduino Nano."""
        packet = PWMCommand(esc_us, shift_us, diff_f_us, diff_r_us).to_packet()
        try:
            self.serial.send_packet(packet)
        except Exception as exc:
            self._warn_fallback("Serial Tx", str(exc))
            self.serial = MockSerialBridge(
                self.config.serial_port, self.config.serial_baud, "Tx error"
            )
            self.serial.send_packet(packet)
            self.hardware_status["serial"] = self.serial.backend_name

        self.last_packet = packet
        return packet

    def _apply_buzzer(self, alarm_requested: bool, now: Optional[float] = None) -> bool:
        """Drive active-LOW piezo buzzer continuously or pulsed based on mode."""
        if not alarm_requested:
            self.gpio.set_buzzer(False)
            return False

        if self.buzzer_mode == "continuous":
            physical_on = True
        else:
            ts = time.monotonic() if now is None else now
            period = 1.0 / max(0.5, self.config.buzzer_pulse_hz)
            physical_on = (ts % period) < (0.5 * period)

        try:
            self.gpio.set_buzzer(physical_on)
        except Exception as exc:
            self._warn_fallback("GPIO Buzzer", str(exc))
        return physical_on

    def select_scenario(self, index_or_id: int, by_id: bool = False) -> ScenarioParams:
        """Select active preset scenario by 0-based index or Scenario_ID."""
        with self._lock:
            if by_id:
                for idx, sc in enumerate(self.scenarios):
                    if sc.scenario_id == index_or_id:
                        self.current_scenario_idx = idx
                        break
                else:
                    self.current_scenario_idx = max(
                        0, min(len(self.scenarios) - 1, index_or_id - 1)
                    )
            else:
                self.current_scenario_idx = index_or_id % len(self.scenarios)

            self.condition_mode = ConditionMode.PRESET
            sc = self.scenarios[self.current_scenario_idx]
            if isinstance(self.imu, MockIMUInterface):
                self.imu.set_simulated_pitch(sc.slope_deg)
        self._record_event(f"Wi-Fi/Scenario -> #{sc.scenario_id} {sc.track_name}")
        return sc

    def set_custom_conditions(self, params: dict[str, Any]) -> ScenarioParams:
        """Apply custom terrain & weather parameters sent from PC Web UI."""
        with self._lock:
            base = self.custom_scenario or self.current_scenario
            slope = float(params.get("slope_deg", base.slope_deg))
            mu = float(params.get("friction_coeff", base.friction_coeff))
            crr = float(params.get("rolling_res", base.rolling_res))
            depth = float(params.get("mud_depth_m", base.mud_depth_m))
            wind = float(params.get("wind_speed_kmh", base.wind_speed_kmh))
            temp = float(params.get("temp_c", base.temp_c))
            track = str(params.get("track_name", base.track_name)).strip() or "Custom Wi-Fi Trail"
            surface = str(params.get("surface_type", base.surface_type)).strip() or "Custom Surface"

            updated = ScenarioParams(
                scenario_id=0,
                track_name=track[:48],
                surface_type=surface[:36],
                slope_deg=max(-60.0, min(60.0, slope)),
                friction_coeff=max(0.05, min(1.5, mu)),
                rolling_res=max(0.001, min(0.6, crr)),
                mud_depth_m=max(0.0, min(0.25, depth)),
                wind_speed_kmh=max(0.0, min(150.0, wind)),
                temp_c=max(-40.0, min(60.0, temp)),
            )
            self.custom_scenario = updated
            self.condition_mode = ConditionMode.CUSTOM
            if isinstance(self.imu, MockIMUInterface):
                self.imu.set_simulated_pitch(updated.slope_deg)

        self._record_event(
            f"Custom conditions -> {updated.surface_type}, {updated.slope_deg:.1f}deg, mu={updated.friction_coeff:.2f}"
        )
        return updated

    def apply_remote_control(
        self,
        control_mode: Optional[str] = None,
        use_live_imu: Optional[bool] = None,
        buzzer_mode: Optional[str] = None,
    ) -> None:
        """Update governor mode override, IMU slope flag, or buzzer mode from Web UI."""
        events: list[str] = []
        with self._lock:
            if control_mode is not None:
                parsed = ControlMode(str(control_mode).upper())
                if parsed != self.control_mode:
                    self.control_mode = parsed
                    events.append(f"Drive mode -> {parsed.value}")
            if use_live_imu is not None:
                flag = bool(use_live_imu)
                if flag != self.use_live_imu:
                    self.use_live_imu = flag
                    events.append(f"Slope source -> {'LIVE_IMU' if flag else 'WEB/PRESET'}")
            if buzzer_mode is not None:
                bm = str(buzzer_mode).lower()
                if bm not in ("pulse", "continuous"):
                    raise ValueError("buzzer_mode must be 'pulse' or 'continuous'")
                if bm != self.buzzer_mode:
                    self.buzzer_mode = bm
                    events.append(f"Buzzer pattern -> {bm.upper()}")
        for ev in events:
            self._record_event(ev)

    def select_model(self, model_id: str) -> dict[str, Any]:
        """Switch active Digital Twin vehicle profile and terramechanics specs."""
        key = str(model_id).strip()
        if key not in MODEL_PROFILES:
            raise ValueError(f"Unknown model_id '{key}'. Available: {list(MODEL_PROFILES)}")
        profile = MODEL_PROFILES[key]
        with self._lock:
            self.active_model_id = key
            if key == "trx4_defender":
                self.specs = self.excel_specs
            else:
                self.specs = VehicleSpecs(**profile["specs"])
        self._record_event(f"Active model -> {profile['name']} ({profile['category']})")
        return dict(profile)

    def set_custom_specs(self, params: dict[str, Any]) -> VehicleSpecs:
        """Apply custom vehicle physical parameters from the Digital Twin Web UI."""
        with self._lock:
            cur = self.specs
            m = max(0.2, min(50.0, float(params.get("m", cur.m))))
            L = max(0.05, min(2.0, float(params.get("L", cur.L))))
            h_cg = max(0.02, min(1.0, float(params.get("h_cg", cur.h_cg))))
            h_clear = max(0.005, min(0.5, float(params.get("h_clear", cur.h_clear))))
            r_tire = max(0.01, min(0.4, float(params.get("r_tire", cur.r_tire))))
            A_f = max(0.005, min(1.0, float(params.get("A_f", cur.A_f))))
            C_d = max(0.05, min(2.5, float(params.get("C_d", cur.C_d))))
            F_motor_low = max(1.0, min(500.0, float(params.get("F_motor_low", cur.F_motor_low))))
            F_motor_high = max(1.0, min(500.0, float(params.get("F_motor_high", cur.F_motor_high))))
            updated = VehicleSpecs(
                m=m,
                L=L,
                h_cg=h_cg,
                h_clear=h_clear,
                r_tire=r_tire,
                A_f=A_f,
                C_d=C_d,
                F_motor_low=F_motor_low,
                F_motor_high=F_motor_high,
                g=cur.g,
                rho_air=cur.rho_air,
                v_target=float(params.get("v_target", cur.v_target)),
                k_sink=float(params.get("k_sink", cur.k_sink)),
                theta_max=_calc_theta_max(L, h_cg),
            )
            self.specs = updated
            self.active_model_id = "custom_model"
        self._record_event(
            f"Custom vehicle specs -> m={m:.2f}kg, L={L:.3f}m, F_low={F_motor_low:.1f}N"
        )
        return updated

    def get_state_snapshot(self) -> dict[str, Any]:
        """Return thread-safe JSON-serializable state snapshot for the Wi-Fi Web UI."""
        with self._lock:
            frame = self.last_telemetry or self.evaluate_governor()
            telem_dict = asdict(frame)
            telem_dict["state"] = frame.state.value
            return {
                "hostname": socket.gethostname(),
                "active_model_id": self.active_model_id,
                "specs": asdict(self.specs),
                "models": MODEL_PROFILES,
                "condition_mode": self.condition_mode.value,
                "control_mode": self.control_mode.value,
                "use_live_imu": self.use_live_imu,
                "buzzer_mode": self.buzzer_mode,
                "enable_touch_button": self.config.enable_touch_button,
                "loop_hz": self.config.loop_hz,
                "web_urls": list(self.web_urls),
                "hardware_status": dict(self.hardware_status),
                "scenarios": [asdict(s) for s in self.scenarios],
                "surface_presets": list(self.surface_presets),
                "telemetry": telem_dict,
                "event_log": list(self.event_log[-25:]),
            }

    def next_scenario(self) -> ScenarioParams:
        """Advance to the next scenario (wrapping after Scenario 10)."""
        with self._lock:
            idx = (self.current_scenario_idx + 1) % len(self.scenarios)
        return self.select_scenario(idx)

    def prev_scenario(self) -> ScenarioParams:
        """Return to the previous scenario."""
        with self._lock:
            idx = (self.current_scenario_idx - 1) % len(self.scenarios)
        return self.select_scenario(idx)

    def step(self) -> TelemetryFrame:
        """Execute one deterministic HIL sense -> compute -> actuate cycle."""
        if self.config.enable_touch_button:
            try:
                if self.gpio.poll_touch_release():
                    sc = self.next_scenario()
                    self._record_event(
                        f"Touch sensor (GPIO {self.config.touch_pin}) released -> #{sc.scenario_id}"
                    )
            except Exception as exc:
                self._warn_fallback("GPIO Touch", str(exc))

        self.read_imu()
        frame = self.evaluate_governor()
        frame.buzzer_physical_active = self._apply_buzzer(frame.buzzer_requested)
        frame.packet_sent = self.send_actuator_packet(
            frame.pwm.esc_us,
            frame.pwm.shift_us,
            frame.pwm.diff_f_us,
            frame.pwm.diff_r_us,
        )
        try:
            self.serial.send_alarm(frame.buzzer_requested)
        except Exception as exc:
            self._warn_fallback("Serial Alarm", str(exc))
        with self._lock:
            self.last_telemetry = frame
        return frame

    def verify_against_excel(self) -> tuple[bool, list[str]]:
        """Verify Python physics calculations match Excel cached values across all scenarios."""
        report: list[str] = []
        all_ok = True
        saved_live = self.use_live_imu
        saved_specs = self.specs
        self.use_live_imu = False
        self.specs = self.excel_specs
        try:
            for sc in self.scenarios:
                frame = self.evaluate_governor(scenario=sc, pitch_deg=sc.slope_deg)
                lam_diff = (
                    abs(frame.load_ratio - sc.excel_lambda)
                    if sc.excel_lambda is not None
                    else 0.0
                )
                status_ok = (
                    sc.excel_status is None or frame.state.value == sc.excel_status
                )
                pwm_ok = (
                    sc.excel_esc_pwm is None or frame.pwm.esc_us == sc.excel_esc_pwm
                )
                row_ok = (lam_diff < 1e-6) and status_ok and pwm_ok
                all_ok = all_ok and row_ok
                mark = "PASS" if row_ok else "FAIL"
                report.append(
                    f"[{mark}] #{sc.scenario_id:2d} {sc.track_name:<23} | "
                    f"Fg={frame.f_g:5.2f} Fr={frame.f_r:5.2f} Fd={frame.f_d:5.2f} "
                    f"Freq={frame.f_req:5.2f} Favail={frame.f_avail:5.2f} | "
                    f"lambda={frame.load_ratio:7.4f} (xl={sc.excel_lambda or 0:7.4f}) | "
                    f"{frame.state.value:<5} | {frame.pwm.to_packet().strip()}"
                )
        finally:
            self.use_live_imu = saved_live
            self.specs = saved_specs
        return all_ok, report

    def _handle_keyboard_command(self, key: str) -> bool:
        """Process interactive keyboard input. Returns False when user requests quit."""
        if key in ("q", "Q", "\x03"):
            self._record_event("Quit requested by operator.")
            return False
        if key in ("n", "N", " ", "\r", "\n"):
            self.next_scenario()
        elif key in ("p", "P"):
            self.prev_scenario()
        elif key in ("1", "2", "3", "4", "5", "6", "7", "8", "9"):
            self.select_scenario(int(key), by_id=True)
        elif key == "0":
            self.select_scenario(10, by_id=True)
        elif key in ("i", "I"):
            self.apply_remote_control(use_live_imu=not self.use_live_imu)
        elif key in ("+", "=", "UP"):
            if isinstance(self.imu, MockIMUInterface):
                val = self.imu.adjust_simulated_pitch(+2.0)
                self._record_event(f"Mock IMU pitch -> {val:+.1f} deg")
        elif key in ("-", "_", "DOWN"):
            if isinstance(self.imu, MockIMUInterface):
                val = self.imu.adjust_simulated_pitch(-2.0)
                self._record_event(f"Mock IMU pitch -> {val:+.1f} deg")
        elif key in ("b", "B"):
            self.apply_remote_control(
                buzzer_mode="continuous" if self.buzzer_mode == "pulse" else "pulse"
            )
        elif key in ("r", "R"):
            try:
                self.load_physics(self.config.excel_path)
            except Exception as exc:
                self._record_event(f"Excel reload failed: {exc}", "ERR")
        return True

    def request_stop(self) -> None:
        """Signal-safe request to exit the main control loop."""
        self._stop_event.set()

    def _start_web_dashboard_if_enabled(self) -> None:
        if not self.config.web_enabled:
            return
        from trx4_web_server import TRX4WebDashboardServer

        srv = TRX4WebDashboardServer(
            controller=self,
            host=self.config.web_host,
            port=self.config.web_port,
        )
        urls = srv.start()
        self._web_server = srv
        self.web_urls = urls
        self._record_event(f"Wi-Fi Web Dashboard started: {urls[0]}")
        self.logger.info("=" * 64)
        self.logger.info("TRX-4 Pi Wi-Fi Web Dashboard running! Open from your computer:")
        for url in urls:
            self.logger.info("  -> %s", url)
        self.logger.info("=" * 64)

    def run(
        self,
        max_cycles: Optional[int] = None,
        interactive: bool = True,
        auto_cycle_sec: Optional[float] = None,
    ) -> None:
        """Run the real-time HIL control loop until SIGINT, quit, or max_cycles."""
        self._stop_event.clear()
        cycle_period = 1.0 / max(1.0, self.config.loop_hz)
        auto_sec = (
            auto_cycle_sec if auto_cycle_sec is not None else self.config.auto_cycle_sec
        )
        last_auto_ts = time.monotonic()

        is_main = threading.current_thread() is threading.main_thread()
        prev_sigint = signal.getsignal(signal.SIGINT) if is_main else None
        prev_sigterm = (
            signal.getsignal(signal.SIGTERM)
            if (is_main and hasattr(signal, "SIGTERM"))
            else None
        )

        def _sig_handler(_signum: int, _frame: Any) -> None:
            self.request_stop()

        if is_main:
            signal.signal(signal.SIGINT, _sig_handler)
            if prev_sigterm is not None:
                signal.signal(signal.SIGTERM, _sig_handler)

        dashboard: Optional[TerminalDashboard] = None
        cycles = 0
        last_logged_sig = ""

        try:
            self._start_web_dashboard_if_enabled()
            use_tui = interactive and sys.stdout.isatty() and not self.config.web_enabled
            dashboard = TerminalDashboard() if use_tui else None
            if dashboard is not None:
                dashboard.enter()

            with NonBlockingKeyboard() as kb:
                while not self._stop_event.is_set():
                    t_start = time.monotonic()

                    if auto_sec and (t_start - last_auto_ts) >= auto_sec:
                        self.next_scenario()
                        last_auto_ts = t_start

                    if interactive:
                        key = kb.read_key()
                        if key is not None and not self._handle_keyboard_command(key):
                            break

                    frame = self.step()
                    cycles += 1

                    if dashboard is not None:
                        dashboard.render(
                            frame=frame,
                            scenarios=self.scenarios,
                            current_idx=self.current_scenario_idx,
                            hw_status=self.hardware_status,
                            use_live_imu=self.use_live_imu,
                            buzzer_mode=self.buzzer_mode,
                            logs=self.event_log,
                            loop_hz=self.config.loop_hz,
                        )
                    else:
                        sig = (
                            f"{frame.scenario.scenario_id}:{frame.scenario.track_name}:"
                            f"{frame.state.value}:{frame.packet_sent.strip()}:{frame.buzzer_requested}"
                        )
                        if not self.config.web_enabled or sig != last_logged_sig:
                            last_logged_sig = sig
                            cond_kind = "CUSTOM" if frame.scenario.scenario_id == 0 else "PRESET"
                            self.logger.info(
                                "Cond [%s] (%s) | slope=%.1f deg | Freq=%.2fN Favail=%.2fN | "
                                "lambda=%.3f | %s | Tx=%s | Buzzer=%s",
                                cond_kind,
                                frame.scenario.track_name,
                                frame.effective_slope_deg,
                                frame.f_req,
                                frame.f_avail,
                                frame.load_ratio,
                                frame.state.value,
                                frame.packet_sent.strip(),
                                "ON" if frame.buzzer_requested else "OFF",
                            )

                    if max_cycles is not None and cycles >= max_cycles:
                        break

                    elapsed = time.monotonic() - t_start
                    remaining = cycle_period - elapsed
                    if remaining > 0.0:
                        self._stop_event.wait(remaining)
        finally:
            if dashboard is not None:
                dashboard.exit()
            if is_main and prev_sigint is not None:
                signal.signal(signal.SIGINT, prev_sigint)
            if is_main and prev_sigterm is not None:
                signal.signal(signal.SIGTERM, prev_sigterm)
            self.shutdown()

    def shutdown(self) -> None:
        """Stop Web Server, safely reset actuators to (1500,1900,1900,1900), silence buzzer."""
        if self._is_shut_down:
            return
        self._is_shut_down = True

        if self._web_server is not None:
            try:
                self._web_server.stop()
            except Exception as exc:
                self.logger.warning("Web server stop raised: %s", exc)
            self._web_server = None

        safe_pkt = self.send_actuator_packet(*SAFE_SHUTDOWN_PWM)
        self._record_event(f"Safe shutdown packet sent: {safe_pkt.strip()}")

        for action_name, action in (
            ("GPIO silence & cleanup", self.gpio.cleanup),
            ("IMU close", self.imu.close),
            ("Serial close", self.serial.close),
        ):
            try:
                action()
            except Exception as exc:
                self.logger.warning("Cleanup step '%s' raised: %s", action_name, exc)

        self.logger.info(
            "Clean exit complete. Actuators reset to %s, Buzzer OFF, GPIOs released.",
            safe_pkt.strip(),
        )

    def __enter__(self) -> "TRX4HILController":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.shutdown()


# ---------------------------------------------------------------------------
# CLI Entrypoint
# ---------------------------------------------------------------------------

def parse_int_auto_base(value: str) -> int:
    """Parse decimal or hex (0x68) integers for CLI options."""
    return int(value, 0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Traxxas TRX-4 Hardware-in-the-Loop (HIL) Raspberry Pi 5 Controller"
    )
    parser.add_argument(
        "--excel",
        type=Path,
        default=None,
        help="Path to trx4_hil_simulation_template.xlsx",
    )
    parser.add_argument(
        "--port",
        default=SERIAL_PORT_DEFAULT,
        help=f"Arduino Nano serial port (default: {SERIAL_PORT_DEFAULT})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=SERIAL_BAUD_DEFAULT,
        help=f"Serial baud rate (default: {SERIAL_BAUD_DEFAULT})",
    )
    parser.add_argument(
        "--i2c-bus",
        type=int,
        default=I2C_BUS_DEFAULT,
        help=f"Linux I2C bus ID for IMU (default: {I2C_BUS_DEFAULT})",
    )
    parser.add_argument(
        "--imu-addr",
        type=parse_int_auto_base,
        default=MPU6050_ADDR_DEFAULT,
        help="IMU I2C address (default: 0x68 for MPU-6050, auto uses 0x53 for ADXL345)",
    )
    parser.add_argument(
        "--imu-type",
        choices=("auto", "mpu6050", "adxl345"),
        default="auto",
        help="I2C IMU chip selection (default: auto)",
    )
    parser.add_argument(
        "--live-imu",
        action="store_true",
        help="Substitute preset/UI slope with live I2C IMU pitch (Fg = m*g*sin(theta_pitch))",
    )
    parser.add_argument(
        "--imu-invert",
        action="store_true",
        help="Invert IMU pitch sign for reversed sensor mounting",
    )
    parser.add_argument(
        "--imu-offset",
        type=float,
        default=0.0,
        help="Static zero-pitch calibration offset in degrees",
    )
    parser.add_argument(
        "--enable-touch",
        action="store_true",
        help="Re-enable legacy GPIO 27 capacitive touch button cycling (disabled in Wi-Fi mode)",
    )
    parser.add_argument(
        "--web",
        action="store_true",
        help="Start Pi-hosted Wi-Fi Web Dashboard server (http://<pi-ip>:5000)",
    )
    parser.add_argument(
        "--web-host",
        default=WEB_HOST_DEFAULT,
        help=f"Web Dashboard bind host (default: {WEB_HOST_DEFAULT})",
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=WEB_PORT_DEFAULT,
        help=f"Web Dashboard HTTP port (default: {WEB_PORT_DEFAULT})",
    )
    parser.add_argument(
        "--benchtwin-cdp",
        default=None,
        metavar="URL",
        help="Forward PWM & alarm packets to live BenchTwin Chrome CDP (e.g. http://127.0.0.1:9222)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Force Virtual Dry-Run mock hardware mode",
    )
    parser.add_argument(
        "--scenario",
        type=int,
        default=1,
        help="Initial scenario ID 1..10 (default: 1)",
    )
    parser.add_argument(
        "--auto-cycle",
        type=float,
        default=None,
        metavar="SEC",
        help="Automatically advance scenario every SEC seconds",
    )
    parser.add_argument(
        "--buzzer-mode",
        choices=("pulse", "continuous"),
        default="pulse",
        help="Piezo alarm pattern during STOP state (default: pulse)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=15.0,
        help="Control loop frequency in Hz (default: 15.0)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=None,
        help="Run a fixed number of HIL cycles then exit safely",
    )
    parser.add_argument(
        "--no-tui",
        action="store_true",
        help="Disable ANSI dashboard and emit plain structured log lines",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Verify all 10 scenarios against Excel workbook formulas and exit",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    config = HardwareConfig(
        excel_path=args.excel,
        enable_touch_button=args.enable_touch,
        i2c_bus=args.i2c_bus,
        imu_address=args.imu_addr,
        imu_type=args.imu_type,
        serial_port=args.port,
        serial_baud=args.baud,
        loop_hz=args.hz,
        buzzer_mode=args.buzzer_mode,
        use_live_imu=args.live_imu,
        imu_invert_pitch=args.imu_invert,
        imu_pitch_offset_deg=args.imu_offset,
        dry_run=args.dry_run,
        initial_scenario=args.scenario,
        auto_cycle_sec=args.auto_cycle,
        web_enabled=args.web,
        web_host=args.web_host,
        web_port=args.web_port,
        benchtwin_cdp_url=args.benchtwin_cdp,
    )

    with TRX4HILController(config=config) as ctrl:
        if args.verify:
            ok, lines = ctrl.verify_against_excel()
            for line in lines:
                print(line)
            print(f"\nExcel Parity Verification: {'ALL PASS' if ok else 'MISMATCH FOUND'}")
            return 0 if ok else 1

        ctrl.run(max_cycles=args.cycles, interactive=not args.no_tui)
    return 0


if __name__ == "__main__":
    sys.exit(main())
