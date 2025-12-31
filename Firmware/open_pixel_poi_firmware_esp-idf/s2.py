#!/usr/bin/env python3
"""
Pixel Poi multi-device full-packet streamer (ESP-IDF/NimBLE)

New:
- Mode `text "<your text>"` renders POV text columns (5x7 font, auto-scaled to your LED count)
- Text options:
    * text speed COLUMNS_PER_SEC     (how fast the text scrolls)
    * text color #RRGGBB             (foreground color)
    * text bg    #RRGGBB             (background color)
- Scans for BLE devices whose name contains "Pixel Poi" (case-insensitive), lets you select devices,
  and streams the SAME frames sequentially to all connected devices to keep them in sync.

Includes fully smooth pulse modes (no hard jumps), plus: walking, stripes, theater, comet, twinkle,
wipe, breathe, noise, sparkle, fire (intensity), rainbow_wave, wave (base color sine), strobe, file streaming.

Interactive console commands:
  scan                         -> discover Pixel Poi devices
  devices                      -> list discovered devices
  connect all                  -> connect to all discovered devices
  connect <idx> [idx ...]      -> connect selected indices (from 'devices' list)
  start / stop / quit
  mode [walking|pulse|pulse_smooth|pulse_palette|stripes|random|theater|comet|twinkle|wipe|breathe|noise|sparkle|fire|rainbow_wave|wave|strobe|text|file]
  brightness N                 -> 0..255 (sent to all devices)
  gear I                       -> 0..5  (sent to all devices)
  speed rainbow Hz             -> walking rainbow hue speed
  speed pulse Hz               -> pulse speed (lower = slower, smoother)
  speed stripes LEDS           -> stripes speed (LEDs/sec)
  speed noise Hz               -> noise rainbow flow
  speed sparkle Hz             -> sparkle drift
  speed wave Hz                -> wave speed
  speed strobe Hz              -> strobe rate
  stripes width N              -> stripe width
  stripes dir +1|-1            -> stripe direction
  pulse min V                  -> min brightness 0..1 for pulse & pulse_smooth
  palette list | palette <name> | palette min V
  fire intensity X             -> 0.5..3.0
  wave len L                   -> wavelength in LEDs
  wave min V                   -> minimum brightness 0..1
  wave color #RRGGBB
  strobe duty D                -> 0.05..0.95
  strobe color #RRGGBB
  text "Your Text"             -> set text and switch to text mode
  text speed COLUMNS_PER_SEC   -> scrolling speed
  text color #RRGGBB           -> text foreground color
  text bg    #RRGGBB           -> text background color
  file ./path/to/file          -> stream raw RGB frames (LED_H*3 per frame)
  loopfile on|off
"""

import asyncio
import argparse
import struct
import sys
import time
import random
import math
from typing import List, Tuple, Optional

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Please install bleak: pip install bleak")
    sys.exit(1)

# ===== UUIDs =====
SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NOTIFY_UUID  = "6e400004-b5a3-f393-e0a9-e50e24dcca9e"

# ===== Command codes =====
CC_SET_BRIGHTNESS   = 2
CC_SET_SPEED_OPTION = 18
CC_START_STREAM     = 21
CC_STOP_STREAM      = 22

# ===== CRC32 =====
def crc32_ieee(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            mask = -(crc & 1)
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
    return (~crc) & 0xFFFFFFFF

# ===== Sub-frame builder =====
def build_subframe(payload: bytes, led_h: int, led_w: int, seq: int) -> bytes:
    payload_len = len(payload)
    if payload_len != led_h * 3:
        raise ValueError(f"payload {payload_len} != {led_h*3}")
    hdr_wo_crc = bytes([
        0x0D, 0x01,            # Type, Ver
        led_h & 0xFF,          # LED_H
        led_w & 0xFF,          # LED_W (reserved)
    ]) + struct.pack(">H", payload_len) + struct.pack(">H", seq)
    crc = crc32_ieee(hdr_wo_crc + payload)
    return bytes([0xD0]) + hdr_wo_crc + struct.pack(">I", crc) + payload + bytes([0xD1])

def wrap_full_packet(sub_frame: bytes) -> bytes:
    pkt = bytes([0xD0, CC_START_STREAM]) + sub_frame
    assert pkt[-1] == 0xD1
    return pkt

# ===== MTU fragmentation =====
def fragment_for_mtu(packet: bytes, mtu: int) -> List[bytes]:
    if mtu <= 0:
        mtu = 180
    mtu_payload = max(20, mtu - 3)
    return [packet[i:i+mtu_payload] for i in range(0, len(packet), mtu_payload)]

# ===== Color helpers =====
def hsv_to_rgb(h: float, s: float, v: float) -> Tuple[int, int, int]:
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = int(v * (1.0 - s) * 255.0)
    q = int(v * (1.0 - f * s) * 255.0)
    t = int(v * (1.0 - (1.0 - f) * s) * 255.0)
    V = int(v * 255.0)
    i &= 5
    return [(V, t, p), (q, V, p), (p, V, t), (p, q, V), (t, p, V), (V, p, q)][i]

def pack_rgb(order: str, r: int, g: int, b: int) -> bytes:
    if order.upper() == "GRB":
        return bytes([g, r, b])
    return bytes([r, g, b])

def apply_gamma_u8(x: int, gamma: float = 2.2) -> int:
    y = pow(max(0.0, min(1.0, x / 255.0)), gamma)
    return max(0, min(255, int(y * 255.0 + 0.5)))

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

def sin01(phase: float) -> float:
    """0..1 sinusoidal wave without discontinuity."""
    return 0.5 * (math.sin(phase) + 1.0)

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

# ===== Simple 1D value noise =====
def _hash_i(i: int) -> float:
    i = (i ^ 2747636419) * 2654435769
    i ^= (i >> 16)
    i = i * 2654435769
    i ^= (i >> 16)
    return (i & 0xFFFFFFFF) / 4294967295.0

def value_noise_1d(x: float) -> float:
    i0 = int(x)
    i1 = i0 + 1
    t = x - i0
    return lerp(_hash_i(i0), _hash_i(i1), t)

# ===== Palettes (HSV stops) =====
PALETTES = {
    "rainbow": [(0.00,1,1),(0.16,1,1),(0.33,1,1),(0.50,1,1),(0.66,1,1),(0.83,1,1),(1.00,1,1)],
    "warm":    [(0.00,1,1),(0.06,1,1),(0.10,1,1),(0.14,1,1),(0.18,1,1)],
    "cool":    [(0.55,1,1),(0.60,1,1),(0.66,1,1),(0.72,1,1),(0.80,1,1)],
    "sunset":  [(0.00,1,1),(0.04,1,1),(0.08,1,1),(0.14,1,1),(0.58,1,0.4)],
    "forest":  [(0.20,1,1),(0.25,1,1),(0.30,1,0.8),(0.33,1,0.6)],
    "ocean":   [(0.50,0.8,1),(0.56,1,1),(0.60,1,0.9),(0.66,1,0.8)],
    "neon":    [(0.83,1,1),(0.66,1,1),(0.33,1,1),(0.16,1,1),(0.00,1,1)],
    "pastel":  [(0.00,0.3,1),(0.20,0.3,1),(0.40,0.3,1),(0.60,0.3,1),(0.80,0.3,1)],
}

def palette_sample_hsv(name: str, t: float) -> Tuple[float,float,float]:
    stops = PALETTES.get(name, PALETTES["rainbow"])
    n = len(stops)
    x = t * (n - 1)
    i0 = int(x)
    i1 = min(n - 1, i0 + 1)
    tt = x - i0
    h = lerp(stops[i0][0], stops[i1][0], tt)
    s = lerp(stops[i0][1], stops[i1][1], tt)
    v = lerp(stops[i0][2], stops[i1][2], tt)
    return clamp01(h), clamp01(s), clamp01(v)

# ===== 5x7 Font (rows) and POV text rendering =====
def _char_rows_5x7():
    """Returns dict: char -> list of 7 strings of length 5 ('.' or '#')."""
    F = {}
    def put(ch, rows):
        F[ch] = rows
    BLANK = ["....."]*7

    # A-Z
    put('A', [".###.","#...#","#...#","#####","#...#","#...#","#...#"])
    put('B', ["####.","#...#","#...#","####.","#...#","#...#","####."])
    put('C', [".####","#....","#....","#....","#....","#....",".####"])
    put('D', ["####.","#...#","#...#","#...#","#...#","#...#","####."])
    put('E', ["#####","#....","#....","####.","#....","#....","#####"])
    put('F', ["#####","#....","#....","####.","#....","#....","#...."])
    put('G', [".####","#....","#....","#.###","#...#","#...#",".###."])
    put('H', ["#...#","#...#","#...#","#####","#...#","#...#","#...#"])
    put('I', ["#####","..#..","..#..","..#..","..#..","..#..","#####"])
    put('J', ["#####","...#.","...#.","...#.","...#.","#..#.",".##.."])
    put('K', ["#...#","#..#.","#.#..","##...","#.#..","#..#.","#...#"])
    put('L', ["#....","#....","#....","#....","#....","#....","#####"])
    put('M', ["#...#","##.##","#.#.#","#.#.#","#...#","#...#","#...#"])
    put('N', ["#...#","##..#","#.#.#","#..##","#...#","#...#","#...#"])
    put('O', [".###.","#...#","#...#","#...#","#...#","#...#",".###."])
    put('P', ["####.","#...#","#...#","####.","#....","#....","#...."])
    put('Q', [".###.","#...#","#...#","#...#","#.#.#","#..#.",".##.#"])
    put('R', ["####.","#...#","#...#","####.","#.#..","#..#.","#...#"])
    put('S', [".####","#....","#....",".###.","....#","....#","####."])
    put('T', ["#####","..#..","..#..","..#..","..#..","..#..","..#.."])
    put('U', ["#...#","#...#","#...#","#...#","#...#","#...#",".###."])
    put('V', ["#...#","#...#","#...#","#...#",".#.#.",".#.#.","..#.."])
    put('W', ["#...#","#...#","#...#","#.#.#","#.#.#","##.##","#...#"])
    put('X', ["#...#",".#.#.","..#..","..#..",".#.#.","#...#","#...#"])
    put('Y', ["#...#",".#.#.","..#..","..#..","..#..","..#..","..#.."])
    put('Z', ["#####","....#","...#.","..#..",".#...","#....","#####"])

    # 0-9
    put('0', [".###.","#...#","#..##","#.#.#","##..#","#...#",".###."])
    put('1', ["..#..",".##..","..#..","..#..","..#..","..#..",".###."])
    put('2', [".###.","#...#","....#","...#.","..#..",".#...","#####"])
    put('3', ["#####","...#.","..#..","...#.","....#","#...#",".###."])
    put('4', ["...#.","..##.",".#.#.","#..#.","#####","...#.","...#."])
    put('5', ["#####","#....","####.","....#","....#","#...#",".###."])
    put('6', ["..##.",".#...","#....","####.","#...#","#...#",".###."])
    put('7', ["#####","....#","...#.","..#..",".#...",".#...",".#..."])
    put('8', [".###.","#...#","#...#",".###.","#...#","#...#",".###."])
    put('9', [".###.","#...#","#...#",".####","....#","...#.",".##.."])

    # punctuation
    put(' ', BLANK)
    put('-', [".....",".....",".....","#####",".....",".....","....."])
    put('.', [".....",".....",".....",".....",".....","..#..","....."])
    put(':', [".....","..#..",".....",".....","..#..",".....","....."])
    put('!', ["..#..","..#..","..#..","..#..","..#..",".....","..#.."])
    put('?', [".###.","#...#","....#","..#..","..#..",".....","..#.."])

    return F

FONT_5x7 = _char_rows_5x7()

class TextPOV:
    """Builds POV columns for a given text using 5x7 font, scaled to led_h."""
    def __init__(self, text: str, led_h: int, spacing: int = 1, margin: int = 2):
        self.led_h = max(1, led_h)
        self.spacing = max(0, spacing)
        self.margin = max(0, margin)
        self.text = text
        self.columns_scaled: List[List[bool]] = []  # each column: list[led_h] -> True/False
        self._build()

    @staticmethod
    def _rows_to_cols(rows: List[str]) -> List[List[bool]]:
        # rows: 7 strings of length 5; return columns (width 5), each column is list of 7 booleans (top->bottom)
        cols = []
        for x in range(5):
            col = []
            for y in range(7):
                col.append(rows[y][x] == '#')
            cols.append(col)
        return cols

    def _build(self):
        # Build unscaled binary columns for the full string
        raw_cols: List[List[bool]] = []
        # left margin
        for _ in range(self.margin):
            raw_cols.append([False]*7)
        # characters
        for ch in self.text:
            key = ch.upper()
            rows = FONT_5x7.get(key, FONT_5x7[' '])  # unknown -> space
            cols = self._rows_to_cols(rows)
            raw_cols.extend(cols)
            # spacing
            for _ in range(self.spacing):
                raw_cols.append([False]*7)
        # right margin
        for _ in range(self.margin):
            raw_cols.append([False]*7)

        # Scale to led_h (nearest-neighbor)
        self.columns_scaled = []
        for c in raw_cols:
            col_scaled = []
            for y in range(self.led_h):
                fy = int(y * 7 / self.led_h)  # 0..6
                col_scaled.append(c[fy])
            self.columns_scaled.append(col_scaled)

    @property
    def width(self) -> int:
        return len(self.columns_scaled)

    def payload_for_column(self, idx: int, order: str, fg: Tuple[int,int,int], bg: Tuple[int,int,int]) -> bytes:
        """Build RGB bytes for the given column index (wraps)."""
        if self.width == 0:
            return bytes([0,0,0] * self.led_h)
        col = self.columns_scaled[idx % self.width]
        payload = bytearray(self.led_h * 3)
        fr, fg_, fb = fg
        br, bg_, bb = bg
        for j in range(self.led_h):
            if col[j]:
                payload[j*3:j*3+3] = pack_rgb(order, fr, fg_, fb)
            else:
                payload[j*3:j*3+3] = pack_rgb(order, br, bg_, bb)
        return bytes(payload)

# ===== Animations =====
def mode_walking_rainbow(led_h: int, t: float, order: str, rate_hz: float = 0.2) -> bytes:
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        h = (t * rate_hz + j / max(1, led_h)) % 1.0
        r, g, b = hsv_to_rgb(h, 1.0, 1.0)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_stripes_moving(led_h: int, t: float, order: str,
                        stripe_width: int = 4, dir_sign: int = +1, speed_leds_per_sec: float = 10.0) -> bytes:
    colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]
    band = max(1, stripe_width)
    offset = int(dir_sign * speed_leds_per_sec * t)
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        idx = j + offset
        stripe_idx = (idx // band)
        color_idx = stripe_idx % len(colors)
        if color_idx < 0: color_idx += len(colors)
        r, g, b = colors[color_idx]
        payload[j*3:j*3+3] = pack_rgb(order, apply_gamma_u8(r), apply_gamma_u8(g), apply_gamma_u8(b))
    return bytes(payload)

def mode_random(led_h: int, t: float, order: str) -> bytes:
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        r, g, b = random.randint(0,255), random.randint(0,255), random.randint(0,255)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_theater_chase(led_h: int, t: float, order: str, step_hz: float = 20.0, v_min: float = 0.2) -> bytes:
    phase = int(t * step_hz) % 3
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        on = ((j + phase) % 3) == 0
        val = int(lerp(255*v_min, 255, 1.0 if on else 0.0))
        payload[j*3:j*3+3] = pack_rgb(order, val, val, val)
    return bytes(payload)

def mode_comet(led_h: int, t: float, order: str, speed_leds_per_sec: float = 30.0, decay: float = 0.5, v_min: float = 0.15) -> bytes:
    head_pos = (t * speed_leds_per_sec) % max(1, led_h)
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        d = abs(j - head_pos)
        v = max(v_min, min(1.0, math.exp(-d * decay)))
        r, g, b = hsv_to_rgb(j / max(1, led_h), 1.0, v)
        r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_twinkle(led_h: int, t: float, order: str, v_min: float = 0.12) -> bytes:
    payload = bytearray(led_h * 3)
    base = int(255 * v_min)
    for j in range(led_h):
        val = base
        if random.randint(0, 160) == 0:
            val = 255
        elif ((int(t*60)+j) % 37) == 0:
            val = int(lerp(base, 255, 0.5))
        payload[j*3:j*3+3] = pack_rgb(order, val, val, val)
    return bytes(payload)

def mode_color_wipe(led_h: int, t: float, order: str, speed_leds_per_sec: float = 60.0,
                    color: Tuple[int,int,int] = (255, 0, 128), v_min: float = 0.12) -> bytes:
    pos = int((t * speed_leds_per_sec)) % max(1, led_h)
    payload = bytearray(led_h * 3)
    dimc = (int(color[0]*v_min), int(color[1]*v_min), int(color[2]*v_min))
    for j in range(led_h):
        r,g,b = (color if j <= pos else dimc)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_gradient_breathe(led_h: int, t: float, order: str, rate_hz: float = 1.0, v_min: float = 0.18) -> bytes:
    phase = (t * rate_hz) * 2.0 * math.pi
    v = lerp(v_min, 1.0, sin01(phase))
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        u = j / max(1, led_h)
        r = apply_gamma_u8(int((1.0 - u) * 255.0 * v))
        g = apply_gamma_u8(0)
        b = apply_gamma_u8(int(u * 255.0 * v))
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_noise_rainbow(led_h: int, t: float, order: str, flow_hz: float = 0.25, v_min: float = 0.22) -> bytes:
    payload = bytearray(led_h * 3)
    x0 = t * flow_hz * 10.0
    for j in range(led_h):
        x = x0 + j * 0.35
        h = value_noise_1d(x)
        r, g, b = hsv_to_rgb(h, 0.95, lerp(v_min, 1.0, 0.9))
        r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_sparkle_trails(led_h: int, t: float, order: str, drift_hz: float = 0.2, v_min: float = 0.15) -> bytes:
    payload = bytearray(led_h * 3)
    centers = []
    for k in range(3):
        c = (0.5 + 0.5 * math.sin(t * (drift_hz + 0.07*k) * 2.0 * math.pi + k)) * (led_h-1)
        centers.append(c)
    for j in range(led_h):
        v = v_min
        for idx, c in enumerate(centers):
            d = abs(j - c)
            v = max(v, math.exp(-0.5 * (d/2.0)**2) * (0.6 if idx == 0 else 0.4))
        h = (0.6 + 0.2 * math.sin(0.15 * t + j * 0.03))
        r, g, b = hsv_to_rgb(h, 0.6, v)
        r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

# ----- Fire (intensity-adjustable, smoother, non-zero min) -----
FIRE_STATE = {}
def _fire_state(led_h: int):
    st = FIRE_STATE.get(led_h)
    if st is None:
        st = {"heat": [0.0]*led_h}
        FIRE_STATE[led_h] = st
    return st

def heat_to_rgb(order: str, heat: float, v_floor: float = 0.15) -> bytes:
    v = max(v_floor, min(1.0, heat))
    if v < 0.33:
        r = int(255 * v); g = int(64 * v); b = 0
    elif v < 0.66:
        r = 255; g = int(180 * (v - 0.33) / 0.33); b = 0
    else:
        r = 255; g = int(220 * (v - 0.66) / 0.34); b = int(80 * (v - 0.66) / 0.34)
    r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
    return pack_rgb(order, r, g, b)

def mode_fire(led_h: int, t: float, order: str,
              intensity: float = 1.5, v_floor: float = 0.15) -> bytes:
    base_cooling   = 0.015
    base_sparking  = 0.12
    base_rise      = 0.92
    cooling  = base_cooling / max(0.5, intensity)
    sparking = min(0.95, base_sparking * intensity)
    rise     = min(0.985, base_rise + 0.02 * (intensity - 1.0))
    st = _fire_state(led_h)
    heat = st["heat"]
    for i in range(led_h):
        heat[i] = max(0.0, heat[i] - random.random() * cooling)
    for i in range(led_h - 1, 1, -1):
        heat[i] = (heat[i] * (1.0 - rise) + (heat[i-1] + heat[i-2]) * 0.5 * rise)
    sparks = max(1, int(2 * intensity))
    for _ in range(sparks):
        if random.random() < sparking:
            idx = random.randint(0, min(4, led_h-1))
            heat[idx] = min(1.0, heat[idx] + random.uniform(0.5, 0.95))
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        payload[j*3:j*3+3] = heat_to_rgb(order, heat[j], v_floor=v_floor)
    return bytes(payload)

# ----- Rainbow wave -----
def mode_rainbow_wave(led_h: int, t: float, order: str,
                      wave_hz: float = 0.35, wavelength_leds: float = 12.0, v_min: float = 0.2) -> bytes:
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        phase = (t * wave_hz) * 2.0 * math.pi + (j / max(1.0, wavelength_leds)) * 2.0 * math.pi
        v = lerp(v_min, 1.0, sin01(phase))
        h = (j / max(1, led_h) + 0.05 * t)
        r, g, b = hsv_to_rgb(h, 0.9, v)
        r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

# ----- Wave mode: sine brightness over a base color -----
def mode_wave(led_h: int, t: float, order: str,
              wave_hz: float = 0.5, wavelength_leds: float = 10.0,
              base_color: Tuple[int,int,int] = (255, 160, 40),
              v_min: float = 0.22) -> bytes:
    payload = bytearray(led_h * 3)
    r0, g0, b0 = base_color
    for j in range(led_h):
        phase = (t * wave_hz) * 2.0 * math.pi + (j / max(1.0, wavelength_leds)) * 2.0 * math.pi
        v = lerp(v_min, 1.0, sin01(phase))
        r = apply_gamma_u8(int(r0 * v)); g = apply_gamma_u8(int(g0 * v)); b = apply_gamma_u8(int(b0 * v))
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

# ----- Strobe (smoothed, never off) -----
def mode_strobe(led_h: int, t: float, order: str,
                rate_hz: float = 8.0, duty: float = 0.25,
                color: Tuple[int,int,int] = (255,255,255), v_min: float = 0.18) -> bytes:
    duty = max(0.05, min(0.95, duty))
    phase = (t * rate_hz) % 1.0
    hi = 1.0 if phase < duty else 0.0
    # smooth on/off edges
    if hi == 1.0:
        env = sin01((phase / duty) * math.pi)
    else:
        env = sin01(((phase - duty) / (1.0 - duty)) * math.pi)
    v = lerp(v_min, 1.0, env)
    r0, g0, b0 = color
    r = apply_gamma_u8(int(r0 * v)); g = apply_gamma_u8(int(g0 * v)); b = apply_gamma_u8(int(b0 * v))
    col = pack_rgb(order, r, g, b)
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        payload[j*3:j*3+3] = col
    return bytes(payload)

# ===== Pulse modes (fully smooth: brightness & hue/palette) =====
def mode_pulse(led_h: int, t: float, order: str,
               rate_hz: float = 0.25, hue_span: float = 0.25,
               hue_drift_hz: float = 0.03, v_min: float = 0.25) -> bytes:
    phase_b = t * rate_hz * 2.0 * math.pi
    v = lerp(v_min, 1.0, sin01(phase_b))  # brightness smooth sine
    phase_h = t * hue_drift_hz * 2.0 * math.pi
    h_center = sin01(phase_h)             # hue center smooth sine 0..1
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        u = (j / max(1, led_h) - 0.5)     # spatial offset
        h = clamp01(h_center + hue_span * u)
        r, g, b = hsv_to_rgb(h, 0.9, v)
        r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_pulse_smooth(led_h: int, t: float, order: str,
                      rate_hz: float = 0.20, hue_span: float = 0.18,
                      hue_drift_hz: float = 0.02, v_min: float = 0.25) -> bytes:
    phase_b = t * rate_hz * 2.0 * math.pi
    v = lerp(v_min, 1.0, sin01(phase_b))
    phase_h = t * hue_drift_hz * 2.0 * math.pi
    h_center = sin01(phase_h)
    payload = bytearray(led_h * 3)
    for j in range(led_h):
        u = (j / max(1, led_h) - 0.5)
        h = clamp01(h_center + hue_span * u)
        r, g, b = hsv_to_rgb(h, 0.85, v)
        r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

def mode_pulse_palette(led_h: int, t: float, order: str,
                       palette_name: str = "sunset", rate_hz: float = 0.25,
                       drift_hz: float = 0.02, v_min: float = 0.25) -> bytes:
    phase_b = t * rate_hz * 2.0 * math.pi
    v_scale = lerp(v_min, 1.0, sin01(phase_b))
    ppos = sin01(t * drift_hz * 2.0 * math.pi)       # smooth 0..1 traversal
    h, s, v0 = palette_sample_hsv(palette_name, ppos)

    payload = bytearray(led_h * 3)
    for j in range(led_h):
        hh = clamp01(h + 0.06 * (j / max(1, led_h) - 0.5))    # subtle spatial tint
        r, g, b = hsv_to_rgb(hh, s, v0 * v_scale)
        r = apply_gamma_u8(r); g = apply_gamma_u8(g); b = apply_gamma_u8(b)
        payload[j*3:j*3+3] = pack_rgb(order, r, g, b)
    return bytes(payload)

# ===== BLE multi-device controller =====
class MultiPixelPoiStreamer:
    def __init__(self, led_h: int, order: str,
                 fps: int, mtu_override: int, chunk_delay_ms: int,
                 write_response: bool, debug: bool,
                 rainbow_rate_hz: float, pulse_rate_hz: float,
                 stripes_width: int, stripes_dir: int, stripes_speed_leds: float,
                 noise_rate_hz: float, sparkle_rate_hz: float,
                 wave_rate_hz: float, wave_wavelength: float, wave_min_v: float, wave_color: Tuple[int,int,int],
                 strobe_rate_hz: float, strobe_duty: float,
                 palette_name: str, pulse_min_v: float, palette_min_v: float,
                 fire_intensity: float, fire_min_v: float):
        self.led_h = led_h
        self.order = order
        self.fps = max(0, fps)
        self.mtu_override = mtu_override
        self.chunk_delay = max(0, chunk_delay_ms) / 1000.0
        self.write_response = write_response
        self.debug = debug

        self.clients: List[BleakClient] = []
        self.client_mtu: List[int] = []
        self.seq = 0
        self.streaming = False

        # Animation params
        self.mode_name = "pulse"  # default to smooth pulse
        self.rainbow_rate_hz = rainbow_rate_hz
        self.pulse_rate_hz = pulse_rate_hz
        self.pulse_min_v = clamp01(pulse_min_v)

        self.stripes_width = stripes_width
        self.stripes_dir = stripes_dir
        self.stripes_speed_leds = stripes_speed_leds
        self.noise_rate_hz = noise_rate_hz
        self.sparkle_rate_hz = sparkle_rate_hz

        self.wave_rate_hz = wave_rate_hz
        self.wave_wavelength = max(1.0, wave_wavelength)
        self.wave_min_v = clamp01(wave_min_v)
        self.wave_color = wave_color

        self.strobe_rate_hz = strobe_rate_hz
        self.strobe_duty = strobe_duty
        self.strobe_color = (255,255,255)

        self.palette_name = palette_name
        self.palette_min_v = clamp01(palette_min_v)

        self.fire_intensity = max(0.5, min(3.0, fire_intensity))
        self.fire_min_v = clamp01(fire_min_v)

        # File streaming
        self.file_path: Optional[str] = None
        self.file_fp: Optional[object] = None
        self.loop_file: bool = True

        # Text POV
        self.text_pov: Optional[TextPOV] = None
        self.text_string: str = ""
        self.text_speed_cols: float = 120.0  # columns per second
        self.text_color: Tuple[int,int,int] = (255, 255, 255)
        self.text_bg:    Tuple[int,int,int] = (0, 0, 0)

        self._stop_task = asyncio.Event()

    async def scan(self, name_hint: str = "Pixel Poi", timeout: float = 6.0):
        devs = await BleakScanner.discover(timeout=timeout)
        found = []
        for d in devs:
            if name_hint.lower() in (d.name or "").lower():
                found.append(d)
        return found

    async def connect_selected(self, devices: List, indices: List[int]):
        self.clients = []
        self.client_mtu = []
        for idx in indices:
            if idx < 0 or idx >= len(devices): 
                continue
            addr = devices[idx].address
            client = BleakClient(addr)
            print(f"Connecting to {devices[idx].name} [{addr}] ...")
            try:
                await client.connect()
                try:
                    await client.start_notify(NOTIFY_UUID, lambda h, d: None)
                except Exception:
                    pass
                mtu = self.mtu_override or getattr(client, "mtu_size", None) or 180
                self.clients.append(client)
                self.client_mtu.append(mtu)
                print(f"Connected: MTU={mtu}")
            except Exception as e:
                print(f"Connect failed: {e}")

        if not self.clients:
            print("No devices connected.")
            return False
        return True

    async def disconnect_all(self):
        for c in self.clients:
            try:
                await c.disconnect()
            except Exception:
                pass
        self.clients = []
        self.client_mtu = []

    async def send_cmd_all(self, code: int, payload: bytes=b""):
        pkt = bytes([0xD0, code]) + payload
        for c in self.clients:
            try:
                await c.write_gatt_char(RX_UUID, pkt, response=self.write_response)
            except Exception as e:
                print(f"cmd write error: {e}")

    async def start_stream(self):
        if not self.clients:
            print("No connected devices. Use 'scan' and 'connect ...' first.")
            return
        await self.send_cmd_all(CC_START_STREAM)
        self.streaming = True
        self._stop_task.clear()
        asyncio.create_task(self._run_stream_loop())

    async def stop_stream(self):
        if not self.clients:
            return
        self.streaming = False
        self._stop_task.set()
        await self.send_cmd_all(CC_STOP_STREAM)

    async def set_brightness(self, value: int):
        v = max(0, min(255, value))
        await self.send_cmd_all(CC_SET_BRIGHTNESS, bytes([v]))
        print(f"Brightness -> {v}")

    async def set_speed_gear(self, index: int):
        idx = max(0, min(5, index))
        await self.send_cmd_all(CC_SET_SPEED_OPTION, bytes([idx]))
        print(f"Speed gear -> {idx}")

    def set_mode(self, name: str):
        valid = {"walking","pulse","pulse_smooth","pulse_palette","stripes","random","theater","comet","twinkle","wipe","breathe","noise","sparkle","fire","rainbow_wave","wave","strobe","text","file"}
        if name not in valid:
            print(f"Unknown mode '{name}'. Valid: {', '.join(sorted(valid))}")
            return
        self.mode_name = name
        print(f"Mode -> {self.mode_name}")

    def next_mode(self):
        order = ["pulse","pulse_smooth","pulse_palette","walking","stripes","random","theater","comet","twinkle","wipe","breathe","noise","sparkle","fire","rainbow_wave","wave","strobe","text","file"]
        i = (order.index(self.mode_name) + 1) % len(order)
        self.mode_name = order[i]
        print(f"Mode -> {self.mode_name}")

    def set_palette(self, name: str):
        if name not in PALETTES:
            print(f"Unknown palette '{name}'. Options: {', '.join(PALETTES.keys())}")
            return
        self.palette_name = name
        print(f"Palette -> {self.palette_name}")

    def set_file(self, path: str):
        npath = path.replace("\\", "/")
        try:
            if self.file_fp:
                try: self.file_fp.close()
                except Exception: pass
            self.file_fp = open(npath, "rb")
            self.file_path = npath
            print(f"File -> {npath}")
        except Exception as e:
            self.file_fp = None
            self.file_path = None
            print(f"File open failed: {e}")

    def _read_file_frame(self) -> Optional[bytes]:
        if not self.file_fp:
            return None
        L = self.led_h * 3
        buf = self.file_fp.read(L)
        if buf is None or len(buf) < L:
            if self.loop_file:
                try:
                    self.file_fp.seek(0)
                    buf = self.file_fp.read(L)
                    if buf is None or len(buf) < L:
                        return None
                except Exception:
                    return None
            else:
                return None
        if self.order.upper() == "GRB":
            b = bytearray(L)
            for j in range(self.led_h):
                r = buf[j*3+0]; g = buf[j*3+1]; bb = buf[j*3+2]
                b[j*3+0] = g; b[j*3+1] = r; b[j*3+2] = bb
            return bytes(b)
        return buf

    def set_text(self, s: str):
        self.text_string = s
        self.text_pov = TextPOV(s, self.led_h, spacing=1, margin=2)
        print(f'Text -> "{s}" (columns={self.text_pov.width}, speed={self.text_speed_cols} col/s)')

    def set_text_speed(self, cols_per_sec: float):
        self.text_speed_cols = max(1.0, float(cols_per_sec))
        print(f"text_speed_cols -> {self.text_speed_cols}")

    def set_text_color(self, rgb: Tuple[int,int,int]):
        self.text_color = tuple(max(0, min(255, v)) for v in rgb)
        print(f"text_color -> #{self.text_color[0]:02X}{self.text_color[1]:02X}{self.text_color[2]:02X}")

    def set_text_bg(self, rgb: Tuple[int,int,int]):
        self.text_bg = tuple(max(0, min(255, v)) for v in rgb)
        print(f"text_bg -> #{self.text_bg[0]:02X}{self.text_bg[1]:02X}{self.text_bg[2]:02X}")

    async def _send_full_packet_all(self, payload: bytes):
        sub_frame = build_subframe(payload, led_h=self.led_h, led_w=0, seq=self.seq)
        full_packet = wrap_full_packet(sub_frame)
        if self.debug:
            tail = full_packet[-16:]
            print(f"TX end: {' '.join(f'{b:02X}' for b in tail)}")
        # Write SEQUENTIALLY to each device to keep ordering aligned
        for c, mtu in zip(self.clients, self.client_mtu):
            try:
                for chunk in fragment_for_mtu(full_packet, mtu):
                    await c.write_gatt_char(RX_UUID, chunk, response=self.write_response)
                    if self.chunk_delay:
                        await asyncio.sleep(self.chunk_delay)
            except Exception as e:
                print(f"stream write error: {e}")
        self.seq = (self.seq + 1) & 0xFFFF

    async def _run_stream_loop(self):
        period = (1.0 / self.fps) if self.fps > 0 else 0.0
        t0 = time.time()
        frames = 0
        while self.streaming and not self._stop_task.is_set():
            t = (time.time() - t0)
            if self.mode_name == "file":
                payload = self._read_file_frame()
                if payload is None:
                    await asyncio.sleep(0.01)
                    continue
            elif self.mode_name == "text":
                if not self.text_pov:
                    # Nothing to display
                    await asyncio.sleep(0.01)
                    continue
                # Determine current column by time (columns per second), wrap across width
                col_idx = int(t * self.text_speed_cols) % max(1, self.text_pov.width)
                payload = self.text_pov.payload_for_column(col_idx, self.order, self.text_color, self.text_bg)
            elif self.mode_name == "pulse":
                payload = mode_pulse(self.led_h, t, self.order, rate_hz=self.pulse_rate_hz, v_min=self.pulse_min_v)
            elif self.mode_name == "pulse_smooth":
                payload = mode_pulse_smooth(self.led_h, t, self.order, rate_hz=self.pulse_rate_hz, v_min=self.pulse_min_v)
            elif self.mode_name == "pulse_palette":
                payload = mode_pulse_palette(self.led_h, t, self.order, palette_name=self.palette_name, rate_hz=self.pulse_rate_hz, v_min=self.palette_min_v)
            elif self.mode_name == "walking":
                payload = mode_walking_rainbow(self.led_h, t, self.order, rate_hz=self.rainbow_rate_hz)
            elif self.mode_name == "stripes":
                payload = mode_stripes_moving(self.led_h, t, self.order, stripe_width=self.stripes_width, dir_sign=self.stripes_dir, speed_leds_per_sec=self.stripes_speed_leds)
            elif self.mode_name == "random":
                payload = mode_random(self.led_h, t, self.order)
            elif self.mode_name == "theater":
                payload = mode_theater_chase(self.led_h, t, self.order)
            elif self.mode_name == "comet":
                payload = mode_comet(self.led_h, t, self.order)
            elif self.mode_name == "twinkle":
                payload = mode_twinkle(self.led_h, t, self.order)
            elif self.mode_name == "wipe":
                payload = mode_color_wipe(self.led_h, t, self.order)
            elif self.mode_name == "breathe":
                payload = mode_gradient_breathe(self.led_h, t, self.order)
            elif self.mode_name == "noise":
                payload = mode_noise_rainbow(self.led_h, t, self.order, flow_hz=self.noise_rate_hz)
            elif self.mode_name == "sparkle":
                payload = mode_sparkle_trails(self.led_h, t, self.order, drift_hz=self.sparkle_rate_hz)
            elif self.mode_name == "fire":
                payload = mode_fire(self.led_h, t, self.order, intensity=self.fire_intensity, v_floor=self.fire_min_v)
            elif self.mode_name == "rainbow_wave":
                payload = mode_rainbow_wave(self.led_h, t, self.order, wave_hz=self.wave_rate_hz, wavelength_leds=self.wave_wavelength, v_min=max(self.wave_min_v, 0.15))
            elif self.mode_name == "wave":
                payload = mode_wave(self.led_h, t, self.order, wave_hz=self.wave_rate_hz, wavelength_leds=self.wave_wavelength, base_color=self.wave_color, v_min=self.wave_min_v)
            elif self.mode_name == "strobe":
                payload = mode_strobe(self.led_h, t, self.order, rate_hz=self.strobe_rate_hz, duty=self.strobe_duty, color=self.strobe_color, v_min=0.18)
            else:
                payload = mode_pulse(self.led_h, t, self.order, rate_hz=self.pulse_rate_hz, v_min=self.pulse_min_v)

            await self._send_full_packet_all(payload)
            frames += 1
            if period > 0.0:
                await asyncio.sleep(period)
            else:
                await asyncio.sleep(0.001)
        print(f"[stream] sent {frames} frames")

# ===== Interactive console =====
async def interactive_loop(streamer: MultiPixelPoiStreamer):
    print("\nCommands:")
    print(" scan                         -> discover Pixel Poi devices")
    print(" devices                      -> list discovered devices")
    print(" connect all                  -> connect to all discovered devices")
    print(" connect <idx> [idx ...]      -> connect selected indices")
    print(" start / stop / quit")
    print(" mode [walking|pulse|pulse_smooth|pulse_palette|stripes|random|theater|comet|twinkle|wipe|breathe|noise|sparkle|fire|rainbow_wave|wave|strobe|text|file]")
    print(" brightness N                 -> 0..255")
    print(" gear I                       -> 0..5")
    print(" speed rainbow Hz             -> walking rainbow hue speed")
    print(" speed pulse Hz               -> pulse speed (lower = slower, smoother)")
    print(" speed stripes LEDS           -> stripes speed (LEDs/sec)")
    print(" speed noise Hz               -> noise rainbow flow")
    print(" speed sparkle Hz             -> sparkle drift")
    print(" speed wave Hz                -> wave speed")
    print(" speed strobe Hz              -> strobe rate")
    print(" stripes width N              -> stripe width")
    print(" stripes dir +1|-1            -> stripe direction")
    print(" pulse min V                  -> min brightness 0..1 for pulse & pulse_smooth")
    print(" palette list | palette <name> | palette min V")
    print(" fire intensity X             -> 0.5..3.0")
    print(" wave len L                   -> wavelength in LEDs")
    print(" wave min V                   -> minimum brightness 0..1")
    print(" wave color #RRGGBB")
    print(" strobe duty D                -> 0.05..0.95")
    print(" strobe color #RRGGBB")
    print(" text \"Your Text\"            -> set text and switch to text mode")
    print(" text speed COLUMNS_PER_SEC   -> scrolling speed")
    print(" text color #RRGGBB           -> text color")
    print(" text bg    #RRGGBB           -> background color")
    print(" file ./path/to/file          -> stream raw RGB frames (LED_H*3 per frame)")
    print(" loopfile on|off\n")

    discovered = []

    loop = asyncio.get_running_loop()
    async def ainput(prompt: str="> "):
        return await loop.run_in_executor(None, lambda: input(prompt))

    def parse_hex_color(s: str) -> Optional[Tuple[int,int,int]]:
        s = s.strip()
        if s.startswith("#"): s = s[1:]
        if len(s) != 6: return None
        try:
            r = int(s[0:2], 16); g = int(s[2:4], 16); b = int(s[4:6], 16)
            return (r,g,b)
        except Exception:
            return None

    def parse_text_cmd_arg(arg: str) -> str:
        arg = arg.strip()
        if arg.startswith('"') and arg.endswith('"') and len(arg) >= 2:
            return arg[1:-1]
        return arg

    while True:
        cmd = (await ainput()).strip()
        if not cmd:
            continue

        if cmd == "scan":
            print("Scanning for Pixel Poi devices...")
            discovered = await streamer.scan("Pixel Poi", timeout=6.0)
            if not discovered:
                print("No Pixel Poi devices found.")
            else:
                print("Discovered devices:")
                for i, d in enumerate(discovered):
                    print(f"  {i}: {d.name} [{d.address}]")
        elif cmd == "devices":
            if not discovered:
                print("No devices discovered. Use 'scan' first.")
            else:
                for i, d in enumerate(discovered):
                    print(f"  {i}: {d.name} [{d.address}]")
        elif cmd.startswith("connect"):
            if not discovered:
                print("No devices discovered. Use 'scan' first.")
                continue
            parts = cmd.split()
            if len(parts) == 2 and parts[1].lower() == "all":
                indices = list(range(len(discovered)))
            else:
                try:
                    indices = [int(x) for x in parts[1:]]
                except Exception:
                    print("Usage: connect all | connect <idx> [idx ...]")
                    continue
            ok = await streamer.connect_selected(discovered, indices)
            if ok:
                print(f"Connected {len(streamer.clients)} device(s).")
        elif cmd == "start":
            await streamer.start_stream()
        elif cmd == "stop":
            await streamer.stop_stream()
        elif cmd == "quit":
            await streamer.stop_stream()
            await streamer.disconnect_all()
            break
        elif cmd == "mode":
            streamer.next_mode()
        elif cmd.startswith("mode "):
            _, name = cmd.split(" ", 1)
            streamer.set_mode(name.strip())
        elif cmd.startswith("brightness"):
            try:
                _, s = cmd.split()
                await streamer.set_brightness(int(s))
            except Exception:
                print("Usage: brightness <0..255>")
        elif cmd.startswith("gear"):
            try:
                _, s = cmd.split()
                await streamer.set_speed_gear(int(s))
            except Exception:
                print("Usage: gear <0..5>")
        elif cmd.startswith("speed "):
            parts = cmd.split()
            if len(parts) >= 3:
                target = parts[1].lower()
                val = parts[2]
                try:
                    if target == "rainbow":
                        streamer.rainbow_rate_hz = float(val); print(f"rainbow_rate_hz -> {streamer.rainbow_rate_hz}")
                    elif target == "pulse":
                        streamer.pulse_rate_hz = float(val); print(f"pulse_rate_hz -> {streamer.pulse_rate_hz}")
                    elif target == "stripes":
                        streamer.stripes_speed_leds = float(val); print(f"stripes_speed_leds -> {streamer.stripes_speed_leds}")
                    elif target == "noise":
                        streamer.noise_rate_hz = float(val); print(f"noise_rate_hz -> {streamer.noise_rate_hz}")
                    elif target == "sparkle":
                        streamer.sparkle_rate_hz = float(val); print(f"sparkle_rate_hz -> {streamer.sparkle_rate_hz}")
                    elif target == "wave":
                        streamer.wave_rate_hz = float(val); print(f"wave_rate_hz -> {streamer.wave_rate_hz}")
                    elif target == "strobe":
                        streamer.strobe_rate_hz = float(val); print(f"strobe_rate_hz -> {streamer.strobe_rate_hz}")
                    else:
                        print("Unknown speed target.")
                except Exception:
                    print("Invalid value.")
            else:
                print("Usage: speed <rainbow|pulse|stripes|noise|sparkle|wave|strobe> <value>")
        elif cmd.startswith("stripes "):
            parts = cmd.split()
            if len(parts) >= 3:
                sub = parts[1].lower()
                val = parts[2]
                if sub == "width":
                    try:
                        streamer.stripes_width = max(1, int(val)); print(f"stripes_width -> {streamer.stripes_width}")
                    except Exception:
                        print("Usage: stripes width <N>")
                elif sub == "dir":
                    try:
                        d = int(val); streamer.stripes_dir = +1 if d >= 0 else -1; print(f"stripes_dir -> {streamer.stripes_dir}")
                    except Exception:
                        print("Usage: stripes dir <+1|-1>")
                else:
                    print("Unknown stripes subcommand.")
            else:
                print("Usage: stripes width <N>  |  stripes dir <+1|-1>")
        elif cmd.startswith("pulse "):
            parts = cmd.split()
            if len(parts) == 3 and parts[1].lower() == "min":
                try:
                    v = float(parts[2]); v = clamp01(v)
                    streamer.pulse_min_v = v
                    print(f"pulse_min_v -> {streamer.pulse_min_v}")
                except Exception:
                    print("Usage: pulse min <0..1>")
            else:
                print("Usage: pulse min <0..1>")
        elif cmd.startswith("palette"):
            parts = cmd.split()
            if len(parts) == 2 and parts[1].lower() == "list":
                print(f"Palettes: {', '.join(PALETTES.keys())}")
            elif len(parts) == 2:
                streamer.set_palette(parts[1].strip().lower())
            elif len(parts) == 3 and parts[1].lower() == "min":
                try:
                    v = float(parts[2]); v = clamp01(v)
                    streamer.palette_min_v = v
                    print(f"palette_min_v -> {streamer.palette_min_v}")
                except Exception:
                    print("Usage: palette min <0..1>")
            else:
                print("Usage: palette list | palette <name> | palette min <0..1>")
        elif cmd.startswith("fire "):
            parts = cmd.split()
            if len(parts) == 3 and parts[1].lower() == "intensity":
                try:
                    x = float(parts[2]); streamer.fire_intensity = max(0.5, min(3.0, x))
                    print(f"fire_intensity -> {streamer.fire_intensity}")
                except Exception:
                    print("Usage: fire intensity <0.5..3.0>")
            else:
                print("Usage: fire intensity <0.5..3.0>")
        elif cmd.startswith("wave "):
            parts = cmd.split()
            if len(parts) == 3 and parts[1].lower() == "len":
                try:
                    L = float(parts[2]); streamer.wave_wavelength = max(1.0, L); print(f"wave_wavelength -> {streamer.wave_wavelength}")
                except Exception:
                    print("Usage: wave len <LEDs>")
            elif len(parts) == 3 and parts[1].lower() == "min":
                try:
                    v = float(parts[2]); streamer.wave_min_v = clamp01(v); print(f"wave_min_v -> {streamer.wave_min_v}")
                except Exception:
                    print("Usage: wave min <0..1>")
            elif len(parts) == 3 and parts[1].lower() == "color":
                color = parse_hex_color(parts[2])
                if color:
                    streamer.wave_color = color; print(f"wave_color -> #{color[0]:02X}{color[1]:02X}{color[2]:02X}")
                else:
                    print("Usage: wave color #RRGGBB")
            else:
                print("Usage: wave len <LEDs> | wave min <0..1> | wave color #RRGGBB")
        elif cmd.startswith("strobe "):
            parts = cmd.split()
            if len(parts) >= 3 and parts[1].lower() == "duty":
                try:
                    streamer.strobe_duty = max(0.05, min(0.95, float(parts[2])))
                    print(f"strobe_duty -> {streamer.strobe_duty}")
                except Exception:
                    print("Usage: strobe duty <0.05..0.95>")
            elif len(parts) >= 3 and parts[1].lower() == "color":
                color = parse_hex_color(parts[2])
                if color:
                    streamer.strobe_color = color
                    print(f"strobe_color -> #{color[0]:02X}{color[1]:02X}{color[2]:02X}")
                else:
                    print("Usage: strobe color #RRGGBB")
            else:
                print("Usage: strobe duty <frac> | strobe color #RRGGBB")
        elif cmd.startswith("text "):
            # Subcommands: text "Your Text" | text speed N | text color #RRGGBB | text bg #RRGGBB
            parts = cmd.split(" ", 2)
            if len(parts) == 2:
                s = parse_text_cmd_arg(parts[1])
                streamer.set_text(s)
                streamer.set_mode("text")
            elif len(parts) == 3:
                sub = parts[1].lower()
                val = parts[2].strip()
                if sub == "speed":
                    try:
                        streamer.set_text_speed(float(val))
                    except Exception:
                        print("Usage: text speed <columns_per_sec>")
                elif sub == "color":
                    color = parse_hex_color(val)
                    if color: streamer.set_text_color(color)
                    else: print("Usage: text color #RRGGBB")
                elif sub == "bg":
                    color = parse_hex_color(val)
                    if color: streamer.set_text_bg(color)
                    else: print("Usage: text bg #RRGGBB")
                else:
                    s = parse_text_cmd_arg(parts[1] + " " + parts[2])
                    streamer.set_text(s)
                    streamer.set_mode("text")
            else:
                print('Usage: text "Your Text" | text speed <columns_per_sec> | text color #RRGGBB | text bg #RRGGBB')
        elif cmd.startswith("file "):
            _, path = cmd.split(" ", 1)
            streamer.set_file(path.strip())
            streamer.set_mode("file")
        elif cmd.startswith("loopfile "):
            _, v = cmd.split(" ", 1)
            v = v.strip().lower()
            streamer.loop_file = (v == "on")
            print(f"loop_file -> {streamer.loop_file}")
        else:
            print("Unknown command.")

# ===== CLI =====
def parse_args():
    ap = argparse.ArgumentParser(description="Pixel Poi multi-device streamer (CC_START_STREAM + sub-frame + CRC32)")
    ap.add_argument("--led_h", type=int, default=20, help="LEDs per column")
    ap.add_argument("--order", type=str, default="RGB", choices=["RGB","GRB"], help="Color order")
    ap.add_argument("--fps", type=int, default=0, help="Frames per second (0 = as fast as possible)")
    ap.add_argument("--mtu", type=int, default=0, help="Override MTU (0 = auto)")
    ap.add_argument("--chunk-delay-ms", type=int, default=1, help="Delay between MTU chunks (ms)")
    ap.add_argument("--write-response", action="store_true", help="Use write-with-response")
    ap.add_argument("--debug", action="store_true", help="Print last 16 bytes of each full packet")

    # Animation defaults
    ap.add_argument("--rainbow-rate-hz", type=float, default=0.2, help="Walking rainbow hue speed")
    ap.add_argument("--pulse-rate-hz", type=float, default=0.25, help="Pulse speed for pulse modes (slower = smoother)")
    ap.add_argument("--stripes-width", type=int, default=4, help="Stripes width (LEDs)")
    ap.add_argument("--stripes-dir", type=int, default=+1, help="Stripes direction (+1/-1)")
    ap.add_argument("--stripes-speed-leds", type=float, default=10.0, help="Stripes speed (LEDs/sec)")
    ap.add_argument("--noise-rate-hz", type=float, default=0.25, help="Noise rainbow flow speed")
    ap.add_argument("--sparkle-rate-hz", type=float, default=0.2, help="Sparkle trails drift speed")

    # Wave defaults
    ap.add_argument("--wave-rate-hz", type=float, default=0.5, help="Wave speed (Hz)")
    ap.add_argument("--wave-len", type=float, default=10.0, help="Wave length in LEDs")
    ap.add_argument("--wave-min", type=float, default=0.22, help="Wave minimum brightness (0..1)")
    ap.add_argument("--wave-color", type=str, default="#FFA028", help="Wave base color (#RRGGBB)")

    # Strobe defaults
    ap.add_argument("--strobe-rate-hz", type=float, default=8.0, help="Strobe rate (Hz)")
    ap.add_argument("--strobe-duty", type=float, default=0.25, help="Strobe duty 0.05..0.95")

    # Palette & pulse mins
    ap.add_argument("--palette", type=str, default="sunset", choices=list(PALETTES.keys()), help="Palette for pulse_palette")
    ap.add_argument("--pulse-min", type=float, default=0.25, help="Minimum brightness for pulse & pulse_smooth (0..1)")
    ap.add_argument("--palette-min", type=float, default=0.25, help="Minimum brightness for pulse_palette (0..1)")

    # Fire defaults
    ap.add_argument("--fire-intensity", type=float, default=1.5, help="Fire intensity (0.5..3.0)")
    ap.add_argument("--fire-min", type=float, default=0.15, help="Fire minimum brightness (0..1)")

    return ap.parse_args()

def _parse_wave_color(s: str) -> Tuple[int,int,int]:
    s = s.strip()
    if s.startswith("#"): s = s[1:]
    try:
        if len(s) == 6:
            return (int(s[0:2],16), int(s[2:4],16), int(s[4:6],16))
    except Exception:
        pass
    return (255,160,40)

async def main():
    args = parse_args()
    streamer = MultiPixelPoiStreamer(
        led_h=args.led_h, order=args.order,
        fps=args.fps, mtu_override=args.mtu,
        chunk_delay_ms=args.chunk_delay_ms,
        write_response=args.write_response,
        debug=args.debug,
        rainbow_rate_hz=args.rainbow_rate_hz,
        pulse_rate_hz=args.pulse_rate_hz,
        stripes_width=args.stripes_width,
        stripes_dir=+1 if args.stripes_dir >= 0 else -1,
        stripes_speed_leds=args.stripes_speed_leds,
        noise_rate_hz=args.noise_rate_hz,
        sparkle_rate_hz=args.sparkle_rate_hz,
        wave_rate_hz=args.wave_rate_hz,
        wave_wavelength=args.wave_len,
        wave_min_v=clamp01(args.wave_min),
        wave_color=_parse_wave_color(args.wave_color),
        strobe_rate_hz=args.strobe_rate_hz,
        strobe_duty=args.strobe_duty,
        palette_name=args.palette,
        pulse_min_v=clamp01(args.pulse_min),
        palette_min_v=clamp01(args.palette_min),
        fire_intensity=max(0.5, min(3.0, args.fire_intensity)),
        fire_min_v=clamp01(args.fire_min),
    )

    print("Type 'scan' to discover Pixel Poi devices, then 'connect all' or 'connect <idx> ...'.")
    print('For text mode: text "Hello World"  |  text speed 120  |  text color #FFFFFF  |  text bg #000000')
    try:
        await interactive_loop(streamer)
    finally:
        await streamer.stop_stream()
        await streamer.disconnect_all()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

