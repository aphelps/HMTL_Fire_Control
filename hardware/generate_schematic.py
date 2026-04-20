#!/usr/bin/env python3
"""
Generates KiCad 9 schematic for the HMTL ESP32 Touch Controller PCB.

Usage (from hardware/ directory):
    python3 generate_schematic.py

Outputs:
    kicad/HMTL_ESP32_TouchController.kicad_sch
    kicad/HMTL_ESP32_TouchController.kicad_pro
"""

import math
import re
import uuid as _uuid_mod
from pathlib import Path

# ─── Configuration ────────────────────────────────────────────────────────────

OUT_DIR   = Path(__file__).parent / "kicad"
KICAD_SYM = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols")
SCH_FILE  = OUT_DIR / "HMTL_ESP32_TouchController.kicad_sch"
PRO_FILE  = OUT_DIR / "HMTL_ESP32_TouchController.kicad_pro"
PROJECT   = "HMTL_ESP32_TouchController"

SCH_UUID  = str(_uuid_mod.uuid4())   # stable per-run; the sheet path reference

# ─── UUID helper ──────────────────────────────────────────────────────────────

def u():
    return str(_uuid_mod.uuid4())

# ─── Symbol extraction ────────────────────────────────────────────────────────

def extract_raw(lib: str, sym: str) -> str:
    """Pull one symbol S-expression from a .kicad_sym file.
    Returns the block at its original 1-tab indentation level."""
    text = (KICAD_SYM / f"{lib}.kicad_sym").read_text()
    # Match with or without leading tab
    for marker in (f'\t(symbol "{sym}"', f'(symbol "{sym}"'):
        start = text.find(marker)
        if start != -1:
            break
    if start == -1:
        raise ValueError(f'Symbol "{sym}" not found in {lib}')
    depth, i = 0, start
    while i < len(text):
        if   text[i] == '(': depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                return text[start:i+1]
        i += 1
    raise ValueError(f'Unbalanced parens for "{sym}" in {lib}')

def _get_sexp_children(block: str) -> list:
    """Return list of (key, block_str) for each top-level child s-expression
    inside a symbol block (everything after the opening (symbol "name")."""
    # Skip past the symbol name: (symbol "name"\n
    i = block.index('(') + 1
    # Skip keyword 'symbol'
    while i < len(block) and block[i] in ' \t': i += 1
    while i < len(block) and block[i] not in ' \t\n': i += 1  # skip 'symbol'
    # Skip the quoted name
    while i < len(block) and block[i] != '"': i += 1
    i += 1  # opening quote
    while i < len(block) and block[i] != '"': i += 1
    i += 1  # closing quote

    children = []
    while i < len(block):
        if block[i] == '(':
            depth = 0
            j = i
            while j < len(block):
                if block[j] == '(': depth += 1
                elif block[j] == ')':
                    depth -= 1
                    if depth == 0: break
                j += 1
            child = block[i:j+1]
            m = re.match(r'\((\S+)', child)
            key = m.group(1) if m else '?'
            children.append((key, child))
            i = j + 1
        elif block[i] == ')':
            break  # closing paren of the symbol itself
        else:
            i += 1
    return children

def resolve_extends(lib: str, sym: str) -> str:
    """For a symbol with (extends "Base"), produce a fully resolved block:
    base geometry (sub-symbols) + derived properties, no extends clause."""
    block = extract_raw(lib, sym)
    m = re.search(r'\(extends "([^"]+)"\)', block)
    if not m:
        return block  # not an extends symbol

    base_name = m.group(1)
    base_block = extract_raw(lib, base_name)

    derived_children = _get_sexp_children(block)
    base_children    = _get_sexp_children(base_block)

    # From derived: all properties; from base: flags + sub-symbols + embedded_fonts
    derived_parts = [c for k, c in derived_children if k == 'property']
    base_flags    = [c for k, c in base_children if k in ('exclude_from_sim', 'in_bom', 'on_board')]
    base_geom     = [c for k, c in base_children if k == 'symbol']  # sub-symbol blocks
    base_misc     = [c for k, c in base_children if k == 'embedded_fonts']

    # Rename sub-symbol names from "Base_N_M" to "Sym_N_M"
    def _ren_geom(c: str) -> str:
        def _r(m2):
            n = m2.group(1)
            if n.startswith(base_name + '_'):
                return f'(symbol "{sym}{n[len(base_name):]}"'
            return m2.group(0)
        return re.sub(r'\(symbol "([^"]+)"', _r, c)

    base_geom = [_ren_geom(c) for c in base_geom]

    # Reassemble: \t(symbol "Sym" flags... properties... geometry... misc)
    inner = (
        '\n\t\t'.join([''] + base_flags)
        + '\n\t\t'.join([''] + derived_parts)
        + '\n\t\t'.join([''] + base_geom)
        + ('\n\t\t'.join([''] + base_misc) if base_misc else '')
    )
    return f'\t(symbol "{sym}"{inner}\n\t)'

# ─── Pin position computation ─────────────────────────────────────────────────

_PIN_CACHE: dict = {}

def get_pin_map(lib: str, sym: str) -> dict:
    """Return {pin_number: (x, y)} for a resolved symbol in Y-Up symbol space.
    Pins are read from the resolved symbol block (extends already merged)."""
    key = (lib, sym)
    if key in _PIN_CACHE:
        return _PIN_CACHE[key]

    block = resolve_extends(lib, sym)
    # Pattern: (pin TYPE LINE\n    (at X Y ANGLE)\n    (length L)\n    ...\n    (number "NUM"
    PIN_RE = re.compile(
        r'\(pin\s+\S+\s+\S+\s*\n\s*'
        r'\(at\s+(-?[\d.]+)\s+(-?[\d.]+)\s+\S+\)\s*\n\s*'
        r'\(length\s+[\d.]+\)\s*\n'
        r'(?:.*\n)*?'      # name block (variable)
        r'\s*\(number\s+"([^"]+)"',
        re.MULTILINE,
    )
    pins = {}
    for m in PIN_RE.finditer(block):
        pins[m.group(3)] = (float(m.group(1)), float(m.group(2)))
    _PIN_CACHE[key] = pins
    return pins

def PA(cx: float, cy: float, px: float, py: float, rotation: float = 0) -> tuple:
    """Pin Absolute position: convert (px,py) in Y-Up symbol space to schematic
    (Y-Down) absolute coordinates, given component centre (cx,cy) and rotation
    (degrees, CCW-positive in screen/Y-Down coordinates).

    Formula: abs_x = cx + px*cos(R) - py*sin(R)
             abs_y = cy - px*sin(R) - py*cos(R)   [Y-Down flip included]
    """
    r = math.radians(rotation)
    c, s = math.cos(r), math.sin(r)
    return (round(cx + px * c - py * s, 4),
            round(cy - px * s - py * c, 4))

def pin_pos(lib: str, sym: str, pin_num: str,
            cx: float, cy: float, rotation: float = 0) -> tuple:
    """Return absolute (x, y) of a named pin for a placed component."""
    pmap = get_pin_map(lib, sym)
    if pin_num not in pmap:
        raise KeyError(f'{lib}:{sym} has no pin "{pin_num}" — available: {sorted(pmap)}')
    px, py = pmap[pin_num]
    return PA(cx, cy, px, py, rotation)

def lib_symbol(lib: str, sym: str) -> str:
    """Extract (and resolve extends) symbol, rename with lib prefix, add one
    indent level so it sits correctly inside the schematic's lib_symbols section."""
    # Resolve extends chain so lib_symbols is self-contained (no base deps)
    block = resolve_extends(lib, sym)
    full  = f"{lib}:{sym}"

    # Rename only the top-level symbol; sub-symbols keep their short name
    # (e.g. "Conn_01x08_1_1" stays as-is, not "Connector_Generic:Conn_01x08_1_1")
    def _ren(m2):
        n = m2.group(1)
        if n == sym: return f'(symbol "{full}"'
        return m2.group(0)
    block = re.sub(r'\(symbol "([^"]+)"', _ren, block)

    # Add one extra tab level to every line (library = 1 tab, schematic lib_symbols = 2 tabs)
    return '\n'.join('\t' + line for line in block.split('\n'))

# ─── Custom ESP32-WROOM-32E symbol ────────────────────────────────────────────
# Pads per datasheet / footprint (RF_Module:ESP32-WROOM-32E)
# Left side: pads 1-15 (top = pad 1)
# Right side: pads 16-38, plus pad 39 = bottom thermal GND

ESP32_LEFT = [
    ("GND",    "1",  "power_in"),
    ("+3.3V",  "2",  "power_in"),
    ("EN",     "3",  "input"),
    ("GPIO36", "4",  "input"),
    ("GPIO39", "5",  "input"),
    ("GPIO34", "6",  "input"),
    ("GPIO35", "7",  "input"),
    ("GPIO32", "8",  "bidirectional"),   # SW3
    ("GPIO33", "9",  "bidirectional"),   # SW4
    ("GPIO25", "10", "bidirectional"),   # unused
    ("GPIO26", "11", "bidirectional"),   # SW1
    ("GPIO27", "12", "bidirectional"),   # SW2
    ("GPIO14", "13", "bidirectional"),   # WS2801 CLOCK (HSPI SCLK)
    ("GPIO12", "14", "bidirectional"),   # strapping – float
    ("GND",    "15", "power_in"),
]

ESP32_RIGHT = [
    ("GPIO13", "16", "bidirectional"),   # WS2801 DATA (HSPI MOSI)
    ("SD2",    "17", "no_connect"),      # internal flash
    ("SD3",    "18", "no_connect"),
    ("CMD",    "19", "no_connect"),
    ("CLK",    "20", "no_connect"),
    ("SD0",    "21", "no_connect"),
    ("SD1",    "22", "no_connect"),
    ("GPIO15", "23", "bidirectional"),   # strapping pull-up
    ("GPIO2",  "24", "bidirectional"),   # status LED + strapping
    ("GPIO0",  "25", "bidirectional"),   # BOOT
    ("GPIO4",  "26", "bidirectional"),   # MPR121 IRQ
    ("GPIO16", "27", "bidirectional"),   # RS485 RX
    ("GPIO17", "28", "bidirectional"),   # RS485 TX
    ("GPIO5",  "29", "bidirectional"),   # unused
    ("GPIO18", "30", "bidirectional"),   # RS485 EN
    ("GPIO19", "31", "bidirectional"),   # unused
    ("NC",     "32", "no_connect"),
    ("GPIO21", "33", "bidirectional"),   # SDA
    ("RXD0",   "34", "bidirectional"),   # UART0 RX ← CP2102N TX
    ("TXD0",   "35", "bidirectional"),   # UART0 TX → CP2102N RX
    ("GPIO22", "36", "bidirectional"),   # SCL
    ("GPIO23", "37", "bidirectional"),   # unused
    ("GND",    "38", "power_in"),
    ("GND",    "39", "power_in"),        # thermal pad
]

def _pin(name, number, ptype, x, y, angle):
    return (
        f'\t\t\t(pin {ptype} line\n'
        f'\t\t\t\t(at {x:.2f} {y:.2f} {angle})\n'
        f'\t\t\t\t(length 2.54)\n'
        f'\t\t\t\t(name "{name}"\n'
        f'\t\t\t\t\t(effects (font (size 1.016 1.016)))\n'
        f'\t\t\t\t)\n'
        f'\t\t\t\t(number "{number}"\n'
        f'\t\t\t\t\t(effects (font (size 1.016 1.016)))\n'
        f'\t\t\t\t)\n'
        f'\t\t\t)'
    )

def esp32_sym() -> str:
    """Build embedded ESP32-WROOM-32E symbol for lib_symbols (2-tab indent)."""
    step   = 2.54
    box_x  = 12.7
    n_r    = len(ESP32_RIGHT)
    top    = step                          # pin 1 y
    bot    = top - (n_r - 1) * step       # bottom extent driven by right side

    left_pins  = [_pin(n, num, pt, -box_x - 2.54, top - i*step, 0)
                  for i, (n, num, pt) in enumerate(ESP32_LEFT)]
    right_pins = [_pin(n, num, pt,  box_x + 2.54, top - i*step, 180)
                  for i, (n, num, pt) in enumerate(ESP32_RIGHT)]

    rect_top = top + step * 0.5
    rect_bot = bot - step * 0.5

    pins_str = '\n'.join(left_pins + right_pins)

    # The whole block at 2-tab indent so it sits inside lib_symbols correctly
    sym = (
        f'\t\t(symbol "Custom:ESP32-WROOM-32E"\n'
        f'\t\t\t(exclude_from_sim no)\n'
        f'\t\t\t(in_bom yes)\n'
        f'\t\t\t(on_board yes)\n'
        f'\t\t\t(property "Reference" "U"\n'
        f'\t\t\t\t(at 0 {rect_top + 2.54:.2f} 0)\n'
        f'\t\t\t\t(effects (font (size 1.27 1.27)))\n'
        f'\t\t\t)\n'
        f'\t\t\t(property "Value" "ESP32-WROOM-32E-N8"\n'
        f'\t\t\t\t(at 0 {rect_bot - 2.54:.2f} 0)\n'
        f'\t\t\t\t(effects (font (size 1.27 1.27)))\n'
        f'\t\t\t)\n'
        f'\t\t\t(property "Footprint" "RF_Module:ESP32-WROOM-32E"\n'
        f'\t\t\t\t(at 0 0 0)\n'
        f'\t\t\t\t(effects (font (size 1.27 1.27)) (hide yes))\n'
        f'\t\t\t)\n'
        f'\t\t\t(property "Datasheet" "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf"\n'
        f'\t\t\t\t(at 0 0 0)\n'
        f'\t\t\t\t(effects (font (size 1.27 1.27)) (hide yes))\n'
        f'\t\t\t)\n'
        f'\t\t\t(property "LCSC" "C701342"\n'
        f'\t\t\t\t(at 0 0 0)\n'
        f'\t\t\t\t(effects (font (size 1.27 1.27)) (hide yes))\n'
        f'\t\t\t)\n'
        f'\t\t\t(symbol "ESP32-WROOM-32E_0_1"\n'
        f'\t\t\t\t(rectangle\n'
        f'\t\t\t\t\t(start {-box_x:.2f} {rect_top:.2f})\n'
        f'\t\t\t\t\t(end   {box_x:.2f}  {rect_bot:.2f})\n'
        f'\t\t\t\t\t(stroke (width 0.254) (type default))\n'
        f'\t\t\t\t\t(fill (type background))\n'
        f'\t\t\t\t)\n'
        f'\t\t\t)\n'
        f'\t\t\t(symbol "ESP32-WROOM-32E_1_1"\n'
        f'{pins_str}\n'
        f'\t\t\t)\n'
        f'\t\t)'
    )
    return sym

# ─── Schematic element builders ───────────────────────────────────────────────

def _prop(key, val, x, y, angle=0, hide=False):
    hide_s = '\n\t\t\t\t(hide yes)' if hide else ''
    return (
        f'\t\t(property "{key}" "{val}"\n'
        f'\t\t\t(at {x:.2f} {y:.2f} {angle})\n'
        f'\t\t\t(effects\n'
        f'\t\t\t\t(font\n'
        f'\t\t\t\t\t(size 1.27 1.27)\n'
        f'\t\t\t\t){hide_s}\n'
        f'\t\t\t)\n'
        f'\t\t)'
    )

_ref_ctr = [0]

def sym(lib_id, x, y, ref, value, footprint, rotation=0,
        lcsc="", extra_props=None, pins=None, unit=1):
    """Build a component instance block."""
    _ref_ctr[0] += 1
    comp_uuid = u()

    props = [
        _prop("Reference", ref,       x, y - 2, rotation),
        _prop("Value",     value,     x, y + 2, rotation),
        _prop("Footprint", footprint, x, y,     rotation, hide=True),
        _prop("Datasheet", "~",       x, y,     rotation, hide=True),
    ]
    if lcsc:
        props.append(_prop("LCSC", lcsc, x, y, rotation, hide=True))
    if extra_props:
        for k, v in extra_props.items():
            props.append(_prop(k, v, x, y, rotation, hide=True))

    pin_lines = ""
    if pins:
        for n in pins:
            pin_lines += f'\t\t(pin "{n}"\n\t\t\t(uuid "{u()}")\n\t\t)\n'

    return (
        f'\t(symbol\n'
        f'\t\t(lib_id "{lib_id}")\n'
        f'\t\t(at {x:.2f} {y:.2f} {rotation})\n'
        f'\t\t(unit {unit})\n'
        f'\t\t(exclude_from_sim no)\n'
        f'\t\t(in_bom yes)\n'
        f'\t\t(on_board yes)\n'
        f'\t\t(dnp no)\n'
        f'\t\t(uuid "{comp_uuid}")\n'
        + '\n'.join(props) + '\n'
        + pin_lines
        + f'\t\t(instances\n'
        f'\t\t\t(project "{PROJECT}"\n'
        f'\t\t\t\t(path "/{SCH_UUID}"\n'
        f'\t\t\t\t\t(reference "{ref}")\n'
        f'\t\t\t\t\t(unit {unit})\n'
        f'\t\t\t\t)\n'
        f'\t\t\t)\n'
        f'\t\t)\n'
        f'\t)'
    )

def pwr(name, x, y):
    """Place a power symbol (GND / +3.3V / +5V)."""
    _ref_ctr[0] += 1
    ref = f"#PWR{_ref_ctr[0]:03d}"
    comp_uuid = u()
    pin_uuid  = u()
    rot = 0
    return (
        f'\t(symbol\n'
        f'\t\t(lib_id "power:{name}")\n'
        f'\t\t(at {x:.2f} {y:.2f} {rot})\n'
        f'\t\t(unit 1)\n'
        f'\t\t(exclude_from_sim no)\n'
        f'\t\t(in_bom yes)\n'
        f'\t\t(on_board yes)\n'
        f'\t\t(dnp no)\n'
        f'\t\t(uuid "{comp_uuid}")\n'
        + _prop("Reference", ref,  x, y-2, hide=True) + '\n'
        + _prop("Value",     name, x, y+2)             + '\n'
        + f'\t\t(pin "1"\n\t\t\t(uuid "{pin_uuid}")\n\t\t)\n'
        + f'\t\t(instances\n'
        f'\t\t\t(project "{PROJECT}"\n'
        f'\t\t\t\t(path "/{SCH_UUID}"\n'
        f'\t\t\t\t\t(reference "{ref}")\n'
        f'\t\t\t\t\t(unit 1)\n'
        f'\t\t\t\t)\n'
        f'\t\t\t)\n'
        f'\t\t)\n'
        f'\t)'
    )

def wire(x1, y1, x2, y2):
    return (
        f'\t(wire\n'
        f'\t\t(pts\n'
        f'\t\t\t(xy {x1:.2f} {y1:.2f}) (xy {x2:.2f} {y2:.2f})\n'
        f'\t\t)\n'
        f'\t\t(stroke (width 0) (type solid))\n'
        f'\t\t(uuid "{u()}")\n'
        f'\t)'
    )

def glabel(name, x, y, shape="passive", angle=0):
    """Global net label."""
    justify = "right" if angle in (180, 2) else "left"
    return (
        f'\t(global_label "{name}"\n'
        f'\t\t(shape {shape})\n'
        f'\t\t(at {x:.2f} {y:.2f} {angle})\n'
        f'\t\t(effects\n'
        f'\t\t\t(font (size 1.27 1.27))\n'
        f'\t\t\t(justify {justify})\n'
        f'\t\t)\n'
        f'\t\t(uuid "{u()}")\n'
        f'\t\t(property "Intersheet References" ""\n'
        f'\t\t\t(at 0 0 0)\n'
        f'\t\t\t(effects (font (size 1.27 1.27)) (hide yes))\n'
        f'\t\t)\n'
        f'\t)'
    )

def nc(x, y):
    return f'\t(no_connect\n\t\t(at {x:.2f} {y:.2f})\n\t\t(uuid "{u()}")\n\t)'

def pwr_flag(x, y):
    """Place a PWR_FLAG to tell KiCad that a power net is externally driven."""
    _ref_ctr[0] += 1
    ref = f"#FLG{_ref_ctr[0]:03d}"
    return sym("power:PWR_FLAG", x, y, ref, "PWR_FLAG", "", pins=["1"])

# ─── Component library list ───────────────────────────────────────────────────

LIB_SYMS = [
    ("Device",              "R"),
    ("Device",              "C"),
    ("Device",              "C_Polarized"),
    ("Device",              "LED"),
    ("Device",              "Polyfuse"),
    ("Switch",              "SW_Push"),
    ("Interface_USB",       "CP2102N-Axx-xQFN28"),
    ("Power_Protection",    "USBLC6-2SC6"),
    ("Connector",           "USB_C_Receptacle"),
    ("Regulator_Linear",    "AMS1117-3.3"),
    ("Interface_UART",      "SP3485EN"),
    ("Sensor_Touch",        "MPR121QR2"),
    ("Interface_Expansion", "PCF8574AT"),
    ("74xx",                "74AHCT125"),
    ("Connector_Generic",   "Conn_01x02"),
    ("Connector_Generic",   "Conn_01x03"),
    ("Connector_Generic",   "Conn_01x04"),
    ("Connector_Generic",   "Conn_01x05"),
    ("Connector_Generic",   "Conn_01x07"),
    ("power",               "GND"),
    ("power",               "+3.3V"),
    ("power",               "+5V"),
]

# ─── Place all components ─────────────────────────────────────────────────────

def build_elements():
    els = []
    e = els.append
    step = 2.54

    # Shorthand: compute absolute (x, y) of a pin for a placed component.
    # pin_pos(lib, sym, pin_num, cx, cy, rotation=0) applies the Y-Down flip.
    pp = pin_pos

    # ── USB-C input J8 ──────────────────────────────────────────────────────
    J8X, J8Y = 22, 35
    all_usbc_pins = ["A1","A2","A3","A4","A5","A6","A7","A8","A9",
                     "A10","A11","A12","B1","B2","B3","B4","B5","B6",
                     "B7","B8","B9","B10","B11","B12","S1"]
    e(sym("Connector:USB_C_Receptacle", J8X, J8Y, "J8", "USB_C_Receptacle",
          "Connector_USB:USB_C_Receptacle_GCT_USB4135-GF-A_Vertical",
          pins=all_usbc_pins))
    # VBUS: A4/B4/A9/B9 all at same position
    vbx, vby = pp("Connector", "USB_C_Receptacle", "A4", J8X, J8Y)
    e(glabel("VBUS_RAW", vbx, vby, "output"))
    # D+: A6 and B6 — both on USB_DP_RAW net
    for _p in ["A6", "B6"]:
        dpx, dpy = pp("Connector", "USB_C_Receptacle", _p, J8X, J8Y)
        e(glabel("USB_DP_RAW", dpx, dpy, "passive"))
    # D-: A7 and B7
    for _p in ["A7", "B7"]:
        dmx, dmy = pp("Connector", "USB_C_Receptacle", _p, J8X, J8Y)
        e(glabel("USB_DM_RAW", dmx, dmy, "passive"))
    # GND: A1, B1, A12, B12, S1 — all same bottom position
    gndx, gndy = pp("Connector", "USB_C_Receptacle", "A1", J8X, J8Y)
    e(pwr("GND", gndx, gndy))
    shx, shy = pp("Connector", "USB_C_Receptacle", "S1", J8X, J8Y)
    e(pwr("GND", shx, shy))
    # CC1/CC2 (A5, B5): no_connect (no CC resistors in this USB2.0-only design)
    for _p in ["A5", "B5"]:
        _nx, _ny = pp("Connector", "USB_C_Receptacle", _p, J8X, J8Y)
        e(nc(_nx, _ny))
    # SuperSpeed USB 3.1 pins: no_connect (USB 2.0 only)
    for _p in ["A2","A3","A8","A10","A11","B2","B3","B8","B10","B11"]:
        _nx, _ny = pp("Connector", "USB_C_Receptacle", _p, J8X, J8Y)
        e(nc(_nx, _ny))

    # F1 Polyfuse 500mA on VBUS (horizontal: pin1=left=VBUS_RAW, pin2=right=VBUS)
    F1X, F1Y = 48, 22
    e(sym("Device:Polyfuse", F1X, F1Y, "F1", "500mA",
          "Fuse:Fuse_1206_3216Metric", lcsc="C16188", rotation=90, pins=["1","2"]))
    f1p1x, f1p1y = pp("Device", "Polyfuse", "1", F1X, F1Y, 90)
    f1p2x, f1p2y = pp("Device", "Polyfuse", "2", F1X, F1Y, 90)
    e(glabel("VBUS_RAW", f1p1x, f1p1y, "input", 180))
    e(glabel("VBUS",     f1p2x, f1p2y, "output"))

    # D1 USBLC6-2SC6 ESD clamp
    D1X, D1Y = 22, 55
    e(sym("Power_Protection:USBLC6-2SC6", D1X, D1Y, "D1", "USBLC6-2SC6",
          "Package_TO_SOT_SMD:SOT-23-6", lcsc="C12044",
          pins=["1","2","3","4","5","6"]))
    # pin 1/6 = I/O1 (D+), pin 3/4 = I/O2 (D-), pin 5 = VBUS, pin 2 = GND
    d1p1x, d1p1y = pp("Power_Protection", "USBLC6-2SC6", "1", D1X, D1Y)
    d1p3x, d1p3y = pp("Power_Protection", "USBLC6-2SC6", "3", D1X, D1Y)
    d1p4x, d1p4y = pp("Power_Protection", "USBLC6-2SC6", "4", D1X, D1Y)
    d1p5x, d1p5y = pp("Power_Protection", "USBLC6-2SC6", "5", D1X, D1Y)
    d1p6x, d1p6y = pp("Power_Protection", "USBLC6-2SC6", "6", D1X, D1Y)
    d1p2x, d1p2y = pp("Power_Protection", "USBLC6-2SC6", "2", D1X, D1Y)
    e(glabel("USB_DP_RAW", d1p1x, d1p1y, "passive", 180))
    e(glabel("USB_DM_RAW", d1p3x, d1p3y, "passive", 180))
    e(glabel("USB_DP_RAW", d1p6x, d1p6y, "passive"))
    e(glabel("USB_DM_RAW", d1p4x, d1p4y, "passive"))
    e(glabel("VBUS", d1p5x, d1p5y, "input"))   # USBLC6 VBUS pin — clamp to VBUS rail
    e(pwr("GND", d1p2x, d1p2y))

    # R1, R2: 27Ω USB series resistors (horizontal: pin1=left=raw, pin2=right=filtered)
    R1X, R1Y = 52, 48
    R2X, R2Y = 52, 58
    e(sym("Device:R", R1X, R1Y, "R1", "27",
          "Resistor_SMD:R_0402_1005Metric", lcsc="C13389", rotation=90, pins=["1","2"]))
    e(sym("Device:R", R2X, R2Y, "R2", "27",
          "Resistor_SMD:R_0402_1005Metric", lcsc="C13389", rotation=90, pins=["1","2"]))
    r1p1x, r1p1y = pp("Device", "R", "1", R1X, R1Y, 90)
    r1p2x, r1p2y = pp("Device", "R", "2", R1X, R1Y, 90)
    r2p1x, r2p1y = pp("Device", "R", "1", R2X, R2Y, 90)
    r2p2x, r2p2y = pp("Device", "R", "2", R2X, R2Y, 90)
    e(glabel("USB_DP_RAW", r1p1x, r1p1y, "input", 180))
    e(glabel("USB_DM_RAW", r2p1x, r2p1y, "input", 180))
    e(glabel("USB_DP", r1p2x, r1p2y, "output"))
    e(glabel("USB_DM", r2p2x, r2p2y, "output"))

    # ── CP2102N USB-serial bridge ────────────────────────────────────────────
    U2X, U2Y = 90, 50
    e(sym("Interface_USB:CP2102N-Axx-xQFN28", U2X, U2Y, "U2", "CP2102N-A02-GQFN28R",
          "Package_DFN_QFN:QFN-28-1EP_5x5mm_P0.5mm_EP3.1x3.1mm",
          lcsc="C964632",
          pins=[str(n) for n in range(1, 30)]))
    # D+ pin 4, D- pin 5 (left side)
    u2dp_x, u2dp_y   = pp("Interface_USB", "CP2102N-Axx-xQFN28", "4",  U2X, U2Y)
    u2dm_x, u2dm_y   = pp("Interface_USB", "CP2102N-Axx-xQFN28", "5",  U2X, U2Y)
    u2rxd_x, u2rxd_y = pp("Interface_USB", "CP2102N-Axx-xQFN28", "25", U2X, U2Y)
    u2txd_x, u2txd_y = pp("Interface_USB", "CP2102N-Axx-xQFN28", "26", U2X, U2Y)
    u2vdd_x, u2vdd_y = pp("Interface_USB", "CP2102N-Axx-xQFN28", "6",  U2X, U2Y)
    u2gnd_x, u2gnd_y = pp("Interface_USB", "CP2102N-Axx-xQFN28", "3",  U2X, U2Y)
    e(glabel("USB_DP", u2dp_x,  u2dp_y,  "input", 180))
    e(glabel("USB_DM", u2dm_x,  u2dm_y,  "input", 180))
    e(glabel("ESP_RX", u2rxd_x, u2rxd_y, "output"))   # CP2102N RXD → ESP UART0 TX
    e(glabel("ESP_TX", u2txd_x, u2txd_y, "output"))   # CP2102N TXD → ESP UART0 RX
    e(pwr("+3.3V", u2vdd_x, u2vdd_y))
    e(pwr("GND",   u2gnd_x, u2gnd_y))
    # VREGIN (pin 7): connect to +3.3V when powered from external LDO
    u2vregin_x, u2vregin_y = pp("Interface_USB", "CP2102N-Axx-xQFN28", "7", U2X, U2Y)
    e(pwr("+3.3V", u2vregin_x, u2vregin_y))
    # VBUS (pin 8): USB VBUS voltage monitoring
    u2vbus_x, u2vbus_y = pp("Interface_USB", "CP2102N-Axx-xQFN28", "8", U2X, U2Y)
    e(glabel("VBUS", u2vbus_x, u2vbus_y, "input", 180))
    # RST# (pin 9): pull up to 3.3V
    u2rst_x, u2rst_y = pp("Interface_USB", "CP2102N-Axx-xQFN28", "9", U2X, U2Y)
    e(pwr("+3.3V", u2rst_x, u2rst_y))
    # Unused CP2102N pins: no_connect
    for _pn in ["1","2","11","12","13","14","15","16","17","18","19","20","21","22","23","24","27","28","29"]:
        _px, _py = pp("Interface_USB", "CP2102N-Axx-xQFN28", _pn, U2X, U2Y)
        e(nc(_px, _py))
    # VDD bypass cap: place vertically at (U2X+12, U2Y-17), pin1=top=+3.3V, pin2=bot=GND
    C10X, C10Y = U2X + 12, U2Y - 17
    e(sym("Device:C", C10X, C10Y, "C10", "100nF",
          "Capacitor_SMD:C_0402_1005Metric", lcsc="C14663", rotation=0, pins=["1","2"]))
    c10p1x, c10p1y = pp("Device", "C", "1", C10X, C10Y, 0)
    c10p2x, c10p2y = pp("Device", "C", "2", C10X, C10Y, 0)
    e(pwr("+3.3V", c10p1x, c10p1y))
    e(pwr("GND",   c10p2x, c10p2y))

    # ── AMS1117-3.3 LDO ─────────────────────────────────────────────────────
    U5X, U5Y = 140, 25
    e(sym("Regulator_Linear:AMS1117-3.3", U5X, U5Y, "U5", "AMS1117-3.3",
          "Package_TO_SOT_SMD:SOT-223-3_TabPin2", lcsc="C6186", pins=["1","2","3"]))
    u5vi_x, u5vi_y   = pp("Regulator_Linear", "AMS1117-3.3", "3", U5X, U5Y)
    u5vo_x, u5vo_y   = pp("Regulator_Linear", "AMS1117-3.3", "2", U5X, U5Y)
    u5gnd_x, u5gnd_y = pp("Regulator_Linear", "AMS1117-3.3", "1", U5X, U5Y)
    e(glabel("VBUS", u5vi_x, u5vi_y, "input", 180))
    e(pwr("+3.3V",   u5vo_x, u5vo_y))
    e(pwr("GND",     u5gnd_x, u5gnd_y))
    # Input bypass cap C7 (vertical, between VBUS and GND)
    C7X, C7Y = 128, 25
    e(sym("Device:C", C7X, C7Y, "C7", "10µF",
          "Capacitor_SMD:C_0805_2012Metric", lcsc="C15850", rotation=0, pins=["1","2"]))
    c7p1x, c7p1y = pp("Device", "C", "1", C7X, C7Y, 0)
    c7p2x, c7p2y = pp("Device", "C", "2", C7X, C7Y, 0)
    e(glabel("VBUS", c7p1x, c7p1y, "input", 180))
    e(pwr("GND",     c7p2x, c7p2y))
    # 3.3V bulk caps (vertical: pin1=top=+3.3V, pin2=bot=GND)
    for cref, cval, lx in [("C8","100µF",162), ("C5","10µF",170), ("C6","100nF",178)]:
        clib = "Device:C_Polarized" if cval == "100µF" else "Device:C"
        cfp  = ("Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" if cval == "100µF"
                else "Capacitor_SMD:C_0805_2012Metric" if cval == "10µF"
                else "Capacitor_SMD:C_0402_1005Metric")
        clcsc = "C16133" if cval == "100µF" else "C15850" if cval == "10µF" else "C14663"
        cpins = ["1","2"]
        e(sym(clib, lx, 22, cref, cval, cfp, lcsc=clcsc, rotation=0, pins=cpins))
        csym = "C" if cval != "100µF" else "C_Polarized"
        cp1x, cp1y = pp("Device", csym, "1", lx, 22, 0)
        cp2x, cp2y = pp("Device", csym, "2", lx, 22, 0)
        e(pwr("+3.3V", cp1x, cp1y))
        e(pwr("GND",   cp2x, cp2y))

    # ── ESP32-WROOM-32E (center) ─────────────────────────────────────────────
    # ESP32 symbol defines pin 1 at y=+step in Y-Up symbol space.
    # In the schematic (Y-Down), abs_y = EY - y_sym = EY - (step - i*step) = EY - step + i*step.
    # So pin index i maps to abs_y = EY - step + i*step (increases downward as i increases).
    EX, EY = 200, 140

    all_pins = [p[1] for p in ESP32_LEFT + ESP32_RIGHT]
    e(sym("Custom:ESP32-WROOM-32E", EX, EY, "U1", "ESP32-WROOM-32E-N8",
          "RF_Module:ESP32-WROOM-32E", lcsc="C701342", pins=all_pins))

    # Left-side pins (1-15): stub endpoint x = EX - 12.7 - 2.54 = EX - 15.24
    lx = EX - 15.24
    def lpin(i): return EY - step + i * step   # i=0→pin1 GND, increases downward

    e(pwr("GND",               lx, lpin(0)))
    e(pwr("+3.3V",             lx, lpin(1)))
    e(glabel("ESP_EN",         lx, lpin(2),  "passive", 180))
    e(nc(lx,                      lpin(3)))   # GPIO36
    e(nc(lx,                      lpin(4)))   # GPIO39
    e(nc(lx,                      lpin(5)))   # GPIO34
    e(nc(lx,                      lpin(6)))   # GPIO35
    e(glabel("SW3",            lx, lpin(7),  "passive", 180))
    e(glabel("SW4",            lx, lpin(8),  "passive", 180))
    e(nc(lx,                      lpin(9)))   # GPIO25 unused
    e(glabel("SW1",            lx, lpin(10), "passive", 180))
    e(glabel("SW2",            lx, lpin(11), "passive", 180))
    e(glabel("WS2801_CLK_3V3", lx, lpin(12), "passive", 180))
    e(nc(lx,                      lpin(13)))  # GPIO12 strapping
    e(pwr("GND",               lx, lpin(14)))

    # Right-side pins (16-39): stub endpoint x = EX + 12.7 + 2.54 = EX + 15.24
    rx = EX + 15.24
    def rpin(i): return EY - step + i * step   # i=0 → pin16 (GPIO13), increases downward

    e(glabel("WS2801_DAT_3V3", rx, rpin(0),  "passive"))
    e(nc(rx,                      rpin(1)))   # SD2
    e(nc(rx,                      rpin(2)))   # SD3
    e(nc(rx,                      rpin(3)))   # CMD
    e(nc(rx,                      rpin(4)))   # CLK (flash)
    e(nc(rx,                      rpin(5)))   # SD0
    e(nc(rx,                      rpin(6)))   # SD1
    e(glabel("GPIO15_PU",      rx, rpin(7),  "passive"))
    e(glabel("STATUS_LED",     rx, rpin(8),  "passive"))
    e(glabel("ESP_BOOT",       rx, rpin(9),  "passive"))
    e(glabel("MPR121_IRQ",     rx, rpin(10), "passive"))
    e(glabel("RS485_RX",       rx, rpin(11), "passive"))
    e(glabel("RS485_TX",       rx, rpin(12), "passive"))
    e(nc(rx,                      rpin(13)))  # GPIO5
    e(glabel("RS485_EN",       rx, rpin(14), "passive"))
    e(nc(rx,                      rpin(15)))  # GPIO19
    e(nc(rx,                      rpin(16)))  # NC
    e(glabel("SDA",            rx, rpin(17), "passive"))
    e(glabel("ESP_RX",         rx, rpin(18), "input",  180))
    e(glabel("ESP_TX",         rx, rpin(19), "output"))
    e(glabel("SCL",            rx, rpin(20), "passive"))
    e(nc(rx,                      rpin(21)))  # GPIO23
    e(pwr("GND",               rx, rpin(22)))  # pin 38
    e(pwr("GND",               rx, rpin(23)))  # pin 39

    # Bypass caps near ESP32 (vertical: pin1=top=+3.3V, pin2=bot=GND)
    for cref, cx in [("C1", EX - 8), ("C2", EX + 8)]:
        CY = EY - 32
        e(sym("Device:C", cx, CY, cref, "100nF",
              "Capacitor_SMD:C_0402_1005Metric", lcsc="C14663",
              rotation=0, pins=["1","2"]))
        cp1x, cp1y = pp("Device", "C", "1", cx, CY, 0)
        cp2x, cp2y = pp("Device", "C", "2", cx, CY, 0)
        e(pwr("+3.3V", cp1x, cp1y))
        e(pwr("GND",   cp2x, cp2y))

    # ── BOOT / RESET buttons, pull-ups, status LED ───────────────────────────
    # Pull-up resistors: vertical (rotation=0) so pin1=top=+3.3V, pin2=bot=signal

    # R3 GPIO0 pull-up (10k, vertical)
    R3X, R3Y = 168, 185
    e(sym("Device:R", R3X, R3Y, "R3", "10k",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C25804", rotation=0, pins=["1","2"]))
    r3p1x, r3p1y = pp("Device", "R", "1", R3X, R3Y, 0)
    r3p2x, r3p2y = pp("Device", "R", "2", R3X, R3Y, 0)
    e(pwr("+3.3V",          r3p1x, r3p1y))
    e(glabel("ESP_BOOT",    r3p2x, r3p2y, "passive"))

    # SW_BOOT button
    SW1X, SW1Y = 168, 200
    e(sym("Switch:SW_Push", SW1X, SW1Y, "SW_BOOT", "BOOT",
          "Button_Switch_SMD:SW_SPST_EVPBT_2.5x1.6mm", pins=["1","2"]))
    sw1p1x, sw1p1y = pp("Switch", "SW_Push", "1", SW1X, SW1Y)
    sw1p2x, sw1p2y = pp("Switch", "SW_Push", "2", SW1X, SW1Y)
    e(glabel("ESP_BOOT", sw1p1x, sw1p1y, "passive", 180))
    e(pwr("GND",         sw1p2x, sw1p2y))

    # R4 EN pull-up (10k, vertical)
    R4X, R4Y = 183, 185
    e(sym("Device:R", R4X, R4Y, "R4", "10k",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C25804", rotation=0, pins=["1","2"]))
    r4p1x, r4p1y = pp("Device", "R", "1", R4X, R4Y, 0)
    r4p2x, r4p2y = pp("Device", "R", "2", R4X, R4Y, 0)
    e(pwr("+3.3V",       r4p1x, r4p1y))
    e(glabel("ESP_EN",   r4p2x, r4p2y, "passive"))

    # C9 100nF EN debounce cap (vertical: pin1=top=ESP_EN, pin2=bot=GND)
    C9X, C9Y = 193, 185
    e(sym("Device:C", C9X, C9Y, "C9", "100nF",
          "Capacitor_SMD:C_0402_1005Metric", lcsc="C14663", rotation=0, pins=["1","2"]))
    c9p1x, c9p1y = pp("Device", "C", "1", C9X, C9Y, 0)
    c9p2x, c9p2y = pp("Device", "C", "2", C9X, C9Y, 0)
    e(glabel("ESP_EN", c9p1x, c9p1y, "passive"))
    e(pwr("GND",       c9p2x, c9p2y))

    # SW_RESET button
    SW2X, SW2Y = 183, 200
    e(sym("Switch:SW_Push", SW2X, SW2Y, "SW_RESET", "RESET",
          "Button_Switch_SMD:SW_SPST_EVPBT_2.5x1.6mm", pins=["1","2"]))
    sw2p1x, sw2p1y = pp("Switch", "SW_Push", "1", SW2X, SW2Y)
    sw2p2x, sw2p2y = pp("Switch", "SW_Push", "2", SW2X, SW2Y)
    e(glabel("ESP_EN", sw2p1x, sw2p1y, "passive", 180))
    e(pwr("GND",       sw2p2x, sw2p2y))

    # R5 GPIO2 STATUS_LED pull-up (10k, vertical)
    R5X, R5Y = 168, 218
    e(sym("Device:R", R5X, R5Y, "R5", "10k",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C25804", rotation=0, pins=["1","2"]))
    r5p1x, r5p1y = pp("Device", "R", "1", R5X, R5Y, 0)
    r5p2x, r5p2y = pp("Device", "R", "2", R5X, R5Y, 0)
    e(pwr("+3.3V",            r5p1x, r5p1y))
    e(glabel("STATUS_LED",    r5p2x, r5p2y, "passive"))

    # R6 GPIO15 pull-up (10k, vertical)
    R6X, R6Y = 183, 218
    e(sym("Device:R", R6X, R6Y, "R6", "10k",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C25804", rotation=0, pins=["1","2"]))
    r6p1x, r6p1y = pp("Device", "R", "1", R6X, R6Y, 0)
    r6p2x, r6p2y = pp("Device", "R", "2", R6X, R6Y, 0)
    e(pwr("+3.3V",          r6p1x, r6p1y))
    e(glabel("GPIO15_PU",   r6p2x, r6p2y, "passive"))

    # D2 status LED (rotation=0: pin1=A=left, pin2=K=right; current flows left→right)
    D2X, D2Y = 168, 233
    e(sym("Device:LED", D2X, D2Y, "D2", "STATUS",
          "LED_SMD:LED_0603_1608Metric", rotation=0, pins=["1","2"]))
    d2p1x, d2p1y = pp("Device", "LED", "1", D2X, D2Y, 0)
    d2p2x, d2p2y = pp("Device", "LED", "2", D2X, D2Y, 0)
    e(glabel("STATUS_LED", d2p1x, d2p1y, "passive", 180))

    # R7 330Ω current-limit (horizontal: pin1=left, from LED K, pin2=right=GND)
    R7X, R7Y = 178, 233
    e(sym("Device:R", R7X, R7Y, "R7", "330",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C23137", rotation=90, pins=["1","2"]))
    r7p1x, r7p1y = pp("Device", "R", "1", R7X, R7Y, 90)
    r7p2x, r7p2y = pp("Device", "R", "2", R7X, R7Y, 90)
    # Wire D2 pin2 (K) to R7 pin1 via same net label
    e(wire(d2p2x, d2p2y, r7p1x, r7p1y))
    e(pwr("GND", r7p2x, r7p2y))

    # ── SP3485EN RS485 transceiver ───────────────────────────────────────────
    U4X, U4Y = 65, 130
    e(sym("Interface_UART:SP3485EN", U4X, U4Y, "U4", "SP3485EN",
          "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", lcsc="C8963",
          pins=["1","2","3","4","5","6","7","8"]))
    u4ro_x,  u4ro_y  = pp("Interface_UART", "SP3485EN", "1", U4X, U4Y)
    u4re_x,  u4re_y  = pp("Interface_UART", "SP3485EN", "2", U4X, U4Y)
    u4de_x,  u4de_y  = pp("Interface_UART", "SP3485EN", "3", U4X, U4Y)
    u4di_x,  u4di_y  = pp("Interface_UART", "SP3485EN", "4", U4X, U4Y)
    u4gnd_x, u4gnd_y = pp("Interface_UART", "SP3485EN", "5", U4X, U4Y)
    u4a_x,   u4a_y   = pp("Interface_UART", "SP3485EN", "6", U4X, U4Y)
    u4b_x,   u4b_y   = pp("Interface_UART", "SP3485EN", "7", U4X, U4Y)
    u4vcc_x, u4vcc_y = pp("Interface_UART", "SP3485EN", "8", U4X, U4Y)
    e(glabel("RS485_RX", u4ro_x, u4ro_y, "output", 180))  # RO → ESP RX
    e(glabel("RS485_EN", u4re_x, u4re_y, "input",  180))  # RE# tied to DE
    e(glabel("RS485_EN", u4de_x, u4de_y, "input",  180))  # DE
    e(glabel("RS485_TX", u4di_x, u4di_y, "input",  180))  # DI ← ESP TX
    e(pwr("GND",    u4gnd_x, u4gnd_y))
    e(glabel("RS485_A", u4a_x, u4a_y, "passive"))
    e(glabel("RS485_B", u4b_x, u4b_y, "passive"))
    e(pwr("+3.3V",  u4vcc_x, u4vcc_y))

    # R8 120Ω termination resistor (vertical between RS485_A and RS485_B)
    R8X, R8Y = 48, 126
    e(sym("Device:R", R8X, R8Y, "R8", "120",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C23422", rotation=0, pins=["1","2"]))
    r8p1x, r8p1y = pp("Device", "R", "1", R8X, R8Y, 0)
    r8p2x, r8p2y = pp("Device", "R", "2", R8X, R8Y, 0)
    e(glabel("RS485_A", r8p1x, r8p1y, "passive"))
    e(glabel("RS485_B", r8p2x, r8p2y, "passive"))

    # R9/R10 bias resistors 680Ω (vertical: +3.3V on top, RS485_A/B on bottom)
    R9X,  R9Y  = 35, 118
    R10X, R10Y = 35, 132
    e(sym("Device:R", R9X,  R9Y,  "R9",  "680",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C23171", rotation=0, pins=["1","2"]))
    e(sym("Device:R", R10X, R10Y, "R10", "680",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C23171", rotation=0, pins=["1","2"]))
    r9p1x,  r9p1y  = pp("Device", "R", "1", R9X,  R9Y,  0)
    r9p2x,  r9p2y  = pp("Device", "R", "2", R9X,  R9Y,  0)
    r10p1x, r10p1y = pp("Device", "R", "1", R10X, R10Y, 0)
    r10p2x, r10p2y = pp("Device", "R", "2", R10X, R10Y, 0)
    e(pwr("+3.3V",      r9p1x,  r9p1y))
    e(glabel("RS485_A", r9p2x,  r9p2y,  "passive"))
    e(pwr("GND",        r10p1x, r10p1y))
    e(glabel("RS485_B", r10p2x, r10p2y, "passive"))

    # C3 RS485 VCC bypass cap (vertical)
    C3X, C3Y = 73, 130
    e(sym("Device:C", C3X, C3Y, "C3", "100nF",
          "Capacitor_SMD:C_0402_1005Metric", lcsc="C14663", rotation=0, pins=["1","2"]))
    c3p1x, c3p1y = pp("Device", "C", "1", C3X, C3Y, 0)
    c3p2x, c3p2y = pp("Device", "C", "2", C3X, C3Y, 0)
    e(pwr("+3.3V", c3p1x, c3p1y))
    e(pwr("GND",   c3p2x, c3p2y))

    # J2 RS485 screw terminal (3P: pin1=A, pin2=B, pin3=GND)
    J2X, J2Y = 22, 128
    e(sym("Connector_Generic:Conn_01x03", J2X, J2Y, "J2", "RS485",
          "Connector_PinArray:PinArray_1x03_Pitch5.00mm", pins=["1","2","3"]))
    j2p1x, j2p1y = pp("Connector_Generic", "Conn_01x03", "1", J2X, J2Y)
    j2p2x, j2p2y = pp("Connector_Generic", "Conn_01x03", "2", J2X, J2Y)
    j2p3x, j2p3y = pp("Connector_Generic", "Conn_01x03", "3", J2X, J2Y)
    e(glabel("RS485_A", j2p1x, j2p1y, "passive", 180))
    e(glabel("RS485_B", j2p2x, j2p2y, "passive", 180))
    e(pwr("GND",        j2p3x, j2p3y))

    # ── MPR121 capacitive touch ──────────────────────────────────────────────
    U3X, U3Y = 310, 110
    e(sym("Sensor_Touch:MPR121QR2", U3X, U3Y, "U3", "MPR121QR2",
          "Package_DFN_QFN:UQFN-20_3x3mm_P0.4mm", lcsc="C91322",
          pins=[str(n) for n in range(1, 21)]))
    # Left side: pin1=IRQ, 2=SCL, 3=SDA, 4=ADDR, 5=VREG, 7=REXT
    u3irq_x, u3irq_y   = pp("Sensor_Touch", "MPR121QR2", "1",  U3X, U3Y)
    u3scl_x, u3scl_y   = pp("Sensor_Touch", "MPR121QR2", "2",  U3X, U3Y)
    u3sda_x, u3sda_y   = pp("Sensor_Touch", "MPR121QR2", "3",  U3X, U3Y)
    u3addr_x, u3addr_y = pp("Sensor_Touch", "MPR121QR2", "4",  U3X, U3Y)
    u3vss_x, u3vss_y   = pp("Sensor_Touch", "MPR121QR2", "6",  U3X, U3Y)
    u3rext_x, u3rext_y = pp("Sensor_Touch", "MPR121QR2", "7",  U3X, U3Y)
    u3vdd_x, u3vdd_y   = pp("Sensor_Touch", "MPR121QR2", "20", U3X, U3Y)
    u3vreg_x, u3vreg_y = pp("Sensor_Touch", "MPR121QR2", "5",  U3X, U3Y)
    e(glabel("MPR121_IRQ", u3irq_x,  u3irq_y,  "output", 180))
    e(glabel("SCL",        u3scl_x,  u3scl_y,  "passive", 180))
    e(glabel("SDA",        u3sda_x,  u3sda_y,  "passive", 180))
    e(pwr("GND",           u3addr_x, u3addr_y))   # ADDR=GND → I2C 0x5A
    e(pwr("GND",           u3vss_x,  u3vss_y))    # VSS
    e(pwr("+3.3V",         u3vdd_x,  u3vdd_y))    # VDD
    e(nc(u3vreg_x, u3vreg_y))                     # VREG: internal regulator output, leave floating

    # R13 REXT 75kΩ (vertical: pin1 connects to REXT, pin2 to GND)
    R13X, R13Y = u3rext_x + 8, u3rext_y
    e(sym("Device:R", R13X, R13Y, "R13", "75k",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C23233", rotation=0, pins=["1","2"]))
    r13p1x, r13p1y = pp("Device", "R", "1", R13X, R13Y, 0)
    r13p2x, r13p2y = pp("Device", "R", "2", R13X, R13Y, 0)
    e(wire(u3rext_x, u3rext_y, r13p1x, r13p1y))
    e(pwr("GND", r13p2x, r13p2y))

    # C4 VDD bypass cap (vertical, near MPR121 VDD)
    C4X, C4Y = U3X + 15, u3vdd_y + 5
    e(sym("Device:C", C4X, C4Y, "C4", "100nF",
          "Capacitor_SMD:C_0402_1005Metric", lcsc="C14663", rotation=0, pins=["1","2"]))
    c4p1x, c4p1y = pp("Device", "C", "1", C4X, C4Y, 0)
    c4p2x, c4p2y = pp("Device", "C", "2", C4X, C4Y, 0)
    e(pwr("+3.3V", c4p1x, c4p1y))
    e(pwr("GND",   c4p2x, c4p2y))

    # Right side ELE0-ELE11 (pins 8-19)
    ele_pins = ["8","9","10","11","12","13","14","15","16","17","18","19"]
    ele_names = ["ELE0","ELE1","ELE2","ELE3","ELE4","ELE5",
                 "ELE6","ELE7","ELE8","ELE9","ELE10","ELE11"]
    for pnum, sig in zip(ele_pins, ele_names):
        ex, ey = pp("Sensor_Touch", "MPR121QR2", pnum, U3X, U3Y)
        e(glabel(sig, ex, ey, "passive"))

    # J6 ELE0-5 + GND (7P connector)
    J6X, J6Y = 360, 90
    e(sym("Connector_Generic:Conn_01x07", J6X, J6Y, "J6", "ELE0-5",
          "Connector_PinArray:PinArray_1x07_Pitch5.00mm",
          pins=["1","2","3","4","5","6","7"]))
    for i, sig in enumerate(["ELE0","ELE1","ELE2","ELE3","ELE4","ELE5"]):
        jpx, jpy = pp("Connector_Generic", "Conn_01x07", str(i+1), J6X, J6Y)
        e(glabel(sig, jpx, jpy, "passive", 180))
    j6p7x, j6p7y = pp("Connector_Generic", "Conn_01x07", "7", J6X, J6Y)
    e(pwr("GND", j6p7x, j6p7y))

    # J7 ELE6-11 + GND (7P connector)
    J7X, J7Y = 360, 112
    e(sym("Connector_Generic:Conn_01x07", J7X, J7Y, "J7", "ELE6-11",
          "Connector_PinArray:PinArray_1x07_Pitch5.00mm",
          pins=["1","2","3","4","5","6","7"]))
    for i, sig in enumerate(["ELE6","ELE7","ELE8","ELE9","ELE10","ELE11"]):
        jpx, jpy = pp("Connector_Generic", "Conn_01x07", str(i+1), J7X, J7Y)
        e(glabel(sig, jpx, jpy, "passive", 180))
    j7p7x, j7p7y = pp("Connector_Generic", "Conn_01x07", "7", J7X, J7Y)
    e(pwr("GND", j7p7x, j7p7y))

    # ── PCF8574AT I2C LCD expander ───────────────────────────────────────────
    U6X, U6Y = 310, 175
    e(sym("Interface_Expansion:PCF8574AT", U6X, U6Y, "U6", "PCF8574AT",
          "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm", lcsc="C398075",
          pins=[str(n) for n in range(1, 17)]))
    # Left side: pin14=SCL, 15=SDA, 13=INT#, 1=A0, 2=A1, 3=A2
    u6scl_x, u6scl_y = pp("Interface_Expansion", "PCF8574AT", "14", U6X, U6Y)
    u6sda_x, u6sda_y = pp("Interface_Expansion", "PCF8574AT", "15", U6X, U6Y)
    u6a0_x,  u6a0_y  = pp("Interface_Expansion", "PCF8574AT", "1",  U6X, U6Y)
    u6a1_x,  u6a1_y  = pp("Interface_Expansion", "PCF8574AT", "2",  U6X, U6Y)
    u6a2_x,  u6a2_y  = pp("Interface_Expansion", "PCF8574AT", "3",  U6X, U6Y)
    u6gnd_x, u6gnd_y = pp("Interface_Expansion", "PCF8574AT", "8",  U6X, U6Y)
    u6vdd_x, u6vdd_y = pp("Interface_Expansion", "PCF8574AT", "16", U6X, U6Y)
    e(glabel("SCL", u6scl_x, u6scl_y, "passive", 180))
    e(glabel("SDA", u6sda_x, u6sda_y, "passive", 180))
    e(pwr("GND",    u6a0_x,  u6a0_y))   # A0 → GND (addr 0x27)
    e(pwr("GND",    u6a1_x,  u6a1_y))   # A1 → GND
    e(pwr("GND",    u6a2_x,  u6a2_y))   # A2 → GND
    e(pwr("GND",    u6gnd_x, u6gnd_y))
    e(pwr("+3.3V",  u6vdd_x, u6vdd_y))
    # INT# (pin 13): not used, no_connect
    u6int_x, u6int_y = pp("Interface_Expansion", "PCF8574AT", "13", U6X, U6Y)
    e(nc(u6int_x, u6int_y))
    # P0-P7 (pins 4-7, 9-12): LCD data bus — physically connected on PCF8574AT backpack board
    for _pn in ["4","5","6","7","9","10","11","12"]:
        _px, _py = pp("Interface_Expansion", "PCF8574AT", _pn, U6X, U6Y)
        e(nc(_px, _py))

    # C11 PCF8574AT VDD bypass cap (vertical)
    C11X, C11Y = U6X + 15, u6vdd_y + 5
    e(sym("Device:C", C11X, C11Y, "C11", "100nF",
          "Capacitor_SMD:C_0402_1005Metric", lcsc="C14663", rotation=0, pins=["1","2"]))
    c11p1x, c11p1y = pp("Device", "C", "1", C11X, C11Y, 0)
    c11p2x, c11p2y = pp("Device", "C", "2", C11X, C11Y, 0)
    e(pwr("+3.3V", c11p1x, c11p1y))
    e(pwr("GND",   c11p2x, c11p2y))

    # I2C pull-ups R11 SDA 4.7kΩ, R12 SCL 4.7kΩ (vertical)
    R11X, R11Y = 285, 160
    R12X, R12Y = 292, 160
    e(sym("Device:R", R11X, R11Y, "R11", "4.7k",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C23162", rotation=0, pins=["1","2"]))
    e(sym("Device:R", R12X, R12Y, "R12", "4.7k",
          "Resistor_SMD:R_0603_1608Metric", lcsc="C23162", rotation=0, pins=["1","2"]))
    r11p1x, r11p1y = pp("Device", "R", "1", R11X, R11Y, 0)
    r11p2x, r11p2y = pp("Device", "R", "2", R11X, R11Y, 0)
    r12p1x, r12p1y = pp("Device", "R", "1", R12X, R12Y, 0)
    r12p2x, r12p2y = pp("Device", "R", "2", R12X, R12Y, 0)
    e(pwr("+3.3V",  r11p1x, r11p1y))
    e(pwr("+3.3V",  r12p1x, r12p1y))
    e(glabel("SDA", r11p2x, r11p2y, "passive"))
    e(glabel("SCL", r12p2x, r12p2y, "passive"))

    # J3 LCD connector (4P: pin1=SDA, 2=SCL, 3=+3.3V, 4=GND)
    J3X, J3Y = 360, 167
    e(sym("Connector_Generic:Conn_01x04", J3X, J3Y, "J3", "LCD",
          "Connector_PinArray:PinArray_1x04_Pitch5.00mm", pins=["1","2","3","4"]))
    j3p1x, j3p1y = pp("Connector_Generic", "Conn_01x04", "1", J3X, J3Y)
    j3p2x, j3p2y = pp("Connector_Generic", "Conn_01x04", "2", J3X, J3Y)
    j3p3x, j3p3y = pp("Connector_Generic", "Conn_01x04", "3", J3X, J3Y)
    j3p4x, j3p4y = pp("Connector_Generic", "Conn_01x04", "4", J3X, J3Y)
    e(glabel("SDA",   j3p1x, j3p1y, "passive", 180))
    e(glabel("SCL",   j3p2x, j3p2y, "passive", 180))
    e(pwr("+3.3V",    j3p3x, j3p3y))
    e(pwr("GND",      j3p4x, j3p4y))

    # ── 74AHCT125 level shifter (WS2801 DATA + CLOCK 3.3V→5V) ───────────────
    # 74AHCT125 is a quad buffer (units A-D = channels, unit E = power VCC/GND).
    # Place unit A (channel 1, pins 1/2/3) for DATA, and unit E (power) separately.
    U7X, U7Y = 200, 235
    # Unit A — channel 1 (DATA): pins 1(OE#), 2(A), 3(Y)
    e(sym("74xx:74AHCT125", U7X, U7Y, "U7", "74AHCT125D",
          "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm", lcsc="C2595",
          pins=["1","2","3"], unit=1))
    u7oe1_x, u7oe1_y = pp("74xx", "74AHCT125", "1",  U7X, U7Y)  # OE# ch1
    u7a1_x,  u7a1_y  = pp("74xx", "74AHCT125", "2",  U7X, U7Y)  # A ch1 (DATA)
    u7y1_x,  u7y1_y  = pp("74xx", "74AHCT125", "3",  U7X, U7Y)  # Y ch1 (DATA out)
    e(pwr("GND", u7oe1_x, u7oe1_y))    # OE# = GND → always enabled
    e(glabel("WS2801_DAT_3V3", u7a1_x, u7a1_y, "input", 180))
    e(glabel("WS2801_DAT_5V",  u7y1_x, u7y1_y, "output"))
    # Unit E — power (VCC pin 14, GND pin 7); placed offset to avoid overlap
    U7EX, U7EY = U7X + 15, U7Y
    e(sym("74xx:74AHCT125", U7EX, U7EY, "U7", "74AHCT125D",
          "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm", lcsc="C2595",
          pins=["7","14"], unit=5))
    u7vcc_x, u7vcc_y = pp("74xx", "74AHCT125", "14", U7EX, U7EY)
    u7gnd_x, u7gnd_y = pp("74xx", "74AHCT125", "7",  U7EX, U7EY)
    e(pwr("+5V", u7vcc_x, u7vcc_y))
    e(pwr("GND", u7gnd_x, u7gnd_y))

    # C12 +5V bypass cap near 74AHCT125 (vertical)
    C12X, C12Y = U7X + 12, U7Y - 8
    e(sym("Device:C", C12X, C12Y, "C12", "100nF",
          "Capacitor_SMD:C_0402_1005Metric", lcsc="C14663", rotation=0, pins=["1","2"]))
    c12p1x, c12p1y = pp("Device", "C", "1", C12X, C12Y, 0)
    c12p2x, c12p2y = pp("Device", "C", "2", C12X, C12Y, 0)
    e(pwr("+5V", c12p1x, c12p1y))
    e(pwr("GND", c12p2x, c12p2y))

    # J4 WS2801 connector (4P: pin1=DATA, 2=CLOCK, 3=+5V, 4=GND)
    J4X, J4Y = 235, 233
    e(sym("Connector_Generic:Conn_01x04", J4X, J4Y, "J4", "WS2801 LEDs",
          "Connector_PinArray:PinArray_1x04_Pitch5.00mm", pins=["1","2","3","4"]))
    j4p1x, j4p1y = pp("Connector_Generic", "Conn_01x04", "1", J4X, J4Y)
    j4p2x, j4p2y = pp("Connector_Generic", "Conn_01x04", "2", J4X, J4Y)
    j4p3x, j4p3y = pp("Connector_Generic", "Conn_01x04", "3", J4X, J4Y)
    j4p4x, j4p4y = pp("Connector_Generic", "Conn_01x04", "4", J4X, J4Y)
    e(glabel("WS2801_DAT_5V",  j4p1x, j4p1y, "passive", 180))
    e(glabel("WS2801_CLK_5V",  j4p2x, j4p2y, "passive", 180))
    e(pwr("+5V",               j4p3x, j4p3y))
    e(pwr("GND",               j4p4x, j4p4y))

    # ── Rocker switches + pull-ups (top-right) ───────────────────────────────
    # R14-R17 pull-ups 10kΩ (vertical: pin1=top=+3.3V, pin2=bot=SW net)
    sw_sigs = [("SW1","R14"), ("SW2","R15"), ("SW3","R16"), ("SW4","R17")]
    for i, (sig, rref) in enumerate(sw_sigs):
        rx_ = 348 + i * 8
        ry_ = 42
        e(sym("Device:R", rx_, ry_, rref, "10k",
              "Resistor_SMD:R_0603_1608Metric", lcsc="C25804",
              rotation=0, pins=["1","2"]))
        rp1x, rp1y = pp("Device", "R", "1", rx_, ry_, 0)
        rp2x, rp2y = pp("Device", "R", "2", rx_, ry_, 0)
        e(pwr("+3.3V",  rp1x, rp1y))
        e(glabel(sig,   rp2x, rp2y, "passive"))

    # J5 Switches (5P: pin1-4=SW1-4, pin5=GND)
    J5X, J5Y = 380, 53
    e(sym("Connector_Generic:Conn_01x05", J5X, J5Y, "J5", "Switches",
          "Connector_PinArray:PinArray_1x05_Pitch5.00mm", pins=["1","2","3","4","5"]))
    for i, sig in enumerate(["SW1","SW2","SW3","SW4"]):
        jpx, jpy = pp("Connector_Generic", "Conn_01x05", str(i+1), J5X, J5Y)
        e(glabel(sig, jpx, jpy, "passive", 180))
    j5p5x, j5p5y = pp("Connector_Generic", "Conn_01x05", "5", J5X, J5Y)
    e(pwr("GND", j5p5x, j5p5y))

    # ── Power input J1 (2P: pin1=VBUS, pin2=GND) ─────────────────────────────
    J1X, J1Y = 148, 265
    e(sym("Connector_Generic:Conn_01x02", J1X, J1Y, "J1", "+5V Power In",
          "Connector_PinArray:PinArray_1x02_Pitch5.00mm", pins=["1","2"]))
    j1p1x, j1p1y = pp("Connector_Generic", "Conn_01x02", "1", J1X, J1Y)
    j1p2x, j1p2y = pp("Connector_Generic", "Conn_01x02", "2", J1X, J1Y)
    e(glabel("VBUS", j1p1x, j1p1y, "input", 180))
    e(pwr("GND",     j1p2x, j1p2y))

    # WS2801_CLK net: tie CLK output from 74AHCT125 ch2 to J4 pin2
    # (74AHCT125 ch2 uses pins 4/5/6 which overlap ch1 in unit=1 placement;
    #  connect via net label only — layout handles physical separation)
    e(glabel("WS2801_CLK_5V", j4p2x, j4p2y, "passive", 180))
    e(glabel("WS2801_CLK_3V3", u7a1_x, u7a1_y + step, "input", 180))

    return els

# ─── Assemble & write ─────────────────────────────────────────────────────────

def generate():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # Build lib_symbols (tab-indented correctly for 2-tab depth)
    lib_parts = []
    for lib, sym_name in LIB_SYMS:
        try:
            lib_parts.append(lib_symbol(lib, sym_name))
        except Exception as e:
            print(f"WARNING: {e}")
    lib_parts.append(esp32_sym())

    elements = build_elements()

    sch = (
        "(kicad_sch\n"
        "\t(version 20250114)\n"
        "\t(generator \"eeschema\")\n"
        "\t(generator_version \"9.0\")\n"
        f"\t(uuid \"{SCH_UUID}\")\n"
        "\t(paper \"A3\")\n"
        "\t(title_block\n"
        "\t\t(title \"HMTL ESP32 Touch Controller\")\n"
        "\t\t(rev \"1.0\")\n"
        "\t\t(company \"HMTL\")\n"
        "\t\t(comment 1 \"ESP32-WROOM-32E-N8 | MPR121QR2 | CP2102N | SP3485EN\")\n"
        "\t\t(comment 2 \"Wickerman Fire Control — Touch Controller PCB\")\n"
        "\t)\n"
        "\t(lib_symbols\n"
        + "\n".join(lib_parts) + "\n"
        + "\t)\n"
        + "\n".join(elements) + "\n"
        + ")\n"
    )

    SCH_FILE.write_text(sch)
    print(f"Written: {SCH_FILE}")

    pro = (
        '{\n'
        '  "meta": {"filename": "' + PROJECT + '.kicad_pro", "version": 1},\n'
        '  "schematic": {\n'
        '    "annotate_start_num": 0,\n'
        '    "drawing": {"default_line_thickness": 6, "default_text_size": 50,\n'
        '      "field_names": [], "junction_size_choice": 3,\n'
        '      "label_size_ratio": 0.25, "pin_symbol_size": 0,\n'
        '      "text_offset_ratio": 0.08},\n'
        '    "legacy_lib_dir": "", "legacy_lib_list": [],\n'
        '    "net_format_name": "", "ngspice_settings": {},\n'
        '    "page_layout_descr_file": "",\n'
        '    "plot_directory": ""\n'
        '  },\n'
        '  "boards": [], "sheets": [], "text_variables": {}\n'
        '}\n'
    )
    PRO_FILE.write_text(pro)
    print(f"Written: {PRO_FILE}")


if __name__ == "__main__":
    generate()
