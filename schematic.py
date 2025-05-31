import schemdraw
import schemdraw.elements as elm
from schemdraw.elements.lines import Rect

# Standalone script: save as schematic.py and run after `pip install schemdraw`

with schemdraw.Drawing() as d:
    # --- Define power rails ---
    rails = {'3.3V': 4, '5V': 2, 'GND': 0}
    rail_x0 = 0    # start of rails
    rail_x1 = 4    # end of rails
    for net, y in rails.items():
        d += (
            elm.Line().at((rail_x0, y))
                     .right().length(rail_x1 - rail_x0)
                     .label(net, loc='left', fontsize=12)
        )

    # --- Draw ESP32 block ---
    esp_w, esp_h = 3, (max(rails.values()) - min(rails.values()) + 2)
    esp_origin = (rail_x1, max(rails.values()) + 1)  # top-left of ESP32
    corner1 = esp_origin
    corner2 = (esp_origin[0] + esp_w, esp_origin[1] - esp_h)
    d += (
        Rect(corner1=corner1, corner2=corner2, lw=2)
          .label('ESP32', loc='center', fontsize=14)
    )

    # --- Define ESP32 pins (label, x_offset, y_offset) ---
    esp_pin_defs = [
        ('3V3',  0,  -1),
        ('5V',   0,  -3),
        ('GND',  0,  -5),
        ('IO34', esp_w, -1),
        ('IO35', esp_w, -2),
        ('IO21', esp_w, -3),
        ('IO18', esp_w, -4),
        ('IO19', esp_w, -5),
        ('IO27', esp_w, -6),
        ('IO25', esp_w, -7),
    ]
    for lbl, xo, yo in esp_pin_defs:
        x = esp_origin[0] + xo
        y = esp_origin[1] + yo
        # draw stub
        d += elm.Line().at((x, y)).right().length(0.5)
        # place label via zero-length line
        d += (
            elm.Line().at((x + 0.7, y))
                     .right().length(0)
                     .label(lbl, loc='left', fontsize=10)
        )

    # --- Define modules and their pins ---
    comp_x = esp_origin[0] + esp_w + 2  # modules placed to right of ESP32
    comps = {
        'Joystick':   {'pos': (comp_x,  4),  'pins': [('VCC','3.3V'), ('GND','GND'), ('VRX','IO34'), ('VRY','IO35'), ('SW','IO21')]},
        'Encoder':    {'pos': (comp_x,  1),  'pins': [('A','IO18'),     ('B','IO19'),     ('C','GND')]},
        'PIR Sensor': {'pos': (comp_x, -2),  'pins': [('VCC','5V'),    ('GND','GND'),   ('OUT','IO27')]},
        'NeoPixel':   {'pos': (comp_x, -5),  'pins': [('VCC','5V'),    ('GND','GND'),   ('DIN','IO25')]},
    }

    # draw modules and connect
    for name, comp in comps.items():
        cx, cy = comp['pos']
        # draw component box
        d += (
            Rect(corner1=(cx, cy + 0.75), corner2=(cx + 3, cy - 0.75), lw=1)
              .label(name, loc='center', fontsize=12)
        )
        # pins and wires
        for i, (p_lbl, net) in enumerate(comp['pins']):
            pin_y = cy + 0.5 - 0.5 * i
            # stub + label
            d += elm.Line().at((cx, pin_y)).left().length(0.5)
            d += (
                elm.Line().at((cx - 1.0, pin_y))
                         .right().length(0)
                         .label(p_lbl, loc='right', fontsize=9)
            )
            # determine source coordinate
            if net in rails:
                src = (rail_x1, rails[net])
            else:
                # match ESP32
                for lbl, xo, yo in esp_pin_defs:
                    if net == lbl:
                        src = (esp_origin[0] + xo + 0.5, esp_origin[1] + yo)
                        break
            # draw orthogonal wire: vertical then horizontal
            d += elm.Line(shape='|-').at(src).to((cx - 0.5, pin_y))

    # save SVG
    d.save('light_controller_schematic.svg')

