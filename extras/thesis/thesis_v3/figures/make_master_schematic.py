#!/usr/bin/env python3
"""The master arm's circuit, every wire to its pin, drawn from the firmware's
own pin map (firmware/master_arm/teensy_final.ino) and the parts in the
purchase records (extras/thesis/thesis_report/_source/artefacts/BOM).

    ~/.venv_schem/bin/python extras/thesis/thesis_v3/figures/make_master_schematic.py

Writes figures/master_arm_schematic.pdf (+ .png).  Teensy 4.1 physical pin
numbers follow the PJRC pinout card: A0-A9 = pins 14-23, A10-A13 = 24-27,
A14-A17 = 38-41, SDA0/SCL0 = 18/19.

schemdraw adds every element built inside the Drawing context; `d += e` on
an element built there forces its placement so its anchors can be read.
"""
import os
import schemdraw
import schemdraw.elements as elm

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "master_arm_schematic")

# --- the pin map, verbatim from teensy_final.ino ---------------------------
K1 = [("A0", 14), ("A1", 15), ("A2", 16), ("A3", 17), ("A6", 20), ("A7", 21), ("A8", 22)]      # left arm j1..j7
K2 = [("A9", 23), ("A10", 24), ("A11", 25), ("A12", 26), ("A13", 27), ("A16", 40), ("A17", 41)]  # right arm j1..j7
FSR1, FSR2 = ("A14", 38), ("A15", 39)
BTN1, BTN2 = 2, 3
SDA, SCL = 18, 19
JOINT = ["J1 roll", "J2 bend", "J3 roll", "J4 bend", "J5 roll", "J6 bend", "J7 roll"]

BLUE, RED, GREEN, GREY, PURPLE = "#2c6fbb", "#c1392b", "#3f9142", "#595959", "#8e44ad"
NSLOT = 15
ROW = 0.7                                 # row pitch on the Teensy symbol
POT_SLOTS = list(range(15, 8, -1))        # J1..J7 at rows 15..9
FSR_SLOT, BTN_SLOT, SDA_SLOT, SCL_SLOT = 7, 5, 3, 2
F_PIN, F_NUM, F_NOTE, F_SMALL, F_TITLE = 8.0, 7.5, 8.0, 7.5, 8.5


def main():
    with schemdraw.Drawing(show=False) as d:
        d.config(unit=2.0, fontsize=8, inches_per_unit=0.5, lw=1.1)

        # ------------------------------------------------------------------
        # Teensy 4.1
        # ------------------------------------------------------------------
        pins = []
        for i, (a, p) in enumerate(K1):
            pins.append(elm.IcPin(name="K1J%d" % (i + 1), pin="%s/%d" % (a, p), side="left", slot="%d/%d" % (POT_SLOTS[i], NSLOT), anchorname="K1J%d" % (i + 1)))
        pins.append(elm.IcPin(name="FSR1", pin="%s/%d" % FSR1, side="left", slot="%d/%d" % (FSR_SLOT, NSLOT), anchorname="FSR1"))
        pins.append(elm.IcPin(name="BTN1", pin="%d" % BTN1, side="left", slot="%d/%d" % (BTN_SLOT, NSLOT), anchorname="BTN1"))
        pins.append(elm.IcPin(name="SDA0", pin="%d" % SDA, side="left", slot="%d/%d" % (SDA_SLOT, NSLOT), anchorname="SDA0"))
        pins.append(elm.IcPin(name="SCL0", pin="%d" % SCL, side="left", slot="%d/%d" % (SCL_SLOT, NSLOT), anchorname="SCL0"))
        for i, (a, p) in enumerate(K2):
            pins.append(elm.IcPin(name="K2J%d" % (i + 1), pin="%s/%d" % (a, p), side="right", slot="%d/%d" % (POT_SLOTS[i], NSLOT), anchorname="K2J%d" % (i + 1)))
        pins.append(elm.IcPin(name="FSR2", pin="%s/%d" % FSR2, side="right", slot="%d/%d" % (FSR_SLOT, NSLOT), anchorname="FSR2"))
        pins.append(elm.IcPin(name="BTN2", pin="%d" % BTN2, side="right", slot="%d/%d" % (BTN_SLOT, NSLOT), anchorname="BTN2"))
        for k, name in enumerate(["3V3", "GND"]):
            pins.append(elm.IcPin(name=name, side="top", slot="%d/2" % (k + 1), anchorname="T" + name))
        pins.append(elm.IcPin(name="GND", side="bottom", slot="1/2", anchorname="BGND"))
        pins.append(elm.IcPin(name="USB", side="bottom", slot="2/2", anchorname="BUSB"))
        T = elm.Ic(pins=pins, pinspacing=ROW, edgepadH=0.6, edgepadW=1.1, leadlen=0.6,
                   lsize=F_PIN, plblsize=F_NUM).theta(0).at((0, 0)).anchor("center")
        d += T
        bb = T.get_bbox(transform=True)
        d += elm.Label().at((0, (T.K1J3.y + T.K1J5.y) / 2)).label(
            "Teensy 4.1\ni.MX RT1062\n3.3 V logic\n12-bit ADC\n100 Hz loop\nUSB CDC 115200 Bd", fontsize=F_NOTE, ofst=0)
        d += elm.Label().at((0, (T.K1J7.y + T.FSR1.y) / 2)).label(
            "per pot:\ne = (64 r + 192 e)/256\n50..4045 -> 0..360 deg", fontsize=F_SMALL, color=GREY, ofst=0)
        d += elm.Label().at((0, (T.BTN1.y + T.SDA0.y) / 2)).label(
            "buttons: INPUT_PULLUP,\n40 ms debounce,\ntoggle per press", fontsize=F_SMALL, color=GREY, ofst=0)

        # ------------------------------------------------------------------
        # one arm: seven pots between a 3V3 rail and a GND rail
        # ------------------------------------------------------------------
        def arm(names, side, rail_v, rail_g, xpot, title):
            nonlocal d
            sgn = -1 if side == "left" else 1
            ys = [getattr(T, nm).y for nm in names]
            for k, nm in enumerate(names):
                pin = getattr(T, nm)
                # the track: a plain resistor between the rails, shorter than the row pitch so
                # neighbouring pots are visibly separate; the wiper: an arrow into the track
                # from the Teensy side, its wire running straight to the pin
                # the track drawn by hand as a small box, so its size is the row's and not the
                # library's fixed resistor body (which overran the row pitch)
                hb, wb = 0.17, 0.08
                for (xa, ya, xb, yb) in ((xpot - wb, pin.y - hb, xpot + wb, pin.y - hb), (xpot + wb, pin.y - hb, xpot + wb, pin.y + hb),
                                         (xpot + wb, pin.y + hb, xpot - wb, pin.y + hb), (xpot - wb, pin.y + hb, xpot - wb, pin.y - hb)):
                    d += elm.Line().at((xa, ya)).to((xb, yb))
                top_pt = type("P", (), {})(); bot_pt = type("P", (), {})()
                top_pt.x, top_pt.y = xpot, pin.y + hb
                bot_pt.x, bot_pt.y = xpot, pin.y - hb
                xw = xpot - sgn * 0.55            # where the wiper wire meets the arrow
                d += elm.Arrow(headwidth=0.16, headlength=0.14).at((xw, pin.y)).to((xpot - sgn * wb, pin.y))
                d += elm.Line().at((xw, pin.y)).tox(pin.x)
                d += elm.Line().at((top_pt.x, top_pt.y)).tox(rail_v)
                d += elm.Dot(radius=0.05).at((rail_v, top_pt.y))
                d += elm.Line().at((bot_pt.x, bot_pt.y)).tox(rail_g)
                d += elm.Dot(radius=0.05).at((rail_g, bot_pt.y))
                d += elm.Label().at((xpot - sgn * 0.62, pin.y + 0.07)).label(
                    "%s-%s" % (("L" if side == "left" else "R"), JOINT[k]), fontsize=F_SMALL, ofst=0,
                    halign="left" if side == "left" else "right", valign="bottom", color=BLUE)
            top, bot = max(ys) + 0.17, min(ys) - 0.17
            d += elm.Line().at((rail_v, top)).toy(bot).color(RED)
            d += elm.Vdd().theta(0).at((rail_v, top)).label("3V3", fontsize=F_SMALL, color=RED)
            d += elm.Line().at((rail_g, top)).toy(bot).color(GREY)
            d += elm.Ground().theta(0).at((rail_g, bot))
            d += elm.Label().at((xpot + sgn * 0.2, top + 1.1)).label(title, fontsize=F_TITLE, color=BLUE, ofst=0)

        xL, xR = -4.0, 4.0
        railL_v, railL_g = -5.6, -6.1
        railR_v, railR_g = 5.6, 6.1
        arm(["K1J%d" % (i + 1) for i in range(7)], "left", railL_v, railL_g, xL,
            "LEFT arm (K1): seven WH148 B10K\n10 k linear pots, wipers to ADC")
        arm(["K2J%d" % (i + 1) for i in range(7)], "right", railR_v, railR_g, xR,
            "RIGHT arm (K2): seven WH148 B10K\n10 k linear pots, wipers to ADC")

        # ------------------------------------------------------------------
        # FSRs: 3V3 -> FSR -> node -> ADC ; node -> R_pd -> GND
        # ------------------------------------------------------------------
        def fsr(pinname, side, xnode, label):
            nonlocal d
            pin = getattr(T, pinname)
            sgn = -1 if side == "left" else 1
            xsup = xnode + sgn * 1.6
            d += elm.Vdd().theta(0).at((xsup, pin.y)).label("3V3", fontsize=F_SMALL, color=RED)
            F = elm.ResistorIEC().at((xsup, pin.y)).tox(xnode).label("FSR", loc="top", fontsize=F_SMALL)
            d += F
            d += elm.Dot(radius=0.05).at((xnode, pin.y))
            d += elm.Line().at((xnode, pin.y)).tox(pin.x)
            R = elm.ResistorIEC().at((xnode, pin.y)).down().length(0.9).label("R_pd", loc="bottom", fontsize=F_SMALL)
            d += R
            d += elm.Ground().theta(0).at(R.end)
            d += elm.Label().at((xsup - sgn * 0.3, pin.y - 0.4)).label(label, fontsize=F_SMALL, valign="top", color=GREEN, ofst=0,
                                                                      halign="right" if side == "left" else "left")

        fsr("FSR1", "left", -3.6, "fsr1: LEFT grip pad,\nthin-film FSR in a\nvoltage divider")
        fsr("FSR2", "right", 3.6, "fsr2: RIGHT grip pad,\nthin-film FSR in a\nvoltage divider")

        # ------------------------------------------------------------------
        # buttons: pin -> momentary switch -> GND, internal pull-up
        # ------------------------------------------------------------------
        def button(pinname, side, xsw, label):
            nonlocal d
            pin = getattr(T, pinname)
            sgn = -1 if side == "left" else 1
            d += elm.Line().at((pin.x, pin.y)).tox(xsw)
            B = elm.Button().at((xsw, pin.y)).tox(xsw + sgn * 1.4)
            d += B
            d += elm.Ground().theta(0).at(B.end)
            d += elm.Label().at((xsw + sgn * 0.8, pin.y - 0.8)).label(label, fontsize=F_SMALL, valign="top", color=PURPLE, ofst=0)

        button("BTN1", "left", -3.4, "btn1: 12 mm momentary\nswitch; the RIGHT-hand\nbutton (measured)")
        button("BTN2", "right", 3.4, "btn2: 12 mm momentary\nswitch; the LEFT-hand\nbutton (measured)")

        # ------------------------------------------------------------------
        # I2C: two GY-521 modules on Wire (SDA0/SCL0 = pins 18/19), 400 kHz,
        # under the Teensy, both hanging off one two-wire bus.
        # ------------------------------------------------------------------
        ybus_sda, ybus_scl = bb.ymin - 0.9, bb.ymin - 1.3
        xj_sda, xj_scl = T.SDA0.x - 0.5, T.SCL0.x - 0.9

        def imu(x, ad0_to, title):
            nonlocal d
            names = ["VCC", "GND", "SCL", "SDA", "AD0"]
            ip = [elm.IcPin(name=n, side="top", slot="%d/5" % (k + 1), anchorname=n) for k, n in enumerate(names)]
            M = elm.Ic(pins=ip, pinspacing=0.7, edgepadH=0.45, edgepadW=0.25, leadlen=0.5, lsize=F_SMALL).theta(0).at((x, ybus_scl - 2.8)).anchor("center")
            d += M
            mb = M.get_bbox(transform=True)
            d += elm.Label().at((x, mb.ymin + 0.95)).label(title, fontsize=F_NOTE, ofst=0)
            d += elm.Label().at((x, mb.ymin + 0.3)).label("GY-521: MPU-6050 with 3.3 V LDO\nand 4.7 k SDA/SCL pull-ups", fontsize=F_SMALL, color=GREY, ofst=0)
            d += elm.Line().at(M.SDA).toy(ybus_sda)
            d += elm.Line().at(M.SCL).toy(ybus_scl)
            if x < 0:
                d += elm.Dot(radius=0.05).at((M.SDA.x, ybus_sda))
                d += elm.Dot(radius=0.05).at((M.SCL.x, ybus_scl))
            d += elm.Vdd().theta(0).at(M.VCC).label("3V3", fontsize=F_SMALL, color=RED)
            d += elm.Ground().theta(0).flip().at(M.GND)
            if ad0_to == "GND":
                d += elm.Ground().theta(0).flip().at(M.AD0)
            else:
                d += elm.Vdd().theta(0).at(M.AD0).label("3V3", fontsize=F_SMALL, color=RED)
            return M

        M1 = imu(-2.5, "GND", "K1 IMU on the LEFT wrist\nAD0 to GND: address 0x68")
        M2 = imu(2.5, "VCC", "K2 IMU on the RIGHT wrist\nAD0 to VCC: address 0x69")
        d += elm.Line().at((T.SDA0.x, T.SDA0.y)).tox(xj_sda)
        d += elm.Line().at((xj_sda, T.SDA0.y)).toy(ybus_sda)
        d += elm.Line().at((xj_sda, ybus_sda)).tox(M2.SDA.x)
        d += elm.Line().at((T.SCL0.x, T.SCL0.y)).tox(xj_scl)
        d += elm.Line().at((xj_scl, T.SCL0.y)).toy(ybus_scl)
        d += elm.Line().at((xj_scl, ybus_scl)).tox(M2.SCL.x)
        d += elm.Label().at((xj_scl - 0.2, ybus_sda)).label("SDA0 (pin 18)", fontsize=F_SMALL, halign="right", valign="center", ofst=0)
        d += elm.Label().at((xj_scl - 0.2, ybus_scl)).label("SCL0 (pin 19), I2C at 400 kHz", fontsize=F_SMALL, halign="right", valign="center", ofst=0)

        # ------------------------------------------------------------------
        # power and the USB link
        # ------------------------------------------------------------------
        d += elm.Vdd().theta(0).at(T.T3V3).label("3V3", fontsize=F_SMALL, color=RED)
        d += elm.Ground().theta(0).flip().at(T.TGND)
        d += elm.Label().at((T.T3V3.x, T.T3V3.y + 1.0)).label(
            "3V3: on-board regulator, supplies\nevery sensor and is the converter's reference", fontsize=F_SMALL, color=RED, valign="bottom", ofst=0)
        d += elm.Ground().theta(0).at(T.BGND)
        d += elm.Line().at(T.BUSB).down(0.35)
        d += elm.Label().at((T.BUSB.x + 0.15, T.BUSB.y - 0.05)).label(
            "USB micro-B to the host PC:\n5 V in, one serial frame out per cycle", fontsize=F_SMALL, halign="left", valign="top", ofst=0)

        d.save(OUT + ".pdf")
        d.save(OUT + ".png", dpi=200)
    print("wrote", OUT + ".pdf")


if __name__ == "__main__":
    main()
