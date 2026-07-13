import numpy as np
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk

# ================= CONFIG =================
ARM_PORT = 10000
ARM_USER = "admin"
ARM_PASS = "admin"

LEFT_IP = "192.168.1.10"
RIGHT_IP = "192.168.1.9"

LEFT_HOME = [259.03, 277.69, 267.74, 286.14, 194.10, 27.48, 55.26]
RIGHT_HOME = [303.65, 77.06, 98.57, 58.57, 317.14, 36.39, 154.71]

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

POT_NOISE_FLOOR = 3.0
DEADBAND = 1.0
KP = 0.5
HOME_KP = 0.5
HOME_SPEED = 5.0
TELE_SPEED = 5.0
COUNTDOWN = 10

ALL_TAGS = [f"k1j{i}" for i in range(1, 8)] + [f"k2j{i}" for i in range(1, 8)]

# =============== SERIAL PARSER ===============
def parse_pots(line):
    out = {}
    for token in line.strip().split(","):
        if ":" in token:
            k, v = token.split(":")
            try:
                out[k.strip()] = float(v)
            except:
                pass
    return out

# =============== GUI APP ===============
class App:
    def __init__(self, root):
        self.root = root
        root.title("WSL SRL TELEOP")
        root.geometry("700x600")

        self.mappings = {}
        self.pot_values = {t: 0.0 for t in ALL_TAGS}
        self.targets = {"LEFT": np.zeros(7), "RIGHT": np.zeros(7)}

        self.running = False
        self.estop = threading.Event()
        self.phase = "SETUP"

        self.logq = queue.Queue()

        self._build()
        self._tick()

    # ---------- UI ----------
    def _build(self):
        top = tk.Frame(self.root)
        top.pack(fill="x")

        self.label = tk.Label(top, text="WSL TEENSY TELEOP", font=("Arial", 14))
        self.label.pack()

        self.start_btn = tk.Button(top, text="START", command=self.start)
        self.start_btn.pack()

        self.log = tk.Text(self.root, height=25)
        self.log.pack(fill="both", expand=True)

    def log_msg(self, msg):
        self.logq.put(msg)

    # ---------- START ----------
    def start(self):
        if self.running:
            return
        self.running = True
        self.estop.clear()

        threading.Thread(target=self.serial_thread, daemon=True).start()
        self.log_msg("Started serial thread")

    # ---------- SERIAL THREAD ----------
    def serial_thread(self):
        try:
            import serial
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
            self.log_msg(f"Connected {SERIAL_PORT}")
        except Exception as e:
            self.log_msg(f"Serial error: {e}")
            return

        while not self.estop.is_set():
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                pots = parse_pots(line)

                for k, v in pots.items():
                    if k in self.pot_values:
                        self.pot_values[k] = v

                self.log_msg(str(pots))

            except Exception as e:
                self.log_msg(f"read error: {e}")
                time.sleep(0.1)

    # ---------- UI LOOP ----------
    def _tick(self):
        while not self.logq.empty():
            self.log.insert("end", self.logq.get() + "\n")
            self.log.see("end")

        self.root.after(100, self._tick)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
