import serial
import time

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
time.sleep(2)

print("Reading Teensy data...\n")

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print(line)
