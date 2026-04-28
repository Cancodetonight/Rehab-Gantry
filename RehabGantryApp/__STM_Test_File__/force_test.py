#!/usr/bin/env python3
"""
force_test.py — Simple Force Sensor Monitor (4 Cell)
Press 'p' + Enter to pause/resume.  Press Enter while paused for a snapshot.
"""

import sys
import time
import re
import threading
import datetime

try:
    import serial
except ImportError:
    print("ERROR: pip install pyserial")
    sys.exit(1)

BAUD_RATE = 115200

# Parse: R:r1,r2,r3,r4 | Z:z1,z2,z3,z4 | N:n1,n2,n3,n4 | F:fx,fy,fz
FORCE_RE = re.compile(
    r"R:\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s*\|\s*"
    r"Z:\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s*\|\s*"
    r"N:\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)\s*\|\s*"
    r"F:\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)"
)

latest = {}
lock = threading.Lock()
log_file = None
log_enabled = False
paused = False


def reader_thread(ser, stop_event):
    global log_file
    buf = ""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting).decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    m = FORCE_RE.search(line)
                    if m:
                        with lock:
                            latest["raw"] = [int(m.group(i)) for i in range(1, 5)]
                            latest["zeroed"] = [int(m.group(i)) for i in range(5, 9)]
                            latest["newton"] = [float(m.group(i)) for i in range(9, 13)]
                            latest["force"] = [float(m.group(i)) for i in range(13, 16)]
                            latest["count"] = latest.get("count", 0) + 1

                        if not paused:
                            r = latest["raw"]
                            f = latest["force"]
                            print(
                                f"  C1:{r[0]:>8}  C2:{r[1]:>8}  C3:{r[2]:>8}  C4:{r[3]:>8}"
                                f"  |  Fx:{f[0]:>+9.2f}  Fy:{f[1]:>+9.2f}  Fz:{f[2]:>+9.2f}"
                            )

                        if log_enabled and log_file:
                            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            vals = ",".join(m.group(i) for i in range(1, 16))
                            log_file.write(f"{ts},{vals}\n")
                            log_file.flush()
                    else:
                        print(f"  STM32> {line}")
            else:
                time.sleep(0.02)
        except (serial.SerialException, OSError):
            break


def snapshot():
    with lock:
        d = dict(latest)
    if not d:
        print("  (no data yet)")
        return
    r = d["raw"]
    z = d["zeroed"]
    n = d["newton"]
    f = d["force"]
    print()
    print("  ── SNAPSHOT ──")
    print(f"  Cell 1 (bottom):  raw={r[0]:>10}  zeroed={z[0]:>10}  N={n[0]:>+10.3f}")
    print(f"  Cell 2 (right) :  raw={r[1]:>10}  zeroed={z[1]:>10}  N={n[1]:>+10.3f}")
    print(f"  Cell 3 (top)   :  raw={r[2]:>10}  zeroed={z[2]:>10}  N={n[2]:>+10.3f}")
    print(f"  Cell 4 (left)  :  raw={r[3]:>10}  zeroed={z[3]:>10}  N={n[3]:>+10.3f}")
    print(f"  Forces:  Fx={f[0]:>+.4f}  Fy={f[1]:>+.4f}  Fz={f[2]:>+.4f}")
    print()


def main():
    global log_file, log_enabled, paused

    port = sys.argv[1] if len(sys.argv) >= 2 else "COM5"

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print(f"Connected to {port} @ {BAUD_RATE}")
    print("Waiting for tare...\n")

    # Wait for tare
    start = time.time()
    while time.time() - start < 15:
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"  STM32> {line}")
                if "Complete" in line or FORCE_RE.search(line):
                    break
        time.sleep(0.05)

    print()
    print("  Commands: p=pause  d=snapshot  t=tare  s=scales  m=matrix")
    print("            s1..s4 <val>=set scale  l=log  q=quit")
    print()
    print("  C1(bot)   C2(right)   C3(top)   C4(left)  |  Fx         Fy         Fz")
    print("  " + "─" * 75)

    stop = threading.Event()
    threading.Thread(target=reader_thread, args=(ser, stop), daemon=True).start()

    try:
        while True:
            try:
                cmd = input("").strip().lower()
            except EOFError:
                break

            if not cmd:
                if paused:
                    snapshot()
                continue
            if cmd == "q":
                break
            elif cmd == "p":
                paused = not paused
                print(f"  {'PAUSED (Enter=snapshot, p=resume)' if paused else 'LIVE'}")
            elif cmd == "d":
                snapshot()
            elif cmd == "t":
                ser.write(b"<TARE>")
                print("  >> tare sent")
            elif cmd == "s" and len(cmd) == 1:
                ser.write(b"<SCALES>")
            elif cmd == "m":
                ser.write(b"<MATRIX>")
            elif cmd.startswith("s") and " " in cmd:
                parts = cmd.split()
                if len(parts) == 2 and parts[0] in ("s1", "s2", "s3", "s4"):
                    try:
                        val = float(parts[1])
                        msg = f"<S{parts[0][1]}:{val:.2f}>"
                        ser.write(msg.encode())
                        print(f"  >> sent {msg}")
                    except ValueError:
                        print("  >> bad number")
            elif cmd == "l":
                log_enabled = not log_enabled
                if log_enabled:
                    fname = f"force_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                    log_file = open(fname, "w")
                    log_file.write("time,raw1,raw2,raw3,raw4,z1,z2,z3,z4,n1,n2,n3,n4,fx,fy,fz\n")
                    print(f"  >> log ON: {fname}")
                else:
                    if log_file:
                        log_file.close()
                        log_file = None
                    print("  >> log OFF")
            else:
                print("  >> ?  (p/d/t/s/m/s1..s4/l/q)")

    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if log_file:
            log_file.close()
        ser.close()
        print("\n  Closed.")


if __name__ == "__main__":
    main()
