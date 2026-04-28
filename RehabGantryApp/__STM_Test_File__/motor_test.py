#!/usr/bin/env python3
"""
motor_test.py — Interactive Motor Control Console for STM32 Gantry
===================================================================
Sends UART commands to the motors.c firmware running on the STM32.

Usage:
    python motor_test.py COM3          (Windows — adjust port as needed)
    python motor_test.py /dev/ttyUSB0  (Linux)

If no port is given, available ports are listed.
"""

import sys
import time
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required.  Install with:  pip install pyserial")
    sys.exit(1)

BAUD_RATE = 115200


# ─── Background receiver ─────────────────────────────────────────────────────
def reader_thread(ser, stop_event):
    """Continuously reads and prints everything the STM32 sends."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(f"\r  STM32> {line}")
                    print("> ", end="", flush=True)
            else:
                time.sleep(0.05)
        except (serial.SerialException, OSError):
            break


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    # --- Port selection (default: COM5) ---
    port = sys.argv[1] if len(sys.argv) >= 2 else "COM5"

    # --- Connect ---
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        sys.exit(1)

    print(f"Connected to {port} at {BAUD_RATE} baud\n")

    print("╔══════════════════════════════════════════════╗")
    print("║        MOTOR TEST CONSOLE  v1.0              ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  x <speed>   X motor speed  (mm/s)           ║")
    print("║  y <speed>   Y motor speed  (mm/s)           ║")
    print("║  z <speed>   Z motor speed  (mm/s)           ║")
    print("║  b on/off    Z-axis brake  (on = clamped)    ║")
    print("║  s           STOP all motors immediately     ║")
    print("║  q           Quit  (auto-stops first)        ║")
    print("║  h           Show this help                  ║")
    print("║                                              ║")
    print("║  Speed range: -100 … +100 mm/s               ║")
    print("║  Negative = reverse direction                ║")
    print("║  Z requires brake OFF first (<b off>)        ║")
    print("╚══════════════════════════════════════════════╝")
    print()

    # --- Start receiver thread ---
    stop_event = threading.Event()
    rx_thread = threading.Thread(
        target=reader_thread, args=(ser, stop_event), daemon=True
    )
    rx_thread.start()

    # Give the STM32 banner time to arrive
    time.sleep(1.5)

    # --- Command loop ---
    try:
        while True:
            try:
                cmd = input("> ").strip()
            except EOFError:
                break

            if not cmd:
                continue

            lower = cmd.lower()

            # ── Quit ──
            if lower == "q":
                ser.write(b"<STOP>")
                print("  >> STOP ALL sent.  Goodbye!")
                time.sleep(0.3)
                break

            # ── Emergency Stop ──
            elif lower == "s":
                ser.write(b"<STOP>")
                print("  >> STOP ALL sent")

            # ── Brake ──
            elif lower.startswith("b "):
                parts = lower.split()
                if len(parts) == 2 and parts[1] in ("on", "1", "off", "0"):
                    val = "1" if parts[1] in ("on", "1") else "0"
                    ser.write(f"<B:{val}>".encode())
                    label = "CLAMP" if val == "1" else "UNCLAMP"
                    print(f"  >> Brake {label} sent")
                else:
                    print("  Usage:  b on   or   b off")

            # ── Motor speed ──
            elif lower[0] in ("x", "y", "z"):
                parts = cmd.split()
                if len(parts) == 2:
                    try:
                        speed = float(parts[1])
                        axis = parts[0].upper()
                        ser.write(f"<{axis}:{speed:.1f}>".encode())
                        print(f"  >> {axis} → {speed:+.1f} mm/s sent")
                    except ValueError:
                        print("  Speed must be a number.  e.g.  x 50  or  z -20")
                else:
                    print(f"  Usage:  {lower[0]} <speed>   e.g.  {lower[0]} 50")

            # ── Help ──
            elif lower in ("h", "help", "?"):
                print("  x/y/z <speed>  Motor speed in mm/s  (negative = reverse)")
                print("  b on/off       Z-axis brake control")
                print("  s              Emergency stop all")
                print("  q              Quit")

            else:
                print("  Unknown command.  Type 'h' for help.")

    except KeyboardInterrupt:
        print("\n  Ctrl-C — stopping motors...")
        ser.write(b"<STOP>")
        time.sleep(0.3)

    finally:
        stop_event.set()
        ser.close()
        print("  Serial port closed.")


if __name__ == "__main__":
    main()
