import serial
import threading
import time
import re

class SerialManager:
    def __init__(self, port, baud_rate, is_test_mode=False):
        self.port = port
        self.baud_rate = baud_rate
        self.is_test_mode = is_test_mode
        self.serial_conn = None
        self.is_running = False
        self.read_thread = None
        self.active_plane = 'XY'
        
        # Telemetry State (Thread-safe implicitly due to GIL for simple assignments/reads)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.frc_x = 0.0
        self.frc_y = 0.0
        self.frc_z = 0.0
        self.is_homed = False  # Set True when STM32 sends <HOMED>
        self.mock_mode = False
        self.last_send_time = 0.0  # Rate limiting
        
        # Regex for parsing: <POS:x,y,z|FRC:fx,fy,fz>
        self.regex = re.compile(r"<POS:([-\d.]+),([-\d.]+),([-\d.]+)\|FRC:([-\d.]+),([-\d.]+),([-\d.]+)>")

    def start(self):
        self.is_running = True
        try:
            # We attempt to connect, but if COM port is placeholder or invalid, it falls back to Mock
            if self.port != "COMX":
                self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
                print(f"Connected to {self.port} at {self.baud_rate} baud.")
            else:
                raise serial.SerialException("Placeholder Port")
        except serial.SerialException as e:
            print(f"Warning: Could not connect to serial port {self.port}. Starting in pure MOCK mode.")
            self.serial_conn = None
            self.mock_mode = True
            self.is_homed = True  # No real hardware to home
            
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def stop(self):
        self.is_running = False
        self.send_stop()  # Graceful exit
        if self.read_thread:
            self.read_thread.join(timeout=2)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    def _read_loop(self):
        buffer = ""
        while self.is_running:
            if not self.mock_mode and self.serial_conn and self.serial_conn.is_open:
                try:
                    if self.is_test_mode:
                        # HIL Mode: ignore raw serial input and use mock_simulation for updates
                        if self.serial_conn.in_waiting > 0:
                            self.serial_conn.read(self.serial_conn.in_waiting)
                        self._mock_simulation()
                        time.sleep(0.01)
                    else:
                        if self.serial_conn.in_waiting > 0:
                            chunk = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                            buffer += chunk
                            
                            # Process full lines
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                line = line.strip()
                                self._parse_line(line)
                except Exception as e:
                    print(f"Serial read error: {e}")
                    time.sleep(0.1)
            else:
                # Pure Mock mode simulation (COM port offline)
                self._mock_simulation()
                time.sleep(0.01)

    def _parse_line(self, line):
        match = self.regex.search(line)
        if match:
            try:
                self.pos_x = float(match.group(1))
                self.pos_y = float(match.group(2))
                self.pos_z = float(match.group(3))
                self.frc_x = float(match.group(4))
                self.frc_y = float(match.group(5))
                self.frc_z = float(match.group(6))
                print(f"[RX] POS:({self.pos_x:.2f},{self.pos_y:.2f},{self.pos_z:.2f}) FRC:({self.frc_x:.2f},{self.frc_y:.2f},{self.frc_z:.2f})")
            except ValueError:
                pass
        elif '<HOMED>' in line:
            self.is_homed = True
            print("[RX] Homing complete — STM32 ready.")
        else:
            if line:
                print(f"[RX RAW] {line}")

    def _mock_simulation(self):
        # Uses actual mouse cursor for mock simulation natively across different planes
        try:
            import pygame
            if pygame.display.get_init():
                mx, my = pygame.mouse.get_pos()
                
                if self.active_plane == 'YZ':
                    from core.constants import map_screen_yz_to_mm
                    y_mm, z_mm = map_screen_yz_to_mm(mx, my)
                    self.pos_x = 0.0
                    self.pos_y = float(y_mm)
                    self.pos_z = float(z_mm)
                elif self.active_plane == 'XZ':
                    from core.constants import map_screen_xz_to_mm
                    x_mm, z_mm = map_screen_xz_to_mm(mx, my)
                    self.pos_x = float(x_mm)
                    self.pos_y = 0.0
                    self.pos_z = float(z_mm)
                elif self.active_plane == 'YZ_ISO':
                    from core.constants import map_iso_screen_to_yz
                    y_mm, z_mm = map_iso_screen_to_yz(mx, my)
                    self.pos_x = 0.0
                    self.pos_y = float(y_mm)
                    self.pos_z = float(z_mm)
                else:
                    from core.constants import map_screen_xy_to_mm
                    mm_x, mm_y = map_screen_xy_to_mm(mx, my)
                    self.pos_x = float(mm_x)
                    self.pos_y = float(mm_y)
                    self.pos_z = 0.0
        except ImportError:
            pass

        import math
        t = time.time()
        # Random mock forces
        self.frc_x = math.sin(t) * 10
        self.frc_y = math.cos(t) * 5
        self.frc_z = 0.0

    def send_command(self, tgt_x, tgt_y, tgt_z, k, z_en=0):
        """Send target position and stiffness to STM32."""
        current_time = time.time()
        if current_time - self.last_send_time < 0.05:
            return  # Rate limited to 20Hz
        self.last_send_time = current_time

        # Format: <TGT:x,y,z|K:value|Z:enable>\r\n
        command_str = f"<TGT:{tgt_x:.2f},{tgt_y:.2f},{tgt_z:.2f}|K:{k:.2f}|Z:{z_en}>\r\n"
        
        print(f"[TX] -> {command_str.strip()}")
        
        if not self.mock_mode and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(command_str.encode('utf-8'))
            except Exception as e:
                print(f"Serial write error: {e}")
        else:
            pass

    def send_stop(self):
        """Send target command with 0.0 stiffness to release motors."""
        command_str = "<TGT:0.00,0.00,0.00|K:0.00|Z:0>\r\n"
        print(f"[TX STOP] -> {command_str.strip()}")
        if not self.mock_mode and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(command_str.encode('utf-8'))
            except Exception as e:
                print(f"Serial write error: {e}")

    def send_tare(self):
        """Send TARE command to re-zero force sensors."""
        command_str = "<TARE>\r\n"
        print(f"[TX TARE] -> {command_str.strip()}")
        if not self.mock_mode and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(command_str.encode('utf-8'))
            except Exception as e:
                print(f"Serial write error: {e}")

    def send_mtest(self, enable=True):
        """Send motor test mode command. When enabled, STM32 ignores forces."""
        cmd = "MTEST" if enable else "MTEST_OFF"
        command_str = f"<{cmd}>\r\n"
        print(f"[TX] -> {command_str.strip()}")
        if not self.mock_mode and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(command_str.encode('utf-8'))
            except Exception as e:
                print(f"Serial write error: {e}")

    def get_telemetry(self):
        """Returns the current state of telemetry safely."""
        return {
            'pos': (self.pos_x, self.pos_y, self.pos_z),
            'frc': (self.frc_x, self.frc_y, self.frc_z)
        }
