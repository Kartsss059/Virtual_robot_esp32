"""
ESP32 Hardware-in-the-Loop (HIL) Robot Simulation
Pygame differential drive simulator communicating with ESP32 via Serial UART.

Usage:
    python sim/robot.py                    # Auto-detect COM port or fallback to mock
    python sim/robot.py --port COM3        # Connect to specific serial port
    python sim/robot.py --mock             # Run in mock/software-in-the-loop mode
    python sim/robot.py --mock --headless  # Run headless for automated testing
"""

import sys
import os
import math
import time
import argparse
import threading
from typing import Optional, Tuple, List

import pygame

# Optional pyserial import with helpful fallback
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# Simulation Constants
WINDOW_WIDTH = 900
WINDOW_HEIGHT = 700
FPS = 60
SIM_DT = 1.0 / FPS

WHEEL_BASE = 40.0       # Distance between wheels (pixels)
ROBOT_RADIUS = 25.0     # Visual radius of robot body
SENSOR_MAX_RANGE = 250.0  # Max distance sensor range


class MockESP32Controller:
    """Software-in-the-loop fallback controller simulating the ESP32 firmware."""
    def __init__(self):
        self.prev_error = 0.0
        self.wheel_base = WHEEL_BASE
        self.max_speed = 120.0
        self.kp = 80.0
        self.kd = 10.0
        self.waypoint_tolerance = 15.0

    def compute(self, x: float, y: float, theta: float, tgt_x: float, tgt_y: float, dist_front: float) -> Tuple[float, float]:
        dx = tgt_x - x
        dy = tgt_y - y
        dist = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)

        error = desired_heading - theta
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi

        d_error = error - self.prev_error
        self.prev_error = error

        if dist <= self.waypoint_tolerance:
            return 0.0, 0.0

        if 0.0 < dist_front < 60.0:
            # Obstacle avoidance reflex
            return -20.0, 60.0

        angular_speed = (self.kp * error) + (self.kd * d_error)
        alignment = math.cos(error)

        if alignment > 0.0:
            linear_speed = self.max_speed * alignment
            if dist < 80.0:
                linear_speed *= (dist / 80.0)
        else:
            linear_speed = 0.0

        left_cmd = linear_speed - (angular_speed * self.wheel_base / 2.0)
        right_cmd = linear_speed + (angular_speed * self.wheel_base / 2.0)

        left_cmd = max(-self.max_speed, min(self.max_speed, left_cmd))
        right_cmd = max(-self.max_speed, min(self.max_speed, right_cmd))
        return left_cmd, right_cmd


class SerialHILClient:
    """Manages Serial communication with the physical ESP32 or mock fallback."""
    def __init__(self, port: Optional[str] = None, baud: int = 115200, mock: bool = False):
        self.port = port
        self.baud = baud
        self.is_mock = mock
        self.ser = None
        self.running = False
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.last_response_time = time.time()
        self.packet_count = 0
        self.status_msg = "Initializing..."

        self.mock_controller = MockESP32Controller() if mock else None
        self.thread: Optional[threading.Thread] = None

        self._connect()

    def _auto_detect_port(self) -> Optional[str]:
        if not SERIAL_AVAILABLE:
            return None
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Common ESP32 USB-to-UART identifiers (CP210x, CH340, FTDI, ESP32)
            desc = (p.description + " " + (p.manufacturer or "")).lower()
            if any(k in desc for k in ["cp210", "ch340", "ch341", "ftdi", "uart", "esp32", "usb to"]):
                return p.device
        if ports:
            return ports[0].device
        return None

    def _connect(self):
        if self.is_mock or not SERIAL_AVAILABLE:
            self.is_mock = True
            self.status_msg = "MOCK MODE (Software-in-the-Loop)"
            return

        target_port = self.port or self._auto_detect_port()
        if not target_port:
            print("[INFO] No active serial port detected. Defaulting to Mock mode.")
            self.is_mock = True
            self.mock_controller = MockESP32Controller()
            self.status_msg = "MOCK MODE (No COM port found)"
            return

        try:
            if "://" in str(target_port):
                self.ser = serial.serial_for_url(target_port, baudrate=self.baud, timeout=0.05)
            else:
                self.ser = serial.Serial(target_port, self.baud, timeout=0.05)
            self.port = target_port
            self.is_mock = False
            self.running = True
            self.status_msg = f"CONNECTED ({self.port})"
            print(f"[INFO] Connected to ESP32 on {self.port}")
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
        except Exception as e:
            print(f"[WARN] Failed to open {target_port}: {e}. Falling back to Mock mode.")
            self.is_mock = True
            self.mock_controller = MockESP32Controller()
            self.status_msg = f"MOCK MODE (Failed {target_port})"

    def _read_loop(self):
        """Asynchronous reader for incoming serial command packets."""
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("CMD,"):
                    parts = line.split(",")
                    if len(parts) >= 3:
                        self.left_cmd = float(parts[1])
                        self.right_cmd = float(parts[2])
                        self.last_response_time = time.time()
                        self.packet_count += 1
                elif line.startswith("STATUS,"):
                    self.status_msg = f"ESP32: {line.split(',', 1)[1]}"
            except Exception:
                pass

    def send_telemetry(self, x: float, y: float, theta: float, tgt_x: float, tgt_y: float, dist_front: float):
        """Send telemetry packet to ESP32 or process in mock."""
        if self.is_mock:
            self.left_cmd, self.right_cmd = self.mock_controller.compute(
                x, y, theta, tgt_x, tgt_y, dist_front
            )
            self.last_response_time = time.time()
            self.packet_count += 1
            return

        if self.ser and self.ser.is_open:
            # Packet: TELEM,x,y,theta,tgt_x,tgt_y,dist_front\n
            msg = f"TELEM,{x:.1f},{y:.1f},{theta:.3f},{tgt_x:.1f},{tgt_y:.1f},{dist_front:.1f}\n"
            try:
                self.ser.write(msg.encode("utf-8"))
            except Exception as e:
                self.status_msg = f"TX Error: {e}"

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"CMD,0.0,0.0\n")
                self.ser.close()
            except Exception:
                pass


class RobotSimulation:
    def __init__(self, client: SerialHILClient, headless: bool = False):
        self.client = client
        self.headless = headless

        # Robot State
        self.x = 200.0
        self.y = 350.0
        self.theta = 0.0
        self.v_l = 0.0
        self.v_r = 0.0

        # Waypoint & Obstacles
        self.target_x = 700.0
        self.target_y = 350.0
        self.obstacles: List[Tuple[float, float, float]] = [
            (450.0, 350.0, 35.0), # (x, y, radius)
        ]

        self.trail: List[Tuple[float, float]] = []
        self.max_trail = 300
        self.front_dist = SENSOR_MAX_RANGE

        # Pygame setup
        if not self.headless:
            pygame.init()
            pygame.display.set_caption("ESP32 Hardware-in-the-Loop Simulation")
            self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
            self.clock = pygame.time.Clock()
            self.font_small = pygame.font.SysFont("Consolas", 14)
            self.font_bold = pygame.font.SysFont("Consolas", 16, bold=True)
            self.font_title = pygame.font.SysFont("Consolas", 20, bold=True)

    def cast_distance_sensor(self) -> float:
        """Raycast forward along robot heading to measure distance to boundaries or obstacles."""
        sensor_dist = SENSOR_MAX_RANGE
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)

        # 1. Screen boundaries
        if cos_t > 0:
            sensor_dist = min(sensor_dist, (WINDOW_WIDTH - self.x) / cos_t)
        elif cos_t < 0:
            sensor_dist = min(sensor_dist, -self.x / cos_t)

        if sin_t > 0:
            sensor_dist = min(sensor_dist, (WINDOW_HEIGHT - self.y) / sin_t)
        elif sin_t < 0:
            sensor_dist = min(sensor_dist, -self.y / sin_t)

        # 2. Obstacles (circle intersection)
        for ox, oy, r in self.obstacles:
            dx = ox - self.x
            dy = oy - self.y
            proj = dx * cos_t + dy * sin_t
            if proj > 0:
                perp_sq = (dx * dx + dy * dy) - (proj * proj)
                if perp_sq < r * r:
                    chord_dist = math.sqrt(max(0.0, r * r - perp_sq))
                    hit_dist = proj - chord_dist
                    if 0 < hit_dist < sensor_dist:
                        sensor_dist = hit_dist

        return max(0.0, sensor_dist)

    def step_physics(self, dt: float):
        # Update motor commands from HIL client
        self.v_l = self.client.left_cmd
        self.v_r = self.client.right_cmd

        # Differential drive forward kinematics
        v = (self.v_l + self.v_r) / 2.0
        w = (self.v_r - self.v_l) / WHEEL_BASE

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Wrap angle
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi

        # Keep within boundaries
        self.x = max(ROBOT_RADIUS, min(WINDOW_WIDTH - ROBOT_RADIUS, self.x))
        self.y = max(ROBOT_RADIUS, min(WINDOW_HEIGHT - ROBOT_RADIUS, self.y))

        # Update trail
        if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 4.0:
            self.trail.append((self.x, self.y))
            if len(self.trail) > self.max_trail:
                self.trail.pop(0)

        # Distance sensor reading
        self.front_dist = self.cast_distance_sensor()

        # Send telemetry back to ESP32
        self.client.send_telemetry(self.x, self.y, self.theta, self.target_x, self.target_y, self.front_dist)

    def draw(self):
        self.screen.fill((20, 24, 30))  # Dark slate background

        # 1. Grid lines
        for gx in range(0, WINDOW_WIDTH, 50):
            pygame.draw.line(self.screen, (30, 36, 46), (gx, 0), (gx, WINDOW_HEIGHT), 1)
        for gy in range(0, WINDOW_HEIGHT, 50):
            pygame.draw.line(self.screen, (30, 36, 46), (0, gy), (WINDOW_WIDTH, gy), 1)

        # 2. Trail
        if len(self.trail) > 1:
            pygame.draw.lines(self.screen, (60, 130, 180), False, self.trail, 2)

        # 3. Obstacles
        for ox, oy, r in self.obstacles:
            pygame.draw.circle(self.screen, (180, 70, 70), (int(ox), int(oy)), int(r))
            pygame.draw.circle(self.screen, (230, 100, 100), (int(ox), int(oy)), int(r), 2)

        # 4. Target Waypoint
        pulse = 4 * math.sin(time.time() * 6.0)
        pygame.draw.circle(self.screen, (255, 180, 50), (int(self.target_x), int(self.target_y)), int(12 + pulse), 2)
        pygame.draw.circle(self.screen, (255, 200, 80), (int(self.target_x), int(self.target_y)), 4)
        pygame.draw.line(self.screen, (255, 180, 50, 100), 
                         (int(self.target_x) - 16, int(self.target_y)), (int(self.target_x) + 16, int(self.target_y)), 1)
        pygame.draw.line(self.screen, (255, 180, 50, 100), 
                         (int(self.target_x), int(self.target_y) - 16), (int(self.target_x), int(self.target_y) + 16), 1)

        # 5. Distance sensor ray
        sensor_end_x = self.x + self.front_dist * math.cos(self.theta)
        sensor_end_y = self.y + self.front_dist * math.sin(self.theta)
        pygame.draw.line(self.screen, (240, 70, 70), (int(self.x), int(self.y)), (int(sensor_end_x), int(sensor_end_y)), 1)
        pygame.draw.circle(self.screen, (255, 50, 50), (int(sensor_end_x), int(sensor_end_y)), 4)

        # 6. Robot Body
        rx, ry = int(self.x), int(self.y)
        pygame.draw.circle(self.screen, (40, 160, 220), (rx, ry), int(ROBOT_RADIUS))
        pygame.draw.circle(self.screen, (200, 240, 255), (rx, ry), int(ROBOT_RADIUS), 2)

        # Wheels
        perp = self.theta + math.pi / 2.0
        w_offset = WHEEL_BASE / 2.0
        for side in [-1, 1]:
            wx = self.x + side * w_offset * math.cos(perp)
            wy = self.y + side * w_offset * math.sin(perp)
            # Wheel rectangle aligned with heading
            w_len, w_th = 16, 6
            p1 = (wx - (w_len/2)*math.cos(self.theta) - (w_th/2)*math.sin(self.theta),
                  wy - (w_len/2)*math.sin(self.theta) + (w_th/2)*math.cos(self.theta))
            p2 = (wx + (w_len/2)*math.cos(self.theta) - (w_th/2)*math.sin(self.theta),
                  wy + (w_len/2)*math.sin(self.theta) + (w_th/2)*math.cos(self.theta))
            p3 = (wx + (w_len/2)*math.cos(self.theta) + (w_th/2)*math.sin(self.theta),
                  wy + (w_len/2)*math.sin(self.theta) - (w_th/2)*math.cos(self.theta))
            p4 = (wx - (w_len/2)*math.cos(self.theta) + (w_th/2)*math.sin(self.theta),
                  wy - (w_len/2)*math.sin(self.theta) - (w_th/2)*math.cos(self.theta))
            pygame.draw.polygon(self.screen, (20, 20, 20), [p1, p2, p3, p4])
            pygame.draw.polygon(self.screen, (100, 100, 100), [p1, p2, p3, p4], 1)

        # Heading indicator arrow
        head_x = self.x + (ROBOT_RADIUS + 8) * math.cos(self.theta)
        head_y = self.y + (ROBOT_RADIUS + 8) * math.sin(self.theta)
        pygame.draw.line(self.screen, (255, 255, 100), (rx, ry), (int(head_x), int(head_y)), 3)

        # 7. Telemetry & Status HUD Overlay
        self._render_hud()

        pygame.display.flip()

    def _render_hud(self):
        panel_rect = pygame.Rect(15, 15, 340, 195)
        panel_surface = pygame.Surface((panel_rect.width, panel_rect.height), pygame.SRCALPHA)
        panel_surface.fill((10, 15, 22, 220))
        self.screen.blit(panel_surface, panel_rect.topleft)
        pygame.draw.rect(self.screen, (60, 80, 100), panel_rect, 1, border_radius=6)

        # Status badge color
        badge_color = (80, 220, 100) if not self.client.is_mock else (240, 160, 40)

        lines = [
            ("ESP32 HIL SIMULATOR", (255, 255, 255), self.font_title),
            (f"Status: {self.client.status_msg}", badge_color, self.font_bold),
            (f"FPS: {self.clock.get_fps():.1f} | Telemetry Pkts: {self.client.packet_count}", (180, 190, 200), self.font_small),
            (f"Robot Pose : X={self.x:.1f}, Y={self.y:.1f}, Th={math.degrees(self.theta):.1f}°", (200, 220, 240), self.font_small),
            (f"Target Waypoint: X={self.target_x:.1f}, Y={self.target_y:.1f}", (240, 220, 120), self.font_small),
            (f"Front Sensor   : {self.front_dist:.1f} px", (250, 140, 140), self.font_small),
            (f"Motor Cmds (L/R): {self.v_l:.1f} / {self.v_r:.1f}", (140, 240, 160), self.font_bold),
            ("Controls: Click = Set Target | R = Reset | O = Add Obstacle", (140, 150, 160), self.font_small),
        ]

        y_offset = 24
        for text, color, font in lines:
            rendered = font.render(text, True, color)
            self.screen.blit(rendered, (25, y_offset))
            y_offset += rendered.get_height() + 4

    def run(self, max_seconds: Optional[float] = None):
        start_time = time.time()
        running = True

        try:
            while running:
                if max_seconds and (time.time() - start_time) > max_seconds:
                    break

                if not self.headless:
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            running = False
                        elif event.type == pygame.MOUSEBUTTONDOWN:
                            mx, my = pygame.mouse.get_pos()
                            if event.button == 1:  # Left click: Set target
                                self.target_x = float(mx)
                                self.target_y = float(my)
                            elif event.button == 3:  # Right click: Add obstacle
                                self.obstacles.append((float(mx), float(my), 25.0))
                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_r:  # Reset
                                self.x = 200.0
                                self.y = 350.0
                                self.theta = 0.0
                                self.trail.clear()
                            elif event.key == pygame.K_o:  # Add obstacle at mouse
                                mx, my = pygame.mouse.get_pos()
                                self.obstacles.append((float(mx), float(my), 25.0))
                            elif event.key == pygame.K_c:  # Clear obstacles
                                self.obstacles.clear()

                self.step_physics(SIM_DT)

                if not self.headless:
                    self.draw()
                    self.clock.tick(FPS)
                else:
                    time.sleep(SIM_DT)
        finally:
            self.client.close()
            if not self.headless:
                pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="ESP32 HIL Robot Simulation")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--wokwi", action="store_true", help="Connect to Wokwi simulation RFC2217 server (localhost:4000)")
    parser.add_argument("--mock", action="store_true", help="Force software mock mode (no ESP32 hardware needed)")
    parser.add_argument("--headless", action="store_true", help="Run without graphical window (for testing)")
    parser.add_argument("--duration", type=float, default=None, help="Run simulation for fixed duration in seconds")
    args = parser.parse_args()

    port = "rfc2217://localhost:4000" if args.wokwi else args.port
    client = SerialHILClient(port=port, baud=args.baud, mock=args.mock)
    sim = RobotSimulation(client, headless=args.headless)
    sim.run(max_seconds=args.duration)


if __name__ == "__main__":
    main()

