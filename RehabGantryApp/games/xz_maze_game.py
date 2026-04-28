import pygame
import math
from collections import deque
from games.game_base import GameBase
from core.constants import *

class XZMazeGame(GameBase):
    def __init__(self, serial_manager, screen):
        super().__init__(serial_manager, screen)
        # Lock simulation to Frontal XZ Plane natively
        self.serial_manager.active_plane = 'XZ'
        
        # Maze Array Grid (Strict W, ' ', S, E)
        self.maze_grid = [
            "WWWWWWWWWWWWWWWWWWWW",
            "W S        W       W",
            "W WWWWWW W W WWW W W",
            "W      W W W W   W W",
            "WWWWWW W W WWW WWW W",
            "W      W W         W",
            "W WWWWWW WWWWWWWWW W",
            "W        W       W W",
            "WWWWWW WWWWW WWW W W",
            "W                W E",
            "WWWWWWWWWWWWWWWWWWWW"
        ]
        
        # State Arrays
        self.walls = []
        self.targets = []
        self.start_physical = (0.0, 0.0)
        self.end_physical = (0.0, 0.0)
        self._parse_maze()
        
        self.game_active = False # Safe Waiting State!
        self.game_won = False # Single Run execution lock
        self.current_target_index = 0
        self.trail_history = []  # Max 20 coords tracking active tremor sequence
        self.flash_red_timer = 0.0
        self.stuck_timer = 0.0
        self.current_k = 0.0
        self.last_sent_k = 0.0
        self.target_stiffness = 50.0
        
        telemetry = self.serial_manager.get_telemetry()
        pos_x = telemetry['pos'][0]
        pos_z = MAX_Z_MM - telemetry['pos'][2]
        self.last_distance = math.hypot(pos_x - self.start_physical[0], pos_z - self.start_physical[1])
        
        # Send initial command immediately to reach start zone
        self.serial_manager.send_command(self.start_physical[0], (MAX_Y_MM + MIN_Y_MM) / 2.0, MAX_Z_MM - self.start_physical[1], self.current_k, z_en=1)
        self.last_sent_k = self.current_k
        print(f"New target spawned at X: {self.start_physical[0]:.1f}, Y: 0.0, Z: {self.start_physical[1]:.1f} (mm)")

    def get_start_position(self):
        """Move gantry to maze start cell before gameplay begins."""
        return (self.start_physical[0], (MAX_Y_MM + MIN_Y_MM) / 2.0, MAX_Z_MM - self.start_physical[1], 1)

    def _bfs_path(self, start, end):
        """Standard queue-based graph traversal mathematically checking adjacent string arrays."""
        queue = deque([(start, [start])])
        visited = {start}
        
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        while queue:
            (r, c), path = queue.popleft()
            if (r, c) == end:
                return path
            
            for dr, dc in dirs:
                nr, nc = r + dr, c + dc
                if 0 <= nr < len(self.maze_grid) and 0 <= nc < len(self.maze_grid[0]):
                    if self.maze_grid[nr][nc] != 'W' and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        queue.append(((nr, nc), path + [(nr, nc)]))
        return []

    def _get_corners(self, path):
        """Filters paths by evaluating coordinate directionality explicitly catching physical intersections natively."""
        if len(path) <= 2:
            return path[1:]
        
        corners = []
        last_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        
        for i in range(1, len(path) - 1):
            curr_dir = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            if curr_dir != last_dir:
                corners.append(path[i])
                last_dir = curr_dir
        
        corners.append(path[-1]) # Safely retain 'E' implicitly.
        return corners

    def _parse_maze(self):
        """Constructs UI boundaries and binds graph algorithms mathematically evaluating pathing explicitly."""
        rows = len(self.maze_grid)
        cols = len(self.maze_grid[0])
        cell_width_mm = (MAX_X_MM - MIN_X_MM) / cols
        cell_height_mm = (MAX_Z_MM - MIN_Z_MM) / rows
        
        cell_px_w = DRAWABLE_WIDTH / cols
        cell_px_h = DRAWABLE_HEIGHT / rows
        
        start_grid = None
        end_grid = None
        
        for r in range(rows):
            for c in range(cols):
                char = self.maze_grid[r][c]
                px_x = MARGIN_PX + c * cell_px_w
                px_y = MARGIN_PX + r * cell_px_h
                
                if char == 'W':
                    wall_margin = 15
                    rect_x = px_x
                    rect_y = px_y
                    rect_w = cell_px_w + 1 # +1 pixel prevents theoretical draw seam gaps
                    rect_h = cell_px_h + 1
                    
                    # Context-Aware Smart Margins
                    # North Check (Up)
                    if r == 0 or self.maze_grid[r-1][c] != 'W':
                        rect_y += wall_margin
                        rect_h -= wall_margin
                    # South Check (Down)
                    if r == rows - 1 or self.maze_grid[r+1][c] != 'W':
                        rect_h -= wall_margin
                    # West Check (Left)
                    if c == 0 or self.maze_grid[r][c-1] != 'W':
                        rect_x += wall_margin
                        rect_w -= wall_margin
                    # East Check (Right)
                    if c == cols - 1 or self.maze_grid[r][c+1] != 'W':
                        rect_w -= wall_margin
                        
                    self.walls.append(pygame.Rect(rect_x, rect_y, rect_w, rect_h))
                elif char == 'S':
                    start_grid = (r, c)
                    self.start_physical = (MIN_X_MM + (c + 0.5) * cell_width_mm, MAX_Z_MM - (r + 0.5) * cell_height_mm)
                elif char == 'E':
                    end_grid = (r, c)
                    self.end_physical = (MIN_X_MM + (c + 0.5) * cell_width_mm, MAX_Z_MM - (r + 0.5) * cell_height_mm)
                    
        # Trace pathing natively
        if start_grid and end_grid:
            path = self._bfs_path(start_grid, end_grid)
            corners = self._get_corners(path)
            
            for (r, c) in corners:
                x_mm = MIN_X_MM + (c + 0.5) * cell_width_mm
                z_mm = MAX_Z_MM - (r + 0.5) * cell_height_mm
                self.targets.append((x_mm, z_mm))

    def draw_telemetry(self):
        """Custom telemetry respecting the Frontal Plane format"""
        telemetry = self.serial_manager.get_telemetry()
        pos_mm = telemetry['pos']
        frc_n = telemetry['frc']

        force_text = self.font.render(f"FRC (N): X:{frc_n[0]:.2f} Y:{frc_n[1]:.2f} Z:{frc_n[2]:.2f}", True, TEXT_COLOR)
        pos_text = self.font.render(f"POS (mm): X:{pos_mm[0]:.2f} Y:{pos_mm[1]:.2f} Z:{pos_mm[2]:.2f}", True, TEXT_COLOR)
        
        self.screen.blit(force_text, (10, 10))
        self.screen.blit(pos_text, (10, 35))

    def update(self):
        telemetry = self.serial_manager.get_telemetry()
        pos_x = telemetry['pos'][0]
        pos_z = MAX_Z_MM - telemetry['pos'][2]
        
        current_time = pygame.time.get_ticks()
        dt = 1.0 / 30.0 # Approximate frame delta assuming locking to 30fps natively
        
        if self.flash_red_timer > 0:
            self.flash_red_timer -= dt

        # Coordinate mappings
        cursor_px_x, cursor_px_y = map_mm_to_screen_xz(pos_x, pos_z)
        
        # Buffer Tracking (Only track trail while actively playing!)
        if self.game_active and not self.game_won:
            self.trail_history.append((cursor_px_x, cursor_px_y))
            if len(self.trail_history) > 20:
                self.trail_history.pop(0)
            
        # Collision Constraints
        if self.game_active and not self.game_won:
            cursor_radius = 12
            cursor_rect = pygame.Rect(cursor_px_x - cursor_radius, cursor_px_y - cursor_radius, cursor_radius * 2, cursor_radius * 2)
            if cursor_rect.collidelist(self.walls) != -1:
                # Flash penalty natively, but DO NOT RESET logic buffers!
                # STM32 Admittance bounds track back dynamically towards checkpoint naturally!
                self.flash_red_timer = 0.5 # 15 frames penalty!

        dist_start = math.hypot(pos_x - self.start_physical[0], pos_z - self.start_physical[1])

        # State Navigation Logic
        if not self.game_active and not self.game_won:
            # Safe Wait State. Do NOT increment targets. MUST touch Start Zone!
            if dist_start < 30 and self.flash_red_timer <= 0:
                self.game_active = True
                self.trail_history.clear()
                self.stuck_timer = 0.0
                self.current_k = max(self.current_k, 1.0)
                # Deploy UART conditionally protecting bounds 
                if self.targets:
                    t_x, t_z = self.targets[self.current_target_index]
                    self.last_distance = math.hypot(pos_x - t_x, pos_z - t_z)
                    
                    self.serial_manager.send_command(t_x, (MAX_Y_MM + MIN_Y_MM) / 2.0, MAX_Z_MM - t_z, self.current_k, z_en=1)
                    self.last_sent_k = self.current_k
                    print(f"New target spawned at X: {t_x:.1f}, Y: 0.0, Z: {t_z:.1f} (mm)")
                    
        elif self.game_active and not self.game_won:
            # AAN Timer & Logic
            if self.targets and self.current_target_index < len(self.targets):
                t_x, t_z = self.targets[self.current_target_index]
                # Calculate local distance for both AAN and regular targeting
                dist_to_target = math.hypot(pos_x - t_x, pos_z - t_z)

                if not self.serial_manager.mock_mode:
                    # 1. Calculate progress since last frame
                    progress_mm = self.last_distance - dist_to_target
                    velocity_toward_target = progress_mm / dt if dt > 0 else 0.0
                    
                    # 2. Evaluate User Performance
                    if velocity_toward_target > 5.0:
                        # Moving TOWARD target well. Relax the robot.
                        self.stuck_timer = 0.0
                        self.current_k = max(1.0, self.current_k - (15.0 * dt))
                    elif velocity_toward_target < 0.0:
                        # Moving AWAY from target (Wrong direction!). Immediate penalty.
                        self.stuck_timer += dt * 2.0 # Ramp up stuck timer twice as fast
                    elif velocity_toward_target < 2.0:
                        # Stuck or moving too slow toward the target.
                        self.stuck_timer += dt
                        
                    # 3. Apply Assistance if stuck
                    if self.stuck_timer > 2.0:
                        ramp_rate = self.target_stiffness / 5.0
                        self.current_k = min(self.target_stiffness, self.current_k + (ramp_rate * dt))
                        
                    # 4. Transmit with Delta-K Thresholding
                    if abs(self.current_k - self.last_sent_k) >= 1.0:
                        self.serial_manager.send_command(t_x, (MAX_Y_MM + MIN_Y_MM) / 2.0, MAX_Z_MM - t_z, self.current_k, z_en=1)
                        self.last_sent_k = self.current_k
                        
                    # 5. Save distance for next frame
                    self.last_distance = dist_to_target

            # Target Navigation Logic
            if self.current_target_index < len(self.targets):
                t_x, t_z = self.targets[self.current_target_index]
                dist = math.hypot(pos_x - t_x, pos_z - t_z)
                
                # Check target intersection threshold locally
                if dist < 30:
                    self.current_target_index += 1
                    if self.current_target_index < len(self.targets):
                        nxt_x, nxt_z = self.targets[self.current_target_index]
                        self.stuck_timer = 0.0
                        self.current_k = max(self.current_k, 1.0)
                        self.last_distance = math.hypot(pos_x - nxt_x, pos_z - nxt_z)
                        
                        self.serial_manager.send_command(nxt_x, (MAX_Y_MM + MIN_Y_MM) / 2.0, MAX_Z_MM - nxt_z, self.current_k, z_en=1)
                        self.last_sent_k = self.current_k
                        print(f"New target spawned at X: {nxt_x:.1f}, Y: 0.0, Z: {nxt_z:.1f} (mm)")
                    else:
                        # Victory Trigger
                        self.game_won = True
                        self.game_active = False # Kill physics loop drag securely
                        self.serial_manager.send_stop()

    def draw(self):
        # Flash states cleanly mapped
        if self.flash_red_timer > 0:
            self.screen.fill((255, 220, 220))
        elif self.game_won:
            self.screen.fill((230, 255, 230))
        else:
            self.screen.fill((248, 249, 250)) # Premium White Canvas Space

        # 1. Environment Render
        for wall in self.walls:
            shadow_rect = wall.copy()
            shadow_rect.x += 4
            shadow_rect.y += 4
            pygame.draw.rect(self.screen, (220, 220, 225), shadow_rect, border_radius=8)
            pygame.draw.rect(self.screen, (240, 240, 245), wall, border_radius=8)
            pygame.draw.rect(self.screen, (210, 210, 215), wall, width=1, border_radius=8)
            
        # 2. Permanent Target Overlays
        sx, sy = map_mm_to_screen_xz(self.start_physical[0], self.start_physical[1])
        if not self.game_active and not self.game_won:
            pulse_s = (math.sin(pygame.time.get_ticks() / 150.0) + 1) / 2
            r_s = int(15 + pulse_s * 10)
            pygame.draw.circle(self.screen, (0, 200, 50), (sx, sy), r_s)
            pygame.draw.circle(self.screen, (50, 255, 100), (sx, sy), int(r_s * 0.7))
        else:
            pygame.draw.circle(self.screen, DARK_GRAY, (sx, sy), 15, width=2)
            
        # Draw End Target permanently regardless of current index progress
        end_px_x, end_px_y = map_mm_to_screen_xz(self.end_physical[0], self.end_physical[1])
        pulse_e = (math.sin(pygame.time.get_ticks() / 150.0) + 1) / 2
        radius_e = int(25 + pulse_e * 8)
        pts = [
            (end_px_x, end_px_y - radius_e), 
            (end_px_x + radius_e, end_px_y), 
            (end_px_x, end_px_y + radius_e), 
            (end_px_x - radius_e, end_px_y)
        ]
        pygame.draw.polygon(self.screen, (255, 50, 50), pts)
        pygame.draw.polygon(self.screen, (200, 0, 0), pts, 3)

        # 3. Active Checkpoint Radiance
        # Do not draw if won, or if the current target IS the end target (Index E implicitly handled visually!)
        if self.game_active and not self.game_won and self.current_target_index < len(self.targets) - 1:
            t_x, t_z = self.targets[self.current_target_index]
            px_x, px_y = map_mm_to_screen_xz(t_x, t_z)
            
            pulse = (math.sin(pygame.time.get_ticks() / 150.0) + 1) / 2
            radius_i = int(20 + pulse * 12)
            pygame.draw.circle(self.screen, (255, 215, 0), (px_x, px_y), radius_i) # Glowing Gold
            pygame.draw.circle(self.screen, (0, 122, 255), (px_x, px_y), int(15 - pulse*3)) # Medical Blue Inner

        # 4. Tremor History Trail
        if self.game_active and len(self.trail_history) > 1:
            for i in range(len(self.trail_history) - 1):
                width = int(8 * (i / len(self.trail_history))) + 1
                pygame.draw.line(self.screen, (43, 108, 176), self.trail_history[i], self.trail_history[i+1], width)
                
        # 5. Native Cursor
        telemetry = self.serial_manager.get_telemetry()
        cursor_px_x, cursor_px_y = map_mm_to_screen_xz(telemetry['pos'][0], MAX_Z_MM - telemetry['pos'][2])
        cursor_radius = 12
        pygame.draw.circle(self.screen, BLUE, (cursor_px_x, cursor_px_y), cursor_radius)
        pygame.draw.circle(self.screen, WHITE, (cursor_px_x, cursor_px_y), 4)

        # 6. Success Modal
        if self.game_won:
            win_text = self.large_font.render("MAZE COMPLETE! Press ESC to Exit", True, (0, 150, 50))
            text_rect = win_text.get_rect(center=(SCREEN_WIDTH//2, SCREEN_HEIGHT//2))
            bg_rect = text_rect.inflate(40, 40)
            pygame.draw.rect(self.screen, WHITE, bg_rect, border_radius=10)
            pygame.draw.rect(self.screen, GREEN, bg_rect, width=3, border_radius=10)
            shadow_rect = bg_rect.copy()
            shadow_rect.inflate_ip(10, 10)
            pygame.draw.rect(self.screen, (200, 230, 200), shadow_rect, border_radius=10, width=5)
            self.screen.blit(win_text, text_rect)

        self.draw_telemetry()
