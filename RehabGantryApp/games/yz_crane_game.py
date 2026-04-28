import pygame
import random
import math
from games.game_base import GameBase
from core.constants import *

class CargoCraneGame(GameBase):
    def __init__(self, serial_manager, screen):
        super().__init__(serial_manager, screen)
        # Configure mock simulator to interpret mouse axes through the inverse isometric plane
        self.serial_manager.active_plane = 'YZ_ISO'
        
        # State Tracking
        self.score = 0
        self.is_holding_crate = False
        self.target_state = 0
        self.hover_timer = 0.0
        self.target_stiffness = 50.0
        self.last_time = pygame.time.get_ticks()
        
        # 3D Physical dimensions in mm (Y is depth, X is fixed track width, Z is height)
        # Represents X across a pseudo width (-50 to +50 mm)
        self.track_width = 100
        
        # Game Objects
        self.crate_size = 40
        
        # Start Platform
        self.start_y = MIN_Y_MM + 40
        self.start_x = 0
        self.start_z = MIN_Z_MM
        self.start_dim_y = 60
        self.start_dim_x = 80
        self.start_dim_z = 30
        
        # Drop Zone Pillar
        self.drop_y = MAX_Y_MM - 60
        self.drop_x = 0
        self.drop_z = MIN_Z_MM
        self.drop_dim_y = 60
        self.drop_dim_x = 80
        self.drop_pillar_z = 80 # Lowered to provide greater vertical differentiation
        
        # Barrier Wall
        self.barrier_y = (MAX_Y_MM + MIN_Y_MM) / 2
        self.barrier_x = 0
        self.barrier_z = MIN_Z_MM
        self.barrier_dim_y = 20
        self.barrier_dim_x = self.track_width + 40
        self.barrier_z_max = MAX_Z_MM * 0.5  # Player must lift over this Z height
        
        # Screen flash
        self.flash_red_timer = 0.0
        
        # Target Sequence Logic
        high_z = MAX_Z_MM - 20
        self.target_pts = {
            0: (self.start_y, self.start_dim_z + self.crate_size / 2),
            1: (self.start_y, high_z),
            2: (self.drop_y, high_z),
            3: (self.drop_y, self.drop_pillar_z + self.crate_size / 2),
            4: (self.drop_y, high_z),
            5: (self.start_y, high_z),
        }
        
        self.req_hover_time = {
            0: 1.5,
            1: 0.5,
            2: 0.5,
            3: 1.5,
            4: 0.5,
            5: 0.5,
        }
        
        self.current_k = 0.0
        self.last_sent_k = 0.0
        
        # Initial Crate Spawn
        self._spawn_crate()

    def get_start_position(self):
        """Move gantry to pickup platform before gameplay starts."""
        return ((MAX_X_MM + MIN_X_MM) / 2.0, self.start_y, MAX_Z_MM - (self.start_dim_z + self.crate_size / 2), 1)

    def _spawn_crate(self):
        self.crate_y_mm = self.start_y
        self.crate_z_mm = self.start_dim_z  # Drops perfectly on top of platform!
        self.is_holding_crate = False
        self.target_state = 0
        self.hover_timer = 0.0
        
        self.stuck_timer = 0.0
        self.current_k = max(self.current_k, 1.0)
        self.last_sent_k = self.current_k
        
        telemetry = self.serial_manager.get_telemetry()
        pos_y = telemetry['pos'][1]
        pos_z = MAX_Z_MM - telemetry['pos'][2]
        
        # Ensure the first target is sent initially or on reset
        if hasattr(self, 'target_pts') and hasattr(self, 'target_stiffness'):
            initial_target = self.target_pts.get(0, (self.start_y, self.start_dim_z + self.crate_size / 2))
            self.last_distance = math.hypot(pos_y - initial_target[0], pos_z - initial_target[1])
            
            self.serial_manager.send_command((MAX_X_MM + MIN_X_MM) / 2.0, initial_target[0], MAX_Z_MM - initial_target[1], self.current_k, z_en=1)
            self.last_sent_k = self.current_k
            print(f"New target spawned at X: 0.0, Y: {initial_target[0]:.1f}, Z: {initial_target[1]:.1f} (mm)")

    def draw_telemetry(self):
        """Draw Standard UI text on top of everything."""
        telemetry = self.serial_manager.get_telemetry()
        pos_mm = telemetry['pos']
        frc_n = telemetry['frc']

        force_text = self.font.render(f"FRC (N): X:{frc_n[0]:.2f} Y:{frc_n[1]:.2f} Z:{frc_n[2]:.2f}", True, TEXT_COLOR)
        pos_text = self.font.render(f"POS (mm): X:{pos_mm[0]:.2f} Y:{pos_mm[1]:.2f} Z:{pos_mm[2]:.2f}", True, TEXT_COLOR)
        score_text = self.large_font.render(f"Boxes Moved: {self.score}", True, WHITE)
        
        self.screen.blit(force_text, (10, 10))
        self.screen.blit(pos_text, (10, 35))
        self.screen.blit(score_text, (SCREEN_WIDTH - 250, 20))

    def update(self):
        telemetry = self.serial_manager.get_telemetry()
        # Ensure we stay inbound horizontally implicitly on the ISO space
        pos_y = max(MIN_Y_MM, min(telemetry['pos'][1], MAX_Y_MM))
        pos_z = MAX_Z_MM - telemetry['pos'][2]
        
        current_time = pygame.time.get_ticks()
        dt = (current_time - self.last_time) / 1000.0
        self.last_time = current_time
        
        if self.flash_red_timer > 0:
            self.flash_red_timer -= dt

        # Collision Check against barrier (Y depth check + Z height check)
        if abs(pos_y - self.barrier_y) < (self.barrier_dim_y / 2 + 15):
            if pos_z < self.barrier_z_max:
                self.flash_red_timer = 0.2
                self._spawn_crate()

        current_target = self.target_pts.get(self.target_state, self.target_pts[0])
        dist = math.hypot(pos_y - current_target[0], pos_z - current_target[1])
        
        previous_state = self.target_state
        
        # Target proximity check
        if dist < 40:
            self.hover_timer += dt
            if self.hover_timer >= self.req_hover_time[self.target_state]:
                self.hover_timer = 0.0
                
                if self.target_state == 0:
                    self.is_holding_crate = True
                    self.target_state = 1
                elif self.target_state == 1:
                    self.target_state = 2
                elif self.target_state == 2:
                    self.target_state = 3
                elif self.target_state == 3:
                    self.score += 1
                    self.is_holding_crate = False
                    self.target_state = 4
                elif self.target_state == 4:
                    self.target_state = 5
                elif self.target_state == 5:
                    self._spawn_crate()
                    
                # Send the new target to STM32 when state advances
                if previous_state != self.target_state:
                    new_target = self.target_pts.get(self.target_state, self.target_pts[0])
                    
                    self.stuck_timer = 0.0
                    self.current_k = max(self.current_k, 1.0)
                    self.last_distance = math.hypot(pos_y - new_target[0], pos_z - new_target[1])
                    
                    self.serial_manager.send_command((MAX_X_MM + MIN_X_MM) / 2.0, new_target[0], MAX_Z_MM - new_target[1], self.current_k, z_en=1)
                    self.last_sent_k = self.current_k
                    print(f"New target spawned at X: 0.0, Y: {new_target[0]:.1f}, Z: {new_target[1]:.1f} (mm)")
                    
        else:
            self.hover_timer = 0.0

        # AAN Logic
        if not self.serial_manager.mock_mode:
            # 1. Calculate progress since last frame (positive means getting closer)
            progress_mm = self.last_distance - dist
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
                # Ramp up stiffness slowly (e.g., takes 5 seconds to reach max stiffness)
                ramp_rate = self.target_stiffness / 5.0 
                self.current_k = min(self.target_stiffness, self.current_k + (ramp_rate * dt))
                
            # 4. Transmit with Delta-K Thresholding
            if abs(self.current_k - self.last_sent_k) >= 1.0:
                self.serial_manager.send_command((MAX_X_MM + MIN_X_MM) / 2.0, current_target[0], MAX_Z_MM - current_target[1], self.current_k, z_en=1)
                self.last_sent_k = self.current_k
                
            # 5. Save distance for next frame
            self.last_distance = dist

        if self.is_holding_crate:
            # Bind crate to cursor bottom
            self.crate_y_mm = pos_y
            self.crate_z_mm = pos_z - self.crate_size/2

    def draw_iso_cube(self, center_y, center_x, base_z, length_y, width_x, height_z, base_color):
        """Builds a solid 3D polygon mapped perfectly to an Isometric Camera projection."""
        y_min = center_y - length_y / 2
        y_max = center_y + length_y / 2
        x_min = center_x - width_x / 2
        x_max = center_x + width_x / 2
        z_min = base_z
        z_max = base_z + height_z

        v = {
            # Base Layer Coordinates
            0: cart_to_iso(y_max, x_max, z_min), # Bottom Front
            1: cart_to_iso(y_max, x_min, z_min), # Bottom Right
            2: cart_to_iso(y_min, x_min, z_min), # Bottom Back
            3: cart_to_iso(y_min, x_max, z_min), # Bottom Left
            # Peak Layer Coordinates
            4: cart_to_iso(y_max, x_max, z_max), # Top Front
            5: cart_to_iso(y_max, x_min, z_max), # Top Right
            6: cart_to_iso(y_min, x_min, z_max), # Top Back
            7: cart_to_iso(y_min, x_max, z_max), # Top Left
        }
        
        r, g, b = base_color
        r_left, g_left, b_left = int(r*0.65), int(g*0.65), int(b*0.65)
        r_right, g_right, b_right = int(r*0.45), int(g*0.45), int(b*0.45)
        r_top, g_top, b_top = min(255, int(r*1.1)), min(255, int(g*1.1)), min(255, int(b*1.1))

        # Face Maps
        pygame.draw.polygon(self.screen, (r_left, g_left, b_left), [v[0], v[3], v[7], v[4]]) # Left
        pygame.draw.polygon(self.screen, (r_right, g_right, b_right), [v[0], v[1], v[5], v[4]]) # Right
        pygame.draw.polygon(self.screen, (r_top, g_top, b_top), [v[4], v[5], v[6], v[7]]) # Cap/Top

    def _draw_isometric_grid(self):
        grid_color = (60, 60, 70)
        step = 50
        # Draw physical tracks
        for x in range(int(-self.track_width), int(self.track_width) + step, step):
            p1 = cart_to_iso(MIN_Y_MM, x, 0)
            p2 = cart_to_iso(MAX_Y_MM, x, 0)
            pygame.draw.line(self.screen, grid_color, p1, p2, 1)
        
        # Draw bounding ticks laterally
        for y in range(int(MIN_Y_MM), int(MAX_Y_MM) + step, step):
            p1 = cart_to_iso(y, -self.track_width, 0)
            p2 = cart_to_iso(y, self.track_width, 0)
            pygame.draw.line(self.screen, grid_color, p1, p2, 1)

    def _draw_shadow(self, cursor_y, z_height):
        shadow_surf = pygame.Surface((60, 40), pygame.SRCALPHA)
        pygame.draw.ellipse(shadow_surf, (0, 0, 0, 150), (0, 0, 60, 40))
        # Shadow cast vertically onto the calculated floor collision level
        sx, sy = cart_to_iso(cursor_y, 0, z_height)
        self.screen.blit(shadow_surf, (sx - 30, sy - 20))

    def draw(self):
        if self.flash_red_timer > 0:
            self.screen.fill((200, 100, 100)) # Darker construction alert red
        else:
            # Fill background (Construction Sky)
            self.screen.fill((226, 232, 240))
            
        # Draw Floor Grid Background completely behind objects
        self._draw_isometric_grid()
        
        telemetry = self.serial_manager.get_telemetry()
        cursor_y = max(MIN_Y_MM, min(telemetry['pos'][1], MAX_Y_MM))
        cursor_z = MAX_Z_MM - telemetry['pos'][2]
        
        # Calculate dynamic shadow altitude (Raycast downwards along Y)
        shadow_z = MIN_Z_MM
        if abs(cursor_y - self.start_y) <= self.start_dim_y / 2:
            shadow_z = max(shadow_z, self.start_z + self.start_dim_z)
        if abs(cursor_y - self.barrier_y) <= self.barrier_dim_y / 2:
            shadow_z = max(shadow_z, self.barrier_z + self.barrier_z_max)
        if abs(cursor_y - self.drop_y) <= self.drop_dim_y / 2:
            shadow_z = max(shadow_z, self.drop_z + self.drop_pillar_z)
        if not self.is_holding_crate:
            if abs(cursor_y - self.crate_y_mm) <= self.crate_size / 2:
                shadow_z = max(shadow_z, self.crate_z_mm + self.crate_size)
                
        # Depth Sorting List!
        # Isometric visual layer order demands objects in the "back" (low Y physical variables) render FIRST!
        # We append a secondary 'z_order' sort priority so objects occurring at the exact same physical Y
        # paint logically: Object (0) -> Shadow (1) -> Cursor (2).
        
        concrete_gray = (156, 163, 175)
        cat_yellow = (234, 179, 8)
        wood_brown = (139, 69, 19)

        objects = [
            {'y': self.start_y, 'z_order': 0, 'draw_fn': lambda: self.draw_iso_cube(self.start_y, self.start_x, self.start_z, self.start_dim_y, self.start_dim_x, self.start_dim_z, concrete_gray)},
            {'y': self.barrier_y, 'z_order': 0, 'draw_fn': lambda: self.draw_iso_cube(self.barrier_y, self.barrier_x, self.barrier_z, self.barrier_dim_y, self.barrier_dim_x, self.barrier_z_max, cat_yellow)},
            {'y': self.drop_y, 'z_order': 0, 'draw_fn': lambda: self.draw_iso_cube(self.drop_y, self.drop_x, self.drop_z, self.drop_dim_y, self.drop_dim_x, self.drop_pillar_z, concrete_gray)},
            {'y': self.crate_y_mm, 'z_order': 0, 'draw_fn': lambda: self.draw_iso_cube(self.crate_y_mm, 0, self.crate_z_mm, self.crate_size, self.crate_size, self.crate_size, wood_brown)},
        ]
        
        # Inject Dynamic Floor Shadow
        objects.append({
            'y': cursor_y,
            'z_order': 1,
            'draw_fn': lambda: self._draw_shadow(cursor_y, shadow_z)
        })
        
        # Push crane magnet hook into render queue
        def draw_hook():
            # The Magnet Block (U-Shape illusion or heavy flat block)
            steel_color = (75, 85, 99)
            # Center it horizontally along X to align with the crate base
            self.draw_iso_cube(cursor_y, 0, cursor_z, 30, 30, 10, steel_color)
            
            # Active Glow Indicator on Top
            if self.is_holding_crate:
                # Add a tiny glowing block on top
                self.draw_iso_cube(cursor_y + 10, 10, cursor_z + 10, 10, 10, 4, (34, 197, 94))
            
            # The Vertical Crane Cable Extrusion
            cx, cy = cart_to_iso(cursor_y + 15, 15, cursor_z + 10)
            pygame.draw.line(self.screen, (55, 65, 81), (cx, 0), (cx, cy), 4)

        objects.append({
            'y': cursor_y, 
            'z_order': 2,
            'draw_fn': draw_hook
        })
        
        # Waypoint visuals
        if hasattr(self, 'target_pts') and hasattr(self, 'target_state'):
            current_target = self.target_pts[self.target_state]
            
            def draw_waypoint():
                ty, tz = current_target
                pulse = (math.sin(pygame.time.get_ticks() / 150.0) + 1) / 2
                z_anim = tz + 5 * pulse
                
                sx, sy = cart_to_iso(ty, 0, z_anim)
    
                # Draw a glowing target ring
                color = (0, 255, 255)
                pygame.draw.circle(self.screen, color, (sx, sy), 15, 3)
                pygame.draw.circle(self.screen, color, (sx, sy), 25, 1)
                
                # Faint line dropping to floor
                floor_pt = cart_to_iso(ty, 0, MIN_Z_MM)
                pygame.draw.line(self.screen, (150, 220, 220), (sx, sy + 15), floor_pt, 1)
    
            objects.append({
                'y': current_target[0],
                'z_order': 1.5,
                'draw_fn': draw_waypoint
            })

        # Execute depth-sort based explicitly on forward-physical depth, then z-order layer logic
        objects.sort(key=lambda obj: (obj['y'], obj['z_order']))
        
        # Fire Polygons!
        for obj in objects:
            obj['draw_fn']()
            
        # Hover visuals rendered absolutely on top of the sorted stack
        if self.hover_timer > 0 and hasattr(self, 'req_hover_time'):
            sx, sy = cart_to_iso(cursor_y, 0, cursor_z + 40)
            progress = self.hover_timer / self.req_hover_time[self.target_state]
            pygame.draw.rect(self.screen, GREEN, (sx - 20, sy, int(40 * progress), 5))

        self.draw_telemetry()
