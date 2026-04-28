import pygame
import random
import math
from games.game_base import GameBase
from core.constants import *

class PaintingGame(GameBase):
    def __init__(self, serial_manager, screen):
        super().__init__(serial_manager, screen)
        
        # Load the background painting
        try:
            import os
            # Get absolute path to the app root directory by going up one level from 'games'
            current_dir = os.path.dirname(os.path.abspath(__file__))
            asset_path = os.path.join(current_dir, "..", "assets", "painting.jpg")
            
            raw_img = pygame.image.load(asset_path).convert()
            self.painting_bg = pygame.transform.scale(raw_img, (SCREEN_WIDTH, SCREEN_HEIGHT))
        except pygame.error as e:
            print(f"Error loading painting.jpg: {e}. Falling back to color background.")
            self.painting_bg = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            self.painting_bg.fill((50, 100, 150))
            
        # 1. The Blurred & Tinted Overlay
        small_w, small_h = SCREEN_WIDTH // 16, SCREEN_HEIGHT // 16
        small_img = pygame.transform.smoothscale(self.painting_bg, (small_w, small_h))
        self.blurred_overlay = pygame.transform.smoothscale(small_img, (SCREEN_WIDTH, SCREEN_HEIGHT)).convert_alpha()
        
        luminance = self._get_average_luminance(self.painting_bg)
        grey_tint = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        
        if luminance < 128:
            # If background is dark, use translucent White
            grey_tint.fill((255, 255, 255, 150))
        else:
            # If background is light, use translucent Gray/Black
            grey_tint.fill((50, 50, 50, 150))
            
        self.blurred_overlay.blit(grey_tint, (0, 0))
        
        self.revealed_spots = []
        
        # Game State
        self.target_radius_mm = 25
        self.target_stiffness = 50.0
        self.hover_timer = 0.0
        self.required_hover_time = 2.0 # 2 seconds
        self.last_distance = 0.0
        self.stuck_timer = 0.0
        self.current_k = 0.0
        self.last_sent_k = 0.0
        
        self.last_time = pygame.time.get_ticks()

        # Animation State
        self.is_animating_reveal = False
        self.current_reveal_radius = 0.0
        self.animation_duration = 0.5
        self.reveal_timer = 0.0
        self.reveal_tgt_screen_pos = (0, 0)

        # Spawn first target
        self.target_x_mm = 0
        self.target_y_mm = 0
        self._spawn_new_target()

    def get_start_position(self):
        """Move gantry to first target before gameplay starts."""
        return (self.target_x_mm, self.target_y_mm, 0.0, 0)

    def _get_average_luminance(self, surface):
        """Calculate the average brightness/luminance of a surface."""
        # Scale down to 1x1 to efficiently get the average color
        avg_surface = pygame.transform.smoothscale(surface, (1, 1))
        color = avg_surface.get_at((0, 0))
        return 0.299 * color.r + 0.587 * color.g + 0.114 * color.b

    def _spawn_new_target(self):
        max_attempts = 100
        margin_x = 150
        margin_y = 100
        for _ in range(max_attempts):
            nx = random.uniform(MIN_X_MM + margin_x, MAX_X_MM - margin_x)
            ny = random.uniform(MIN_Y_MM + margin_y, MAX_Y_MM - margin_y)
            
            overlap = False
            for rx, ry, rad in self.revealed_spots:
                if math.hypot(nx - rx, ny - ry) < self.target_radius_mm * 2:
                    overlap = True
                    break
                    
            if not overlap:
                self.target_x_mm = nx
                self.target_y_mm = ny
                self._send_command_to_gantry()
                return
                
        # Failsafe
        self.target_x_mm = nx
        self.target_y_mm = ny
        self._send_command_to_gantry()

    def _send_command_to_gantry(self):
        """Sends the current target coordinate to the gantry."""
        self.stuck_timer = 0.0
        # Keep a minimum K so gantry stays responsive between targets (no dead zone)
        self.current_k = max(self.current_k, 1.0)
        
        telemetry = self.serial_manager.get_telemetry()
        pos_mm = telemetry['pos']
        dx = pos_mm[0] - self.target_x_mm
        dy = pos_mm[1] - self.target_y_mm
        self.last_distance = math.sqrt(dx**2 + dy**2)
        
        self.serial_manager.send_command(self.target_x_mm, self.target_y_mm, 0.0, self.current_k, z_en=0)
        self.last_sent_k = self.current_k
        print(f"New target spawned at X: {self.target_x_mm:.1f}, Y: {self.target_y_mm:.1f}, Z: 0.0 (mm)")

    def update(self):
        telemetry = self.serial_manager.get_telemetry()
        pos_mm = telemetry['pos']
        
        current_time = pygame.time.get_ticks()
        dt = (current_time - self.last_time) / 1000.0
        self.last_time = current_time
        
        dx = pos_mm[0] - self.target_x_mm
        dy = pos_mm[1] - self.target_y_mm
        distance = math.sqrt(dx**2 + dy**2)

        if not self.is_animating_reveal:
            if not self.serial_manager.mock_mode:
                # 1. Calculate progress since last frame (positive means getting closer)
                progress_mm = self.last_distance - distance 
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
                    self.serial_manager.send_command(self.target_x_mm, self.target_y_mm, 0.0, self.current_k, z_en=0)
                    self.last_sent_k = self.current_k
                    
                # 5. Save distance for next frame
                self.last_distance = distance

        if self.is_animating_reveal:
            self.reveal_timer += dt
            progress = min(1.0, self.reveal_timer / self.animation_duration)
            
            scale_x = DRAWABLE_WIDTH / (MAX_X_MM - MIN_X_MM)
            max_hole_radius_px = int(self.target_radius_mm * scale_x * 2.5)
            self.current_reveal_radius = max_hole_radius_px * progress
            
            # Punch growing hole using Pygame's transparent circle overwrite
            pygame.draw.circle(self.blurred_overlay, (0, 0, 0, 0), self.reveal_tgt_screen_pos, int(self.current_reveal_radius))
            
            if progress >= 1.0:
                # Finish animation
                self.is_animating_reveal = False
                self.revealed_spots.append((self.target_x_mm, self.target_y_mm, self.target_radius_mm))
                self.hover_timer = 0.0
                self._spawn_new_target()
        else:
            if distance < self.target_radius_mm:
                self.hover_timer += dt
                if self.hover_timer >= self.required_hover_time:
                    # Hover complete! Start sequence
                    tgt_screen_x, tgt_screen_y = map_mm_to_screen_xy(self.target_x_mm, self.target_y_mm)
                    self.reveal_tgt_screen_pos = (tgt_screen_x, tgt_screen_y)
                    
                    self.is_animating_reveal = True
                    self.reveal_timer = 0.0
                    self.current_reveal_radius = 0.0
            else:
                self.hover_timer = 0.0

    def draw(self):
        # 1. Base Painting
        self.screen.blit(self.painting_bg, (0, 0))
        
        # 2. Fog of war mask (blurred overlay with holes)
        self.screen.blit(self.blurred_overlay, (0, 0))
        
        # 3. Draw Target & Hover effects
        tgt_screen_x, tgt_screen_y = map_mm_to_screen_xy(self.target_x_mm, self.target_y_mm)
        scale_x = DRAWABLE_WIDTH / (MAX_X_MM - MIN_X_MM)
        tgt_radius_px = int(self.target_radius_mm * scale_x)
        
        # Color interpolate based on timer (RED -> GREEN)
        progress = min(1.0, self.hover_timer / self.required_hover_time)
        r = int(RED[0] * (1 - progress) + GREEN[0] * progress)
        g = int(RED[1] * (1 - progress) + GREEN[1] * progress)
        b = int(RED[2] * (1 - progress) + GREEN[2] * progress)
        
        # Draw target ring
        pygame.draw.circle(self.screen, (r, g, b), (tgt_screen_x, tgt_screen_y), tgt_radius_px, 3)
        
        # Draw expanding fill if hovering
        if progress > 0:
            fill_radius = int(tgt_radius_px * progress)
            if fill_radius > 0:
                pygame.draw.circle(self.screen, (r, g, b, 150), (tgt_screen_x, tgt_screen_y), fill_radius)
                
        # 4. Standard telemetry (cursor)
        super().draw_telemetry()
