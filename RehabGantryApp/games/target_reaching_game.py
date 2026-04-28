import pygame
import random
import math
from games.game_base import GameBase
from core.constants import *

class TargetReachingGame(GameBase):
    def __init__(self, serial_manager, screen):
        super().__init__(serial_manager, screen)
        # Initialize target position (in mm)
        self.target_x_mm = self._get_random_x()
        self.target_y_mm = self._get_random_y()
        self.target_radius_mm = 20 # Collision radius in physical mm
        self.target_stiffness = 50.0 # Example stiffness to send to STM32

        self._send_command_to_gantry()

    def _get_random_x(self):
        # Keeps target somewhat within bounds so player can reach it
        margin = 100
        return random.uniform(MIN_X_MM + margin, MAX_X_MM - margin)

    def _get_random_y(self):
        margin = 50
        return random.uniform(MIN_Y_MM + margin, MAX_Y_MM - margin)

    def _send_command_to_gantry(self):
        """Sends the current target coordinate and K value to the gantry."""
        # For the XY game, assume Z=0 for target
        self.serial_manager.send_command(self.target_x_mm, self.target_y_mm, 0.0, self.target_stiffness)
        print(f"New target spawned at X: {self.target_x_mm:.1f}, Y: {self.target_y_mm:.1f} (mm)")

    def update(self):
        """Spawns a new target if the player reaches the current one."""
        telemetry = self.serial_manager.get_telemetry()
        pos_mm = telemetry['pos']
        
        # Check Cartesian distance in physical mm
        dx = pos_mm[0] - self.target_x_mm
        dy = pos_mm[1] - self.target_y_mm
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.target_radius_mm:
            # Reached target!
            self.target_x_mm = self._get_random_x()
            self.target_y_mm = self._get_random_y()
            self._send_command_to_gantry()
            
    def draw(self):
        # Fill background
        self.screen.fill(BG_COLOR)
        
        # Draw target in screen coordinates
        tgt_screen_x, tgt_screen_y = map_mm_to_screen_xy(self.target_x_mm, self.target_y_mm)
        
        # Scale physical radius to screen pixels
        scale_x = DRAWABLE_WIDTH / (MAX_X_MM - MIN_X_MM)
        tgt_radius_px = int(self.target_radius_mm * scale_x)
        
        pygame.draw.circle(self.screen, RED, (tgt_screen_x, tgt_screen_y), tgt_radius_px)
        
        # Draw standard telemetry overlay (cursor and text) on top
        super().draw_telemetry()
