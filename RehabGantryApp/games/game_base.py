import pygame
from core.constants import *

class GameBase:
    def __init__(self, serial_manager, screen):
        self.serial_manager = serial_manager
        self.screen = screen
        self.font = pygame.font.SysFont(None, 24)
        self.large_font = pygame.font.SysFont(None, 48)

    def draw_telemetry(self):
        """Draws the player cursor and text force overlay based on serial telemetry."""
        telemetry = self.serial_manager.get_telemetry()
        pos_mm = telemetry['pos']
        frc_n = telemetry['frc']

        # Map to screen pixels (using XY plane as default for games inheriting base)
        screen_x, screen_y = map_mm_to_screen_xy(pos_mm[0], pos_mm[1])

        # Draw Cursor
        pygame.draw.circle(self.screen, BLUE, (screen_x, screen_y), 15)
        # Inner dot
        pygame.draw.circle(self.screen, WHITE, (screen_x, screen_y), 5)

        # Draw Force and Pos Overlays
        force_text = self.font.render(f"FRC (N): X:{frc_n[0]:.2f} Y:{frc_n[1]:.2f} Z:{frc_n[2]:.2f}", True, TEXT_COLOR)
        pos_text = self.font.render(f"POS (mm): X:{pos_mm[0]:.2f} Y:{pos_mm[1]:.2f} Z:{pos_mm[2]:.2f}", True, TEXT_COLOR)
        
        self.screen.blit(force_text, (10, 10))
        self.screen.blit(pos_text, (10, 35))

    def get_start_position(self):
        """Returns (x_mm, y_mm, z_mm, z_enable) for the game's starting position.
        Override in subclasses to specify game-specific start locations."""
        return ((MAX_X_MM + MIN_X_MM) / 2.0, (MAX_Y_MM + MIN_Y_MM) / 2.0, (MAX_Z_MM + MIN_Z_MM) / 2.0, 1)

    def update(self):
        """Update game logic. To be overridden by subclass if needed."""
        pass

    def draw(self):
        """Standard draw loop."""
        self.screen.fill(BG_COLOR)
        self.draw_telemetry()
