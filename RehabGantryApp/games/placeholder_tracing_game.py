import pygame
from games.game_base import GameBase
from core.constants import *

class TracingGame(GameBase):
    def update(self):
        pass

    def draw(self):
        self.screen.fill(BG_COLOR)
        
        # Draw "Under Construction"
        text_surface = self.large_font.render("XZ Plane - Tracing Game - Under Construction", True, GRAY)
        text_rect = text_surface.get_rect(center=(SCREEN_WIDTH//2, SCREEN_HEIGHT//2))
        self.screen.blit(text_surface, text_rect)
        
        # Standard telemetry on top
        self.draw_telemetry()
