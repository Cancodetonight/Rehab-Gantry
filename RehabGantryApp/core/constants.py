import pygame

# --- Display Settings ---
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
FPS = 60

# --- Colors ---
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (50, 50, 50)
RED = (200, 50, 50)
GREEN = (50, 200, 50)
BLUE = (50, 50, 200)
TEXT_COLOR = (0, 0, 0) # Black for high visibility
BG_COLOR = (20, 20, 25)

# UI Elements
BUTTON_COLOR = (70, 130, 180)
BUTTON_HOVER_COLOR = (100, 150, 200)

# --- Physical Coordinates Settings (mm) ---
# XY plane boundaries
MIN_X_MM = 0
MAX_X_MM = 510
MIN_Y_MM = 0
MAX_Y_MM = 230

# Z plane boundaries
MIN_Z_MM = 0
MAX_Z_MM = 200

# Mapping constants to map physical mm bounds to screen bounds
MARGIN_PX = 50
DRAWABLE_WIDTH = SCREEN_WIDTH - 2 * MARGIN_PX
DRAWABLE_HEIGHT = SCREEN_HEIGHT - 2 * MARGIN_PX

def map_mm_to_screen_xy(x_mm, y_mm):
    """Maps physical (x, y) coordinates in mm to screen (x, y) pixels for the primary game."""
    # Clamp values
    x_mm = max(MIN_X_MM, min(x_mm, MAX_X_MM))
    y_mm = max(MIN_Y_MM, min(y_mm, MAX_Y_MM))
    
    # Scale calculation
    x_pixels = MARGIN_PX + (x_mm - MIN_X_MM) / (MAX_X_MM - MIN_X_MM) * DRAWABLE_WIDTH
    
    # Y=0 at top of screen (front of gantry)
    y_pixels = MARGIN_PX + (y_mm - MIN_Y_MM) / (MAX_Y_MM - MIN_Y_MM) * DRAWABLE_HEIGHT
    
    return int(x_pixels), int(y_pixels)

def map_screen_xy_to_mm(x_pixels, y_pixels):
    """Maps screen (x, y) pixels back to physical (x, y) coordinates in mm."""
    # Inverse map X
    x_scaled = (x_pixels - MARGIN_PX) / DRAWABLE_WIDTH
    x_mm = MIN_X_MM + x_scaled * (MAX_X_MM - MIN_X_MM)
    
    # Inverse map Y (Y=0 at top)
    y_scaled = (y_pixels - MARGIN_PX) / DRAWABLE_HEIGHT
    y_mm = MIN_Y_MM + y_scaled * (MAX_Y_MM - MIN_Y_MM)
    
    # Clamp to boundaries
    x_mm = max(MIN_X_MM, min(x_mm, MAX_X_MM))
    y_mm = max(MIN_Y_MM, min(y_mm, MAX_Y_MM))
    return x_mm, y_mm

# --- Isometric Projection Helpers ---
ISO_CENTER_X = (SCREEN_WIDTH // 2) - 360  
ISO_BOTTOM_Y = (SCREEN_HEIGHT // 2) + 150   
ISO_SCALE_X = 1.8  
ISO_SCALE_Y = 0.9   
ISO_Z_SCALE = 1.5

def cart_to_iso(y_world, x_world, z_world):
    """Translates a 3D physical point (mm) onto the 2D display surface as an Isometric projection."""
    sx = ISO_CENTER_X + (y_world - x_world) * ISO_SCALE_X
    sy = ISO_BOTTOM_Y + (y_world + x_world) * ISO_SCALE_Y - (z_world * ISO_Z_SCALE)
    return int(sx), int(sy)

def map_iso_screen_to_yz(sx, sy):
    """Mock Simulator algebra inversing from screen X/Y to isometric physical Y/Z with constant pseudo_X=0."""
    y_world = (sx - ISO_CENTER_X) / ISO_SCALE_X
    z_world = (ISO_BOTTOM_Y + y_world * ISO_SCALE_Y - sy) / ISO_Z_SCALE
    y_mm = max(MIN_Y_MM, min(y_world, MAX_Y_MM))
    z_mm = max(MIN_Z_MM, min(z_world, MAX_Z_MM))
    return y_mm, z_mm

def map_mm_to_screen_xz(x_mm, z_mm):
    """Maps physical (x, z) coordinates in mm to screen (x, y) pixels for frontal games."""
    x_mm = max(MIN_X_MM, min(x_mm, MAX_X_MM))
    z_mm = max(MIN_Z_MM, min(z_mm, MAX_Z_MM))
    
    # Scale X -> Screen X
    x_pixels = MARGIN_PX + (x_mm - MIN_X_MM) / (MAX_X_MM - MIN_X_MM) * DRAWABLE_WIDTH
    
    # Scale Z -> Screen Y (inverted)
    y_scaled = (z_mm - MIN_Z_MM) / (MAX_Z_MM - MIN_Z_MM) * DRAWABLE_HEIGHT
    y_pixels = SCREEN_HEIGHT - MARGIN_PX - y_scaled
    
    return int(x_pixels), int(y_pixels)

def map_screen_xz_to_mm(x_pixels, y_pixels):
    """Maps screen (x, y) pixels back to physical (x, z) coordinates in mm."""
    # Inverse map X
    x_scaled = (x_pixels - MARGIN_PX) / DRAWABLE_WIDTH
    x_mm = MIN_X_MM + x_scaled * (MAX_X_MM - MIN_X_MM)
    
    # Inverse map Z
    y_scaled = SCREEN_HEIGHT - MARGIN_PX - y_pixels
    z_mm = MIN_Z_MM + (y_scaled / DRAWABLE_HEIGHT) * (MAX_Z_MM - MIN_Z_MM)
    
    x_mm = max(MIN_X_MM, min(x_mm, MAX_X_MM))
    z_mm = max(MIN_Z_MM, min(z_mm, MAX_Z_MM))
    
    return x_mm, z_mm

def map_mm_to_screen_yz(y_mm, z_mm):
    """Maps physical (y, z) coordinates in mm to screen (x, y) pixels for sagittal games."""
    y_mm = max(MIN_Y_MM, min(y_mm, MAX_Y_MM))
    z_mm = max(MIN_Z_MM, min(z_mm, MAX_Z_MM))
    
    # Scale Y -> Screen X
    x_pixels = MARGIN_PX + (y_mm - MIN_Y_MM) / (MAX_Y_MM - MIN_Y_MM) * DRAWABLE_WIDTH
    
    # Scale Z -> Screen Y (inverted)
    y_scaled = (z_mm - MIN_Z_MM) / (MAX_Z_MM - MIN_Z_MM) * DRAWABLE_HEIGHT
    y_pixels = SCREEN_HEIGHT - MARGIN_PX - y_scaled
    
    return int(x_pixels), int(y_pixels)

def map_screen_yz_to_mm(x_pixels, y_pixels):
    """Maps screen (x, y) pixels back to physical (y, z) coordinates in mm."""
    # Inverse map Y
    x_scaled = (x_pixels - MARGIN_PX) / DRAWABLE_WIDTH
    y_mm = MIN_Y_MM + x_scaled * (MAX_Y_MM - MIN_Y_MM)
    
    # Inverse map Z
    y_scaled = SCREEN_HEIGHT - MARGIN_PX - y_pixels
    z_mm = MIN_Z_MM + (y_scaled / DRAWABLE_HEIGHT) * (MAX_Z_MM - MIN_Z_MM)
    
    y_mm = max(MIN_Y_MM, min(y_mm, MAX_Y_MM))
    z_mm = max(MIN_Z_MM, min(z_mm, MAX_Z_MM))
    
    return y_mm, z_mm

# --- Serial Settings ---
SERIAL_PORT = "COM5" # Hardware Port
BAUD_RATE = 115200

# --- Game States ---
STATE_MENU = 0
STATE_GAME_1 = 1 # XY Target Reaching
STATE_GAME_2 = 2 # YZ Maze
STATE_GAME_3 = 3 # XZ Tracing
STATE_GAME_4 = 4 # Painting Game
