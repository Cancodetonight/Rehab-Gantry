import sys
import pygame
import math
from core.constants import *
from core.serial_manager import SerialManager
from games.yz_crane_game import CargoCraneGame
from games.xz_maze_game import XZMazeGame
from games.painting_game import PaintingGame

# Additional states for lifecycle management
STATE_CALIBRATING = -1
STATE_MOVING_TO_START = -2

def main():
    print("Select Mode:")
    print("1: Full Hardware Run (Real Telemetry + Real TX)")
    print("2: Hardware-in-the-Loop Test (Mouse Telemetry + Real TX)")
    print("3: Motorless Hardware Run (Real Telemetry + Real TX, no homing)")
    print("4: Motor Test (No force input, auto-move to targets)")
    try:
        mode_input = input("Enter choice (1/2/3/4): ").strip()
    except EOFError:
        mode_input = "1"
    
    is_test_mode = (mode_input == "2")
    is_motorless = (mode_input == "3")
    is_motor_test = (mode_input == "4")

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SCALED)
    pygame.display.set_caption("Upper-Limb Rehabilitation Game Interface")
    clock = pygame.time.Clock()
    is_fullscreen = False

    # Start Serial Manager
    serial_manager = SerialManager(SERIAL_PORT, BAUD_RATE, is_test_mode=is_test_mode)
    serial_manager.start()

    current_state = STATE_CALIBRATING
    active_game = None
    calibration_start_time = pygame.time.get_ticks()
    tare_toast_time = 0  # Timestamp when TARE was last sent (0 = no toast)

    # Colors for New UI
    UI_BG_COLOR = (248, 249, 250)
    UI_PRIMARY = (43, 108, 176)   # Medical Slate Blue
    UI_HOVER = (44, 82, 130)      # Darker Blue
    UI_TEXT_DARK = (33, 37, 41)
    UI_TEXT_LIGHT = (255, 255, 255)
    UI_GRAY = (150, 150, 150)
    UI_SHADOW = (209, 213, 219)   # Drop shadow gray

    # Fonts for Menu
    font_prefs = 'segoe ui,helvetica,arial'
    menu_font = pygame.font.SysFont(font_prefs, 64, bold=True)
    subtitle_font = pygame.font.SysFont(font_prefs, 28)
    btn_font = pygame.font.SysFont(font_prefs, 26, bold=True)
    footer_font = pygame.font.SysFont(font_prefs, 18)
    status_font = pygame.font.SysFont(font_prefs, 22)

    # Define menu buttons (Rects)
    btn_width = 500
    btn_height = 60
    btn_x = (SCREEN_WIDTH - btn_width) // 2
    start_y = 250
    btn_spacing = 30
    
    btn_game1 = pygame.Rect(btn_x, start_y + (btn_height + btn_spacing) * 0, btn_width, btn_height)
    btn_game2 = pygame.Rect(btn_x, start_y + (btn_height + btn_spacing) * 1, btn_width, btn_height)
    btn_game3 = pygame.Rect(btn_x, start_y + (btn_height + btn_spacing) * 2, btn_width, btn_height)

    def draw_waves(screen, time_elapsed):
        wave_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        # Layers: (amplitude, frequency, phase_speed, alpha)
        layers = [
            (50, 0.005, 0.001, 30),
            (35, 0.008, 0.0015, 50),
            (20, 0.012, 0.002, 70),
        ]
        
        base_y = SCREEN_HEIGHT - 120
        
        for amp, freq, speed, alpha in layers:
            points = [(0, SCREEN_HEIGHT)]
            for x in range(0, SCREEN_WIDTH + 10, 10):
                y = base_y + amp * math.sin(x * freq + time_elapsed * speed)
                points.append((x, y))
            points.append((SCREEN_WIDTH, SCREEN_HEIGHT))
            
            pygame.draw.polygon(wave_surface, (43, 108, 176, alpha), points)
            
        screen.blit(wave_surface, (0, 0))

    def draw_calibrating_screen():
        """Premium homing/calibration screen shown at startup."""
        screen.fill((15, 18, 25))
        
        t = pygame.time.get_ticks() / 1000.0
        
        # Animated spinner ring
        cx, cy = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 - 40
        ring_radius = 50
        for i in range(12):
            angle = math.radians(i * 30 + t * 180)
            alpha = int(80 + 175 * ((i / 12 + t * 0.5) % 1.0))
            alpha = min(255, alpha)
            dot_x = cx + int(ring_radius * math.cos(angle))
            dot_y = cy + int(ring_radius * math.sin(angle))
            dot_size = 4 + int(3 * ((i / 12 + t * 0.5) % 1.0))
            color = (43, 108, 176, alpha)
            dot_surf = pygame.Surface((dot_size * 2, dot_size * 2), pygame.SRCALPHA)
            pygame.draw.circle(dot_surf, color, (dot_size, dot_size), dot_size)
            screen.blit(dot_surf, (dot_x - dot_size, dot_y - dot_size))
        
        # Title
        title = menu_font.render("System Calibrating", True, (220, 225, 235))
        screen.blit(title, (SCREEN_WIDTH // 2 - title.get_width() // 2, cy + 80))
        
        # Subtitle with pulsing dots
        dots = "." * (int(t * 2) % 4)
        sub = subtitle_font.render(f"Homing all axes{dots}", True, UI_GRAY)
        screen.blit(sub, (SCREEN_WIDTH // 2 - sub.get_width() // 2, cy + 150))
        
        # Status text
        elapsed = (pygame.time.get_ticks() - calibration_start_time) / 1000.0
        status = status_font.render(f"Elapsed: {elapsed:.1f}s", True, (80, 85, 95))
        screen.blit(status, (SCREEN_WIDTH // 2 - status.get_width() // 2, cy + 190))

    def draw_moving_to_start_screen():
        """Overlay shown while gantry auto-navigates to game start position."""
        # Draw the game underneath (frozen)
        if active_game:
            active_game.draw()
        
        # Dark overlay
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        screen.blit(overlay, (0, 0))
        
        t = pygame.time.get_ticks() / 1000.0
        cx, cy = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
        
        # Pulsing ring
        pulse = (math.sin(t * 3) + 1) / 2
        ring_r = int(30 + 10 * pulse)
        pygame.draw.circle(screen, (43, 108, 176), (cx, cy - 50), ring_r, 3)
        pygame.draw.circle(screen, (43, 108, 176, 100), (cx, cy - 50), ring_r + 10, 1)
        
        # Inner dot
        inner_r = int(8 + 4 * pulse)
        pygame.draw.circle(screen, (100, 160, 230), (cx, cy - 50), inner_r)
        
        # Text
        title = subtitle_font.render("Moving to Start Position", True, UI_TEXT_LIGHT)
        screen.blit(title, (cx - title.get_width() // 2, cy + 10))
        
        dots = "." * (int(t * 2) % 4)
        sub = status_font.render(f"Please wait{dots}", True, UI_GRAY)
        screen.blit(sub, (cx - sub.get_width() // 2, cy + 50))
        
        # Show distance remaining
        telemetry = serial_manager.get_telemetry()
        pos = telemetry['pos']
        if hasattr(main, '_move_target'):
            tgt = main._move_target
            dist = math.sqrt((pos[0]-tgt[0])**2 + (pos[1]-tgt[1])**2 + (pos[2]-tgt[2])**2)
            dist_text = status_font.render(f"Distance: {dist:.1f} mm", True, (80, 130, 200))
            screen.blit(dist_text, (cx - dist_text.get_width() // 2, cy + 85))

    def draw_menu():
        screen.fill(UI_BG_COLOR)
        
        # 1. Animated wave background
        time_elapsed = pygame.time.get_ticks()
        draw_waves(screen, time_elapsed)
        
        # Draw Title and Subtitle
        title = menu_font.render("Rehabilitation Module", True, UI_TEXT_DARK)
        screen.blit(title, (SCREEN_WIDTH//2 - title.get_width()//2, 100))
        
        subtitle = subtitle_font.render("Select a therapy exercise to begin", True, UI_GRAY)
        screen.blit(subtitle, (SCREEN_WIDTH//2 - subtitle.get_width()//2, 170))

        mouse_pos = pygame.mouse.get_pos()

        # 2 + 3. Draw shadow -> Draw buttons
        for btn, text in [(btn_game1, "Hover-to-Reveal Painting (XY Plane)"), 
                          (btn_game2, "Cargo Crane Game (YZ Plane)"), 
                          (btn_game3, "Maze Navigation (XZ Plane)")]:
            
            # Drop shadow layer
            shadow_rect = pygame.Rect(btn.x + 4, btn.y + 4, btn.width, btn.height)
            pygame.draw.rect(screen, UI_SHADOW, shadow_rect, border_radius=15)
            
            # Primary button layer
            color = UI_HOVER if btn.collidepoint(mouse_pos) else UI_PRIMARY
            pygame.draw.rect(screen, color, btn, border_radius=15)
            
            # 4. Text layer
            lbl = btn_font.render(text, True, UI_TEXT_LIGHT)
            screen.blit(lbl, (btn.x + (btn.width - lbl.get_width())//2, btn.y + (btn.height - lbl.get_height())//2))
            
        # Draw Footer
        footer_text = footer_font.render("NUST Final Year Project - Multi DOF Upper Limb Rehabilitation Device", True, UI_GRAY)
        screen.blit(footer_text, (SCREEN_WIDTH//2 - footer_text.get_width()//2, SCREEN_HEIGHT - 40))

    running = True
    
    # Fade State Machine
    fade_state = None  # None, 'FADING_OUT', 'FADING_IN'
    fade_alpha = 0
    target_state = None
    target_game_class = None
    
    show_quit_modal = False
    quit_modal_btn = pygame.Rect(SCREEN_WIDTH//2 - 150, SCREEN_HEIGHT//2 + 50, 300, 60)
    
    fade_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
    fade_surface.fill((0, 0, 0))

    # Moving-to-start state tracking
    move_target = (0.0, 0.0, 0.0)
    move_z_enable = 0
    mtest_active = False  # Track whether MTEST auto-pull is currently on
    MOVE_ARRIVAL_THRESHOLD = 20.0  # mm
    MOVE_STIFFNESS = 50.0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
            # Block input during fades and calibration
            if fade_state is not None or current_state == STATE_CALIBRATING:
                continue
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_F11:
                    is_fullscreen = not is_fullscreen
                    if is_fullscreen:
                        pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.FULLSCREEN | pygame.SCALED)
                    else:
                        pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SCALED)
                # Global ESC to return to menu with safety intercept
                elif event.key == pygame.K_ESCAPE:
                    if current_state not in (STATE_MENU, STATE_CALIBRATING, STATE_MOVING_TO_START):
                        if not show_quit_modal:
                            show_quit_modal = True
                            serial_manager.send_stop()
                # T key to tare force sensors
                elif event.key == pygame.K_t:
                    serial_manager.send_tare()
                    tare_toast_time = pygame.time.get_ticks()
                    print("[TARE] Force sensor tare requested")
                            
            if show_quit_modal:
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    mouse_pos = event.pos
                    if quit_modal_btn.collidepoint(mouse_pos):
                        show_quit_modal = False
                        target_state = STATE_MENU
                        target_game_class = None
                        fade_state = 'FADING_OUT'
                    
            if current_state == STATE_MENU and not show_quit_modal:
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    mouse_pos = event.pos
                    if btn_game1.collidepoint(mouse_pos):
                        target_state = STATE_GAME_1
                        target_game_class = PaintingGame
                        fade_state = 'FADING_OUT'
                    elif btn_game2.collidepoint(mouse_pos):
                        target_state = STATE_GAME_2
                        target_game_class = CargoCraneGame
                        fade_state = 'FADING_OUT'
                    elif btn_game3.collidepoint(mouse_pos):
                        target_state = STATE_GAME_3
                        target_game_class = XZMazeGame
                        fade_state = 'FADING_OUT'

        # ==========================================
        #       STATE: CALIBRATING (Homing)
        # ==========================================
        if current_state == STATE_CALIBRATING:
            draw_calibrating_screen()
            
            # Motorless mode only: auto-complete homing after 5 seconds
            if is_motorless:
                if pygame.time.get_ticks() - calibration_start_time > 5000:
                    serial_manager.is_homed = True
                    print("[MOTORLESS] Homing auto-completed (5s timeout)")
            
            # Transition only when STM sends <HOMED> (or motorless timer fires)
            if serial_manager.is_homed:
                # Mode 4: enable motor test after real homing completes
                if is_motor_test:
                    serial_manager.send_mtest(enable=True)
                    print("[MOTOR TEST] Mode enabled — forces ignored, auto-pull active")
                fade_state = 'FADING_OUT'
                target_state = STATE_MENU
                target_game_class = None

        # ==========================================
        #     STATE: MOVING TO START POSITION
        # ==========================================
        elif current_state == STATE_MOVING_TO_START:
            draw_moving_to_start_screen()
            
            # Enable auto-pull so gantry navigates to start without force input
            if not mtest_active:
                serial_manager.send_mtest(enable=True)
                mtest_active = True
            
            # Send the start position with high stiffness to auto-navigate
            serial_manager.send_command(
                move_target[0], move_target[1], move_target[2],
                MOVE_STIFFNESS, z_en=move_z_enable
            )
            
            # Check if arrived — only on enabled axes
            telemetry = serial_manager.get_telemetry()
            pos = telemetry['pos']
            dx = pos[0] - move_target[0]
            dy = pos[1] - move_target[1]
            
            if move_z_enable:
                dz = pos[2] - move_target[2]
            else:
                dz = 0.0  # Z is brake-clamped, ignore it
            
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist < MOVE_ARRIVAL_THRESHOLD:
                # Arrived — disable auto-pull for normal force-driven gameplay
                if not is_motor_test:
                    serial_manager.send_mtest(enable=False)
                    mtest_active = False
                current_state = target_state
                print(f"[MOVE] Arrived at start position (dist={dist:.1f}mm)")

        # ==========================================
        #           STATE: MENU
        # ==========================================
        elif current_state == STATE_MENU:
            draw_menu()
            
        # ==========================================
        #         STATE: ACTIVE GAME
        # ==========================================
        else:
            if active_game:
                if fade_state is None and not show_quit_modal:
                    active_game.update()
                active_game.draw()
                
                # Render Safety Verification Popup if triggered
                if show_quit_modal:
                    overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
                    overlay.fill((0, 0, 0, 180))
                    screen.blit(overlay, (0, 0))
                    
                    popup_w, popup_h = 600, 320
                    popup_rect = pygame.Rect(SCREEN_WIDTH//2 - popup_w//2, SCREEN_HEIGHT//2 - popup_h//2, popup_w, popup_h)
                    pygame.draw.rect(screen, UI_BG_COLOR, popup_rect, border_radius=15)
                    
                    warn_title = menu_font.render("SAFETY WARNING", True, (200, 50, 50))
                    warn_desc1 = subtitle_font.render("Device Brakes are now securely clamped.", True, UI_TEXT_DARK)
                    warn_desc2 = subtitle_font.render("Please let go of the handle.", True, UI_TEXT_DARK)
                    
                    screen.blit(warn_title, (SCREEN_WIDTH//2 - warn_title.get_width()//2, popup_rect.y + 30))
                    screen.blit(warn_desc1, (SCREEN_WIDTH//2 - warn_desc1.get_width()//2, popup_rect.y + 110))
                    screen.blit(warn_desc2, (SCREEN_WIDTH//2 - warn_desc2.get_width()//2, popup_rect.y + 150))
                    
                    mouse_pos = pygame.mouse.get_pos()
                    color = UI_HOVER if quit_modal_btn.collidepoint(mouse_pos) else UI_PRIMARY
                    pygame.draw.rect(screen, color, quit_modal_btn, border_radius=12)
                    
                    btn_lbl = btn_font.render("I have let go", True, UI_TEXT_LIGHT)
                    screen.blit(btn_lbl, (quit_modal_btn.x + (quit_modal_btn.width - btn_lbl.get_width())//2, quit_modal_btn.y + (quit_modal_btn.height - btn_lbl.get_height())//2))

        # ==========================================
        #          FADE TRANSITIONS
        # ==========================================
        if fade_state == 'FADING_OUT':
            fade_alpha = min(255, fade_alpha + 15)
            if fade_alpha == 255:
                if current_state == STATE_CALIBRATING:
                    # Calibration done -> go to menu
                    current_state = STATE_MENU
                    active_game = None
                    fade_state = 'FADING_IN'
                elif target_state == STATE_MENU:
                    # Returning to menu -> stop motors
                    current_state = STATE_MENU
                    serial_manager.active_plane = 'XY'
                    active_game = None
                    # Ensure auto-pull is disabled when leaving a game
                    if mtest_active and not is_motor_test:
                        serial_manager.send_mtest(enable=False)
                        mtest_active = False
                    serial_manager.send_stop()
                    fade_state = 'FADING_IN'
                elif target_game_class:
                    # Starting a new game -> create it, then move to start
                    serial_manager.active_plane = 'XY'  # Reset, game __init__ will override
                    active_game = target_game_class(serial_manager, screen)
                    
                    # Get start position and enter moving state
                    start_pos = active_game.get_start_position()
                    move_target = (start_pos[0], start_pos[1], start_pos[2])
                    move_z_enable = start_pos[3]
                    
                    current_state = STATE_MOVING_TO_START
                    print(f"[MOVE] Navigating to start: ({move_target[0]:.1f}, {move_target[1]:.1f}, {move_target[2]:.1f})")
                    fade_state = 'FADING_IN'
                else:
                    current_state = target_state
                    active_game = None
                    fade_state = 'FADING_IN'
                    
        elif fade_state == 'FADING_IN':
            fade_alpha = max(0, fade_alpha - 15)
            if fade_alpha == 0:
                fade_state = None

        if fade_alpha > 0:
            fade_surface.set_alpha(fade_alpha)
            screen.blit(fade_surface, (0, 0))

        # --- Tare Toast Notification ---
        if tare_toast_time > 0:
            elapsed_toast = pygame.time.get_ticks() - tare_toast_time
            if elapsed_toast < 2000:
                toast_alpha = max(0, 255 - int(elapsed_toast / 2000 * 255))
                toast_surf = pygame.Surface((300, 50), pygame.SRCALPHA)
                toast_surf.fill((30, 30, 30, toast_alpha))
                toast_text = status_font.render("TARE Sent", True, (100, 220, 100))
                toast_surf.blit(toast_text, (150 - toast_text.get_width() // 2, 12))
                screen.blit(toast_surf, (SCREEN_WIDTH // 2 - 150, 20))
            else:
                tare_toast_time = 0

        pygame.display.flip()
        clock.tick(FPS)

    # Cleanup
    if is_motor_test:
        serial_manager.send_mtest(enable=False)
    serial_manager.stop()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
