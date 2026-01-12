import pygame
import cv2
import numpy as np
import json
import os
import serial
import serial.tools.list_ports
import threading
import queue
import time

pygame.init()
pygame.joystick.init()

def load_config(config_path='config.json'):
    default_config = {
        "gcode_settings": {
            "coordinate_mode_absolute": "G90",
            "coordinate_mode_relative": "G91",
            "stop_command": "M5",
            "pause_command": "!",
            "resume_command": "~"
        },
        "movement_settings": {
            "default_feed_rate": 1000,
            "default_analog_speed": 3.0,
            "default_step_size": 1.0,
            "gcode_send_rate": 10  # Commands per second
        },
        "joystick_mapping": {
            "axis_x": 0,
            "axis_y": 1,
            "axis_z": 3  # Right stick Y-axis
        },
        "serial_settings": {
            "baudrate": 115200,
            "timeout": 1.0
        }
    }
    
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                loaded = json.load(f)
                for section in default_config:
                    if section in loaded:
                        default_config[section].update(loaded[section])
        except:
            pass
    else:
        with open(config_path, 'w') as f:
            json.dump(default_config, f, indent=2)
    
    return default_config

CONFIG = load_config()

# Get actual screen size
display_info = pygame.display.Info()
SCREEN_WIDTH = display_info.current_w
SCREEN_HEIGHT = display_info.current_h

LEFT_PANEL_WIDTH = 300
TERMINAL_HEIGHT = 220
CAMERA_WIDTH = (SCREEN_WIDTH - LEFT_PANEL_WIDTH - 60) // 2
CAMERA_HEIGHT = SCREEN_HEIGHT - TERMINAL_HEIGHT - 100

BG_COLOR = (240, 242, 245)
PANEL_COLOR = (255, 255, 255)
ACCENT_COLOR = (41, 128, 185)
TEXT_COLOR = (44, 62, 80)
BORDER_COLOR = (189, 195, 199)
TERMINAL_BG = (33, 37, 43)
TERMINAL_TEXT = (204, 204, 204)
SUCCESS_COLOR = (46, 204, 113)
WARNING_COLOR = (230, 126, 34)
ERROR_COLOR = (231, 76, 60)
BUTTON_OFF = (236, 240, 241)
BUTTON_OFF_HOVER = (189, 195, 199)
WAYPOINT_COLOR = (155, 89, 182)
RECORD_COLOR = (231, 76, 60)
PLAYBACK_COLOR = (46, 204, 113)

class ControlMode:
    ANALOG = 1
    STEP = 2

class MachineState:
    IDLE = 1
    JOGGING = 2
    PAUSED = 3

class RecordState:
    NOT_RECORDING = 0
    RECORDING = 1
    READY_TO_EXECUTE = 2
    EXECUTING = 3

class SerialConnection:
    def __init__(self):
        self.serial_port = None
        self.connected = False
        self.command_queue = queue.Queue()
        self.send_thread = None
        self.receive_thread = None
        self.running = False
        self.send_rate = CONFIG['movement_settings']['gcode_send_rate']
        self.test_mode = False
        self.response_callback = None
        self.grbl_mode = True  # Assume GRBL-compatible controller
        
    def get_available_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port):
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=CONFIG['serial_settings']['baudrate'],
                timeout=CONFIG['serial_settings']['timeout']
            )
            time.sleep(2)  # Wait for Arduino to reset
            
            self.connected = True
            self.running = True
            
            # Clear any startup messages
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # Send wake-up command for GRBL
            if self.grbl_mode:
                self.serial_port.write(b"\r\n\r\n")
                time.sleep(0.5)
                self.serial_port.reset_input_buffer()
            
            self.send_thread = threading.Thread(target=self._send_worker, daemon=True)
            self.send_thread.start()
            
            self.receive_thread = threading.Thread(target=self._receive_worker, daemon=True)
            self.receive_thread.start()
            
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        self.running = False
        if self.send_thread:
            self.send_thread.join(timeout=2)
        if self.receive_thread:
            self.receive_thread.join(timeout=2)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connected = False
        # Clear the queue
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except:
                break
    
    def send_command(self, command):
        if not command.strip():
            return
        self.command_queue.put(command)
    
    def _send_worker(self):
        delay = 1.0 / self.send_rate
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                if not self.test_mode and self.serial_port and self.serial_port.is_open:
                    self.serial_port.write((command + '\n').encode())
                    self.serial_port.flush()
                time.sleep(delay)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Send error: {e}")
    
    def _receive_worker(self):
        """Receive and process responses from controller"""
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        print(f"MCU: {response}")
                        if self.response_callback:
                            self.response_callback(response)
            except Exception as e:
                print(f"Receive error: {e}")
            time.sleep(0.01)

class WelcomeScreen:
    def __init__(self, screen):
        self.screen = screen
        self.font_large = pygame.font.Font(None, 56)
        self.font_medium = pygame.font.Font(None, 36)
        self.font_small = pygame.font.Font(None, 26)
        
        self.cameras = self.detect_cameras()
        self.joysticks = self.detect_joysticks()
        
        self.selected_xy_camera = self.cameras[0] if self.cameras else None
        self.selected_z_camera = self.cameras[1] if len(self.cameras) > 1 else None
        self.selected_joystick = 0 if self.joysticks else None
        
        center_x = SCREEN_WIDTH // 2
        center_y = SCREEN_HEIGHT // 2
        
        self.buttons = {
            'xy_camera': pygame.Rect(center_x - 250, center_y - 120, 500, 60),
            'z_camera': pygame.Rect(center_x - 250, center_y - 40, 500, 60),
            'joystick': pygame.Rect(center_x - 250, center_y + 40, 500, 60),
            'start': pygame.Rect(center_x - 150, center_y + 140, 300, 70)
        }
    
    def detect_cameras(self):
        cameras = []
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        for i in range(10):
            for backend in backends:
                try:
                    cap = cv2.VideoCapture(i, backend)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            cameras.append(i)
                            cap.release()
                            break
                    cap.release()
                except:
                    continue
        return cameras
    
    def detect_joysticks(self):
        return [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    
    def draw(self):
        self.screen.fill(BG_COLOR)
        title = self.font_large.render("Micron CNC Controller", True, ACCENT_COLOR)
        self.screen.blit(title, (SCREEN_WIDTH//2 - title.get_width()//2, 100))
        
        xy_text = f"XY Camera: {self.selected_xy_camera if self.selected_xy_camera is not None else 'None'}"
        z_text = f"Z Camera: {self.selected_z_camera if self.selected_z_camera is not None else 'None (optional)'}"
        joystick_text = f"Joystick: {self.joysticks[self.selected_joystick].get_name()[:30] if self.selected_joystick is not None else 'None'}"
        
        for key, rect in self.buttons.items():
            pygame.draw.rect(self.screen, PANEL_COLOR, rect, border_radius=8)
            pygame.draw.rect(self.screen, BORDER_COLOR, rect, 2, border_radius=8)
        
        xy_surf = self.font_medium.render(xy_text, True, TEXT_COLOR)
        z_surf = self.font_medium.render(z_text, True, TEXT_COLOR)
        joy_surf = self.font_medium.render(joystick_text, True, TEXT_COLOR)
        
        self.screen.blit(xy_surf, (self.buttons['xy_camera'].centerx - xy_surf.get_width()//2, self.buttons['xy_camera'].centery - xy_surf.get_height()//2))
        self.screen.blit(z_surf, (self.buttons['z_camera'].centerx - z_surf.get_width()//2, self.buttons['z_camera'].centery - z_surf.get_height()//2))
        self.screen.blit(joy_surf, (self.buttons['joystick'].centerx - joy_surf.get_width()//2, self.buttons['joystick'].centery - joy_surf.get_height()//2))
        
        start_color = SUCCESS_COLOR if (self.selected_xy_camera is not None and self.selected_joystick is not None) else BORDER_COLOR
        pygame.draw.rect(self.screen, start_color, self.buttons['start'], border_radius=10)
        start_text = self.font_large.render("START", True, PANEL_COLOR)
        self.screen.blit(start_text, (SCREEN_WIDTH//2 - start_text.get_width()//2, self.buttons['start'].centery - start_text.get_height()//2))
        
        info = self.font_small.render("Click to cycle through options", True, TEXT_COLOR)
        self.screen.blit(info, (SCREEN_WIDTH//2 - info.get_width()//2, SCREEN_HEIGHT - 100))
    
    def handle_click(self, pos):
        if self.buttons['xy_camera'].collidepoint(pos) and self.cameras:
            current_idx = self.cameras.index(self.selected_xy_camera) if self.selected_xy_camera in self.cameras else -1
            self.selected_xy_camera = self.cameras[(current_idx + 1) % len(self.cameras)]
        elif self.buttons['z_camera'].collidepoint(pos) and self.cameras:
            if self.selected_z_camera is None:
                for cam in self.cameras:
                    if cam != self.selected_xy_camera:
                        self.selected_z_camera = cam
                        break
            else:
                current_idx = self.cameras.index(self.selected_z_camera) if self.selected_z_camera in self.cameras else -1
                next_idx = (current_idx + 1) % (len(self.cameras) + 1)
                self.selected_z_camera = self.cameras[next_idx] if next_idx < len(self.cameras) else None
        elif self.buttons['joystick'].collidepoint(pos) and self.joysticks:
            self.selected_joystick = (self.selected_joystick + 1) % len(self.joysticks)
        elif self.buttons['start'].collidepoint(pos):
            if self.selected_xy_camera is not None and self.selected_joystick is not None:
                return True
        return False

class JoystickNormalizer:
    def __init__(self, joystick):
        self.joystick = joystick
        self.dead_zone = 0.15
        self.z_inverted = False  # Set to True if Z axis is inverted
    
    def normalize_axis(self, axis_index, invert=False):
        try:
            raw_value = self.joystick.get_axis(axis_index)
        except:
            return 0.0
            
        if abs(raw_value) < self.dead_zone:
            return 0.0
        
        sign = 1 if raw_value >= 0 else -1
        if invert:
            sign = -sign
            
        normalized = (abs(raw_value) - self.dead_zone) / (1.0 - self.dead_zone)
        return sign * min(normalized, 1.0)

class CameraFeed:
    def __init__(self, camera_id, label="Camera"):
        self.camera = None
        self.camera_id = camera_id
        self.label = label
        
        if camera_id is None:
            return
        
        backends = [cv2.CAP_V4L2, cv2.CAP_DSHOW, cv2.CAP_ANY]
        for backend in backends:
            try:
                self.camera = cv2.VideoCapture(camera_id, backend)
                if self.camera.isOpened():
                    ret, test_frame = self.camera.read()
                    if ret and test_frame is not None:
                        print(f"{label} {camera_id} initialized")
                        break
                self.camera.release()
            except:
                continue
        
        if not self.camera or not self.camera.isOpened():
            self.camera = None
            return
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    
    def get_frame(self):
        if not self.camera or not self.camera.isOpened():
            return None
        ret, frame = self.camera.read()
        if ret and frame is not None:
            try:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = np.rot90(frame)
                frame = pygame.surfarray.make_surface(frame)
                return pygame.transform.scale(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
            except:
                return None
        return None
    
    def release(self):
        if self.camera:
            self.camera.release()

class Button:
    def __init__(self, rect, text, active_color, text_color=TEXT_COLOR):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.active_color = active_color
        self.text_color = text_color
        self.font = pygame.font.Font(None, 20)
        self.hover = False
        self.active = False
    
    def draw(self, surface):
        if self.active:
            bg_color = tuple(min(255, c + 20) for c in self.active_color) if self.hover else self.active_color
            text_col = PANEL_COLOR
        else:
            bg_color = BUTTON_OFF_HOVER if self.hover else BUTTON_OFF
            text_col = self.text_color
        
        pygame.draw.rect(surface, bg_color, self.rect, border_radius=6)
        pygame.draw.rect(surface, BORDER_COLOR, self.rect, 1, border_radius=6)
        text_surf = self.font.render(self.text, True, text_col)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)
    
    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)
    
    def update_hover(self, pos):
        self.hover = self.rect.collidepoint(pos)

class MainWindow:
    def __init__(self, screen, xy_camera_id, z_camera_id, joystick_id):
        self.screen = screen
        self.xy_camera = CameraFeed(xy_camera_id, "XY Camera")
        self.z_camera = CameraFeed(z_camera_id, "Z Camera")
        
        self.joystick = pygame.joystick.Joystick(joystick_id)
        self.joystick.init()
        self.normalizer = JoystickNormalizer(self.joystick)
        
        self.serial_conn = SerialConnection()
        self.available_ports = self.serial_conn.get_available_ports()
        self.selected_port_idx = 0
        self.mcu_responses = []  # Store MCU responses
        self.serial_conn.response_callback = self.handle_mcu_response
        
        self.font_title = pygame.font.Font(None, 28)
        self.font_medium = pygame.font.Font(None, 24)
        self.font_small = pygame.font.Font(None, 18)
        self.font_mono = pygame.font.Font(None, 16)
        
        self.control_mode = ControlMode.ANALOG
        self.machine_state = MachineState.IDLE
        self.record_state = RecordState.NOT_RECORDING
        
        # Crosshair position in pixels on screen
        self.cursor_x = CAMERA_WIDTH / 2
        self.cursor_y = CAMERA_HEIGHT / 2
        self.cursor_z = CAMERA_HEIGHT / 2
        
        # Machine positions in mm
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        
        self.analog_speed = CONFIG['movement_settings']['default_analog_speed']
        self.step_size = CONFIG['movement_settings']['default_step_size']
        self.feed_rate = CONFIG['movement_settings']['default_feed_rate']
        
        self.xy_camera_x = LEFT_PANEL_WIDTH + 20
        self.xy_camera_y = 20
        self.z_camera_x = self.xy_camera_x + CAMERA_WIDTH + 20
        self.z_camera_y = 20
        
        # Waypoint recording
        self.recorded_path = []
        self.execution_index = 0
        
        self.gcode_commands = []
        self.max_gcode_lines = 10
        
        # Calculate button positions - fixed spacing
        btn_x = 20
        btn_y = 400
        btn_w = LEFT_PANEL_WIDTH - 40
        btn_h = 32
        spacing = 8
        
        self.buttons = {
            'jog_enable': Button((btn_x, btn_y, btn_w, btn_h), 'START JOG', SUCCESS_COLOR),
            'pause': Button((btn_x, btn_y + (btn_h + spacing), btn_w, btn_h), 'PAUSE', WARNING_COLOR),
            'stop': Button((btn_x, btn_y + (btn_h + spacing) * 2, btn_w, btn_h), 'STOP', ERROR_COLOR),
            'mode_toggle': Button((btn_x, btn_y + (btn_h + spacing) * 3, btn_w, btn_h), 'TOGGLE MODE', ACCENT_COLOR),
            'record_wp': Button((btn_x, btn_y + (btn_h + spacing) * 4, btn_w, btn_h), 'RECORD WAYPOINT', RECORD_COLOR),
            'save_wp': Button((btn_x, btn_y + (btn_h + spacing) * 5, btn_w, btn_h), 'SAVE', SUCCESS_COLOR),
            'execute_wp': Button((btn_x, btn_y + (btn_h + spacing) * 6, btn_w, btn_h), 'START EXECUTION', PLAYBACK_COLOR)
        }
        
        # Back button in right column at bottom
        back_btn_x = SCREEN_WIDTH - 220
        back_btn_y = SCREEN_HEIGHT - TERMINAL_HEIGHT - 50
        self.back_button = Button((back_btn_x, back_btn_y, 200, 35), 'BACK TO SETUP', TEXT_COLOR)
        
        # Connection controls in top right
        conn_x = SCREEN_WIDTH - 220
        conn_y = 20
        conn_w = 200
        conn_h = 30
        
        self.connection_buttons = {
            'port_select': Button((conn_x, conn_y, conn_w, conn_h), 'Select Port', ACCENT_COLOR),
            'connect': Button((conn_x, conn_y + conn_h + 5, conn_w, conn_h), 'CONNECT', SUCCESS_COLOR),
            'test_mode': Button((conn_x, conn_y + (conn_h + 5) * 2, conn_w, conn_h), 'TEST MODE', WARNING_COLOR),
            'home': Button((conn_x, conn_y + (conn_h + 5) * 3, conn_w, conn_h), 'HOME ($H)', ACCENT_COLOR),
            'unlock': Button((conn_x, conn_y + (conn_h + 5) * 4, conn_w, conn_h), 'UNLOCK ($X)', WARNING_COLOR)
        }
    
    def handle_mcu_response(self, response):
        """Handle responses from the microcontroller"""
        self.mcu_responses.append(response)
        if len(self.mcu_responses) > 10:
            self.mcu_responses.pop(0)
    
    def send_gcode(self, command):
        self.gcode_commands.append(command)
        if len(self.gcode_commands) > self.max_gcode_lines:
            self.gcode_commands.pop(0)
        
        if self.serial_conn.test_mode or self.serial_conn.connected:
            self.serial_conn.send_command(command)
        
        print(f"G-code: {command}")
    
    def start_recording(self):
        self.record_state = RecordState.RECORDING
        self.recorded_path = []
        print("★ RECORDING STARTED")
    
    def save_recording(self):
        if len(self.recorded_path) > 0:
            self.record_state = RecordState.READY_TO_EXECUTE
            print(f"★ SAVED {len(self.recorded_path)} waypoints")
        else:
            self.record_state = RecordState.NOT_RECORDING
            print("✗ No waypoints recorded")
    
    def start_execution(self):
        if self.record_state == RecordState.READY_TO_EXECUTE and len(self.recorded_path) > 0:
            self.record_state = RecordState.EXECUTING
            self.execution_index = 0
            print("→ EXECUTING WAYPOINTS")
    
    def draw_crosshair(self, x, y, camera_rect):
        color = SUCCESS_COLOR if self.control_mode == ControlMode.ANALOG else WARNING_COLOR
        
        screen_x = camera_rect.x + int(x)
        screen_y = camera_rect.y + int(y)
        
        pygame.draw.line(self.screen, color, (screen_x - 25, screen_y), (screen_x + 25, screen_y), 3)
        pygame.draw.line(self.screen, color, (screen_x, screen_y - 25), (screen_x, screen_y + 25), 3)
        pygame.draw.circle(self.screen, color, (screen_x, screen_y), 4)
        pygame.draw.circle(self.screen, color, (screen_x, screen_y), 18, 2)
    
    def draw_recorded_path(self, camera_rect):
        if len(self.recorded_path) < 2:
            return
        
        for i in range(len(self.recorded_path) - 1):
            p1 = self.recorded_path[i]
            p2 = self.recorded_path[i + 1]
            
            if self.record_state == RecordState.EXECUTING and i < self.execution_index:
                color = PLAYBACK_COLOR
            else:
                color = RECORD_COLOR
            
            x1 = camera_rect.x + int(p1[3])
            y1 = camera_rect.y + int(p1[4])
            x2 = camera_rect.x + int(p2[3])
            y2 = camera_rect.y + int(p2[4])
            
            pygame.draw.line(self.screen, color, (x1, y1), (x2, y2), 3)
            pygame.draw.circle(self.screen, color, (x1, y1), 5)
    
    def handle_analog_input(self):
        if self.machine_state != MachineState.JOGGING:
            return
        
        x_axis = self.normalizer.normalize_axis(CONFIG['joystick_mapping']['axis_x'])
        y_axis = self.normalizer.normalize_axis(CONFIG['joystick_mapping']['axis_y'])
        z_axis = -self.normalizer.normalize_axis(CONFIG['joystick_mapping']['axis_z'])  # Inverted for natural control

        # print("Debug Z:", z_axis)
        
        # Only process if there's actual movement
        if abs(x_axis) > 0 or abs(y_axis) > 0 or abs(z_axis) > 0:
            # Move crosshair
            self.cursor_x += x_axis * self.analog_speed
            self.cursor_y += y_axis * self.analog_speed
            self.cursor_z -= z_axis * self.analog_speed  # Inverted for visual feedback
            
            self.cursor_x = max(0, min(CAMERA_WIDTH, self.cursor_x))
            self.cursor_y = max(0, min(CAMERA_HEIGHT, self.cursor_y))
            self.cursor_z = max(0, min(CAMERA_HEIGHT, self.cursor_z))
            
            # Update machine position (in mm)
            delta_x = x_axis * self.analog_speed * 0.1
            delta_y = y_axis * self.analog_speed * 0.1
            delta_z = z_axis * self.analog_speed * 0.1
            
            self.pos_x += delta_x
            self.pos_y += delta_y
            self.pos_z += delta_z
            
            # Record if recording
            if self.record_state == RecordState.RECORDING:
                self.recorded_path.append((self.pos_x, self.pos_y, self.pos_z, self.cursor_x, self.cursor_y, self.cursor_z))
            
            # Send G-code only if movement is significant
            if abs(delta_x) > 0.01 or abs(delta_y) > 0.01 or abs(delta_z) > 0.01:
                self.send_gcode(CONFIG['gcode_settings']['coordinate_mode_relative'])
                self.send_gcode(f"G1 X{delta_x:.3f} Y{-delta_y:.3f} Z{delta_z:.3f} F{self.feed_rate}")
    
    def handle_step_input(self, keys, hat_motion):
        if self.machine_state != MachineState.JOGGING:
            return
        
        step = self.step_size
        moved = False
        dx, dy, dz = 0, 0, 0
        
        # XY movement with D-pad or arrow keys
        if keys[pygame.K_LEFT] or hat_motion[0] == -1:
            dx = -step
            self.cursor_x = max(0, self.cursor_x - 10)
            moved = True
        if keys[pygame.K_RIGHT] or hat_motion[0] == 1:
            dx = step
            self.cursor_x = min(CAMERA_WIDTH, self.cursor_x + 10)
            moved = True
        if keys[pygame.K_UP] or hat_motion[1] == 1:
            dy = -step
            self.cursor_y = max(0, self.cursor_y - 10)
            moved = True
        if keys[pygame.K_DOWN] or hat_motion[1] == -1:
            dy = step
            self.cursor_y = min(CAMERA_HEIGHT, self.cursor_y + 10)
            moved = True
        
        # Z movement with Page Up/Down (joystick buttons 6/7 can also be mapped)
        # if keys[pygame.K_PAGEUP]:
        #     dz = step
        #     self.cursor_z = min(CAMERA_HEIGHT, self.cursor_z + 10)  # Visual goes down when Z goes up
        #     moved = True
        # elif keys[pygame.K_PAGEDOWN]:
        #     dz = -step
        #     self.cursor_z = max(0, self.cursor_z - 10)  # Visual goes up when Z goes down
        #     moved = True
        
        if moved:
            self.pos_x += dx
            self.pos_y += dy
            self.pos_z += dz
            
            if self.record_state == RecordState.RECORDING:
                self.recorded_path.append((self.pos_x, self.pos_y, self.pos_z, self.cursor_x, self.cursor_y, self.cursor_z))
            
            self.send_gcode(CONFIG['gcode_settings']['coordinate_mode_relative'])
            self.send_gcode(f"G1 X{dx:.2f} Y{-dy:.2f} Z{dz:.2f} F{self.feed_rate}")
    
    def execute_waypoints(self):
        if self.record_state == RecordState.EXECUTING and self.execution_index < len(self.recorded_path):
            wp = self.recorded_path[self.execution_index]
            
            self.send_gcode(CONFIG['gcode_settings']['coordinate_mode_absolute'])
            self.send_gcode(f"G0 X{wp[0]:.2f} Y{wp[1]:.2f} Z{wp[2]:.2f} F{self.feed_rate}")
            
            self.pos_x = wp[0]
            self.pos_y = wp[1]
            self.pos_z = wp[2]
            self.cursor_x = wp[3]
            self.cursor_y = wp[4]
            self.cursor_z = wp[5]
            
            self.execution_index += 1
            
            if self.execution_index >= len(self.recorded_path):
                print("✓ EXECUTION COMPLETE")
                self.record_state = RecordState.READY_TO_EXECUTE
                self.execution_index = 0
    
    def draw_connection_panel(self):
        conn_x = SCREEN_WIDTH - 220
        conn_y = 20
        
        # Background panel - taller for more buttons
        panel_rect = pygame.Rect(conn_x - 10, conn_y - 10, 220, 200)
        pygame.draw.rect(self.screen, PANEL_COLOR, panel_rect, border_radius=8)
        pygame.draw.rect(self.screen, BORDER_COLOR, panel_rect, 2, border_radius=8)
        
        # Connection status
        if self.serial_conn.connected:
            status_text = "Connected"
            status_color = SUCCESS_COLOR
        elif self.serial_conn.test_mode:
            status_text = "Test Mode"
            status_color = WARNING_COLOR
        else:
            status_text = "Disconnected"
            status_color = ERROR_COLOR
        
        status_surf = self.font_small.render(status_text, True, status_color)
        self.screen.blit(status_surf, (conn_x + 100 - status_surf.get_width()//2, conn_y - 30))
        
        # Update button texts
        if self.available_ports and self.selected_port_idx < len(self.available_ports):
            port_name = self.available_ports[self.selected_port_idx].split('/')[-1]
            self.connection_buttons['port_select'].text = f"Port: {port_name}"
        else:
            self.connection_buttons['port_select'].text = "No Ports"
        
        if self.serial_conn.connected:
            self.connection_buttons['connect'].text = "DISCONNECT"
            self.connection_buttons['connect'].active = True
        else:
            self.connection_buttons['connect'].text = "CONNECT"
            self.connection_buttons['connect'].active = False
        
        self.connection_buttons['test_mode'].active = self.serial_conn.test_mode
        
        for button in self.connection_buttons.values():
            button.draw(self.screen)
    
    def draw_left_panel(self):
        pygame.draw.rect(self.screen, PANEL_COLOR, (0, 0, LEFT_PANEL_WIDTH, SCREEN_HEIGHT))
        pygame.draw.line(self.screen, BORDER_COLOR, (LEFT_PANEL_WIDTH, 0), (LEFT_PANEL_WIDTH, SCREEN_HEIGHT), 2)
        
        title = self.font_title.render("CONTROL PANEL", True, TEXT_COLOR)
        self.screen.blit(title, (LEFT_PANEL_WIDTH // 2 - title.get_width() // 2, 20))
        pygame.draw.line(self.screen, BORDER_COLOR, (20, 50), (LEFT_PANEL_WIDTH - 20, 50), 1)
        
        state_text = {
            MachineState.IDLE: ("IDLE", TEXT_COLOR),
            MachineState.JOGGING: ("JOGGING", SUCCESS_COLOR),
            MachineState.PAUSED: ("PAUSED", WARNING_COLOR)
        }
        
        y = 60
        labels = [
            ("State:", state_text[self.machine_state][0], state_text[self.machine_state][1]),
            ("Mode:", "ANALOG" if self.control_mode == ControlMode.ANALOG else "STEP", ACCENT_COLOR),
            ("X:", f"{self.pos_x:.2f} mm", TEXT_COLOR),
            ("Y:", f"{self.pos_y:.2f} mm", TEXT_COLOR),
            ("Z:", f"{self.pos_z:.2f} mm", WAYPOINT_COLOR),
            ("Speed:", f"{self.analog_speed:.1f}" if self.control_mode == ControlMode.ANALOG else f"{self.step_size:.1f} mm", TEXT_COLOR),
            ("Feed:", f"{self.feed_rate} mm/min", TEXT_COLOR)
        ]
        
        for label, value, color in labels:
            label_surf = self.font_small.render(label, True, TEXT_COLOR)
            value_surf = self.font_medium.render(value, True, color)
            self.screen.blit(label_surf, (30, y))
            self.screen.blit(value_surf, (30, y + 18))
            y += 42
        
        # Recording status
        y += 5
        if self.record_state == RecordState.RECORDING:
            status_color = RECORD_COLOR
            status_text = f"● RECORDING ({len(self.recorded_path)} pts)"
        elif self.record_state == RecordState.READY_TO_EXECUTE:
            status_color = SUCCESS_COLOR
            status_text = f"✓ READY ({len(self.recorded_path)} pts)"
        elif self.record_state == RecordState.EXECUTING:
            status_color = PLAYBACK_COLOR
            status_text = f"→ EXECUTING {self.execution_index}/{len(self.recorded_path)}"
        else:
            status_color = BORDER_COLOR
            status_text = "No recording"
        
        status_surf = self.font_small.render(status_text, True, status_color)
        self.screen.blit(status_surf, (30, y))
        
        for button in self.buttons.values():
            button.draw(self.screen)
        
        # Draw back button separately
        self.back_button.draw(self.screen)
    
    def draw_terminal(self):
        terminal_y = SCREEN_HEIGHT - TERMINAL_HEIGHT
        pygame.draw.rect(self.screen, TERMINAL_BG, (LEFT_PANEL_WIDTH, terminal_y, SCREEN_WIDTH - LEFT_PANEL_WIDTH, TERMINAL_HEIGHT))
        pygame.draw.line(self.screen, BORDER_COLOR, (LEFT_PANEL_WIDTH, terminal_y), (SCREEN_WIDTH, terminal_y), 2)
        
        title = self.font_medium.render("G-CODE TERMINAL", True, TERMINAL_TEXT)
        self.screen.blit(title, (LEFT_PANEL_WIDTH + 20, terminal_y + 10))
        
        y_offset = terminal_y + 40
        
        # Show sent commands
        for cmd in reversed(self.gcode_commands[-5:]):
            cmd_surf = self.font_mono.render(f"> {cmd}", True, SUCCESS_COLOR)
            self.screen.blit(cmd_surf, (LEFT_PANEL_WIDTH + 20, y_offset))
            y_offset += 18
        
        # Show MCU responses
        for response in reversed(self.mcu_responses[-5:]):
            resp_surf = self.font_mono.render(f"< {response}", True, ACCENT_COLOR)
            self.screen.blit(resp_surf, (LEFT_PANEL_WIDTH + 20, y_offset))
            y_offset += 18
    
    def handle_button_click(self, pos):
        # Connection buttons
        if self.connection_buttons['port_select'].is_clicked(pos):
            if self.available_ports:
                self.selected_port_idx = (self.selected_port_idx + 1) % len(self.available_ports)
        
        elif self.connection_buttons['connect'].is_clicked(pos):
            if self.serial_conn.connected:
                self.serial_conn.disconnect()
            else:
                if self.available_ports and self.selected_port_idx < len(self.available_ports):
                    port = self.available_ports[self.selected_port_idx]
                    if self.serial_conn.connect(port):
                        print(f"Connected to {port}")
                    else:
                        print(f"Failed to connect to {port}")
        
        elif self.connection_buttons['test_mode'].is_clicked(pos):
            self.serial_conn.test_mode = not self.serial_conn.test_mode
            print(f"Test mode: {'ON' if self.serial_conn.test_mode else 'OFF'}")
        
        elif self.connection_buttons['home'].is_clicked(pos):
            if self.serial_conn.connected or self.serial_conn.test_mode:
                self.send_gcode("$H")  # GRBL homing command
                print("Homing cycle started")
        
        elif self.connection_buttons['unlock'].is_clicked(pos):
            if self.serial_conn.connected or self.serial_conn.test_mode:
                self.send_gcode("$X")  # GRBL unlock command
                print("Machine unlocked")
        
        # Main control buttons
        if self.buttons['jog_enable'].is_clicked(pos):
            if self.machine_state == MachineState.JOGGING:
                self.machine_state = MachineState.IDLE
                self.send_gcode(CONFIG['gcode_settings']['coordinate_mode_absolute'])
            else:
                self.machine_state = MachineState.JOGGING
                self.send_gcode(CONFIG['gcode_settings']['coordinate_mode_relative'])
        
        elif self.buttons['pause'].is_clicked(pos):
            if self.machine_state == MachineState.JOGGING:
                self.machine_state = MachineState.PAUSED
                self.send_gcode(CONFIG['gcode_settings']['pause_command'])
            elif self.machine_state == MachineState.PAUSED:
                self.machine_state = MachineState.JOGGING
                self.send_gcode(CONFIG['gcode_settings']['resume_command'])
        
        elif self.buttons['stop'].is_clicked(pos):
            self.machine_state = MachineState.IDLE
            self.send_gcode(CONFIG['gcode_settings']['stop_command'])
            self.send_gcode(CONFIG['gcode_settings']['coordinate_mode_absolute'])
        
        elif self.buttons['mode_toggle'].is_clicked(pos):
            self.control_mode = ControlMode.STEP if self.control_mode == ControlMode.ANALOG else ControlMode.ANALOG
        
        elif self.buttons['record_wp'].is_clicked(pos):
            if self.record_state == RecordState.NOT_RECORDING:
                self.start_recording()
            elif self.record_state == RecordState.RECORDING:
                self.record_state = RecordState.NOT_RECORDING
                self.recorded_path = []
        
        elif self.buttons['save_wp'].is_clicked(pos):
            if self.record_state == RecordState.RECORDING:
                self.save_recording()
        
        elif self.buttons['execute_wp'].is_clicked(pos):
            self.start_execution()
        
        elif self.back_button.is_clicked(pos):
            return True
        
        return False
    
    def run(self):
        clock = pygame.time.Clock()
        running = True
        hat_motion = (0, 0)
        last_step_time = 0
        step_delay = 0.2
        execution_timer = 0
        
        while running:
            current_time = pygame.time.get_ticks() / 1000.0
            mouse_pos = pygame.mouse.get_pos()
            
            for button in self.buttons.values():
                button.update_hover(mouse_pos)
            for button in self.connection_buttons.values():
                button.update_hover(mouse_pos)
            self.back_button.update_hover(mouse_pos)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if self.handle_button_click(event.pos):
                        self.serial_conn.disconnect()
                        return True
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:
                        self.control_mode = ControlMode.STEP if self.control_mode == ControlMode.ANALOG else ControlMode.ANALOG
                    elif event.button == 4:
                        if self.control_mode == ControlMode.ANALOG:
                            self.analog_speed = min(10.0, self.analog_speed + 0.5)
                        else:
                            self.step_size = min(10.0, self.step_size + 0.5)
                    elif event.button == 5:
                        if self.control_mode == ControlMode.ANALOG:
                            self.analog_speed = max(0.5, self.analog_speed - 0.5)
                        else:
                            self.step_size = max(0.1, self.step_size - 0.5)
                elif event.type == pygame.JOYHATMOTION:
                    hat_motion = self.joystick.get_hat(0)
            
            keys = pygame.key.get_pressed()
            
            if self.control_mode == ControlMode.ANALOG:
                self.handle_analog_input()
            else:
                if current_time - last_step_time > step_delay:
                    if any(hat_motion) or any([keys[pygame.K_LEFT], keys[pygame.K_RIGHT], keys[pygame.K_UP], keys[pygame.K_DOWN], keys[pygame.K_PAGEUP], keys[pygame.K_PAGEDOWN]]):
                        self.handle_step_input(keys, hat_motion)
                        last_step_time = current_time
                        hat_motion = (0, 0)
            
            if self.record_state == RecordState.EXECUTING:
                execution_timer += 1
                if execution_timer >= 30:
                    self.execute_waypoints()
                    execution_timer = 0
            
            if self.machine_state == MachineState.JOGGING:
                self.buttons['jog_enable'].active = True
                self.buttons['jog_enable'].text = 'STOP JOG'
            else:
                self.buttons['jog_enable'].active = False
                self.buttons['jog_enable'].text = 'START JOG'
            
            self.buttons['pause'].active = (self.machine_state == MachineState.PAUSED)
            self.buttons['mode_toggle'].active = (self.control_mode == ControlMode.STEP)
            self.buttons['record_wp'].active = (self.record_state == RecordState.RECORDING)
            self.buttons['save_wp'].active = (self.record_state == RecordState.RECORDING)
            self.buttons['execute_wp'].active = (self.record_state == RecordState.READY_TO_EXECUTE)
            
            self.screen.fill(BG_COLOR)
            
            xy_camera_rect = pygame.Rect(self.xy_camera_x, self.xy_camera_y, CAMERA_WIDTH, CAMERA_HEIGHT)
            pygame.draw.rect(self.screen, PANEL_COLOR, xy_camera_rect)
            pygame.draw.rect(self.screen, BORDER_COLOR, xy_camera_rect, 2)
            
            xy_frame = self.xy_camera.get_frame()
            if xy_frame:
                self.screen.blit(xy_frame, (self.xy_camera_x, self.xy_camera_y))
            
            if self.record_state != RecordState.NOT_RECORDING:
                self.draw_recorded_path(xy_camera_rect)
            
            self.draw_crosshair(self.cursor_x, self.cursor_y, xy_camera_rect)
            
            xy_label = self.font_small.render("XY VIEW", True, TEXT_COLOR)
            self.screen.blit(xy_label, (self.xy_camera_x + 10, self.xy_camera_y + CAMERA_HEIGHT + 5))
            
            if self.z_camera.camera:
                z_camera_rect = pygame.Rect(self.z_camera_x, self.z_camera_y, CAMERA_WIDTH, CAMERA_HEIGHT)
                pygame.draw.rect(self.screen, PANEL_COLOR, z_camera_rect)
                pygame.draw.rect(self.screen, BORDER_COLOR, z_camera_rect, 2)
                
                z_frame = self.z_camera.get_frame()
                if z_frame:
                    self.screen.blit(z_frame, (self.z_camera_x, self.z_camera_y))
                
                self.draw_crosshair(CAMERA_WIDTH / 2, self.cursor_z, z_camera_rect)
                
                z_label = self.font_small.render("Z VIEW", True, WAYPOINT_COLOR)
                self.screen.blit(z_label, (self.z_camera_x + 10, self.z_camera_y + CAMERA_HEIGHT + 5))
            
            self.draw_left_panel()
            self.draw_connection_panel()
            self.draw_terminal()
            
            pygame.display.flip()
            clock.tick(60)
        
        self.xy_camera.release()
        self.z_camera.release()
        self.serial_conn.disconnect()
        return False

def main():
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Micron CNC Controller")
    
    welcome = WelcomeScreen(screen)
    clock = pygame.time.Clock()
    running = True
    start_main = False
    
    while running and not start_main:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if welcome.handle_click(event.pos):
                    start_main = True
        
        welcome.draw()
        pygame.display.flip()
        clock.tick(60)
    
    if start_main:
        while True:
            main_window = MainWindow(screen, welcome.selected_xy_camera, welcome.selected_z_camera, welcome.selected_joystick)
            go_back = main_window.run()
            if not go_back:
                break
    
    pygame.quit()

if __name__ == "__main__":
    main()