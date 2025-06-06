import os
os.environ["QT_LOGGING_RULES"] = "qt5.*=true"

import subprocess
# subprocess.Popen(["squeekboard"])  # Use Florence virtual keyboard

import sys
# Add the service_logic directory to sys.path
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

import json
import pygame
import numpy as np
import cv2  # Import OpenCV
from ultralytics import YOLO
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QWidget, QVBoxLayout
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import QTimer, Qt, pyqtSignal

# Import the original scenes from the scenes subdirectory
from scenes.main_scene import MainScene
from scenes.settings_scene import SettingsScene
from scenes.control_device_scene import ControlDeviceScene
from scenes.camera_scene import CameraScene
from scenes.updating_scene import UpdatingScene

class DuckDetector:
    def __init__(self):
        pygame.init()
        self.cap = cv2.VideoCapture(0)  # Use the correct camera ID
        if not self.cap.isOpened():
            print("Error: Could not open camera at /dev/video0")
            raise RuntimeError("Could not open camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame_width = 640
        self.frame_height = 480
        # Load YOLOv11 model from the specified path
        self.model = YOLO("/home/ubuntu/emily/src/service_logic/models/yolo11n.pt")

    def process_frame(self, draw_labels=True):
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Failed to capture frame from camera")
            return None

        # Resize frame to match video_label size (320x240)
        frame = cv2.resize(frame, (320, 240))

        # Convert frame from BGR (OpenCV) to RGB (Pygame-compatible)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run YOLOv11 inference
        results = self.model(frame_rgb)

        if draw_labels:
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    if box.cls == 0:  # Class 0 is typically "person" in COCO dataset
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = box.conf.item()
                        if confidence > 0.5:  # Confidence threshold
                            # Draw bounding box
                            cv2.rectangle(frame_rgb, (x1, y1), (x2, y2), (255, 0, 0), 2)
                            # Draw label
                            label = f"Person: {confidence:.2f}"
                            cv2.putText(frame_rgb, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Convert the frame to Pygame surface
        frame_surface = pygame.surfarray.make_surface(np.transpose(frame_rgb, (1, 0, 2)))
        return frame_surface

    def cleanup(self):
        self.cap.release()
        pygame.quit()

class EyeWidget(QWidget):
    def __init__(self, eyes_data, parent=None):
        super().__init__(parent)
        print("Initializing EyeWidget")
        self.eyes_data = eyes_data
        self.is_blinking = False
        self.setGeometry(0, 0, 800, 480)
        self.setStyleSheet("background-color: black;")  # Black background as requested

        self.blink_interval = 2000
        self.blink_duration = 200

        # Precompute positions and colors for performance
        self.eye_info = []
        self.pupil_info = []
        self.eyebrow_info = []
        self.eyelash_info = []
        self.mouth_info = None
        scale_x = 800 / 150
        scale_y = 480 / 150

        # Ensure we only process the correct number of eyes, pupils, etc.
        for layer in self.eyes_data:
            pos_x = layer["ks"]["p"]["k"][0] * scale_x
            pos_y = layer["ks"]["p"]["k"][1] * scale_y
            name = layer["nm"].lower()

            if "eye" in name and len(self.eye_info) < 2:  # Limit to 2 eyes
                outline_shape = layer["shapes"][0]
                outline_color = QColor.fromRgbF(*outline_shape["it"][0]["c"]["k"])
                if len(outline_shape["it"]) > 1:
                    fill_shape = outline_shape["it"][1]
                    fill_color = QColor.fromRgbF(*fill_shape["c"]["k"])
                else:
                    fill_color = QColor.fromRgbF(1, 1, 1)  # Default to white
                self.eye_info.append({
                    "pos_x": pos_x,
                    "pos_y": pos_y,
                    "outline_color": outline_color,
                    "fill_color": fill_color,
                    "eye_width": 30 * scale_x,
                    "eye_height": 30 * scale_y
                })
            elif "pupil" in name and len(self.pupil_info) < 2:  # Limit to 2 pupils
                pupil_shape = layer["shapes"][0]
                pupil_color = QColor.fromRgbF(*pupil_shape["it"][0]["c"]["k"])
                self.pupil_info.append({
                    "pos_x": pos_x,
                    "pos_y": pos_y,
                    "pupil_color": pupil_color,
                    "pupil_width": 15 * scale_x,
                    "pupil_height": 15 * scale_y
                })
            elif "eyebrow" in name and len(self.eyebrow_info) < 2:  # Limit to 2 eyebrows
                eyebrow_shape = layer["shapes"][0]
                eyebrow_color = QColor.fromRgbF(*eyebrow_shape["it"][0]["c"]["k"])
                self.eyebrow_info.append({
                    "pos_x": pos_x,
                    "pos_y": pos_y,
                    "eyebrow_color": eyebrow_color,
                    "eyebrow_size": 5 * scale_x
                })
            elif "eyelash" in name and len(self.eyelash_info) < 2:  # Limit to 2 eyelashes
                eyelash_shape = layer["shapes"][0]
                eyelash_color = QColor.fromRgbF(*eyelash_shape["it"][0]["c"]["k"])
                self.eyelash_info.append({
                    "pos_x": pos_x,
                    "pos_y": pos_y,
                    "eyelash_color": eyelash_color,
                    "eyelash_length": 5 * scale_x
                })
            elif "mouth" in name and self.mouth_info is None:  # Only one mouth
                mouth_shape = layer["shapes"][0]
                mouth_color = QColor.fromRgbF(*mouth_shape["it"][0]["c"]["k"])
                self.mouth_info = {
                    "pos_x": pos_x,
                    "pos_y": pos_y,
                    "mouth_color": mouth_color,
                    "mouth_width": 20 * scale_x,
                    "mouth_height": 10 * scale_y
                }

        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.toggle_blink)
        self.blink_timer.start(self.blink_interval)

        self.blink_duration_timer = QTimer(self)
        self.blink_duration_timer.timeout.connect(self.end_blink)

        self.hide()
        print("EyeWidget initialized")

    def toggle_blink(self):
        self.is_blinking = True
        self.update()
        self.blink_duration_timer.start(self.blink_duration)

    def end_blink(self):
        self.is_blinking = False
        self.update()

    def paintEvent(self, event):
        print("Painting EyeWidget")
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Explicitly draw the black background
        painter.fillRect(self.rect(), QColor(0, 0, 0))

        # Draw eyes
        for eye in self.eye_info:
            pos_x = eye["pos_x"]
            pos_y = eye["pos_y"]
            eye_width = eye["eye_width"]
            eye_height = eye["eye_height"]
            outline_color = eye["outline_color"]
            fill_color = eye["fill_color"]

            if not self.is_blinking:
                # Draw open eye (outline and fill)
                painter.setBrush(Qt.NoBrush)
                painter.setPen(outline_color)
                painter.drawEllipse(int(pos_x - eye_width // 2), int(pos_y - eye_height // 2), int(eye_width), int(eye_height))

                painter.setBrush(fill_color)
                painter.setPen(Qt.NoPen)
                painter.drawEllipse(int(pos_x - eye_width // 2), int(pos_y - eye_height // 2), int(eye_width), int(eye_height))
            else:
                # Draw closed eye (a horizontal line with slight curve)
                painter.setPen(QPen(outline_color, 2))
                painter.setBrush(Qt.NoBrush)
                painter.drawArc(int(pos_x - eye_width // 2), int(pos_y - 5), int(eye_width), 10, 0 * 16, 180 * 16)

        # Draw pupils (only when eyes are open)
        if not self.is_blinking:
            for pupil in self.pupil_info:
                pos_x = pupil["pos_x"]
                pos_y = pupil["pos_y"]
                pupil_width = pupil["pupil_width"]
                pupil_height = pupil["pupil_height"]
                pupil_color = pupil["pupil_color"]

                painter.setBrush(pupil_color)
                painter.setPen(Qt.NoPen)
                painter.drawEllipse(int(pos_x - pupil_width // 2), int(pos_y - pupil_height // 2), int(pupil_width), int(pupil_height))

        # Draw eyebrows
        for eyebrow in self.eyebrow_info:
            pos_x = eyebrow["pos_x"]
            pos_y = eyebrow["pos_y"]
            eyebrow_size = eyebrow["eyebrow_size"]
            eyebrow_color = eyebrow["eyebrow_color"]

            painter.setBrush(eyebrow_color)
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(int(pos_x - eyebrow_size // 2), int(pos_y - eyebrow_size // 2), int(eyebrow_size), int(eyebrow_size))

        # Draw eyelashes (only when eyes are open)
        if not self.is_blinking:
            for eyelash in self.eyelash_info:
                pos_x = eyelash["pos_x"]
                pos_y = eyelash["pos_y"]
                eyelash_length = eyelash["eyelash_length"]
                eyelash_color = eyelash["eyelash_color"]

                painter.setPen(eyelash_color)
                painter.setBrush(Qt.NoBrush)
                # Draw three eyelashes per eye
                painter.drawLine(int(pos_x - eyelash_length), int(pos_y), int(pos_x - eyelash_length // 2), int(pos_y - eyelash_length))
                painter.drawLine(int(pos_x), int(pos_y), int(pos_x), int(pos_y - eyelash_length))
                painter.drawLine(int(pos_x + eyelash_length), int(pos_y), int(pos_x + eyelash_length // 2), int(pos_y - eyelash_length))

        # Draw smiling mouth
        if self.mouth_info:
            pos_x = self.mouth_info["pos_x"]
            pos_y = self.mouth_info["pos_y"]
            mouth_width = self.mouth_info["mouth_width"]
            mouth_height = self.mouth_info["mouth_height"]
            mouth_color = self.mouth_info["mouth_color"]

            painter.setPen(QPen(mouth_color, 2))
            painter.setBrush(Qt.NoBrush)
            # Draw a smiling arc
            painter.drawArc(int(pos_x - mouth_width // 2), int(pos_y - mouth_height // 2), int(mouth_width), int(mouth_height), 0 * 16, -180 * 16)

    def cleanup(self):
        self.blink_timer.stop()
        self.blink_duration_timer.stop()

class ScreensaverManager:
    def __init__(self, parent, container):
        self.parent = parent
        self.container = container
        self.current_scene = None
        self.screensaver_timer = QTimer(self.parent)
        self.screensaver_timer.setInterval(15000)
        self.screensaver_timer.timeout.connect(self.show_screensaver)

        self.eyes_data = []
        try:
            with open("/home/pi/byduck/service_logic/assets/eye_animation.json", "r") as f:
                data = json.load(f)
                self.eyes_data = data["layers"]
            print("Successfully loaded eye animation data")
        except Exception as e:
            print(f"Error loading eye_animation.json: {e}")
            self.eyes_data = [
                {
                    "nm": "left eye Outlines",
                    "ks": {"p": {"k": [112.5, 75]}},
                    "shapes": [{"it": [{"c": {"k": [0, 0, 0]}}, {"c": {"k": [1, 1, 1]}}]}]
                },
                {
                    "nm": "right eye Outlines",
                    "ks": {"p": {"k": [87.5, 75]}},
                    "shapes": [{"it": [{"c": {"k": [0, 0, 0]}}, {"c": {"k": [1, 1, 1]}}]}]
                }
            ]

        self.eye_widget = EyeWidget(self.eyes_data, self.container)
        self.container.addWidget(self.eye_widget)

        self.screensaver_timer.start()
        print("Screensaver timer started")

    def show_screensaver(self):
        print("Showing screensaver")
        self.current_scene = self.container.currentWidget()
        self.container.setCurrentWidget(self.eye_widget)
        self.eye_widget.show()
        self.eye_widget.update()

    def hide_screensaver(self):
        print("Hiding screensaver")
        if self.current_scene:
            self.container.setCurrentWidget(self.current_scene)
        else:
            self.container.setCurrentWidget(self.parent.stacked_widget)
        self.eye_widget.hide()

    def reset_timer(self):
        print("Resetting screensaver timer")
        self.screensaver_timer.stop()
        self.hide_screensaver()
        self.screensaver_timer.start()

    def cleanup(self):
        self.screensaver_timer.stop()
        if self.eye_widget:
            self.eye_widget.cleanup()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        print("Initializing MainWindow")
        self.setWindowTitle("ByDuck Application")
        self.setGeometry(0, 0, 800, 480)

        self.duck_detector = DuckDetector()
        print("DuckDetector initialized")

        # Create a container widget to hold both the stacked widget and EyeWidget
        self.container = QStackedWidget(self)
        self.setCentralWidget(self.container)

        # Create the main stacked widget for scenes
        self.stacked_widget = QStackedWidget()
        self.container.addWidget(self.stacked_widget)

        # Initialize the screensaver_manager before scenes
        self.screensaver_manager = ScreensaverManager(self, self.container)
        print("ScreensaverManager initialized")

        # Use the original scenes
        self.main_scene = MainScene(self)
        print("MainScene initialized")
        self.settings_scene = SettingsScene(self)
        print("SettingsScene initialized")
        self.control_device_scene = ControlDeviceScene(self)
        print("ControlDeviceScene initialized")
        self.camera_scene = CameraScene(self, self.duck_detector)
        print("CameraScene initialized")
        self.updating_scene = UpdatingScene(self)
        print("UpdatingScene initialized")

        self.stacked_widget.addWidget(self.main_scene)
        self.stacked_widget.addWidget(self.settings_scene)
        self.stacked_widget.addWidget(self.control_device_scene)
        self.stacked_widget.addWidget(self.camera_scene)
        self.stacked_widget.addWidget(self.updating_scene)

        self.stacked_widget.setCurrentWidget(self.main_scene)

        self.main_scene.switch_to_settings.connect(self.switch_to_settings)
        self.main_scene.switch_to_control_device.connect(self.switch_to_control_device)
        self.main_scene.switch_to_camera.connect(self.switch_to_camera)
        self.settings_scene.switch_to_main.connect(self.switch_to_main)
        self.control_device_scene.switch_to_main.connect(self.switch_to_main)
        self.control_device_scene.switch_to_camera.connect(self.switch_to_camera)
        self.control_device_scene.switch_to_updating.connect(self.switch_to_updating)
        self.camera_scene.switch_to_control_device.connect(self.switch_to_control_device)
        self.updating_scene.switch_to_control_device.connect(self.switch_to_control_device)

        print("MainWindow initialized")

        # Ensure fullscreen on Raspberry Pi LCD
        self.showFullScreen()

    def switch_to_settings(self):
        print("Switching to SettingsScene")
        self.stacked_widget.setCurrentWidget(self.settings_scene)
        self.screensaver_manager.reset_timer()

    def switch_to_control_device(self):
        print("Switching to ControlDeviceScene")
        self.stacked_widget.setCurrentWidget(self.control_device_scene)
        self.screensaver_manager.reset_timer()

    def switch_to_camera(self):
        print("Switching to CameraScene")
        self.stacked_widget.setCurrentWidget(self.camera_scene)
        self.screensaver_manager.reset_timer()

    def switch_to_main(self):
        print("Switching to MainScene")
        self.stacked_widget.setCurrentWidget(self.main_scene)
        self.screensaver_manager.reset_timer()

    def switch_to_updating(self):
        print("Switching to UpdatingScene")
        self.stacked_widget.setCurrentWidget(self.updating_scene)
        self.screensaver_manager.reset_timer()

    def closeEvent(self, event):
        self.screensaver_manager.cleanup()
        self.duck_detector.cleanup()
        event.accept()

    def mousePressEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().keyPressEvent(event)

    def touchEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().touchEvent(event)

if __name__ == "__main__":
    print("Starting application")
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    print("Window shown")
    sys.exit(app.exec_())
