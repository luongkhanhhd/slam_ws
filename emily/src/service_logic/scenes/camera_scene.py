import os
os.environ["QT_LOGGING_RULES"] = "qt5.*=false"
os.environ["OPENCV_VIDEOIO_BACKEND"] = "FFMPEG"

import numpy as np
from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QVBoxLayout
from PyQt5.QtCore import pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.Qt import Qt as QtGui_Qt
#from picamera2 import Picamera2
import cv2

class CameraScene(QWidget):
    switch_to_control_device = pyqtSignal()

    def __init__(self, parent=None, duck_detector=None):
        super().__init__(parent)
        self.detector = duck_detector  # Keep for compatibility, but not used
        self.picam2 = None
        self.init_camera()
        self.init_ui()
        self.start_camera()
        print("CameraScene initialized")

    def init_camera(self):
        try:
            print("Attempting to initialize camera with Picamera2...")
            self.picam2 = Picamera2()
            print("Picamera2 object created")
            config = self.picam2.create_preview_configuration(main={"size": (320, 240)})
            print("Preview configuration created:", config)
            self.picam2.configure(config)
            print("Camera configured")
            self.picam2.start()
            print("Camera started successfully with Picamera2")
        except Exception as e:
            print(f"Error initializing camera: {str(e)}")
            self.picam2 = None

    def init_ui(self):
        layout = QVBoxLayout(self)

        self.title_label = QLabel("Camera View", self)
        self.title_label.setStyleSheet("font-size: 36px; color: black;")
        self.title_label.setAlignment(QtGui_Qt.AlignCenter)
        layout.addWidget(self.title_label)

        self.video_label = QLabel(self)
        self.video_label.setFixedSize(320, 240)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 16px;")
        self.video_label.setAlignment(QtGui_Qt.AlignCenter)
        self.video_label.setText("Loading camera...")
        layout.addWidget(self.video_label, alignment=QtGui_Qt.AlignCenter)

        self.back_button = QPushButton("< Back", self)
        self.back_button.setStyleSheet("background-color: red; color: white; font-size: 24px;")
        self.back_button.setFixedSize(100, 40)
        self.back_button.clicked.connect(self.on_back_clicked)
        layout.addWidget(self.back_button)

    def start_camera(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Update every 30ms (~33 FPS)

    def update_frame(self):
        if self.picam2 is None:
            print("CameraScene: Camera not initialized")
            self.video_label.setText("Error: Camera not available")
            return

        try:
            # Capture frame from picamera2
            frame = self.picam2.capture_array()
            # Convert RGB (picamera2) to BGR (OpenCV)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # Convert BGR to RGB for display in PyQt5
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Convert to QImage for PyQt5 display
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(image)

            self.video_label.setPixmap(pixmap)
            self.video_label.setText("")
            print("CameraScene: Frame displayed")
        except Exception as e:
            print(f"CameraScene: Error displaying frame: {e}")
            self.video_label.setText("Error: Failed to display frame")

    def on_back_clicked(self):
        print("Back button pressed")
        self.switch_to_control_device.emit()

    def closeEvent(self, event):
        if self.picam2 is not None:
            self.picam2.stop()
            print("CameraScene: Camera stopped")
        print("CameraScene: Cleanup called (handled in MainWindow)")
        event.accept()
