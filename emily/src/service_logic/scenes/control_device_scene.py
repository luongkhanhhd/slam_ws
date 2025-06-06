from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtGui import QIcon, QFont, QColor
from PyQt5.QtWidgets import QGraphicsDropShadowEffect
from PyQt5.QtCore import pyqtSignal, Qt

class CustomButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setFixedSize(180, 60)
        self.setFont(QFont("Arial", 12, QFont.Bold))
        self.setStyleSheet("""
            QPushButton {
                background-color: #632bfe;
                color: white;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #5a29e5;
            }
            QPushButton:pressed {
                background-color: #4f23cc
            }
        """)
        self.glow_effect = QGraphicsDropShadowEffect(self)
        self.glow_effect.setColor(QColor(255, 255, 255, 200))
        self.glow_effect.setBlurRadius(15)
        self.glow_effect.setOffset(0, 0)
        self.setGraphicsEffect(self.glow_effect)

    def enterEvent(self, event):
        self.glow_effect.setEnabled(True)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.glow_effect.setEnabled(False)
        super().leaveEvent(event)

class ControlDeviceScene(QWidget):
    switch_to_main = pyqtSignal()
    switch_to_camera = pyqtSignal()
    switch_to_updating = pyqtSignal()

    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.screensaver_manager = self.main_window.screensaver_manager  # Reference to MainWindow's screensaver_manager
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        header_layout = QHBoxLayout()

        back_button = QPushButton()
        back_button.setFixedSize(100, 50)
        back_button.setStyleSheet("background-color: red; border-radius: 5px; color: white;")
        back_button.setIcon(QIcon("/home/pi/byduck/service_logic/assets/back_top.png"))
        back_button.setText("Back")
        back_button.clicked.connect(self.switch_to_main.emit)
        header_layout.addWidget(back_button)

        title_label = QLabel("Control Devices")
        title_label.setFont(QFont("Arial", 30, QFont.Bold))
        title_label.setStyleSheet("color: black;")
        title_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(title_label, 1)

        layout.addLayout(header_layout)

        layout.addSpacing(20)

        buttons_container = QHBoxLayout()

        column1_layout = QVBoxLayout()
        camera_button = CustomButton("Camera")
        camera_button.clicked.connect(self.switch_to_camera.emit)
        column1_layout.addWidget(camera_button)

        light_button = CustomButton("Light")
        light_button.clicked.connect(self.switch_to_updating.emit)
        column1_layout.addWidget(light_button)

        ac_button = CustomButton("Air Conditioner")
        ac_button.clicked.connect(self.switch_to_updating.emit)
        column1_layout.addWidget(ac_button)

        buttons_container.addLayout(column1_layout)

        buttons_container.addSpacing(20)

        column2_layout = QVBoxLayout()
        fan_button = CustomButton("Fan")
        fan_button.clicked.connect(self.switch_to_updating.emit)
        column2_layout.addWidget(fan_button)

        water_heater_button = CustomButton("Water Heater")
        water_heater_button.clicked.connect(self.switch_to_updating.emit)
        column2_layout.addWidget(water_heater_button)

        buttons_container.addLayout(column2_layout)

        layout.addLayout(buttons_container)
        layout.addStretch()

        self.setLayout(layout)

    def mousePressEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().mouseReleaseEvent(event)

    def touchEvent(self, event):
        self.screensaver_manager.reset_timer()
        super().touchEvent(event)