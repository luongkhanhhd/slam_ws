from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPixmap, QFont, QColor, QPainter, QBrush, QPen
from PyQt5.QtWidgets import QGraphicsDropShadowEffect

class CustomButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setFixedSize(180, 60)
        self.setFont(QFont("Arial", 12, QFont.Bold))  # Bold font for button text
        self.setStyleSheet("""
            QPushButton {
                background-color: #632bfe;  /* Purple background for buttons */
                color: white;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #5a29e5;  /* Slightly lighter purple on hover */
            }
            QPushButton:pressed {
                background-color: #4f23cc;  /* Darker purple when pressed */
            }
        """)
        # Glow effect on hover
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

class MainScene(QWidget):
    switch_to_settings = pyqtSignal()
    switch_to_control_device = pyqtSignal()
    switch_to_camera = pyqtSignal()
    switch_to_idle = pyqtSignal()

    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.screensaver_manager = main_window.screensaver_manager  # Reference to MainWindow's screensaver_manager
        self.init_ui()

    def init_ui(self):
        # Main layout
        main_layout = QVBoxLayout()

        # Title "ByDuck Assistance" with black color
        title_label = QLabel("ByDuck Assistance")
        title_label.setFont(QFont("Arial", 30, QFont.Bold))
        title_label.setStyleSheet("color: black;")  # Black color for title
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)

        # Content layout (image on the left, buttons on the right)
        content_layout = QHBoxLayout()

        # Add spacing to push buttons to the left
        content_layout.addSpacing(50)  # Push buttons to the left

        # Left side: Buttons in 2 vertical columns, each with 3 buttons
        buttons_container = QHBoxLayout()

        # Column 1 (3 buttons)
        column1_layout = QVBoxLayout()
        control_button = CustomButton("Control Devices")
        control_button.clicked.connect(self.switch_to_control_device.emit)
        column1_layout.addWidget(control_button)

        delivery_button = CustomButton("Delivery")
        column1_layout.addWidget(delivery_button)

        cruise_button = CustomButton("Cruise")
        column1_layout.addWidget(cruise_button)

        buttons_container.addLayout(column1_layout)

        # Column 2 (3 buttons)
        column2_layout = QVBoxLayout()
        patrolling_button = CustomButton("Patrolling")
        column2_layout.addWidget(patrolling_button)

        birthday_button = CustomButton("Birthday Greeting")
        column2_layout.addWidget(birthday_button)

        settings_button = CustomButton("Settings")
        settings_button.clicked.connect(self.switch_to_settings.emit)
        column2_layout.addWidget(settings_button)

        buttons_container.addLayout(column2_layout)

        content_layout.addLayout(buttons_container)

        # Add spacing to push the image to the right
        content_layout.addSpacing(50)  # Push image to the right

        # Right side: Display robot_home.jpg (smaller by 20%)
        image_label = QLabel()
        pixmap = QPixmap("/home/pi/byduck/service_logic/assets/robot_home.jpg")
        if not pixmap.isNull():
            # Original size scaled to 400x480, now reduce by 20% (320x384)
            image_label.setPixmap(pixmap.scaled(320, 384, Qt.KeepAspectRatio))
        else:
            image_label.setText("Image not found")
        image_label.setAlignment(Qt.AlignCenter)
        content_layout.addWidget(image_label)

        main_layout.addLayout(content_layout)
        self.setLayout(main_layout)

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