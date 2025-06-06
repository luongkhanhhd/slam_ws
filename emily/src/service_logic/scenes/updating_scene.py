from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtGui import QFont, QIcon
from PyQt5.QtCore import pyqtSignal, Qt

class UpdatingScene(QWidget):
    switch_to_control_device = pyqtSignal()

    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.screensaver_manager = self.main_window.screensaver_manager  # Reference to MainWindow's screensaver_manager
        self.init_ui()

    def init_ui(self):
        self.setStyleSheet("background-color: #632bfe;")

        layout = QVBoxLayout()

        title_label = QLabel("Updating")
        title_label.setFont(QFont("Arial", 20, QFont.Bold))
        title_label.setStyleSheet("color: white;")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        back_button = QPushButton()
        back_button.setFixedSize(50, 50)
        back_button.setStyleSheet("background-color: red; border-radius: 5px;")
        back_button.setIcon(QIcon("/home/pi/byduck/service_logic/assets/back_top.png"))
        back_button.clicked.connect(self.switch_to_control_device.emit)
        layout.addWidget(back_button, alignment=Qt.AlignLeft)

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