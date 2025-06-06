from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSlider, QComboBox, QLineEdit, QScrollArea
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont, QIcon
import subprocess
import os
import glob

class CustomLineEdit(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)

class SettingsScene(QWidget):
    switch_to_main = pyqtSignal()

    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.screensaver_manager = self.main_window.screensaver_manager  # Reference to MainWindow's screensaver_manager
        self.backlight_path = self.find_backlight_path()
        self.current_menu = "Basic Settings"
        self.wifi_list = []
        self.init_ui()

    def find_backlight_path(self):
        backlight_dirs = glob.glob("/sys/class/backlight/*/brightness")
        if backlight_dirs:
            return backlight_dirs[0]
        return None

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

        title_label = QLabel("Settings")
        title_label.setFont(QFont("Arial", 30, QFont.Bold))
        title_label.setStyleSheet("color: black;")
        title_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(title_label, 1)

        layout.addLayout(header_layout)

        layout.addSpacing(10)

        settings_layout = QHBoxLayout()

        menu_layout = QVBoxLayout()

        menu_label = QLabel("Menu")
        menu_label.setFont(QFont("Arial", 14, QFont.Bold))
        menu_label.setStyleSheet("color: black;")
        menu_layout.addWidget(menu_label)

        self.menu_buttons = {}
        menu_items = ["Basic Settings", "WLAN", "Map Settings", "Volume Settings", "Voice Settings", "Speech Settings", "Tray Settings"]
        for item in menu_items:
            menu_button = QPushButton(item)
            menu_button.setFixedSize(150, 40)
            if item == self.current_menu:
                menu_button.setStyleSheet("""
                    QPushButton {
                        background-color: #632bfe;
                        color: white;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #5a29e5;
                    }
                    QPushButton:pressed {
                        background-color: #4f23cc;
                    }
                """)
            else:
                menu_button.setStyleSheet("""
                    QPushButton {
                        background-color: #d3d3d3;
                        color: black;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #5a29e5;
                    }
                    QPushButton:pressed {
                        background-color: #4f23cc;
                    }
                """)
            menu_button.clicked.connect(lambda checked, i=item: self.switch_menu(i))
            menu_layout.addWidget(menu_button)
            self.menu_buttons[item] = menu_button

        settings_layout.addLayout(menu_layout)

        self.middle_layout = QVBoxLayout()

        self.wifi_form_widget = QWidget()
        self.wifi_form_layout = QVBoxLayout()

        wifi_label = QLabel("Wi-Fi Settings")
        wifi_label.setFont(QFont("Arial", 14))
        wifi_label.setStyleSheet("color: black;")
        self.wifi_form_layout.addWidget(wifi_label)

        ssid_label = QLabel("SSID:")
        ssid_label.setFont(QFont("Arial", 12))
        ssid_label.setStyleSheet("color: black;")
        self.wifi_form_layout.addWidget(ssid_label)

        self.ssid_input = CustomLineEdit()
        self.ssid_input.setFixedHeight(30)
        self.ssid_input.setStyleSheet("background-color: white; color: black; border: 1px solid gray;")
        self.wifi_form_layout.addWidget(self.ssid_input)

        password_label = QLabel("Password:")
        password_label.setFont(QFont("Arial", 12))
        password_label.setStyleSheet("color: black;")
        self.wifi_form_layout.addWidget(password_label)

        self.password_input = CustomLineEdit()
        self.password_input.setFixedHeight(30)
        self.password_input.setStyleSheet("background-color: white; color: black; border: 1px solid gray;")
        self.password_input.setEchoMode(QLineEdit.Password)
        self.wifi_form_layout.addWidget(self.password_input)

        connect_button = QPushButton("Connect")
        connect_button.setFixedSize(100, 40)
        connect_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        connect_button.clicked.connect(self.connect_wifi)
        self.wifi_form_layout.addWidget(connect_button)

        self.status_label = QLabel("")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setStyleSheet("color: green;")
        self.wifi_form_layout.addWidget(self.status_label)

        self.wifi_form_widget.setLayout(self.wifi_form_layout)
        self.wifi_form_widget.hide()

        self.volume_widget = QWidget()
        volume_layout = QVBoxLayout()

        volume_label = QLabel("Volume Settings")
        volume_label.setFont(QFont("Arial", 14))
        volume_label.setStyleSheet("color: black;")
        volume_layout.addWidget(volume_label)

        self.volume_slider = QSlider(Qt.Horizontal)
        self.volume_slider.setMinimum(0)
        self.volume_slider.setMaximum(100)
        self.volume_slider.setValue(50)
        self.volume_slider.valueChanged.connect(self.adjust_volume)
        volume_layout.addWidget(self.volume_slider)

        self.volume_widget.setLayout(volume_layout)
        self.volume_widget.hide()

        self.wlan_widget = QWidget()
        wlan_layout = QVBoxLayout()

        wlan_label = QLabel("Available Wi-Fi Networks")
        wlan_label.setFont(QFont("Arial", 14))
        wlan_label.setStyleSheet("color: black;")
        wlan_layout.addWidget(wlan_label)

        self.wifi_scroll = QScrollArea()
        self.wifi_scroll.setWidgetResizable(True)
        self.wifi_scroll_content = QWidget()
        self.wifi_scroll_layout = QVBoxLayout(self.wifi_scroll_content)
        self.wifi_scroll.setWidget(self.wifi_scroll_content)
        wlan_layout.addWidget(self.wifi_scroll)

        self.wlan_widget.setLayout(wlan_layout)
        self.wlan_widget.hide()

        self.placeholder_widget = QWidget()
        placeholder_layout = QVBoxLayout()
        placeholder_label = QLabel("Đang cập nhật...")
        placeholder_label.setFont(QFont("Arial", 14))
        placeholder_label.setStyleSheet("color: black;")
        placeholder_layout.addWidget(placeholder_label)
        self.placeholder_widget.setLayout(placeholder_layout)
        self.placeholder_widget.hide()

        self.middle_layout.addWidget(self.wifi_form_widget)
        self.middle_layout.addWidget(self.volume_widget)
        self.middle_layout.addWidget(self.wlan_widget)
        self.middle_layout.addWidget(self.placeholder_widget)
        settings_layout.addLayout(self.middle_layout)

        self.right_widget = QWidget()
        self.right_layout = QVBoxLayout()

        brightness_label = QLabel("Brightness Settings")
        brightness_label.setFont(QFont("Arial", 14))
        brightness_label.setStyleSheet("color: black;")
        self.right_layout.addWidget(brightness_label)

        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setMinimum(0)
        self.brightness_slider.setMaximum(100)
        self.brightness_slider.setValue(50)
        self.brightness_slider.valueChanged.connect(self.adjust_brightness)
        self.right_layout.addWidget(self.brightness_slider)

        self.right_layout.addSpacing(20)

        multilingual_label = QLabel("Multilingual Settings")
        multilingual_label.setFont(QFont("Arial", 14))
        multilingual_label.setStyleSheet("color: black;")
        self.right_layout.addWidget(multilingual_label)

        self.language_combo = QComboBox()
        self.language_combo.addItems(["English", "Spanish", "French", "Vietnamese"])
        self.right_layout.addWidget(self.language_combo)

        self.right_widget.setLayout(self.right_layout)
        settings_layout.addWidget(self.right_widget)

        layout.addLayout(settings_layout)

        layout.addStretch()

        self.setLayout(layout)

        self.switch_menu("Basic Settings")

    def switch_menu(self, item):
        self.current_menu = item

        for menu_item, button in self.menu_buttons.items():
            if menu_item == self.current_menu:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #632bfe;
                        color: white;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #5a29e5;
                    }
                    QPushButton:pressed {
                        background-color: #4f23cc;
                    }
                """)
            else:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #d3d3d3;
                        color: black;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #5a29e5;
                    }
                    QPushButton:pressed {
                        background-color: #4f23cc;
                    }
                """)

        self.wifi_form_widget.hide()
        self.volume_widget.hide()
        self.wlan_widget.hide()
        self.placeholder_widget.hide()
        self.right_widget.hide()
        self.status_label.setText("")

        if item == "Basic Settings":
            self.right_widget.show()
        elif item == "WLAN":
            self.show_wifi_list()
            self.wlan_widget.show()
        elif item == "Volume Settings":
            self.volume_widget.show()
        elif item in ["Map Settings", "Voice Settings", "Speech Settings", "Tray Settings"]:
            self.placeholder_widget.show()

    def show_wifi_list(self):
        for i in reversed(range(self.wifi_scroll_layout.count())):
            widget = self.wifi_scroll_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()

        try:
            result = subprocess.run(["nmcli", "-f", "SSID,SECURITY", "device", "wifi", "list"], capture_output=True, text=True)
            wifi_lines = result.stdout.splitlines()[1:]
            self.wifi_list = []
            for line in wifi_lines:
                if line.strip():
                    parts = line.split()
                    ssid = " ".join(parts[:-1])
                    security = parts[-1] if parts[-1] in ["WPA1", "WPA2", "WEP"] else "None"
                    self.wifi_list.append((ssid, security))
        except Exception as e:
            print(f"Error scanning Wi-Fi: {e}")
            self.wifi_list = [("Error scanning Wi-Fi", "None")]

        for ssid, security in self.wifi_list:
            wifi_button = QPushButton(f"{ssid} ({'Khóa' if security != 'None' else 'Không'})")
            wifi_button.setFixedHeight(40)
            wifi_button.setStyleSheet("""
                QPushButton {
                    background-color: #f0f0f0;
                    color: black;
                    border-radius: 5px;
                    text-align: left;
                    padding: 5px;
                }
                QPushButton:hover {
                    background-color: #e0e0e0;
                }
                QPushButton:pressed {
                    background-color: #d0d0d0;
                }
            """)
            wifi_button.clicked.connect(lambda checked, s=ssid, sec=security: self.show_wifi_form(s, sec))
            self.wifi_scroll_layout.addWidget(wifi_button)

    def show_wifi_form(self, ssid, security):
        self.wlan_widget.hide()
        self.wifi_form_widget.show()
        self.ssid_input.setText(ssid)
        self.ssid_input.setReadOnly(True if security != "None" else False)
        self.password_input.setText("")
        self.status_label.setText("")

    def connect_wifi(self):
        ssid = self.ssid_input.text()
        password = self.password_input.text()

        if not ssid:
            print("SSID cannot be empty")
            return

        if self.ssid_input.isReadOnly() and not password:
            print("Password cannot be empty for secured Wi-Fi")
            return

        try:
            wifi_config = f"""
ctrl_interface=DIR=/var/run/ wpa_supplicant GROUP=netdev
update_config=1
country=VN

network={{
    ssid="{ssid}"
    psk="{password}"
    key_mgmt=WPA-PSK
}}
"""
            with open("/tmp/wpa_supplicant.conf", "w") as f:
                f.write(wifi_config)

            subprocess.run(["sudo", "mv", "/tmp/wpa_supplicant.conf", "/etc/wpa_supplicant/wpa_supplicant.conf"])
            subprocess.run(["sudo", "systemctl", "restart", "dhcpcd"])
            print(f"Connect thành công")
            self.status_label.setText(f"Đang kết nối đến Wi-Fi: {ssid}")
        except Exception as e:
            print(f"Error connecting to Wi-Fi: {e}")
            self.status_label.setText("Kết nối thất bại")

    def adjust_brightness(self):
        if not self.backlight_path:
            print("Error: No backlight device found in /sys/class/backlight/")
            return

        brightness_value = self.brightness_slider.value()
        backlight_value = int(brightness_value * 255 / 100)

        try:
            with open(self.backlight_path, "w") as f:
                f.write(str(backlight_value))
        except Exception as e:
            print(f"Error adjusting brightness: {e}")

    def adjust_volume(self):
        volume_value = self.volume_slider.value()

        try:
            subprocess.run(["amixer", "set", "PCM", f"{volume_value}%"])
        except Exception as e:
            print(f"Error adjusting volume: {e}")

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

    def closeEvent(self, event):
        event.accept()