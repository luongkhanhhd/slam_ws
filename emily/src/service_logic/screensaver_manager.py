from PyQt5.QtWidgets import QLabel, QWidget
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QTimer, Qt

class ScreensaverManager:
    def __init__(self, parent):
        self.parent = parent
        self.screensaver_timer = QTimer(self.parent)
        self.screensaver_timer.setInterval(15000)  # 15 giây
        self.screensaver_timer.timeout.connect(self.show_screensaver)
        self.screensaver_timer.start()

        self.screensaver_widget = QLabel(self.parent)
        self.screensaver_widget.setAlignment(Qt.AlignCenter)
        self.screensaver_widget.setStyleSheet("background-color: black;")
        self.movie = QMovie("/home/pi/byduck/service_logic/assets/chop_mat.gif")
        if self.movie.isValid():
            self.screensaver_widget.setMovie(self.movie)
        else:
            print("Error: Could not load GIF at /home/pi/byduck/service_logic/assets/chop_mat.gif")
        self.screensaver_widget.hide()

    def show_screensaver(self):
        self.screensaver_widget.setGeometry(self.parent.rect())
        self.movie.start()
        self.screensaver_widget.show()
        self.screensaver_widget.raise_()

    def hide_screensaver(self):
        self.movie.stop()
        self.screensaver_widget.hide()

    def reset_timer(self):
        self.screensaver_timer.stop()
        self.hide_screensaver()
        self.screensaver_timer.start()
