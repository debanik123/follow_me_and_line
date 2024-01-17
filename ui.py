import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtGui import QPixmap

class WonbotControlUI(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):
        # Set up the main layout
        layout = QVBoxLayout()

        # Add image to the layout
        wonbot_image = QLabel(self)
        pixmap = QPixmap("logo/wonbot_2_인쇄용-02.png")  # Replace with your image file path
        wonbot_image.setPixmap(pixmap)
        layout.addWidget(wonbot_image)

        # Follow Me button
        follow_me_button = QPushButton('Follow Me', self)
        follow_me_button.clicked.connect(self.follow_me_clicked)
        layout.addWidget(follow_me_button)

        # Follow Line button
        follow_line_button = QPushButton('Follow Line', self)
        follow_line_button.clicked.connect(self.follow_line_clicked)
        layout.addWidget(follow_line_button)

        # Status label
        self.status_label = QLabel('Mode: Idle', self)
        layout.addWidget(self.status_label)

        # Set the main layout
        self.setLayout(layout)

        # Set window properties
        self.setWindowTitle('Wonbot Control')
        self.setGeometry(100, 100, 400, 400)

    def follow_me_clicked(self):
        self.status_label.setText('Mode: Follow Me')
        # Add your code to handle "Follow Me" functionality here

    def follow_line_clicked(self):
        self.status_label.setText('Mode: Follow Line')
        # Add your code to handle "Follow Line" functionality here


if __name__ == '__main__':
    app = QApplication(sys.argv)
    wonbot_ui = WonbotControlUI()
    wonbot_ui.show()
    sys.exit(app.exec_())
