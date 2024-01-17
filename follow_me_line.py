from PyQt5 import QtCore, QtGui, QtWidgets
import sys

class Ui_MainWindow(object):
    def __init__(self):
        super().__init__()
        self.deep_violet = "background-color: #9400D3;"
        self.blue_text = "color: blue;"
        self.green_text = "color: green;"
        self.bg_blue = "background-color: blue;"
        self._translate = QtCore.QCoreApplication.translate
        self.start = None
        self.end = None

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(918, 521)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        # self.outer_frame = QtWidgets.QLabel(self.centralwidget)
        # self.outer_frame.setGeometry(QtCore.QRect(8, 28, 903, 393))
        # self.outer_frame.setObjectName("outer_frame")
        # self.outer_frame.setStyleSheet("border: 2px solid black;")

        self.header = QtWidgets.QLabel(self.centralwidget)
        self.header.setGeometry(QtCore.QRect(10, 30, 901, 71))
        self.header.setObjectName("header")
        self.header.setStyleSheet("background-color: gray;")

        self.wonbot = QtWidgets.QToolButton(self.centralwidget)
        self.wonbot.setGeometry(QtCore.QRect(20, 40, 141, 51))
        self.wonbot.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icons/wonbot_2_인쇄용-02.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.wonbot.setIcon(icon)
        self.wonbot.setIconSize(QtCore.QSize(200, 200))
        self.wonbot.setObjectName("wonbot")
        self.obstracle_icon = QtWidgets.QToolButton(self.centralwidget)
        self.obstracle_icon.setGeometry(QtCore.QRect(660, 50, 41, 41))
        self.obstracle_icon.setText("")

        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("icons/obstracle_yes.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        icon1.addPixmap(QtGui.QPixmap("icons/obstracle_no.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)

        self.obstracle_icon.setIcon(icon1)
        self.obstracle_icon.setIconSize(QtCore.QSize(200, 200))
        self.obstracle_icon.setObjectName("obstracle_icon")


        self.obstracle_alert_status = QtWidgets.QLabel(self.centralwidget)
        self.obstracle_alert_status.setGeometry(QtCore.QRect(530, 50, 121, 31))
        self.obstracle_alert_status.setObjectName("obstracle_alert_status")

        self.human_following_status = QtWidgets.QLabel(self.centralwidget)
        self.human_following_status.setGeometry(QtCore.QRect(180, 50, 171, 31))
        self.human_following_status.setObjectName("human_following_status")
        # self.human_following_status.setStyleSheet("background-color: gray;")

        self.line_following_status = QtWidgets.QLabel(self.centralwidget)
        self.line_following_status.setGeometry(QtCore.QRect(360, 50, 151, 31))
        self.line_following_status.setObjectName("line_following_status")
        # self.line_following_status.setStyleSheet("background-color: gray;")


        self.push_followme = QtWidgets.QPushButton(self.centralwidget)
        self.push_followme.setGeometry(QtCore.QRect(130, 120, 121, 41))
        self.push_followme.setObjectName("push_followme")
        self.push_followme.setStyleSheet(self.bg_blue)
        self.push_followme.clicked.connect(self.follow_me_callback)
        

        self.push_folllow_line = QtWidgets.QPushButton(self.centralwidget)
        self.push_folllow_line.setGeometry(QtCore.QRect(300, 120, 121, 41))
        self.push_folllow_line.setObjectName("push_folllow_line")
        self.push_folllow_line.setStyleSheet(self.bg_blue)
        self.push_folllow_line.clicked.connect(self.follow_line_callback)

        self.push_one = QtWidgets.QPushButton(self.centralwidget)
        self.push_one.setGeometry(QtCore.QRect(580, 210, 89, 51))
        self.push_one.setObjectName("push_one")
        self.push_one.setStyleSheet(self.deep_violet)
        self.push_one.clicked.connect(self.push_one_callback)

        self.push_three = QtWidgets.QPushButton(self.centralwidget)
        self.push_three.setGeometry(QtCore.QRect(580, 270, 89, 51))
        self.push_three.setObjectName("push_three")
        self.push_three.setStyleSheet(self.deep_violet)
        self.push_three.clicked.connect(self.push_three_callback)

        self.push_two = QtWidgets.QPushButton(self.centralwidget)
        self.push_two.setGeometry(QtCore.QRect(680, 210, 89, 51))
        self.push_two.setObjectName("push_two")
        self.push_two.setStyleSheet(self.deep_violet)
        self.push_two.clicked.connect(self.push_two_callback)

        self.push_four = QtWidgets.QPushButton(self.centralwidget)
        self.push_four.setGeometry(QtCore.QRect(680, 270, 89, 51))
        self.push_four.setObjectName("push_four")
        self.push_four.setStyleSheet(self.deep_violet)
        self.push_four.clicked.connect(self.push_four_callback)


        self.go_to_station = QtWidgets.QLabel(self.centralwidget)
        self.go_to_station.setGeometry(QtCore.QRect(630, 170, 101, 31))
        self.go_to_station.setTextFormat(QtCore.Qt.AutoText)
        self.go_to_station.setObjectName("go_to_station")
        self.go_to_station.setStyleSheet(self.blue_text)


        self.push_go = QtWidgets.QPushButton(self.centralwidget)
        self.push_go.setGeometry(QtCore.QRect(620, 340, 81, 31))
        self.push_go.setCheckable(False)
        self.push_go.setChecked(False)
        self.push_go.setAutoRepeat(False)
        self.push_go.setObjectName("push_go")
        self.push_go.setStyleSheet(self.green_text)

        self.station_rectangle = QtWidgets.QLabel(self.centralwidget)
        self.station_rectangle.setGeometry(QtCore.QRect(560, 380, 220, 31))
        self.station_rectangle.setStyleSheet("background-color: gray;")
        # self.drawStations(self.station_rectangle)

        self.start_station = QtWidgets.QLabel(self.centralwidget)
        self.start_station.setGeometry(QtCore.QRect(560, 380, 111, 31))
        self.start_station.setObjectName("start_station")
        # self.start_station.setStyleSheet(blue_text)
        
        self.end_station = QtWidgets.QLabel(self.centralwidget)
        self.end_station.setGeometry(QtCore.QRect(680, 380, 111, 31))
        self.end_station.setObjectName("end_station")
        # self.end_station.setStyleSheet(blue_text)

        self.display_img = QtWidgets.QLabel(self.centralwidget)
        self.display_img.setGeometry(QtCore.QRect(140, 210, 341, 201))
        self.display_img.setStyleSheet("border: 2px solid black; background-color: white;")
        self.display_img.setText("")
        self.display_img.setObjectName("display_img")

        self.battery = QtWidgets.QToolButton(self.centralwidget)
        self.battery.setGeometry(QtCore.QRect(720, 50, 31, 41))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("icons/battery-status.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.battery.setIcon(icon2)
        self.battery.setIconSize(QtCore.QSize(30, 30))
        self.battery.setObjectName("battery")


        self.power_off = QtWidgets.QPushButton(self.centralwidget)
        self.power_off.setGeometry(QtCore.QRect(780, 120, 61, 41))
        self.power_off.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("icons/power-off.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.power_off.setIcon(icon3)
        self.power_off.setIconSize(QtCore.QSize(40, 40))
        self.power_off.setObjectName("power_off")

        self.reset = QtWidgets.QPushButton(self.centralwidget)
        self.reset.setGeometry(QtCore.QRect(680, 120, 61, 41))
        self.reset.setText("")
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap("icons/reset.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.reset.setIcon(icon4)
        self.reset.setIconSize(QtCore.QSize(40, 40))
        self.reset.setObjectName("reset")
        self.reset.clicked.connect(self.reset_callback)


        self.resume = QtWidgets.QPushButton(self.centralwidget)
        self.resume.setGeometry(QtCore.QRect(580, 120, 61, 41))
        self.resume.setText("")
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap("icons/resume.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.resume.setIcon(icon5)
        self.resume.setIconSize(QtCore.QSize(30, 30))
        self.resume.setObjectName("resume")


        self.emergency_stop = QtWidgets.QPushButton(self.centralwidget)
        self.emergency_stop.setGeometry(QtCore.QRect(470, 120, 61, 41))
        self.emergency_stop.setText("")
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap("icons/stop-button.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.emergency_stop.setIcon(icon6)
        self.emergency_stop.setIconSize(QtCore.QSize(40, 40))
        self.emergency_stop.setObjectName("emergency_stop")


        self.dateTimeEdit = QtWidgets.QDateTimeEdit(self.centralwidget)
        self.dateTimeEdit.setGeometry(QtCore.QRect(760, 60, 151, 26))
        self.dateTimeEdit.setObjectName("dateTimeEdit")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 918, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        
        MainWindow.setWindowTitle(self._translate("MainWindow", "MainWindow"))
        self.obstracle_alert_status.setText(self._translate("MainWindow", "Obstracle Status"))
        self.human_following_status.setText(self._translate("MainWindow", "Human Follow Status Act"))
        self.line_following_status.setText(self._translate("MainWindow", "Line Follow status Act"))
        self.push_followme.setText(self._translate("MainWindow", "Follow Me"))
        self.push_folllow_line.setText(self._translate("MainWindow", "Follow Line"))
        self.push_one.setText(self._translate("MainWindow", "One"))
        self.push_three.setText(self._translate("MainWindow", "three"))
        self.push_two.setText(self._translate("MainWindow", "two"))
        self.push_four.setText(self._translate("MainWindow", "four"))
        self.go_to_station.setText(self._translate("MainWindow", "Go to Station"))
        self.push_go.setText(self._translate("MainWindow", "Go"))
        
        self.battery.setText(self._translate("MainWindow", "..."))
    
    def follow_me_callback(self):
        # print("Follow Me button clicked")
        # Add your logic for the "Follow Me" action here
        self.human_following_status.setStyleSheet("background-color: green;")
        self.line_following_status.setStyleSheet("")

    def follow_line_callback(self):
        # print("Follow Line button clicked")
        # Add your logic for the "Follow Line" action here
        self.line_following_status.setStyleSheet("background-color: green;")
        self.human_following_status.setStyleSheet("")
    
    def push_one_callback(self):
        if self.start:
            self.end = 1
            
        if self.start is None:
            self.start = 1
            self.end = None
        
        self.update_station()
        
    def push_two_callback(self):
        if self.start:
            self.end = 2

        if self.start is None:
            self.start = 2
            self.end = None
        
        self.update_station()

    def push_three_callback(self):
        if self.start:
            self.end = 3

        if self.start is None:
            self.start = 3
            self.end = None
        
        self.update_station()

    def push_four_callback(self):
        if self.start:
            self.end = 4
        
        if self.start is None:
            self.start = 4
            self.end = None
        
        self.update_station()
    
    def reset_callback(self):
        self.start = None
        self.end = None
        self.update_station()
        
    def update_station(self):
        if self.start is not None:
            self.start_station.setText(self._translate("MainWindow", "Start Station : "+str(self.start)))
        else:
            self.start_station.setText(self._translate("MainWindow", "Start Station : "))
        
        if self.end is not None:
            self.end_station.setText(self._translate("MainWindow", "End Station : "+str(self.end)))
        else:
            self.end_station.setText(self._translate("MainWindow", "End Station : "))
        

    def drawStations(self, rectangle):
        painter = QtGui.QPainter(rectangle)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        pen = QtGui.QPen(QtGui.QColor(169, 169, 169), 2, QtCore.Qt.SolidLine)
        painter.setPen(pen)
        painter.drawRect(0, 0, rectangle.width(), rectangle.height())
        painter.end()

def main():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        # Handle KeyboardInterrupt (Ctrl+C) to exit the application gracefully
        print("Application terminated by user.")
        app.quit()

if __name__ == "__main__":
    main()