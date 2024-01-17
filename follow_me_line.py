from PyQt5 import QtCore, QtGui, QtWidgets
import sys

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.wonbot = QtWidgets.QToolButton(self.centralwidget)
        self.wonbot.setGeometry(QtCore.QRect(20, 40, 121, 51))
        self.wonbot.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icons/wonbot_2_인쇄용-02.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.wonbot.setIcon(icon)
        self.wonbot.setIconSize(QtCore.QSize(200, 200))
        self.wonbot.setObjectName("wonbot")
        self.obstracle_icon = QtWidgets.QToolButton(self.centralwidget)
        self.obstracle_icon.setGeometry(QtCore.QRect(510, 50, 41, 41))
        self.obstracle_icon.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("icons/obstracle_yes.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon1.addPixmap(QtGui.QPixmap("icons/obstracle_no.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.obstracle_icon.setIcon(icon1)
        self.obstracle_icon.setIconSize(QtCore.QSize(200, 200))
        self.obstracle_icon.setObjectName("obstracle_icon")
        self.obstracle_alert_status = QtWidgets.QLabel(self.centralwidget)
        self.obstracle_alert_status.setGeometry(QtCore.QRect(346, 56, 161, 31))
        self.obstracle_alert_status.setObjectName("obstracle_alert_status")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.obstracle_alert_status.setText(_translate("MainWindow", "Obstracle Alert Status"))


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