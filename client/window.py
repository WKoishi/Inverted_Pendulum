# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\window.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1014, 606)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(820, 30, 161, 231))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout_button = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_button.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_button.setObjectName("verticalLayout_button")
        self.Button_connect = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.Button_connect.setObjectName("Button_connect")
        self.verticalLayout_button.addWidget(self.Button_connect)
        self.Button_disconnect = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.Button_disconnect.setObjectName("Button_disconnect")
        self.verticalLayout_button.addWidget(self.Button_disconnect)
        self.Button_start = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.Button_start.setObjectName("Button_start")
        self.verticalLayout_button.addWidget(self.Button_start)
        self.Button_change_target = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.Button_change_target.setObjectName("Button_change_target")
        self.verticalLayout_button.addWidget(self.Button_change_target)
        self.label_target_val = QtWidgets.QLabel(self.centralwidget)
        self.label_target_val.setGeometry(QtCore.QRect(20, 470, 151, 41))
        self.label_target_val.setObjectName("label_target_val")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(10, 10, 791, 451))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_5 = QtWidgets.QWidget()
        self.tab_5.setObjectName("tab_5")
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.tab_5)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(20, 20, 741, 301))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_visual = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_visual.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_visual.setObjectName("verticalLayout_visual")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.tab_5)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(20, 350, 741, 51))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_ruler = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_ruler.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_ruler.setObjectName("verticalLayout_ruler")
        self.tabWidget.addTab(self.tab_5, "")
        self.tab_6 = QtWidgets.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.verticalLayoutWidget_4 = QtWidgets.QWidget(self.tab_6)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(0, 0, 781, 421))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.verticalLayout_figure = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_figure.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_figure.setObjectName("verticalLayout_figure")
        self.tabWidget.addTab(self.tab_6, "")
        self.label_figure_ok = QtWidgets.QLabel(self.centralwidget)
        self.label_figure_ok.setGeometry(QtCore.QRect(180, 470, 231, 41))
        self.label_figure_ok.setObjectName("label_figure_ok")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1014, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.Button_connect.setText(_translate("MainWindow", "连接"))
        self.Button_disconnect.setText(_translate("MainWindow", "断开"))
        self.Button_start.setText(_translate("MainWindow", "开始"))
        self.Button_change_target.setText(_translate("MainWindow", "修改目标值"))
        self.label_target_val.setText(_translate("MainWindow", "TextLabel"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), _translate("MainWindow", "实时动画"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), _translate("MainWindow", "曲线图"))
        self.label_figure_ok.setText(_translate("MainWindow", "TextLabel"))