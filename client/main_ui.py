'''
Author: WKoishi \\
Creation date: 2021-05-21 \\
Description: 客户端主程序
'''

from PyQt5 import QtCore
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import QApplication, QMainWindow, QInputDialog, QMessageBox
from window import Ui_MainWindow
from protocol import MyProtocol
from ruler import RulerWidget
from visualise import ModelVisualise
from figure import MyFigure
import sys, socket
import struct

class MyWindows(QMainWindow, Ui_MainWindow, MyProtocol):
    '''主UI界面
    '''

    def __init__(self, socket, qt_lock):
        super(MyWindows, self).__init__()
        self.socket = socket
        #self.qt_lock = qt_lock

        self.setupUi(self)

        # self.setStyleSheet('background-color: rgb(240,240,255);')
        # self.setWindowOpacity(0.95)
        # self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

        self.tcp_receivers = StateReceiver(self.socket, qt_lock)
        self.ruler = RulerWidget()
        self.model_visual = ModelVisualise()
        self.model_figure = MyFigure()
        self.verticalLayout_ruler.addWidget(self.ruler)
        self.verticalLayout_visual.addWidget(self.model_visual)
        self.verticalLayout_figure.addWidget(self.model_figure)

        self.posi_target = 0
        targe_val_text = '位置目标值: {:.2f}'.format(self.posi_target)
        self.label_target_val.setText(targe_val_text)

        self.label_figure_ok.setText('')
        self.label_figure_ok_printed = False

        self.lineEdit_lqr_param_list = [self.lineEdit_lqr_fast_K1, 
                                        self.lineEdit_lqr_fast_K2,
                                        self.lineEdit_lqr_fast_K3,
                                        self.lineEdit_lqr_fast_K4,
                                        self.lineEdit_lqr_slow_K1,
                                        self.lineEdit_lqr_slow_K2,
                                        self.lineEdit_lqr_slow_K3,
                                        self.lineEdit_lqr_slow_K4]

        # 添加浮点数输入限制
        pDoubleValidator = QDoubleValidator()
        for object in self.lineEdit_lqr_param_list:
            object.setValidator(pDoubleValidator)

        # 信号连接
        self.tcp_receivers.received_signal.connect(self.received_process)
        self.tcp_receivers.terminate_signal.connect(self.tcp_receivers.stop)
        
        self.Button_connect.clicked.connect(self.pushButton_clicked)
        self.Button_disconnect.clicked.connect(self.pushButton_clicked)
        self.Button_start.clicked.connect(self.pushButton_clicked)
        self.Button_change_target.clicked.connect(self.pushButton_clicked)
        self.Button_lqr_param_read.clicked.connect(self.pushButton_clicked)
        self.Button_lqr_param_write.clicked.connect(self.pushButton_clicked)

        self.is_connect = False
        self.ip_address = ''
        self.ip_port = 9999

    def pushButton_clicked(self):
        '''按钮事件处理函数
        '''
        sender = self.sender()

        if sender == self.Button_connect:
            if not self.is_connect:
                host = '127.0.0.1'
                text, ok = QInputDialog.getText(self, '设置IP地址', 
                                        '请输入IP地址或主机名', text=host)
                if ok:
                    self.ip_address = text
                    text, ok = QInputDialog.getInt(self, '设置端口号',
                                                '请输入端口号', min=1024)
                    if ok:
                        self.ip_port = text
                        self.server_connect()

            else:
                QMessageBox.information(self, '提示', '已连接到 '\
                                +self.ip_address+':'+str(self.ip_port))

        elif sender == self.Button_disconnect:
            pass

        elif sender == self.Button_start:
            send_buf = self.mp_send_buf_pack(self.CMD_START, None)
            self.my_sendall(send_buf)

        elif sender == self.Button_change_target:
            text, ok = QInputDialog.getDouble(self, '设置目标值', '请输入目标值', 
                                                min=-4, max=4, decimals=4)
            if ok:
                send_buf = self.mp_send_buf_pack(self.CMD_CHANGE_TARGET, 
                                            struct.pack('<f', float(text)))
                self.my_sendall(send_buf)
                self.posi_target = text

                self.label_figure_ok.setText('')
                self.label_figure_ok_printed = False

        elif sender == self.Button_lqr_param_read:
            send_buf = self.mp_send_buf_pack(self.CMD_READ_C_PARAM, None)
            self.my_sendall(send_buf)

        elif sender == self.Button_lqr_param_write:
            lqr_param = []
            for object in self.lineEdit_lqr_param_list:
                text = object.text()
                if text:
                    lqr_param.append(float(text))
                else:
                    QMessageBox.information(self, '提示', '没有输入数据！')
                    return

            float_buf = struct.pack('<8f', *lqr_param)
            send_buf = self.mp_send_buf_pack(self.CMD_CHANGE_C_PARAM, float_buf)
            self.my_sendall(send_buf)
            QMessageBox.information(self, '提示', '写入成功')


    def my_sendall(self, send_buf):
        try:
            self.socket.sendall(send_buf)
        except OSError:
            QMessageBox.information(self, '提示', '连接已断开')
            self.model_figure.reset()


    def server_connect(self):
        self.socket.connect((self.ip_address, self.ip_port))
        self.is_connect = True
        self.tcp_receivers.start()

    def received_process(self):
        command = self.tcp_receivers.command

        if command == self.CMD_MODEL_STATE:
            # 更新目标值显示
            targe_val_text = '位置目标值: {:.2f}'.format(self.posi_target)
            self.label_target_val.setText(targe_val_text)

            # 更新倒立摆可视化模型
            data = self.tcp_receivers.get_recv_detail()
            position, theta = struct.unpack('<2f', data)
            self.model_visual.set_value(position, theta)
            self.ruler.set_value(position)
            self.model_figure.update_data(self.posi_target, theta, position)
            if self.model_figure.is_painted and not self.label_figure_ok_printed:
                self.label_figure_ok.setText('模型已稳定，曲线图已更新')
                self.label_figure_ok_printed = True

        elif command == self.CMD_READ_C_PARAM:
            data = self.tcp_receivers.get_recv_detail()
            lqr_param = struct.unpack('<8f', data)
            for i in range(len(self.lineEdit_lqr_param_list)):
                self.lineEdit_lqr_param_list[i].setText('{:.4f}'.format(lqr_param[i]))



    def closeEvent(self, event):
        reply = QMessageBox.question(self, '确认', '确认退出吗',
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.tcp_receivers.terminate_signal.emit()
            event.accept()
        else:
            event.ignore()


class StateReceiver(QtCore.QThread, MyProtocol):
    '''倒立摆状态（位移、摆角）接收机
    '''

    received_signal = QtCore.pyqtSignal()
    terminate_signal = QtCore.pyqtSignal()

    def __init__(self, socket, qt_lock):
        super(StateReceiver, self).__init__()
        self.socket = socket
        self.qt_lock = qt_lock
        self.command = b''
        self.receive_data = []

    def run(self):
        while 1:
            ret, length = self.mp_receive_head(self.socket)
            if ret:
                self.qt_lock.lock()
                self.receive_data = self.mp_receive_data(self.socket, length)
                if self.receive_data[0] == self.receive_data[1]:
                    self.command = self.receive_data[:2]
                    # 发射接收完成信号
                    self.qt_lock.unlock()
                    self.received_signal.emit()
                else:
                    self.qt_lock.unlock()

    def get_recv_detail(self):
        return self.receive_data[2:]

    def stop(self):
        print('receiver terminate')
        self.terminate()



if __name__ == '__main__':
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    Qt_lock = QtCore.QMutex()
    app = QApplication(sys.argv)
    main_ui = MyWindows(client, Qt_lock)
    main_ui.show()
    sys.exit(app.exec_())








