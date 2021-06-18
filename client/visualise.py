'''
Author: WKoishi \\
Creation date: 2021-05-21 \\
Description: 倒立摆动画绘制
'''

from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QWidget
from math import sin, cos, pi

class ModelVisualise(QWidget):
    """
    倒立摆模型可视化
    """
    def __init__(self):      
        super().__init__()
        
        self.MAX_VIEW = 8
        
        self.position = int(self.MAX_VIEW / 2)
        self.theta = pi
        self.car_size = (80, 40)  #(w,h)
        self.stick_length = 110


    def update_state(self, position, theta):
        self.position = position + int(self.MAX_VIEW / 2)
        self.theta = theta
        self.repaint()

    def paintEvent(self, event):
        qp = QPainter()
        qp.begin(self)

        size = self.size()
        Qwidth = size.width()
        Qheight = size.height()
        #width_mid = int(Qwidth/2)

        # 计算小车在绘画区域的位置坐标
        x_posi = int(((Qwidth-self.car_size[0]) / self.MAX_VIEW) * self.position)
        y_posi = Qheight - self.stick_length
        rotate_center = (int(x_posi+self.car_size[0]/2), y_posi)

        pen = QPen()

        # 绘制摆杆
        pen.setWidth(12)
        pen.setColor(QColor(114, 136, 208))
        qp.setPen(pen)
        end_point_x = rotate_center[0] - int(self.stick_length * sin(-self.theta))
        end_point_y = rotate_center[1] - int(self.stick_length * cos(-self.theta))
        qp.drawLine(rotate_center[0], rotate_center[1], end_point_x, end_point_y)

        # 绘制小车

        left_wheel_center = (x_posi + 15, y_posi + self.car_size[1])
        right_wheel_center = (x_posi + 65, y_posi + self.car_size[1])

        pen.setWidth(2)
        pen.setColor(QColor(20, 20, 20))
        qp.setPen(pen)
        qp.setBrush(QColor(0, 0, 200))
        qp.drawEllipse(left_wheel_center[0] - 7, left_wheel_center[1] - 7, 14, 14)
        qp.drawEllipse(right_wheel_center[0] - 7, right_wheel_center[1] - 7, 14, 14)

        pen.setColor(QColor(255, 255, 255))
        qp.setPen(pen)
        qp.setBrush(QColor(255, 255, 184))
        qp.drawEllipse(rotate_center[0] - 9, rotate_center[1] - 9, 18, 18)
        qp.drawRect(x_posi, y_posi, self.car_size[0], self.car_size[1])

        qp.end()





