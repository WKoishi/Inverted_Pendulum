'''
Author: WKoishi \\
Creation date: 2021-05-21 \\
Description: 倒立摆动画绘制
'''

from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QWidget
from math import sin, cos, pi

class ModelVisualise(QWidget):

    def __init__(self):      
        super().__init__()
        
        self.MAX_VIEW = 8
        
        self.position = int(self.MAX_VIEW / 2)
        self.theta = pi
        self.car_size = (80, 40)  #(w,h)
        self.stick_length = 110


    def set_value(self, position, theta):
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

        x_posi = int(((Qwidth-self.car_size[0]) / self.MAX_VIEW) * self.position)
        y_posi = Qheight - self.stick_length
        rotate_center = (int(x_posi+self.car_size[0]/2), y_posi)

        pen = QPen()

        pen.setWidth(12)
        pen.setColor(QColor(0, 0, 179))
        qp.setPen(pen)
        end_point_x = rotate_center[0] - int(self.stick_length * sin(-self.theta))
        end_point_y = rotate_center[1] - int(self.stick_length * cos(-self.theta))
        qp.drawLine(rotate_center[0], rotate_center[1], end_point_x, end_point_y)

        pen.setWidth(2)
        pen.setColor(QColor(255, 255, 255))
        qp.setPen(pen)
        qp.setBrush(QColor(255, 255, 184))
        qp.drawRect(x_posi, y_posi, self.car_size[0], self.car_size[1])

        qp.end()





