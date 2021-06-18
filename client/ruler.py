'''
Author: WKoishi \\
Creation date: 2021-05-21 \\
Description: 标尺绘制
'''

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QFont, QColor, QPen
import numpy as np


class RulerWidget(QWidget):

    def __init__(self):      
        super().__init__()

        self.MAX_VIEW = 8

        #self.setMinimumSize(1, 30)
        self.car_size = (80, 40)  #(w,h)
        self.value = 0
        self.num = np.array([-4,-3.5,-3,-2.5,-2,-1.5,-1,0.5,0,
                            0.5,1,1.5,2,2.5,3,3.5,4])

    def update_value(self, value):
        self.value = value
        self.repaint()


    def paintEvent(self, e):

        qp = QPainter()
        qp.begin(self)

        font = QFont('Arial', 8, QFont.Light)
        qp.setFont(font)

        size = self.size()
        Qwidth = size.width()
        Qheight = size.height()

        width_mid = int(Qwidth/2)

        posi_shift_gain = int((Qwidth-self.car_size[0]) / self.MAX_VIEW)

        # 绘制进度条

        qp.setPen(QColor(255, 255, 255))

        if self.value >= 0:
            x_delta = int(posi_shift_gain * self.value)
            x_posi = x_delta

            qp.setBrush(QColor(255, 255, 184))
            qp.drawRect(width_mid, 0, x_posi, Qheight)

        elif self.value < 0:
            x_delta = int(posi_shift_gain * abs(self.value))
            x_posi = width_mid - x_delta

            qp.setBrush(QColor(255, 184, 255))
            qp.drawRect(x_posi, 0, width_mid-x_posi, Qheight) # x,y,w,h

        # 绘制边框
        pen = QPen(QColor(20, 20, 20), 1, Qt.SolidLine)
        qp.setPen(pen)
        qp.setBrush(Qt.NoBrush)
        qp.drawRect(0, 0, Qwidth-1, Qheight-1)

        # 绘制刻度
        step = int(posi_shift_gain / 2)
        j = 0
        for posi in range(int(self.car_size[0]/2), 17*step, step):
            posi += 1
            qp.drawLine(posi, 0, posi, 5)
            metrics = qp.fontMetrics()
            fw = metrics.width(str(self.num[j]))
            qp.drawText(int(posi-fw/2), int(Qheight/2), str(self.num[j]))
            j += 1


        qp.end()



