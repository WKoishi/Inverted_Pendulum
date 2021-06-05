from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QFont, QColor, QPen
import numpy as np


class RulerWidget(QWidget):

    def __init__(self):      
        super().__init__()

        self.MAX_VIEW = 8
        self.OVER_VIEW_HALF = 3

        #self.setMinimumSize(1, 30)
        self.car_size = (80, 40)  #(w,h)
        self.value = 0
        self.num = np.array([-4,-3,-2,-1,0,1,2,3,4])

    def set_value(self, value):
        self.value = value
        self.repaint()


    def paintEvent(self, e):

        qp = QPainter()
        qp.begin(self)

        font = QFont('Serif', 8, QFont.Light)
        qp.setFont(font)

        size = self.size()
        Qwidth = size.width()
        Qheight = size.height()

        width_mid = int(Qwidth/2)

        posi_shift_gain = int((Qwidth-self.car_size[0]) / self.MAX_VIEW)

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


        pen = QPen(QColor(20, 20, 20), 1, Qt.SolidLine)

        qp.setPen(pen)
        qp.setBrush(Qt.NoBrush)
        qp.drawRect(0, 0, Qwidth-1, Qheight-1)

        step = int((Qwidth-self.car_size[0]) / 8)

        j = 0
        for i in range(int(Qwidth/18), 9*step, step):
            qp.drawLine(i, 0, i, 5)
            metrics = qp.fontMetrics()
            fw = metrics.width(str(self.num[j]))
            qp.drawText(int(i-fw/2), int(Qheight/2), str(self.num[j]))
            j = j + 1


        qp.end()



