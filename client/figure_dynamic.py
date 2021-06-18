'''
Author: WKoishi \\
Creation date: 2021-06-10 \\
Description: pyqtgraph动态曲线图绘制
'''

import pyqtgraph as pg
import numpy as np

class DynamicFigure(pg.PlotWidget):

    def __init__(self):
        super(DynamicFigure, self).__init__()

        self.setBackground('w')

        self.BUF_MAX_LEN = 1000  # 数组最大长度
        self.THETA_DEADBAND = 0.01  # 稳定精度
        self.POSI_DEADBAND = 0.01
        self.STABLE_TIMES = 100  # 稳定判断所需连续稳定的次数
        self.stable_count = 0  # 连续稳定的次数

        self.x_dim = np.arange(1, self.BUF_MAX_LEN+1)
        self.target_array = np.zeros(self.BUF_MAX_LEN)
        self.posi_array = np.zeros(self.BUF_MAX_LEN)
        self.theta_array = np.zeros(self.BUF_MAX_LEN)
        self.count = 0    # 数组索引
        self.is_stable = False  # 模型稳定标志
        self.is_painted = False

        self.posi_target = 0
        self.theta_target = 0

        self.addLegend()
        self.target_curve = self.getPlotItem().plot(
            pen = pg.mkPen('g', width=2), name='target (m)'
        )
        self.posi_curve = self.getPlotItem().plot(
            pen = pg.mkPen('r', width=2), name='position (m)'
        )
        self.theta_curve = self.getPlotItem().plot(
            pen = pg.mkPen('b', width=2), name='theta (rad)'
        )

        self.showGrid(x=True, y=True)


    def update_data(self, posi_target, theta, position):
        """
        曲线图更新数据
        """
        # 目标值发生变化
        if self.posi_target != posi_target:
            self.posi_target = posi_target
            self.is_stable = False
            self.is_painted = False
            self.stable_count = 0

        # 只有当模型未达到目标值时才绘制曲线图
        if not self.is_stable:
            if self.count < self.BUF_MAX_LEN:
                self.theta_array[self.count] = theta
                self.posi_array[self.count] = position
                self.target_array[self.count] = self.posi_target
                self.count += 1

            elif self.count == self.BUF_MAX_LEN:
                self.theta_array = np.roll(self.theta_array, -1)
                self.posi_array = np.roll(self.posi_array, -1)
                self.target_array = np.roll(self.target_array, -1)
                self.theta_array[-1] = theta
                self.posi_array[-1] = position
                self.target_array[-1] = self.posi_target
            
            else:
                print('缓冲区错误')

            if abs(position - self.posi_target) < self.POSI_DEADBAND and \
                abs(theta - 0) < self.THETA_DEADBAND:
                self.stable_count += 1
            else:
                self.stable_count = 0

            if self.stable_count >= self.STABLE_TIMES:
                self.is_stable = True

            # 进行曲线绘制
            self.target_curve.setData(self.x_dim[:self.count], self.target_array[:self.count])
            self.posi_curve.setData(self.x_dim[:self.count], self.posi_array[:self.count])
            self.theta_curve.setData(self.x_dim[:self.count], self.theta_array[:self.count])





