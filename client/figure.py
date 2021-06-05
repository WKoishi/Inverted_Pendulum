import matplotlib
matplotlib.use("Qt5Agg")  # 声明使用QT5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np

class MyFigure(FigureCanvasQTAgg):

    def __init__(self, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(MyFigure,self).__init__(self.fig)

        self.BUF_MAX_LEN = 1000
        self.THETA_DEADBAND = 0.01
        self.POSI_DEADBAND = 0.01
        self.STABLE_TIMES = 100
        self.stable_count = 0

        self.x_dim = np.arange(1, self.BUF_MAX_LEN+1)
        self.target_array = np.zeros(self.BUF_MAX_LEN)
        self.posi_array = np.zeros(self.BUF_MAX_LEN)
        self.theta_array = np.zeros(self.BUF_MAX_LEN)
        self.count = 0
        self.is_stable = False
        self.is_painted = False

        self.posi_target = 0
        self.theta_target = 0

        self.axes = self.fig.add_subplot(111)


    def update_data(self, posi_target, theta, position):
        if self.posi_target != posi_target:
            self.posi_target = posi_target
            self.is_stable = False
            self.is_painted = False
            #self.count = 0
            self.stable_count = 0

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

        else:
            if not self.is_painted:
                self.axes.cla()
                self.axes.plot(self.x_dim[:self.count], self.theta_array[:self.count],
                                label='theta')
                self.axes.plot(self.x_dim[:self.count], self.posi_array[:self.count],
                                label='position')
                self.axes.plot(self.x_dim[:self.count], self.target_array[:self.count],
                                label='target')
                self.axes.legend()
                self.axes.grid()
                self.fig.canvas.draw()
                #self.fig.canvas.flush_events()
                self.is_painted = True


















