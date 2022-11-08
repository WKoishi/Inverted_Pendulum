import numpy as np

class RK4_Solver:

    def __init__(self, ode_func, sample_time) -> None:
        
        self.ode_func = ode_func  # 待求解的函数
        self.__sample_time = sample_time  # 采样周期
        self.__sample_time_half = sample_time / 2
        self.__t = 0

    def set_sample_time(self, sample_time):
        self.__sample_time = sample_time
        self.__sample_time_half = sample_time / 2

    def step(self, y):
        k1 = self.ode_func(self.__t, y)
        k2 = self.ode_func(self.__t + self.__sample_time_half, y + self.__sample_time_half * k1)
        k3 = self.ode_func(self.__t + self.__sample_time_half, y + self.__sample_time_half * k2)
        k4 = self.ode_func(self.__t + self.__sample_time, y + self.__sample_time * k3)
        self.__t += self.__sample_time
        
        return y + (k1 + 2*k2 + 2*k3 + k4) * self.__sample_time / 6


class EulerImprovedSolver:

    def __init__(self, ode_func, sample_time) -> None:
        
        self.ode_func = ode_func  # 待求解的函数
        self.__sample_time = sample_time  # 采样周期
        self.__sample_time_half = sample_time / 2
        self.__t = 0

    def set_sample_time(self, sample_time):
        self.__sample_time = sample_time
        self.__sample_time_half = sample_time / 2

    def step(self, y):
        k0 = y + self.__sample_time * self.ode_func(self.__t, y)
        k1 = y + self.__sample_time_half * (self.ode_func(self.__t, y) + self.ode_func(self.__t + self.__sample_time, k0))
        self.__t += self.__sample_time

        return k1







