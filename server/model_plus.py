'''
Author: WKoishi \\
Creation date: 2021-03-31 \\
Description: 倒立摆离散模型-四阶龙格库塔法迭代
'''

from math import sin, cos, pi
import numpy as np
from enum import unique, IntEnum
from ode_solver import RK4_Solver, EulerImprovedSolver

class FirstOrderInvertedPendulum:
    """一阶倒立摆离散模型

    Parameter:
    ----------
    M_car: 小车质量 \\
    M_stick: 摆杆质量 \\
    stick_length: 摆杆长度 \\
    friction: 摩擦系数 \\
    initial_theta: 摆角初始角度 \\
    sample_time: 采样时间
    """

    @unique
    class State(IntEnum):
        X = 0       # 小车位置
        DX = 1      # 小车位置一阶微分
        theta = 2   # 摆角
        Dtheta = 3  # 摆角一阶微分
        LENGTH = 4
    
    __state = np.zeros((State.LENGTH, 1), dtype=np.float64)

    def __init__(self, M_car, M_stick, stick_lenght, initial_theta, sample_time, 
                    friction=0):
        
        # 小车质量
        self.M_car = M_car

        # 摆杆质量
        self.M_stick = M_stick

        # 摆杆1/2长度
        self.l_stick = stick_lenght / 2.0

        # 摩擦系数
        self.friction = friction

        # 作用在小车上的水平方向的力
        self.input_force = 0

        # 采样时间
        self.sample_time = sample_time

        # 转动惯量J = (ml^2)/3
        J_stick = (self.M_stick * self.l_stick**2) / 3.0

        # (J + ml^2)
        self.term_stick_Jml = J_stick + self.M_stick * self.l_stick**2

        # (m^2 * l^2)
        self.term_stick_m2l2 = self.M_stick**2 * self.l_stick**2

        self.term_stick_ml = self.M_stick * self.l_stick

        self.term_M_sum = self.M_car + self.M_stick

        self.__G__ = 9.8

        # 摆角初始条件
        self.__state[self.State.theta] = initial_theta

        # 设置求解器
        self.solver = RK4_Solver(self.ode_func, self.sample_time)


    def forward(self, input_force):
        """使系统进行一次迭代

        Parameter:
        -----------
        input_force: 施加在小车上的力
        """
        self.input_force = input_force
        rk_h = self.sample_time

        # 四阶龙格库塔算法
        self.__state = self.solver.step(self.__state)

    
    def ode_func(self, t, y):
        '''模型微分方程组
        '''
        dydt = np.zeros((self.State.LENGTH, 1), dtype=np.float64)
        dydt[self.State.X, 0] = y[self.State.DX, 0]
        dydt[self.State.DX, 0] = self.__x_state_cal(self.input_force, y[self.State.DX, 0], y[self.State.theta, 0], y[self.State.Dtheta, 0])
        dydt[self.State.theta, 0] = y[self.State.Dtheta, 0]
        dydt[self.State.Dtheta, 0] = self.__theta_state_cal(self.input_force, y[self.State.DX, 0], y[self.State.theta, 0], y[self.State.Dtheta, 0])

        return dydt

    def __x_state_cal(self, input_val, x_1, theta, theta_1):
        """位移x二阶微分方程
        """
        sin_theta = sin(theta)
        cos_theta = cos(theta)

        num = self.term_stick_Jml * input_val \
            - self.term_stick_Jml * self.friction * x_1 \
            + self.term_stick_ml * self.term_stick_Jml * sin_theta * theta_1**2 \
            - self.term_stick_m2l2 * self.__G__ * sin_theta * cos_theta

        den = self.term_stick_Jml * self.term_M_sum \
            - self.term_stick_m2l2 * cos_theta**2

        return num / den

    def __theta_state_cal(self, input_val, x_1, theta, theta_1):
        """摆角theta二阶微分方程
        """
        sin_theta = sin(theta)
        cos_theta = cos(theta)

        num = self.term_stick_ml * cos_theta * input_val \
            - self.term_stick_ml * cos_theta * self.friction * x_1 \
            + self.term_stick_m2l2 * sin_theta * cos_theta * theta_1**2 \
            - self.term_M_sum * self.term_stick_ml * self.__G__ * sin_theta

        den = self.term_stick_m2l2 * cos_theta**2 \
            - self.term_M_sum * self.term_stick_Jml

        return num / den

    def get_position(self):
        """
        获取当前小车位置
        """
        return self.__state[self.State.X, 0]

    def get_theta(self):
        """
        获取当前摆角
        """
        return self.__state[self.State.theta, 0]

    def reset_param(self, M_car, M_stick, stick_lenght):
        """
        倒立摆参数重设
        """
        # 小车质量
        self.M_car = M_car

        # 摆杆质量
        self.M_stick = M_stick

        # 摆杆1/2长度
        self.l_stick = stick_lenght / 2.0

        # 转动惯量J = (ml^2)/3
        J_stick = (self.M_stick * self.l_stick**2) / 3.0

        # (J + ml^2)
        self.term_stick_Jml = J_stick + self.M_stick * self.l_stick**2

        # (m^2 * l^2)
        self.term_stick_m2l2 = self.M_stick**2 * self.l_stick**2

        self.term_stick_ml = self.M_stick * self.l_stick

        self.term_M_sum = self.M_car + self.M_stick




