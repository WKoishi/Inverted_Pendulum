'''
Author: WKoishi \\
Creation date: 2021-03-31 \\
Description: 倒立摆离散模型-四阶龙格库塔法迭代
'''

from math import sin, cos, pi

class TustinIntegrator:
    '''经过双线性变换法离散化的积分器
    '''
    def __init__(self, initial_state=0):
        self.integrator_DSTATE = initial_state
        self.last_input = 0

    def Add(self, input_val, sample_time):
        '''进行一次周期的积分

        Parameter:
        ------------
        input_val: 积分输入值 \\
        sample_time: 采样时间
        '''
        self.integrator_DSTATE += (sample_time / 2.0) * (input_val + self.last_input)
        self.last_input = input_val

        return self.integrator_DSTATE


class Differentiator:
    '''一阶离散微分器
    '''
    def __init__(self):
        self.last_input = 0

    def Sub(self, input_val, sample_time):
        '''进行一次一阶差分

        Parameter:
        -----------
        input_val: 输出值 \\
        sample_time: 采样时间
        '''
        F = 1.0 / sample_time
        output = F * (input_val - self.last_input)
        self.last_input = input_val

        return output


class FirstOrderInvertedPendulum:
    '''一阶倒立摆离散模型

    Parameter:
    ----------
    M_car: 小车质量 \\
    M_stick: 摆杆质量 \\
    stick_length: 摆杆长度 \\
    friction: 摩擦系数 \\
    initial_theta: 摆角初始角度 \\
    sample_time: 采样时间
    '''

    def __init__(self, M_car, M_stick, stick_lenght, friction, initial_theta, 
                    sample_time):
        
        # 小车质量
        self.M_car = M_car

        # 摆杆质量
        self.M_stick = M_stick

        # 摆杆1/2长度
        self.l_stick = stick_lenght / 2.0

        # 摩擦系数
        self.friction = friction

        # 摆角
        self.theta = 0

        # 作用在小车上的水平方向的力
        self.input_force = 0

        # 小车的位置
        self.X_car = 0

        # 采样时间
        self.sample_time = sample_time

        self.__X_car_nabla = 0  # 小车位置一阶微分

        self.__theta_nabla = 0  # 摆角一阶微分

        # 转动惯量J = (ml^2)/3
        J_stick = (self.M_stick * self.l_stick**2) / 3.0

        # (J + ml^2)
        self.term_Jml = J_stick + self.M_stick * self.l_stick**2

        # (m^2 * l^2)
        self.term_m2l2 = self.M_stick**2 * self.l_stick**2

        self.__G__ = 9.8

        # 摆角初始条件
        self.theta = initial_theta


    def forward(self, input_force):
        '''使系统进行一次迭代

        Parameter:
        -----------
        input_force: 施加在小车上的力
        '''
        self.input_force = input_force
        rk_h = self.sample_time

        # 四阶龙格库塔算法
        # t(theta), x(position)
        # k = y'
        # l = k' = y"
        # y" = f()

        t_k1 = self.__theta_nabla
        t_l1 = self.__theta_state_cal(self.input_force, self.__X_car_nabla,
                                self.theta, self.__theta_nabla)
        
        x_k1 = self.__X_car_nabla
        x_l1 = self.__x_state_cal(self.input_force, self.__X_car_nabla, 
                                self.theta, self.__theta_nabla)

        t_k2 = self.__theta_nabla + t_l1*rk_h/2
        x_k2 = self.__X_car_nabla + x_l1*rk_h/2

        t_l2 = self.__theta_state_cal(self.input_force + rk_h/2, x_k2, 
                                self.theta + t_k1*rk_h/2, t_k2)

        x_l2 = self.__x_state_cal(self.input_force + rk_h/2, x_k2,
                                self.theta + t_k1*rk_h/2, t_k2)

        t_k3 = self.__theta_nabla + t_l2*rk_h/2
        x_k3 = self.__X_car_nabla + x_l2*rk_h/2

        t_l3 = self.__theta_state_cal(self.input_force + rk_h/2, x_k3, 
                                self.theta + t_k2*rk_h/2, t_k3)

        x_l3 = self.__x_state_cal(self.input_force + rk_h/2, x_k3, 
                                self.theta + t_k2*rk_h/2, t_k3)

        t_k4 = self.__theta_nabla + t_l3*rk_h
        x_k4 = self.__X_car_nabla + x_l3*rk_h

        t_l4 = self.__theta_state_cal(self.input_force + rk_h, x_k4, 
                                self.theta + t_k3*rk_h, t_k4)

        x_l4 = self.__x_state_cal(self.input_force + rk_h, x_k4,
                                self.theta + t_k3*rk_h, t_k4)


        self.theta += (t_k1 + 2*t_k2 + 2*t_k3 + t_k4) * rk_h / 6
        self.__theta_nabla += (t_l1 + 2*t_l2 + 2*t_l3 + t_l4) * rk_h / 6

        self.X_car += (x_k1 + 2*x_k2 + 2*x_k3 + x_k4) * rk_h / 6
        self.__X_car_nabla += (x_l1 + 2*x_l2 + 2*x_l3 + x_l4) * rk_h / 6


    def __x_state_cal(self, input_val, x_1, theta, theta_1):
        sin_theta = sin(theta)
        cos_theta = cos(theta)

        num = self.term_Jml * input_val \
            - self.term_Jml * self.friction * x_1 \
            + self.l_stick * self.M_stick * self.term_Jml * sin_theta * theta_1**2 \
            - self.term_m2l2 * self.__G__ * sin_theta * cos_theta

        den = self.term_Jml * (self.M_car + self.M_stick) \
            - self.term_m2l2 * cos_theta**2

        return num / den

    def __theta_state_cal(self, input_val, x_1, theta, theta_1):
        sin_theta = sin(theta)
        cos_theta = cos(theta)

        num = self.M_stick * self.l_stick * cos_theta * input_val \
            - self.M_stick * self.l_stick * cos_theta * self.friction * x_1 \
            + self.term_m2l2 * sin_theta * cos_theta * theta_1**2 \
            - (self.M_car + self.M_stick) * self.M_stick * self.l_stick * self.__G__ * sin_theta

        den = self.term_m2l2 * cos_theta**2 \
            - (self.M_car + self.M_stick) * self.term_Jml

        return num / den

    def get_position(self):
        '''获取当前小车位置
        '''
        return self.X_car

    def get_theta(self):
        '''获取当前摆角
        '''
        return self.theta

    def get_car_velocity(self):
        '''获取小车当前速度
        '''
        return self.__X_car_nabla


