'''
Author: WKoishi \\
Creation date: 2021-03-31 \\
Description: 倒立摆离散模型
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
    sample_time: 采样时间 \\
    internal_iter_times: 内部迭代次数
        此参数控制着系统在一个采样时间周期里更新状态的次数，该次数越大，梯形积分的逼近程度越好，
        在一个采样周期后得到的系统输出也就更准确，但该行为会带来更大的计算量。
        默认值为4。
    '''

    def __init__(self, M_car, M_stick, stick_lenght, initial_theta, 
                    sample_time, friction=0, internal_iter_times=4):

        # 检查参数是否合法
        if internal_iter_times < 1:
            raise ValueError('internal_iter_times参数的值必须大于0')
        
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

        # 内部迭代次数
        self.__internal_iter_times = internal_iter_times
        # 梯形积分所用的真实采样时间
        self.__internal_Ts = sample_time / internal_iter_times

        self.X_car_nabla = 0  # 小车位置一阶微分
        self.X_car_nabla_2 = 0  # 小车位置二阶微分

        self.theta_nabla = 0  # 摆角一阶微分
        self.theta_nabla_2 = 0  # 摆角二阶微分

        # 转动惯量J = (ml^2)/3
        J_stick = (self.M_stick * self.l_stick**2) / 3.0

        # (J + ml^2)
        self.__term_stick_Jml = J_stick + self.M_stick * self.l_stick**2

        # (m^2 * l^2)
        self.__term_stick_m2l2 = self.M_stick**2 * self.l_stick**2

        self.__term_stick_ml = self.M_stick * self.l_stick

        self.__term_M_sum = self.M_car + self.M_stick

        self.__G = 9.8

        # 系统包含的积分器
        self.integrator_x_1 = TustinIntegrator()
        self.integrator_x_2 = TustinIntegrator()
        self.integrator_theta_1 = TustinIntegrator()
        # 给系统赋予初始摆角
        self.integrator_theta_2 = TustinIntegrator(initial_state=initial_theta)


    def forward(self, input_force):
        '''使系统进行一次迭代

        Parameter:
        -----------
        input_force: 施加在小车上的力
        '''
        self.input_force = input_force

        for _ in range(self.__internal_iter_times):

            #************* 摆角theta *****************

            sin_theta = sin(self.theta)
            cos_theta = cos(self.theta)

            # 计算摆角二阶微分方程

            num = self.__term_stick_ml * cos_theta * self.input_force \
                - self.__term_stick_ml * cos_theta * self.friction * self.X_car_nabla \
                + self.__term_stick_m2l2 * sin_theta * cos_theta * self.theta_nabla**2 \
                - self.__term_M_sum * self.__term_stick_ml * self.__G * sin_theta

            den = self.__term_stick_m2l2 * cos_theta**2 \
                - self.__term_M_sum * self.__term_stick_Jml

            self.theta_nabla_2 = num / den

            # 一次积分，输出摆角一阶微分
            self.theta_nabla = self.integrator_theta_1.Add(self.theta_nabla_2, self.__internal_Ts)
            # 二次积分，输出摆角
            self.theta = self.integrator_theta_2.Add(self.theta_nabla, self.__internal_Ts)
        
            #************** 位移X ********************

            sin_theta = sin(self.theta)
            cos_theta = cos(self.theta)

            # 计算位移二阶微分方程

            num = self.__term_stick_Jml * self.input_force \
                - self.__term_stick_Jml * self.friction * self.X_car_nabla \
                + self.__term_stick_ml * self.__term_stick_Jml * sin_theta * self.theta_nabla**2 \
                - self.__term_stick_m2l2 * self.__G * sin_theta * cos_theta

            den = self.__term_stick_Jml * self.__term_M_sum \
                - self.__term_stick_m2l2 * cos_theta**2

            self.X_car_nabla_2 = num / den

            # 一次积分，输出位置一阶微分
            self.X_car_nabla = self.integrator_x_1.Add(self.X_car_nabla_2, self.__internal_Ts)
            # 二次积分，输出位置
            self.X_car = self.integrator_x_2.Add(self.X_car_nabla, self.__internal_Ts)


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
        return self.X_car_nabla


