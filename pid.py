'''
Author: WKoishi \\
Creation date: 2021-04-01 \\
Description: PID控制器
'''

from model_RK4 import TustinIntegrator, Differentiator

class PID_controller:
    '''位置式PID控制器

    Parameter:
    -----------
    Kp, Ki, Kd: 比例、积分、微分参数 \\
    output_lower_limit: 输出下限值 \\
    output_upper_limit: 输出上限值 \\
    sample_time: 采样时间，即PID控制器的控制频率
    '''

    def __init__(self, Kp, Ki, Kd, 
                    output_lower_limit, output_upper_limit, sample_time):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # 输出下限
        self.output_lower_limit = output_lower_limit

        # 输出上限
        self.output_upper_limit = output_upper_limit

        # 采样时间
        self.sample_time = sample_time

        # # 目标值
        # self.target = 0

        # # 上一拍的输出（传感器的数值）
        # self.sensor_val = 0

        # 积分器
        self.integrator = TustinIntegrator()

        # 微分器
        self.differentiator = Differentiator()

    def pid_cal(self, input_val):
        '''进行一次PID计算，输出PID控制量
        '''
        integral = 0
        differential = 0

        if self.Ki:
            integral = self.integrator.Add(input_val, self.sample_time)

        differential = self.differentiator.Sub(input_val, self.sample_time)

        output = self.Kp * input_val + self.Ki * integral + self.Kd * differential

        # 输出限幅
        if output > self.output_upper_limit:
            output = self.output_upper_limit
        elif output < self.output_lower_limit:
            output = self.output_lower_limit

        return output

    def update_state(self, integral=0, last_value=0):
        
        self.integrator.integrator_DSTATE = integral
        self.differentiator.last_input = last_value




