'''
Author: WKoishi \\
Creation date: 2021-04-01 \\
Description: PID控制器
'''

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
        output = (input_val - self.last_input) / sample_time
        self.last_input = input_val

        return output


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




