'''
Author: WKoishi \\
Creation date: 2021-05-31 \\
Description: 滤波器
'''

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


class AntiPeakFilter:
    '''防尖峰脉冲滤波器
    '''
    def __init__(self, max_increment, sample_time, deadband=0.1) -> None:
        self.max_Increment = (max_increment / sample_time) + (deadband / sample_time)
        self.sample_time = sample_time
        self.last_value = 0
        self.differ = Differentiator()

    def get_value(self, input_val):
        nabla = self.differ.Sub(input_val, self.sample_time)
        if abs(nabla) > self.max_Increment:
            output = self.last_value
            self.differ.last_input = self.last_value
        else:
            output = input_val
            self.last_value = input_val

        return output

        

