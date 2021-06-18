'''
Author: WKoishi \\
Creation date: 2021-05-20 \\
Description: 倒立摆控制主控制流程
'''

import numpy as np
from pid import PID_controller, Differentiator
from math import pi, cos


class TotalController:
    """
    总控制器
    """
    def __init__(self):

        self.sample_time = 0.014

        self.theta_PD_controller = PID_controller(Kp=3.392, Ki=0, Kd=0.75,
                                            output_lower_limit = -1000,
                                            output_upper_limit = 1000,
                                            sample_time = self.sample_time)

        self.posi_PD_controller = PID_controller(Kp=1, Ki=0, Kd=1.55,
                                            output_lower_limit = -1000,
                                            output_upper_limit = 1000,
                                            sample_time = self.sample_time)

        # PID/LQR切换开关
        self.LQR_CONTROL_ENABLE = 1

        self.now_position = 0
        self.now_theta = 0
        self.last_position = 0
        self.last_theta = 0

        self.posi_differ = Differentiator()
        self.theta_differ = Differentiator()
        self.posi_nabla = 0
        self.theta_nabla = 0

        # 通过LQR算法得出的状态反馈增益
        self.lqr_feedback_gain_fast = [-82.8127, -17.4970, -10.0, -12.0002]
        self.lqr_feedback_gain_slow = [-47.6602, -11.8337, -0.3162, -1.3513]
        self.is_slow_mode = False

        self.position_target = 0

        # 是否已经起摆
        self.is_swing_up = False
        self.is_impulse = 1

    def swing_up_control(self, gain, theta, theta_nabla):
        '''
        能量起摆控制器

        Return
        ------------
        力（force）输入
        '''
        kinetic_energy = (1/240) * theta_nabla**2

        potential_energy = (cos(theta)-1) * 9.8 * 0.25 * 0.1

        sign = 0
        if theta_nabla * cos(theta) >= 0:
            sign = 1
        else:
            sign = -1

        output = gain * sign * (kinetic_energy + potential_energy)

        output_max = 9.8 * 2
        if output > output_max:
            output = output_max
        elif output < -output_max:
            output = -output_max

        return output

    def lqr_control(self, target, theta, theta_nabla, posi, posi_nabla):
        """
        状态反馈LQR控制器
        """
        if self.is_slow_mode:
            feedback_gain = self.lqr_feedback_gain_slow
        else:
            feedback_gain = self.lqr_feedback_gain_fast

        # 计算状态反馈量
        feedback = theta * feedback_gain[0] \
                + theta_nabla * feedback_gain[1] \
                + posi * feedback_gain[2] \
                + posi_nabla * feedback_gain[3]

        output = target * feedback_gain[2] - feedback

        return output

    def pid_control(self, target, theta, last_theta, posi, last_posi):
        """
        PID控制器
        """
        position_gain = 0.3
        force_gain = -50

        self.posi_PD_controller.update_state(last_value=last_posi)
        self.theta_PD_controller.update_state(last_value=last_theta)

        # 外环反馈通道PD控制器
        posi_pid_output = self.posi_PD_controller.pid_cal(posi)

        # 外环前向通道增益
        theta_input = position_gain * (target - posi_pid_output)

        # 内环反馈通道PD控制器
        theta_pid_output = self.theta_PD_controller.pid_cal(theta)

        # 得到被控对象的输入（内环前向通道增益）
        output = force_gain * (theta_input - theta_pid_output)

        return output


    def runtime(self, posi_target, now_position, now_theta):
        """
        控制器主流程
        """
        # 获取当前位置并计算位置的微分
        self.last_position = self.now_position
        self.now_position = now_position
        self.posi_nabla = self.posi_differ.Sub(self.now_position, self.sample_time)

        # 获取当前角度并计算角度的微分
        self.last_theta = self.now_theta
        self.now_theta = now_theta
        self.theta_nabla = self.theta_differ.Sub(self.now_theta, self.sample_time)

        if self.is_swing_up:
            if abs(self.now_theta) > pi/6 and self.is_slow_mode == False:
                self.is_slow_mode = True
            elif abs(self.now_theta) < pi/18 and self.is_slow_mode == True:
                self.is_slow_mode = False

        # 起摆时摆角小于30°即切换成稳摆控制
        if self.is_swing_up == False and abs(self.now_theta) < pi/6:
            self.is_swing_up = True

        self.position_target = posi_target

        if self.is_swing_up:

            if self.LQR_CONTROL_ENABLE:
                force_input = self.lqr_control(self.position_target, self.now_theta, 
                                self.theta_nabla, self.now_position, self.posi_nabla)
            
            else:
                force_input = self.pid_control(self.position_target, self.now_theta, 
                                self.last_theta, self.now_position, self.last_position)

        else:

            force_input = self.swing_up_control(350, self.last_theta, self.theta_nabla)

            # 开始起摆时产生一个单位冲激信号
            if self.is_impulse:
                force_input = 1
                self.is_impulse += 1
                if self.is_impulse > 3:
                    self.is_impulse = 0

        return force_input


    def get_theta(self):
        return self.now_theta

    def get_position(self):
        return self.now_position

