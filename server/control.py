'''
Author: WKoishi \\
Creation date: 2021-05-20 \\
Description: 倒立摆控制主控制流程
'''

import numpy as np
from model_RK4 import FirstOrderInvertedPendulum, Differentiator
from pid import PID_controller
from math import pi, cos

class InvertedPendulumSimulator:

    def __init__(self, sample_time):

        self.sample_time = sample_time

        self.ip_model = FirstOrderInvertedPendulum(M_car=1, 
                                            M_stick=1,
                                            stick_lenght=0.6,
                                            initial_theta=pi,
                                            sample_time = sample_time)

        self.theta_PD_controller = PID_controller(Kp=3.392, Ki=0, Kd=0.75,
                                            output_lower_limit = -1000,
                                            output_upper_limit = 1000,
                                            sample_time = sample_time)

        self.posi_PD_controller = PID_controller(Kp=1, Ki=0, Kd=1.55,
                                            output_lower_limit = -1000,
                                            output_upper_limit = 1000,
                                            sample_time = sample_time)

        self.LQR_CONTROL_ENABLE = 1

        self.now_position = 0
        self.now_theta = 0
        self.last_position = 0
        self.last_theta = 0

        self.posi_differ = Differentiator()
        self.theta_differ = Differentiator()
        self.posi_nabla = 0
        self.theta_nabla = 0

        self.feedback_gain_fast = np.array([-82.8127, -17.4970, -10.0, -12.0002])
        self.feedback_gain_slow = np.array([-47.6602, -11.8337, -0.3162, -1.3513])
        self.is_slow_mode = False

        self.position_target = 0

        self.is_swing_up = False
        self.is_impulse = 1

    def swing_up(self, gain, theta, theta_nabla):
        
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
        if self.is_slow_mode:
            feedback_gain = self.feedback_gain_slow
        else:
            feedback_gain = self.feedback_gain_fast

        # 计算状态反馈量
        feedback = theta * feedback_gain[0] \
                + theta_nabla * feedback_gain[1] \
                + posi * feedback_gain[2] \
                + posi_nabla * feedback_gain[3]

        output = target * feedback_gain[2] - feedback

        return output

    def pid_control(self, target, theta, last_theta, posi, last_posi):
        position_gain = 0.3
        force_gain = -50

        self.posi_PD_controller.update_state(last_value=last_posi)
        self.theta_PD_controller.update_state(last_value=last_theta)

        # 外环反馈通道PD控制器
        posi_pid_output = self.posi_PD_controller.pid_cal(posi)

        theta_input = position_gain * (target - posi_pid_output)

        # 内环反馈通道PD控制器
        theta_pid_output = self.theta_PD_controller.pid_cal(theta)

        # 得到被控对象的输入
        output = force_gain * (theta_input - theta_pid_output)

        return output


    def runtime(self, posi_target):

        self.position_target = posi_target

        if self.is_swing_up:

            if self.LQR_CONTROL_ENABLE:
                force_input = self.lqr_control(self.position_target, self.now_theta, 
                                self.theta_nabla, self.now_position, self.posi_nabla)
            
            else:
                force_input = self.pid_control(self.position_target, self.now_theta, 
                                self.last_theta, self.now_position, self.last_position)

        else:

            force_input = self.swing_up(350, self.last_theta, self.theta_nabla)

            # 产生一个冲激信号
            if self.is_impulse:
                force_input = 1
                self.is_impulse += 1
                if self.is_impulse > 3:
                    self.is_impulse = 0

        self.ip_model.forward(force_input)

        # 获取当前位置并计算位置的微分
        self.last_position = self.now_position
        self.now_position = self.ip_model.get_position()
        self.posi_nabla = self.posi_differ.Sub(self.now_position, self.sample_time)

        # 获取当前角度并计算角度的微分
        self.last_theta = self.now_theta
        self.now_theta = self.ip_model.get_theta()
        self.theta_nabla = self.theta_differ.Sub(self.now_theta, self.sample_time)

        if self.is_swing_up:
            if abs(self.now_theta) > pi/6 and self.is_slow_mode == False:
                self.is_slow_mode = True
            elif abs(self.now_theta) < pi/18 and self.is_slow_mode == True:
                self.is_slow_mode = False

        if self.is_swing_up == False and abs(self.now_theta) < pi/6:
            self.is_swing_up = True


    def get_theta(self):
        return self.now_theta

    def get_position(self):
        return self.now_position

