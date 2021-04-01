from model import FirstOrderInvertedPendulum
from math import pi
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    # 建立模型
    ip_model = FirstOrderInvertedPendulum(M_car=1, 
                                        M_stick=1,
                                        stick_lenght=0.6,
                                        friction=1,
                                        initial_theta=0,
                                        sample_time = 0.002
                                        )

    # 迭代次数
    times = 10000

    posi_array = np.zeros((times,))
    theta_array = np.zeros((times,))
    velocity_array = np.zeros((times,))
    input_val = 0

    # 进行迭代
    for i in range(times):
        if i < 100:
            input_val = 1
        else:
            input_val = 0

        ip_model.forward(input_val)

        # 记录系统当前状态
        theta_array[i] = ip_model.get_theta()
        posi_array[i] = ip_model.get_position()
        velocity_array[i] = ip_model.get_car_velocity()

    x = np.arange(times)
    plt.plot(x, theta_array, label='theta')
    plt.plot(x, posi_array, label='car position')
    plt.plot(x, velocity_array, label='car velocity')
    plt.legend()
    plt.grid()
    plt.show()



