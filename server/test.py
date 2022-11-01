import numpy as np
import matplotlib.pyplot as plt
from model_RK4 import FirstOrderInvertedPendulum
import math

iteration = 3000

if __name__ == '__main__':

    input_force = 0
    sample_time = 0.014
    surplus_time = 0.014
    ip_model = FirstOrderInvertedPendulum(M_car = 1, 
                                M_stick = 1,
                                stick_lenght = 0.6,
                                initial_theta = math.pi / 6,
                                sample_time = sample_time)

    theta_array = np.zeros((iteration))
    posi_array = np.zeros((iteration))

    for i in range(iteration):

        ip_model.forward(0)

        theta_array[i] = ip_model.get_theta()
        posi_array[i] = ip_model.get_position()

    x = np.arange(iteration)
    plt.plot(x, theta_array, label='theta')
    plt.plot(x, posi_array, label='car position')
    #plt.plot(x, force_array, label='force')
    plt.legend()
    plt.grid()
    plt.show()


