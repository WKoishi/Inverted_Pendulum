import numpy as np
import matplotlib.pyplot as plt
from control import InvertedPendulumSimulator
import socket

sample_time = 0.005

iteration = 3000

if __name__ == '__main__':

    simulator = InvertedPendulumSimulator(sample_time)

    theta_array = np.zeros((iteration))
    posi_array = np.zeros((iteration))

    for i in range(iteration):

        simulator.runtime(0)

        theta_array[i] = simulator.get_theta()
        posi_array[i] = simulator.get_position()

    x = np.arange(iteration)
    plt.plot(x, theta_array, label='theta')
    plt.plot(x, posi_array, label='car position')
    #plt.plot(x, force_array, label='force')
    plt.legend()
    plt.grid()
    plt.show()


