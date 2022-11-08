import numpy as np
import matplotlib.pyplot as plt
import model_RK4
import model_plus
import math

iteration = 1000

if __name__ == '__main__':

    input_force = 0
    sample_time = 0.014
    surplus_time = 0.014
    ip_model_rk4 = model_RK4.FirstOrderInvertedPendulum(M_car = 1, 
                                M_stick = 1,
                                stick_lenght = 0.6,
                                initial_theta = math.pi / 1.5,
                                sample_time = sample_time)

    ip_model_plus = model_plus.FirstOrderInvertedPendulum(M_car = 1, 
                                M_stick = 1,
                                stick_lenght = 0.6,
                                initial_theta = math.pi / 1.5,
                                sample_time = sample_time)

    theta_array_rk4 = np.zeros((iteration))
    posi_array_rk4 = np.zeros((iteration))

    theta_array_plus = np.zeros((iteration))
    posi_array_plus = np.zeros((iteration))

    for i in range(iteration):

        ip_model_rk4.forward(0)
        ip_model_plus.forward(0)

        # theta_array_rk4[i] = ip_model_rk4.get_theta()
        # posi_array_rk4[i] = ip_model_rk4.get_position()

        theta_array_plus[i] = ip_model_plus.get_theta()
        posi_array_plus[i] = ip_model_plus.get_position()

    x = np.arange(iteration)
    plt.plot(x, theta_array_plus, label='theta')
    plt.plot(x, posi_array_plus, label='car position')
    #plt.plot(x, force_array, label='force')
    plt.legend()
    plt.grid()
    plt.show()

    theta_error = np.linalg.norm(theta_array_plus - theta_array_rk4)
    posi_error = np.linalg.norm(posi_array_plus - posi_array_rk4)

    print(theta_error, posi_error)

    np.savetxt("state_rk4.csv", np.vstack((posi_array_plus, theta_array_plus)), delimiter=',')


