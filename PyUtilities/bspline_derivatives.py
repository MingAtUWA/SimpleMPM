import os
import numpy as np
import matplotlib.pyplot as plt

def head0_der(u):
    if (u >= 0.0):
        if (u < 0.5):
            return -8.0/3.0 * u
        elif (u <= 1.5):
            return 4.0/3.0 * u - 2.0
    return 0.0

def head1_der(u):
    if (u >= -1.0):
        if (u < -0.5):
            return 8.0/3.0 * (u+1.0)
        elif (u < 0.5):
            return -7.0/3.0 * u + 1.0/6.0
        elif (u <= 1.5):
            return (2.0*u - 3.0) * 0.5
    return 0.0

def middle_der(u):
    if (u >= -1.5):
        if (u < -0.5):
            return (3.0 + 2.0*u) * 0.5
        elif (u < 0.5):
            return -2.0 * u
        elif (u < 1.5):
            return (2.0*u - 3.0) * 0.5
    return 0.0

def tail1_der(u):
    if (u >= -1.5):
        if (u < -0.5):
            return (3.0 + 2.0*u) * 0.5
        elif (u <= 0.5):
            return -7.0/3.0 * u - 1.0/6.0
        elif (u <= 1.0):
            return -8.0/3.0 * (1.0-u)
    return 0.0

def tail0_der(u):
    if (u >= -1.5):
        if (u < -0.5):
            return 4.0/3.0 * u + 2.0
        elif (u <= 0.0):
            return -8.0/3.0 * u
    return 0.0

def cal_func(func, x1, x2, u1, u2, num):
    x = np.zeros(num+1)
    y = np.zeros(num+1)
    x_div = (x2 - x1) / num
    u_div = (u2 - u1) / num
    for i in range(num+1):
        x[i] = x1 + x_div * i
        y[i] = func(u1 + u_div * i)
    return (x, y)

if __name__ == "__main__":
    fig = plt.figure()
    plot1 = fig.subplots(1, 1)
    plot1.set_xlim([0.0, 5.0])
    #plot1.set_ylim([0.0, 1.0])
    plot1.plot([0.0, 5.0], [0.0, 0.0], "k")
    # head0
    x, y = cal_func(head0_der, 0.0, 1.5, 0.0, 1.5, 100)
    plot1.plot(x, y)
    # head1
    x, y = cal_func(head1_der, 0.0, 2.5, -1.0, 1.5, 100)
    plot1.plot(x, y)
    # middle1
    x, y = cal_func(middle_der, 0.5, 3.5, -1.5, 1.5, 100)
    plot1.plot(x, y)
    # middle2
    x, y = cal_func(middle_der, 1.5, 4.5, -1.5, 1.5, 100)
    plot1.plot(x, y)
    # tail1
    x, y = cal_func(tail1_der, 2.5, 5.0, -1.5, 1.0, 100)
    plot1.plot(x, y)
    # tail0
    x, y = cal_func(tail0_der, 3.5, 5.0, -1.5, 0.0, 100)
    plot1.plot(x, y)
    plt.show()
