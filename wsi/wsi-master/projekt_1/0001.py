#zakres [-100, 100]
#parametry: losowy punt początkowy, współczynnik uczenia
#rysunek funkcji na wykresie
#sprawko: czego dotyczy, wykresy, parametr, wnioski

import matplotlib.pyplot as plt
import numpy as np



min_x = -3.5
max_x = 2.5
eps = 0.01
option = 2
max_iter = 50
xk_list = []


def poly(x):
    if option == 1 :
        return 2*(x**2)
    elif option == 2:
        return 1.5*(x**4) + 2.2*(x**3) - 3.3*(x**2) - 1.3*x + 2.8


def derivative(x):
    if option == 1 :
        return 4*x
    elif option == 2:
        return 6*(x**3) + 6.6*(x**2) -6.6*x - 1.3

def make_plot(start, learn_rate):
    y_list = []
    points_list = []
    for x in np.arange(min_x, max_x, 0.01):
        y_list.append(poly(x))
    for x in xk_list:
        points_list.append(poly(x))
    plt.plot(np.arange(min_x, max_x, 0.01), y_list)
    plt.plot(xk_list, points_list, c='r', marker='x')
    plt.title("Punkt startowy: " + str(start) + " , Wspołczynnik uczenia się: " + str(learn_rate))
    plt.show()


def gradient_descent(start, learn_rate):
    xk = start
    for i in range(max_iter):
        xk_list.append(xk)
        d = learn_rate * derivative(xk)
        if np.all(np.abs(d) <= eps):
            break
        xk += - d
    return xk

start=-0.17
learn_rate=0.1
print(gradient_descent(start, learn_rate))
make_plot(start, learn_rate)
