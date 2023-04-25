from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


def f(x, y):
    a = 1
    b = 100
    return (a - x)**2 + b*(y - x**2)**2

# def f(x):
#     a = 1
#     b = 100
#     return (a - x[0])**2 + b*(x[1] - x[0]**2)**2
best_indv_list = [[np.array([-2.39193531]), np.array([8.54836191])], [np.array([-2.69206981]), np.array([7.83681872])], [np.array([-2.69206981]), np.array([7.83681872])]]
x_list = []
y_list = []
z_list = []

for indv in best_indv_list:
    x_list.append(indv[0])
    y_list.append(indv[1])
    z_list.append(f(indv[0], indv[1]))

x = np.linspace(-5, 5, 2)
y = np.linspace(-5, 5, 2)

X, Y = np.meshgrid(x, y)
Z = f(X, Y)
fig = plt.figure()
ax = plt.axes(projection='3d')
# ax.plot_surface(X, Y, Z, rstride=1, cstride=1,cmap='viridis',edgecolor='none')
ax.scatter(x_list, y_list, z_list)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()



