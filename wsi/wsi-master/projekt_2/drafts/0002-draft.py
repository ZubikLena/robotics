#algorytm ewolucyjny

import numpy as np
import matplotlib.pyplot as plt
import math
import random
from copy import deepcopy

#wymiar
dimension = 1
#liczba iteracji
max_iter = 100
#wielkość populacji inicjalnej
P_size = 20
#typ populacji inicjalnej sterowany center i dev rozkłady normalnego
P_type = 1
#rozmiar turnieju
Comp_player_num = 2
#rozmiar elity
Elite_size = 1
#alfa do krzyżowania (rekombinacji)
alpha = 0.1
#siła mutacji
sigma = 1
#prawdopodobieństwo mutacji
Pm = 1/10
#- funkcja celu (nie jako parametr ale w jednym miejscu na podst. tablicy parametrów)
assesment_f_type = 1

    
def f(x):
    return x**2
    
def generate_herd(P_size):
    herd = []
    mem = np.random.random()*10
    for i in range (0, P_size):
        herd.append(mem)
    return herd


def get_score(herd, func):
    score = []
    for elem in herd:
        score.append((elem, func(elem)))

    x_best, o_best = get_best(score)
   
    return list(score), x_best, o_best

def get_best(herd_with_scores):
    x_best = herd_with_scores[0][0]
    o_best = herd_with_scores[0][1]
    for elem in herd_with_scores:
        if elem[1] < o_best:
            o_best = elem[1]
            x_best = elem[0]
    return x_best, o_best

def get_worst(herd_with_scores):
    x_best = herd_with_scores[0][0]
    o_best = herd_with_scores[0][1]
    for elem in herd_with_scores:
        if elem[1] > o_best:
            o_best = elem[1]
            x_best = elem[0]
    return x_best, o_best

def reproduction(herd_with_scores):
    """Reprodukcja turniejowa wybieramy dwóch ze zbioru z powtarzaniem i lepszego z nich"""
    descendants = []
    for i in range(1, P_size):
        rnd_index_1 = random.randrange(19)
        rnd_index_2 = random.randrange(19)
        if herd_with_scores[rnd_index_1][1] >= herd_with_scores[rnd_index_1][1]:
            descendants.append(herd_with_scores[rnd_index_1])
        else:
            descendants.append(herd_with_scores[rnd_index_2])
    return descendants

def recombination(herd_with_scores):
    """
    child_1 = alfa * parent_1 + (1 - alfa) * parent_2
    child_2 = alfa * parent_2 + (1 - alfa) * parent_1
    """


def mutation(herd_with_scores, sigma):
    new_herd = []
    for elem in herd_with_scores:
        mem,score = elem
        mem += sigma * np.random.normal(0.0, 1.0, dimension)
        new_herd.append(mem)

    return new_herd

def succession(P_scores, M_scores):
    P_next = []
    for i in range (0, Elite_size):
        best = get_best(P_scores)
        P_next.append(best)
        P_scores.remove(best)
    for elem in M_scores:
        P_next.append(elem)
    for i in range (0, Elite_size):
        worst = get_worst(P_next)
        P_next.remove(worst)
    return P_next
    


x_best = 0.0
o_best = 0.0
P = generate_herd(P_size)
i = 0
P_scores,x_best,o_best = get_score(P, f)
x_best_list = []

while (i <= max_iter):
    x_best_list.append(x_best)

    R = reproduction(P_scores)
    # R = recombination()
    M = mutation(R, sigma)
    M_scores,x_local_best,o_local_best = get_score(M, f)
    
    if o_local_best <= o_best:
        o_best = o_local_best
        x_best = x_local_best
    
    P_scores = succession(P_scores, M_scores)
    
    i += 1


x = np.linspace(-100, 100,100)
plt.plot(x, f(x))
y = []
for x in x_best_list:
    y.append(f(x))
plt.plot(x_best_list, y, '-xr')
plt.show()
print("x_worst", x_best_list[0],"x_best", x_best)









