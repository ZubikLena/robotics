import numpy as np
import matplotlib.pyplot as plt
from operator import itemgetter


#liczba iteracji
max_iter = 500
#wielkość populacji inicjalnej
pop_size = 10
#typ populacji 1-clones // 2-independent
pop_type = 2
#rozmiar turnieju
tournament_size = 0
#rozmiar elity
elite_size = 10
#siła rekombinacji
alpha = 0.1
#siła mutacji
sigma = 2.0
#prawdopodobieństwo mutacji
pm = 0.2
#typ wykresu 1-best // 2-avg
plot_type = 2


def f1(x,y):
    a = 1
    b = 100
    return (a - x)**2 + b*(y - x**2)**2

def f2(x,y):
    return (np.sin(x)*np.e**(1 - np.cos(y))**2) + (np.cos(y)*np.e**(1 - np.sin(x))**2) + (x - y)**2


def generate_init_population(pop_size):
    center = 0
    dev = 5
    population = []

    if pop_type == 1:
        #clones
        individual = [np.random.normal(center, dev, 1), np.random.normal(center, dev, 1)]
        for j in range(0, pop_size):
            population.append(individual)

        return population
    elif pop_type == 2:
        #independent
        for j in range(0, pop_size):
            individual = [np.random.normal(center, dev, 1), np.random.normal(center, dev, 1)]
            population.append(individual)
        
        return population

def get_scores(population):
    scored_population = []
    for indv in population:
        score = f2(indv[0], indv[1])
        scored_population.append([indv, score])
    return scored_population

def get_best(scored_population):
    best_local_indv = scored_population[0][0]
    best_local_score = scored_population[0][1]

    for indv in scored_population:
        if indv[1] < best_local_score:
            best_local_indv = indv[0]
            best_local_score = indv[1]
    
    return best_local_indv, best_local_score

def get_worst(scored_population):
    worst_local_indv = scored_population[0][0]
    worst_local_score = scored_population[0][1]

    for indv in scored_population:
        if indv[1] > worst_local_score:
            worst_local_indv = indv[0]
            worst_local_score = indv[1]
    
    return worst_local_indv, worst_local_score

def reproduction(scored_population):
    descendants = []
    if tournament_size > 0:
        for j in range(0, pop_size):
            index_list = []
            for n in range(0, tournament_size):
                index_list.append(np.random.randint(0, pop_size))
            parents = []
            for index in index_list:
                parents.append(scored_population[index])
    else:
        descendants = remove_scores(scored_population)
    
    return descendants


def crossing(descendants, alpha):
    crossed_descendants = []
    while len(crossed_descendants) < pop_size:
        index1 = np.random.randint(0, pop_size)
        index2 = np.random.randint(0, pop_size)
        parent1 = descendants[index1]
        parent2 = descendants[index2]

        crossed_descendants.append([alpha*parent1[0] + (1 - alpha)*parent2[0], alpha*parent1[1] + (1 - alpha)*parent2[1]])
        crossed_descendants.append([(1 - alpha)*parent1[0] + alpha*parent2[0], (1 - alpha)*parent1[1] + alpha*parent2[1]])
        
    return crossed_descendants

def mutation(descendants, sigma):
    mutated_descendants = []
    for dscnt in descendants:
        if np.random.uniform(0, 1.0) < pm:
            mutated_descendants.append([dscnt[0] + sigma*np.random.normal(0,1,1), dscnt[1] + sigma*np.random.normal(0,1,1)])
        else:
            mutated_descendants.append(dscnt)

    return mutated_descendants

def succession(scored_population, scored_descendants, elite_size):
    succ_population = []
    for j in range(0, elite_size):
        elite_member = get_best(scored_population)
        succ_population.append(list(elite_member))
        index = scored_population.index(list(elite_member))
        scored_population.pop(index)
    for dscnt in scored_descendants:
        succ_population.append(dscnt)
    for j in range(0, elite_size):
        excluded_member = get_worst(succ_population)
        index = succ_population.index(list(excluded_member))
        succ_population.pop(index)

    return remove_scores(succ_population), succ_population

def remove_scores(scored_population):
    population = []
    for mem in scored_population:
        population.append(mem[0])
    
    return population

def evolve(population_init):
    i = 0
    best_indv_list = []
    avg_pop_list = []
    last_sig_iter = 0

    population = population_init
    scored_population = get_scores(population)
    best_individual, best_score = get_best(scored_population)

    while i < max_iter:
        best_indv_list.append(best_individual)
        #reprodukcja turniejowa
        descendants = reproduction(scored_population)
        #krzyżownaie arytmetyczne
        descendants = crossing(descendants, alpha)
        #mutacja gaussowska
        descendants = mutation(descendants, sigma)

        scored_descendants = get_scores(descendants)

        best_individual_d, best_score_d = get_best(scored_descendants)    

        #parametry zbierame do statystyk
        if best_score_d < best_score:
            last_sig_iter = i
            best_score = best_score_d
            best_individual = best_individual_d
        avg_pop_list.append([sum(sub_list) / len(sub_list) for sub_list in zip(*population)])

        #sukcesja elitarna
        population, scored_population = succession(scored_population, scored_descendants, elite_size)

        i += 1

    print("Best individual: ", best_individual, " scored: ", best_score, " last significat iter:", last_sig_iter)
    plot_result(best_indv_list, avg_pop_list)

def plot_result(best_indv_list, avg_pop_list):
    min_val = -5
    max_val = 5

    #wykres funkcji
    x = np.linspace(min_val, max_val, 60)
    y = np.linspace(min_val, max_val, 60)
    X, Y = np.meshgrid(x, y)
    Z = f2(X, Y)

    #wykres najlepszych osobników // średniej wartości populacji
    x_list = []
    y_list = []
    z_list = []
    if plot_type == 1:
        for indv in best_indv_list:
            x_list.append(indv[0])
            y_list.append(indv[1])
            z_list.append(f2(indv[0], indv[1]))
    elif plot_type == 2:
        for indv in avg_pop_list:
            x_list.append(indv[0])
            y_list.append(indv[1])
            z_list.append(f2(indv[0], indv[1]))

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, Z, rstride=1, cstride=1,cmap='viridis',edgecolor='none')
    ax.scatter(x_list, y_list, z_list, c='red')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

if __name__ == "__main__":
    pop = generate_init_population(pop_size)
    evolve(pop)
