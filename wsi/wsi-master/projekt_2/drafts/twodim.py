import numpy as np
import matplotlib.pyplot as plt


#liczba iteracji
max_iter = 500
#wielkość populacji inicjalnej
pop_size = 10
#typ populacji 1-clones // 2-independent
pop_type = 1
#rozmiar turnieju
tournament_size = 2
#rozmiar elity
elite_size = 1
#siła rekombinacji
alpha = 0.1
#siła mutacji
sigma = 1

def f(x):
    a = 1
    b = 100
    return (a - x[0])**2 + b*(x[1] - x[0]**2)**2


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
        score = f(indv)
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
    for j in range(0, pop_size):
        index1 = np.random.randint(0, pop_size)
        index2 = np.random.randint(0, pop_size)
        parent1 = scored_population[index1]
        parent2 = scored_population[index2]

        if parent1[1] <= parent2[1]:
            descendants.append(parent1[0])
        else:
            descendants.append(parent2[0])
        
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
        mutated_descendants.append([dscnt[0] + sigma*np.random.normal(0,1,1), dscnt[1] + sigma*np.random.normal(0,1,1)])

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

    population = population_init
    scored_population = get_scores(population)
    best_individual, best_score = get_best(scored_population)

    while i < max_iter:
        #reprodukcja turniejowa
        descendants = reproduction(scored_population)
        #krzyżownaie arytmetyczne
        descendants = crossing(descendants, alpha)
        #mutacja gaussowska
        descendants = mutation(descendants, sigma)

        scored_descendants = get_scores(descendants)

        best_individual_d, best_score_d = get_best(scored_descendants)    

        if best_score_d < best_score:
            best_score = best_score_d
            best_individual = best_individual_d
            pass

        #sukcesja elitarna
        population, scored_population = succession(scored_population, scored_descendants, elite_size)

        i += 1

    print("Best individual: ", best_individual, " scored: ", best_score)



if __name__ == "__main__":
    pop = generate_init_population(pop_size)
    # print("OCENA")
    # pop_scores = get_scores(pop)
    # print(get_scores(pop))
    # print("BEST")
    # print(get_best(pop_scores))
    # print("REPRODUKCJA")
    # rep = reproduction(pop_scores)
    # print(rep)
    # print("KRZYŻOWANIE")
    # cross = crossing(rep, alpha)
    # print(cross)
    # print("MUTACJA")
    # mut = mutation(cross, sigma)
    # print(len(get_scores(mut)))
    # print(mut)
    # mut_scores = get_scores(mut)
    # print("SUKCESJA")
    # succ, succ_scores = succession(pop_scores, mut_scores, elite_size)
    # print(succ_scores)

    evolve(pop)