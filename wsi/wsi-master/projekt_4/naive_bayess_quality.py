import pandas
import numpy as np
import math

def read_csv(filename):
    wine_data = pandas.read_csv(filename, sep = ';', header = None)
    return np.array(wine_data, dtype = np.float)

def prior_probability(data):
    three = 0
    four = 0
    five = 0
    six = 0
    seven = 0
    eight = 0
    for item in data:
        if item[11] == 3:
            three += 1
        elif item[11] == 4:
            four += 1
        elif item[11] == 5:
            five += 1
        elif item[11] == 6:
            six += 1
        elif item[11] == 7:
            seven += 1
        elif item[11] == 8:
            eight += 1
    return [three/len(data), four/len(data), five/len(data), six/len(data), seven/len(data), eight/len(data)]

def conditional_probability(data):
    attrs = [([ [] for i in range(10)]) for i in range(6)]
    conditional_probabilities = [([ [] for i in range(10)]) for i in range(6)]

    for i in range(0, 10):
        for item in data:
            if item[11] == 3:
                attrs[0][i].append(item[i])
            elif item[11] == 4:
                attrs[1][i].append(item[i])
            elif item[11] == 5:
                attrs[2][i].append(item[i])
            elif item[11] == 6:
                attrs[3][i].append(item[i])
            elif item[11] == 7:
                attrs[4][i].append(item[i])
            elif item[11] == 8:
                attrs[5][i].append(item[i])

    #attrs list grouped by qualities and then by 10 attributres
    for i in range(0, 6):
        for j in range(0, 10):
            attr_touple = np.average(attrs[i][j]), np.var(attrs[i][j])
            conditional_probabilities[i][j].append(attr_touple)
    return conditional_probabilities

def gauss_conditional_probability(attr_val, attr_mean, stddev):
    exponent = math.exp(-((attr_val - attr_mean) ** 2 / (2 * stddev ** 2)))
    return (1 / (math.sqrt(2 * math.pi) * stddev)) * exponent
    #poor result
    # return np.exp(-np.power((attr_val - attr_mean)/sig, 2.)/2.)+0.00001

def classify(attr_vals_list, data):
    con_prob = conditional_probability(data)
    prior_prob = prior_probability(data)
    gauss_con_prob = [[] for i in range(6)]
    multiplied_gauss_con_prob = []
    bayess_prob = []

    for i in range(0, 6):
        for j in range(0, 10):
            gauss_con_prob[i].append(gauss_conditional_probability(attr_vals_list[j], con_prob[i][j][0][0], math.sqrt(con_prob[i][j][0][1])))
            
    #multiplying all conditional probabilities
    for i in range(0, 6):
        multiplied_gauss_con_prob.append(np.prod(gauss_con_prob[i]))
        
    denominator = 0
    for i in range(0, 6):
        denominator += multiplied_gauss_con_prob[i]*prior_prob[i]

    #the final probabilities 
    for i in range(0, 6):
        bayess_prob.append(multiplied_gauss_con_prob[i]*prior_prob[i]/denominator)

    return bayess_prob

def test(train_percent):
    white = read_csv('projekt_4\winequality-white.csv')
    train_data = white[0:int(len(white)*(train_percent/100))]
    test_data = white[(int(len(white)*(train_percent/100))+1):(len(white)-1)]

    error = 0
    
    for test in test_data:
        result = classify(test, train_data)
        quality = test[11]
        maximum = max(result)
        for i in range(0, 6):
            if maximum == result[i]:
                # print('Klasa: ', i+3)
                if (i+3) != quality:
                    error += 1
    
    print('Error num', error, len(train_data), len(test_data))
    print('Accuracy', 1 - error/len(test_data))

def k_cross_validation(k):
    data = read_csv('projekt_4\winequality-white.csv')
    division = int(len(data)/k)
    sets = []
    errors = []
    train_sets = np.array(np.empty)
    
    for i in range(0, k):
        if i == 0:
            sets.append(data[0:division])
        else:
            sets.append(data[(i*division+1):((i+1)*division)])
    
    for i in range(0, k):
        indexes = list(range(0, k))
        test_set = sets[i]
        indexes.remove(i)
        train_set = sets[indexes[0]]
        for idx in indexes:
            np.concatenate((train_set, sets[idx]))

        err = 0

        for test in test_set:
            result = classify(test, train_set)
            quality = test[11]
            maximum = max(result)
            for i in range(0, 6):
                if maximum == result[i]:
                    # print('Klasa: ', i+3)
                    if (i+3) != quality:
                        err += 1
        errors.append(err)
        
    print('Test set length', division, 'Errors:', errors)
    print('Average number of errors', np.average(errors))
    print('Itertion\'s accuracy', [(division-i)/division for i in errors])
    print('Overall accuracy: ', 1-np.average(errors)/division)


    pass
if __name__ == "__main__":

    # white = read_csv('projekt_4\winequality-white.csv')
    # print('Prior', prior_probability(white))
    # result = classify([6.1,0.53,0.02,0.9,0.035,6,81,0.99234,3.24,0.35,9.5,4], white[0:3000])
    # print('Result: ', result)

    test(60)
    k_cross_validation(5)


