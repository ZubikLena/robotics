import pandas
import numpy as np
from functools import reduce

def read_csv(filename):
    wine_data = pandas.read_csv(filename, sep = ';', header = None)
    return tag_set(np.array(wine_data, dtype = np.float), filename)

def tag_set(data, filename):
    tagged_data = []
    for item in data:
        if filename == 'projekt_4\winequality-white.csv':
            item = np.append(item, 1)
        elif filename == 'projekt_4\winequality-red.csv':
            item = np.append(item, 0)
        tagged_data.append(item)
    return tagged_data

def mixed_set(white_data, white_len, red_data, red_len):
    return white_data[0:white_len] + red_data[0:red_len]

def prior_probability(data):
    white_count = 0
    red_count = 0
    for item in data:
        if item[12] == 1:
            white_count += 1
        elif item[12] == 0:
            red_count += 1
    return white_count/len(data), red_count/len(data)

def conditional_probability(data):
    white_attrs = []
    red_attrs = []
    white_conditional_probabilities = []
    red_conditional_probabilities = []
    for i in range(0, 11):
        wattr = []
        rattr = []
        for item in data:
            if item[12] == 1:
                #white wine
                wattr.append(item[i])
            elif item[12] == 0:
                #red wine
                rattr.append(item[i])

        white_attrs.append(wattr)
        red_attrs.append(rattr)
        
    for w_attr, r_attr in zip(white_attrs, red_attrs):
        w_touple = (np.average(w_attr), np.var(w_attr))
        white_conditional_probabilities.append(w_touple)
        r_touple = (np.average(r_attr), np.var(r_attr))
        red_conditional_probabilities.append(r_touple)

    return white_conditional_probabilities, red_conditional_probabilities

def gauss_conditional_probability(attr_val, attr_mean, sig):
    return np.exp(-np.power((attr_val - attr_mean)/sig, 2.)/2.)+0.000001
    # return 1./(np.sqrt(2.*np.pi)*sig)*np.exp(-np.power((attr_val - attr_mean)/sig, 2.)/2.) + 0.000001
    # return 1/np.sqrt((2*np.pi*attr_variance)) * np.exp(-0.5*((attr_val - attr_mean)**2)/attr_variance)

def classify(attr_vals_list, data):
    wcp, rcp = conditional_probability(data)
    wp, rp = prior_probability(data)

    new_white_conditional_probability = []
    new_red_conditional_probability = []
    
    for attr, w_attr, r_attr in zip(attr_vals_list, wcp, rcp):
        new_white_conditional_probability.append(gauss_conditional_probability(attr, w_attr[0], w_attr[1]))
        new_red_conditional_probability.append(gauss_conditional_probability(attr, r_attr[0], r_attr[1]))

    #multiplying all conditional probabilities
    new_is_white = reduce((lambda x, y: x * y), new_white_conditional_probability)
    new_is_red = reduce((lambda x, y: x * y), new_red_conditional_probability)

    #the final probabilities 
    wp_bayess = (new_is_white*wp)/((new_is_white*wp) + (new_is_red*rp))
    rp_bayess = (new_is_red*rp)/((new_is_white*wp) + (new_is_red*rp))

    return wp_bayess, rp_bayess



if __name__ == "__main__":
    print("begin\n")
    white = read_csv('projekt_4\winequality-white.csv')
    red = read_csv('projekt_4\winequality-red.csv')
    mix = mixed_set(white, 2, red, 2)
    # print('Mixed: ', mix)
    # print('Prior prob: ', prior_probability(mix))
    # print('Conditional: ', conditional_probability(mix))
    print('Result: ', classify([6.6,0.32,0.26,4.6,0.031,26,120,0.99198,3.4,0.73,12.5,7], mix))
    print('res', classify([8,0.48,0.34,2.2,0.073,16,25,0.9936,3.28,0.66,12.4,6], mix))
