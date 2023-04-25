import json
import yaml
import math


def get_x_sign(item):
    '''
    get the sign(direction) of element coordinates 
    with reference to last coordinate system
    '''
    alfa = item['alfa']

    if alfa ==0:
        #alfa = 0 brak obrotu
        return 1
    elif alfa <= 1.5708:
        #mniejsza lub równa 90 st
        return -1
    elif alfa <= 3.14:
        # alfa do 90 do 180
        return -1
    elif alfa <=4.7124:
        #alfa od 180 do 270
        return 1
    else:
        #alfa od 270 do 360
        return 1

def get_z_sign(item):
    '''get the sign(direction) of element coordinates 
    with reference to last coordinate system
    '''
    alfa = item['alfa']
    
    if alfa == 0:
        #alfa = 0 brak obrotu
        return 1
    elif alfa <= 1.5708:
        #mniejsza lub równa 90 st
        return 1
    elif alfa <= 3.14:
        # alfa do 90 do 180
        return -1
    elif alfa <=4.7124:
        #alfa od 180 do 270
        return -1
    else:
        #alfa od 270 do 360
        return 1

def rotate_coordinates(item,coor_list):
    '''
    rotating an element doesnt influence the position on y-axis
    swaps the position of z-axis with x-axis - 
    due to the fact that y doesnt move at all
    and we're rotating by 90 degrees
    that is the function that perfoms the rotation
    swaps x with z if such rotation declared
    '''
    alfa = item['alfa']
    if(len(item) == 5):
        #its a box
        if alfa == 0:
            coor_list[0], coor_list[1] = coor_list[1], coor_list[0]
        elif alfa == 1.5708:
            coor_list[0], coor_list[1] = coor_list[1], coor_list[0]
        elif alfa ==3.14:
            coor_list[0], coor_list[1] = coor_list[0], coor_list[1]
        elif alfa == 4.7124:
            coor_list[0], coor_list[1] = coor_list[0], coor_list[1]
        else:
            coor_list[0], coor_list[1] = coor_list[1], coor_list[0]
        return coor_list
    else:
        return coor_list


def get_geometric_center(item):
    '''
    due to the fact that in urdf notation 
    we locate elements by declaring
    the position of theirs geometrical center,
    so we must get its coordinates
    '''

    #assigning the values
    alfa = item['alfa']
    x_sign = get_x_sign(item)
    z_sign = get_z_sign(item)


    if len(item) == 6:
        #its a cylider
        return [0,item['length']/2]

    elif len(item) == 5 and alfa == 0:
        #coordinates of box in home coordinates swap x with z 
        measures = item['size']
        return [x_sign*measures['x']/2,z_sign*measures['z']/2 ]
    else:
        #both x and z axiz changes directions -> -
        measures = item['size']
        return [x_sign*measures['x']/2,z_sign*measures['z']/2 ]


def convert(jsonfile):
    ''' 
    method converting dh matrix to rpy
    takes data from jsonfile, returns dict
    '''

    data = read_dh(jsonfile)
  



    for item in data.values():
        alfa = item['alfa']

        a,d = rotate_coordinates(item,get_geometric_center(item))

        
        item['a'] = a
        item['d'] = d

        
        item['rpy'] = {}
        item['xyz'] = {}
        item['rpy']['r'] = 0
        item['rpy']['p'] = item['alfa']
        item['rpy']['y'] = 0

        item['xyz']['x'] = item['a']
        item['xyz']['y'] = 0
        item['xyz']['z'] = item['d']

        item.pop('alfa')
        item.pop('theta')
        item.pop('a')
        item.pop('d')
    
    return data


def read_dh(jsonfile):
    ''' 
    method reading json file
    returns dict
    '''
    with open(jsonfile, "r") as file:
        data = json.load(file)    
    return data

def write_yaml(data, filename):
    '''
    method writing dict data to yaml file
    '''
    with open(filename, "w") as yamlfile:
        yaml.dump(data, yamlfile)


if __name__ == "__main__":
    data = convert("dh.json")
    write_yaml(data, "dh.yaml")



