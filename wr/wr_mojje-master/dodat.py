 #pobieranie punktu
def get_coordinate():
    num=""
    while(type(num)!=float):
        try:
            num = float(input())
        except Exception:
            print("Złe dane, sprobuj jeszcze raz")
    return num

def get_user_goal():
    print("Podaj współrzędną x:")
    x_goal=get_coordinate()
    print("Podaj współrzędną y:")
    y_goal=get_coordinate()
    return (x_goal,y_goal)


print(get_user_goal())

#pobieranie listy
def wrong_coordinates(x_goal,y_goal):
    if(0<x_goal<10 and 0<y_goal<10):
        return False
    else:
        return True


goals_len=0
while(goals_len<=0):
    try:
        goals_len = int(input("Podaj liczbę punktów, z której będzie się składała trasa:"))
    except Exception:
        print("Złe dane, sprobuj jeszcze raz")
    
def get_coordinate():
    num=""
    while(type(num)!=float):
        try:
            num = float(input())
        except Exception:
            print("Złe dane, sprobuj jeszcze raz")
    return num

def get_user_goal():
    print("Podaj współrzędną x:")
    x_goal=get_coordinate()
    print("Podaj współrzędną y:")
    y_goal=get_coordinate()
    return (x_goal,y_goal)


def get_user_goals_list():
    goals=[]
    for i in range(0, goals_len):
        x_goal=-0.5
        y_goal=-0.5
        while(wrong_coordinates(x_goal,y_goal)):
            x_goal,y_goal=get_user_goal()
        goals.append((x_goal,y_goal))
    return goals

print(get_user_goals_list())