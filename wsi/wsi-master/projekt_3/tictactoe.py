from math import inf
import random

k = 3
 
terminal_states = [
    [1,1,1,0,0,0,0,0,0],
    [0,0,0,1,1,1,0,0,0],
    [0,0,0,0,0,0,1,1,1],
    [1,0,0,1,0,0,1,0,0],
    [0,1,0,0,1,0,0,1,0],
    [0,0,1,0,0,1,0,0,1],
    [1,0,0,0,1,0,0,0,1],
    [0,0,1,0,1,0,1,0,0]
]


def print_current_state(state):
    print("Ruch", state[k**2])
    print(state[0], state[1], state[2])
    print(state[3], state[4], state[5])
    print(state[6], state[7], state[8])


def get_moves(state):
    player = state[k**2]
    moves = []
    for i in range(k**2):
        if state[i] == 0:
            next_state = state.copy()
            if player == 1:
                next_state[k**2] = 2
            else:
                next_state[k**2] = 1 
            next_state[i] = player
            moves.append(next_state)
    return moves


def get_heuristic(state):
    #tictactoe heuristics 3x3
    H = [3, 2, 3, 2, 4, 2, 3, 2, 3]
    max = 0
    min = 0
    for i in range(k**2):
        if state[i] == 1:
            max += H[i]
        elif state[i] == 2:
            min += H[i]
    return max - min


def is_winning(state):
    player = 1
    if state[k**2] == 1:
        player = 2
    
    for t in terminal_states:
        count = k
        for i in range(k**2):
            if t[i] == 1 and state[i] == player:
                count-=1 
        if count == 0:
            return True  
    return False


def is_tie(state):
    for i in range(k**2):
        if state[i] == 0:
            return False
    else:
        return True


def find_max_state(states):
    value = k**2+1
    best_state = states[0]
    best_value = states[0][value]
    for state in states:
        if state[value] > best_value:
            best_value = state[value]
            best_state = state
    return best_state


def find_min_state(states):
    value = k**2+1
    best_state = states[0]
    best_value = states[0][value]
    for state in states:
        if state[value] < best_value:
            best_value = state[value]
            best_state = state
    return best_state


def Minimax(state, d):
    states_searched = 0
    player = state[k**2]
    if is_winning(state) and player == 2:
        return 10, states_searched
    elif is_winning(state) and player == 1:
        return -10, states_searched
    elif is_tie(state):
        return 0, states_searched
    elif d == 0:
        return get_heuristic(state), states_searched
    
    moves = get_moves(state)
    states_searched += len(moves)

    w = []
    for m in moves:
        new_w, new_states = Minimax(m, d-1)
        states_searched += new_states
        w.append(new_w)
    if player == 1:
        return max(w), states_searched
    elif player == 2:
        return min(w), states_searched


def alphabeta(state, d, alpha, beta):
    player = state[k**2]
    state_searched = 0
    if is_winning(state) and player == 2:
        return 10, state_searched
    elif is_winning(state) and player == 1:
        return -10, state_searched
    elif is_tie(state):
        return 0, state_searched
    elif d == 0:
        return get_heuristic(state), state_searched
    
    moves = get_moves(state)
    state_searched += len(moves)

    if player == 1:
        for m in moves:
            new_alpha, new_states = alphabeta(m, d-1, alpha, beta)
            alpha = max(alpha, new_alpha)
            state_searched += new_states
            if alpha >= beta:
                return beta, state_searched
        return alpha, state_searched
    elif player == 2:
        for m in moves:
            new_beta, new_states = alphabeta(m, d-1, alpha, beta)
            state_searched += new_states
            beta = min(beta, new_beta)
            if alpha >= beta:
                return alpha, state_searched
        return beta, state_searched


def make_move(state, d, ab=False):
    player = state[k**2]
    value = k**2+1
    moves = get_moves(state)
    states_searched = len(moves)
    w = []
    
    for m in moves:
        if ab:
            m[value], new_states = alphabeta(m, d-1, -inf, inf)
            states_searched += new_states
        else:
            m[value], new_states = Minimax(m, d-1)
            states_searched += new_states
        w.append(m)
    if player == 1:
        return find_max_state(w), states_searched
    elif player == 2:
        return find_min_state(w), states_searched


def tictactoe(cpu1_d, cpu1_ab, cpu2_d, cpu2_ab):
    value = k**2+1
    state = [0,0,0,0,0,0,0,0,0,1,0]

    cpu1_states = 0
    cpu2_states = 0
    cpu1_wins = False

    while(True):
        if cpu1_ab == True:
            state, states_searched = make_move(state, cpu1_d, True)
            cpu1_states += states_searched
        else:
            state, states_searched = make_move(state, cpu1_d)
            cpu1_states += states_searched
        if state[value] == 10:
            print("Komp1: ", cpu1_states)
            print("Komp2: ", cpu2_states)
            cpu1_wins = True
            return cpu1_wins
        elif (is_tie(state)):
            print("Komp1: ", cpu1_states)
            print("Komp2: ", cpu2_states)
            return 0
        if cpu2_ab == True:
            state, states_searched = make_move(state, cpu2_d, True)
            cpu2_states += states_searched
        else:
            state, states_searched = make_move(state, cpu2_d)
            cpu2_states += states_searched
        if state[value] == -10:
            print("Komp1: ", cpu1_states)
            print("Komp2: ", cpu2_states)
            return cpu1_wins
        elif (is_tie(state)):
            print("Komp1: ", cpu1_states)
            print("Komp2: ", cpu2_states)
            return 0


def random_move(state,d):
    player = state[k**2]

    if is_winning(state) and player == 2:
        return 10
    elif is_winning(state) and player == 1:
        return -10
    elif is_tie(state):
        return 0

    empty_states = [i for i in range(0, len(state[0:9])) if state[i] == 0]
    state[empty_states[random.randint(0, len(empty_states)-1)]] = 2
    state[k**2] = 1

    return state


def tictactoe_rd(cpu_d, cpu_ab):
    value = k**2+1
    state = [0,0,0,0,0,0,0,0,0,1,0]

    cpu_states = 0
    cpu1_wins = False

    while(True):
        if cpu_ab == True:
            state, states_searched = make_move(state, cpu_d, True)
            cpu_states += states_searched
        else:
            state, states_searched = make_move(state, cpu_d)
            cpu_states += states_searched
        if state[value] == 10:
            print("Komp1: ", cpu_states)
            cpu1_wins = True
            return cpu1_wins
        elif (is_tie(state)):
            print("Komp1: ", cpu_states)
            return 0

        state = random_move(state,0)
        if state[value] == -10:
            print("Komp1: ", cpu_states)
            return cpu1_wins
        elif (is_tie(state)):
            print("Komp1: ", cpu_states)
            return 0
 
        

if __name__ == "__main__":
    print(tictactoe_rd(1,True))    
    print(tictactoe(9,False,9,False))    #tie
    print(tictactoe(9,False,9,True))     #tie
    print(tictactoe(9,False,3,False))    #tie
    print(tictactoe(3,False,9,True))     #2
    print(tictactoe(3,False,3,False))    #2
    print(tictactoe(3,False,3,True))     #2
    print(tictactoe(1,False,0,False))    #2
