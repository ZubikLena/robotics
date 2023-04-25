import gym
import sys
import pyglet
import numpy as np
import matplotlib.pyplot as plt 
import time

def get_discrete_state(state):
    discrete_state = (state - env.observation_space.low)/discrete_os_win_size
    return tuple(discrete_state.astype(np.int))

def exploit_or_explore():
    if np.random.random() > epsilon:
        action = np.argmax(q_table[discrete_state])
    else:
        action = np.random.randint(0, env.action_space.n)
    return action

def random_action():    
    return np.random.randint(0, env.action_space.n)

def actoion_according_to_heuristics(state):
    if state[2] < 0:
        # leaning to the left then push cart to the left
        action = 0
    else:
        # leaning to the right then push cart to the right
        action = 1
    return action

def adjust_exploration_coefficient(epsilon, episode):
    if END_EPSILON_DECAYING >= episode >= START_EPSILON_DECAYING:
        epsilon -= epsilon_decay_value
    return epsilon
    
# For stats
ep_rewards = []
aggr_ep_rewards = {'ep': [], 'avg': [], 'max': [], 'min': []}
STATS_EVERY = 100
episode_counter = 0

DISCRETE_OS_SIZE = [20, 20, 20, 20]
LEARNING_RATE = 0.1
DISCOUNT = 0.9
EPISODES = 4000
SHOW_EVERY = 1000
epsilon = 0.1
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILON_DECAYING - START_EPSILON_DECAYING)


env = gym.make("CartPole-v0")

discrete_os_win_size = (env.observation_space.high - env.observation_space.low)/DISCRETE_OS_SIZE
q_table = np.random.uniform(low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n]))
discrete_state = get_discrete_state(env.reset())

print(env.action_space.n)
print(env.reset())
done = False


    # Starting State:
    # All observations are assigned a uniform random value in [-0.05..0.05]
    
    # Episode Termination:
    #     Pole Angle is more than 12 degrees (from stability).
    #     Cart Position is more than 2.4 (center of the cart reaches the edge of
    #     the display).
    #     Episode length is greater than 200.
    
    # Solved Requirements:
    #     Considered solved when the average return is greater than or equal to
    #     195.0 over 100 consecutive trials.

for episode in range(EPISODES):
    
    episode_counter += 1
    start_time = time.time()
    ep_reward = 0 
    discrete_state = get_discrete_state(env.reset())
    done = False

    if episode % SHOW_EVERY == 0:
        render = True
        print(episode)
    else:
        render = False

    while not done:
        
        # Actions:
        # 0: Push cart to the left
        # 1: Push cart to the right
        
        action = exploit_or_explore()
        # action = random_action()
        # action = actoion_according_to_heuristics(discrete_state)
        # print("Action: " + str(action))
        
        # Observation:
        # Cart Position: [-2.4, 2,4]
        # Cart Velocity: [-Inf, Inf]
        # Pole Angle: [-0.209 rad (-12 deg), 0.209 rad (12 deg)] from stability
        # Pole angular velocity: [-Inf, Inf]
        
        # Reward:
        # 1: for every taken action, including terminal state
        
        new_state, reward, done, _ = env.step(action)
        new_discrete_state = get_discrete_state(new_state)
        ep_reward += reward
        # print("Reward: ", reward)
        # print("New state: ", new_state)
        # print("New discrete state: ", new_state)
        
        if episode % SHOW_EVERY == 0:
            env.render()
        
        if not done:

            # Maximum possible Q value in next step (for new state)
            max_future_q = np.max(q_table[new_discrete_state])

            # Current Q value (for current state and performed action)
            current_q = q_table[discrete_state + (action,)]

            # And here's our equation for a new Q value for current state and action
            new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)

            # Update Q table with new Q value
            q_table[discrete_state + (action,)] = new_q

        # Simulation ended (for any reson) - if goal position is achived - update Q value with reward directly
        elif new_state[2] >= - 0.209 and new_state[2] <= 0.209:
            q_table[discrete_state + (action,)] = 0

        discrete_state = new_discrete_state
        
    # Update of epsilon (exploration coefficient)
        
    epsilon = adjust_exploration_coefficient(epsilon, episode)
    ep_rewards.append(ep_reward)
    
    if not episode % STATS_EVERY:
        average_reward = sum(ep_rewards[-STATS_EVERY:])/STATS_EVERY
        aggr_ep_rewards['ep'].append(episode)
        aggr_ep_rewards['avg'].append(average_reward)
        aggr_ep_rewards['max'].append(max(ep_rewards[-STATS_EVERY:]))
        aggr_ep_rewards['min'].append(min(ep_rewards[-STATS_EVERY:]))
        print(f'Episode: {episode:>5d}, average reward: {average_reward:>4.1f}, current epsilon: {epsilon:>1.2f}')
    
    if episode % 10 == 0:
        pass
        # np.save(f"qtables/{episode}-qtable.npy", q_table)
        
    end_time = time.time()
    print("Episode" + str(episode_counter) + "time: " + str(end_time - start_time))
    

plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['avg'], label="average rewards")
plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['max'], label="max rewards")
plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['min'], label="min rewards")
plt.legend(loc=4)
plt.show()

env.close()
