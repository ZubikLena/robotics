import gym
import numpy as np
import math
import csv

# containers for statistics data
aggr_ep_rewards = {'ep': [], 'reward': []}

class CartPoleAgent():
    def __init__(self, buckets=(3, 6, 3, 6), num_episodes=2000, min_lr=0.1, min_epsilon=0.1, discount=0.95, decay=25, alpha=0.4, gamma=0.85):
        self.buckets = buckets
        self.num_episodes = num_episodes
        self.min_lr = min_lr
        self.min_epsilon = min_epsilon
        self.discount = discount
        self.decay = decay
        self.alpha = alpha
        self.episode_counter = 0
        self.actions = (0, 1)
        self.list_of_visited_states_actions = []
        self.gamma = gamma

        self.env = gym.make('CartPole-v0')

        # [position, velocity, angle, angular velocity]
        self.upper_bounds = [self.env.observation_space.high[0], 0.5, self.env.observation_space.high[2], math.radians(50) / 1.]
        self.lower_bounds = [self.env.observation_space.low[0], -0.5, self.env.observation_space.low[2], -math.radians(50) / 1.]

        self.q_table = np.zeros(self.buckets + (self.env.action_space.n,))

    def discretize_state(self, obs):
        discretized = list()
        for i in range(len(obs)):
            scaling = (obs[i] + abs(self.lower_bounds[i])) / (self.upper_bounds[i] - self.lower_bounds[i])
            new_obs = int(round((self.buckets[i] - 1) * scaling))
            new_obs = min(self.buckets[i] - 1, max(0, new_obs))
            discretized.append(new_obs)
        return tuple(discretized)

    def choose_action(self, state):
        if (np.random.random() < self.epsilon):
            return self.env.action_space.sample() 
        else:
            return np.argmax(self.q_table[state])

    def update_td0(self, state, action, reward, new_state):
        current_q = self.q_table[state][action]
        next_q = self.q_table[new_state][action]
        self.q_table[state][action] = current_q+self.alpha*(reward+self.gamma*next_q-current_q)
            
    def get_epsilon(self, t):
        return max(self.min_epsilon, min(1., 1. - math.log10((t + 1) / self.decay)))

    def get_learning_rate(self, t):
        return max(self.min_lr, min(1., 1. - math.log10((t + 1) / self.decay)))

    def train(self):
        for e in range(self.num_episodes):
            current_state = self.discretize_state(self.env.reset())
            self.learning_rate = 0.97
            self.epsilon = self.get_epsilon(e)
            done = False
            episode_reward_sum = 0
            
            while not done:
                action = self.choose_action(current_state)
                obs, reward, done, _ = self.env.step(action)
                episode_reward_sum += reward
                new_state = self.discretize_state(obs)
                self.update_td0(current_state, action, reward, new_state)
                current_state = new_state

            aggr_ep_rewards['ep'].append(e)
            aggr_ep_rewards['reward'].append(episode_reward_sum)
            
            self.episode_counter += 1
            print('Episode: ' + str(self.episode_counter))
        
        #saving results to csv file
        with open('td_'+str(self.buckets[0])+str(self.buckets[1])+str(self.buckets[2])+str(self.buckets[3])+'_ep_'+str(self.num_episodes)+'.csv', 'w', newline='') as result_file:
            result_writer = csv.writer(result_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)            
            for e, r in zip(aggr_ep_rewards['ep'], aggr_ep_rewards['reward']):
                result_writer.writerow([e, r])
            
        print('Finished training!')            


if __name__ == "__main__":
    agent = CartPoleAgent()
    agent.train()
    
    
