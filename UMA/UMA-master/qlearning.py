import gym
import numpy as np
import math
import csv

# containers for statistics data
aggr_ep_rewards = {'ep': [], 'reward': []}

class CartPoleQAgent():
    def __init__(self, buckets=(3, 6, 3, 6), num_episodes=1000, min_lr=0.1, min_epsilon=0.1, discount=1.0, decay=25):
        self.buckets = buckets
        self.num_episodes = num_episodes
        self.min_lr = min_lr
        self.min_epsilon = min_epsilon
        self.discount = discount
        self.decay = decay
        self.episode_counter = 0

        self.env = gym.make('CartPole-v0')

        # [position, velocity, angle, angular velocity]
        self.upper_bounds = [self.env.observation_space.high[0], 0.5, self.env.observation_space.high[2], math.radians(50) / 1.]
        self.lower_bounds = [self.env.observation_space.low[0], -0.5, self.env.observation_space.low[2], -math.radians(50) / 1.]

        self.Q_table = np.zeros(self.buckets + (self.env.action_space.n,))

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
            return np.argmax(self.Q_table[state])

    def update_q(self, state, action, reward, new_state):
        self.Q_table[state][action] += self.learning_rate * (reward + self.discount * np.max(self.Q_table[new_state]) - self.Q_table[state][action])

    def get_epsilon(self, t):
        return max(self.min_epsilon, min(1., 1. - math.log10((t + 1) / self.decay)))

    def get_learning_rate(self, t):
        return max(self.min_lr, min(1., 1. - math.log10((t + 1) / self.decay)))

    def train(self):
        for e in range(self.num_episodes):
            current_state = self.discretize_state(self.env.reset())

            self.learning_rate = self.get_learning_rate(e)
            self.epsilon = self.get_epsilon(e)
            done = False
            episode_reward_sum = 0
            
            while not done:
                action = self.choose_action(current_state)
                obs, reward, done, _ = self.env.step(action)
                episode_reward_sum += reward
                new_state = self.discretize_state(obs)
                self.update_q(current_state, action, reward, new_state)                            
                current_state = new_state

            aggr_ep_rewards['ep'].append(e)
            aggr_ep_rewards['reward'].append(episode_reward_sum)
            
            self.episode_counter += 1
            print('Episode: ' + str(self.episode_counter))
        
        #saving results to csv file
        with open('ql_'+str(self.buckets[0])+str(self.buckets[1])+str(self.buckets[2])+str(self.buckets[3])+'_ep_'+str(self.num_episodes)+'.csv', 'w', newline='') as result_file:
            result_writer = csv.writer(result_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)            
            for e, r in zip(aggr_ep_rewards['ep'], aggr_ep_rewards['reward']):
                result_writer.writerow([e, r])
                
        print('Finished training!')


if __name__ == "__main__":
    agent = CartPoleQAgent()
    agent.train()
