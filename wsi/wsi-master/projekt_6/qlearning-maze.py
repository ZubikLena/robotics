import numpy as np
import matplotlib.pyplot as plt

aggr_ep_data = {'ep': [], 'steps': [], 'reward': []}


class QLearningAgent():
    def __init__(self, epsilon=0.9, discount_factor=0.9, learning_rate=0.9) -> None:
        self.content = []
        self.rewards = None
        self.environment_rows = 0
        self.environment_columns = 0
        self.start_x = 0
        self.start_y = 0
        self.q_table = None
        #numeric action codes: 0 = up, 1 = right, 2 = down, 3 = left
        self.actions = ['up', 'right', 'down', 'left']
        #qlearning hiperparams
        self.epsilon = epsilon
        self.discount_factor = discount_factor
        self.learning_rate = learning_rate 
    
    def get_maze_content(self):
        with open('maze.txt') as f:
            lines = f.readlines()
            self.environment_rows = len(lines)
            for line in lines:
                self.environment_columns = len(line.strip())
                self.content.append(line.strip())
                
    def get_maze_rewards(self):
        self.rewards = np.full((self.environment_rows, self.environment_columns), -100.)
        for r_index in range(len(self.content)):
            for c_index in range(len(self.content[r_index])):
                if self.content[r_index][c_index] == 'S':
                    self.start_x = r_index
                    self.start_y = c_index
                    self.rewards[r_index][c_index] = -1.
                elif self.content[r_index][c_index] == '.':
                    self.rewards[r_index][c_index] = -1.
                elif self.content[r_index][c_index] == '#':
                    # -100.
                    pass
                elif self.content[r_index][c_index] == 'F':
                    self.rewards[r_index][c_index] = 100.
        
    def create_q_table(self):
        self.q_table = np.zeros((self.environment_rows, self.environment_columns, 4))

    def initialize_env(self):
        self.get_maze_content()
        self.get_maze_rewards()
        self.create_q_table()
        
    def is_terminal_state(self, current_row_index, current_column_index):
        if self.rewards[current_row_index, current_column_index] == -1.:
            return False
        else:
            return True
    
    def get_next_action(self, current_row_index, current_column_index, epsilon):
        if np.random.random() < epsilon:
            return np.argmax(self.q_table[current_row_index, current_column_index])
        else:
            return np.random.randint(4)
        
    def get_next_location(self, current_row_index, current_column_index, action_index):
        new_row_index = current_row_index
        new_column_index = current_column_index
        if self.actions[action_index] == 'up' and current_row_index > 0:
            new_row_index -= 1
        elif self.actions[action_index] == 'right' and current_column_index < self.environment_columns - 1:
            new_column_index += 1
        elif self.actions[action_index] == 'down' and current_row_index < self.environment_rows - 1:
            new_row_index += 1
        elif self.actions[action_index] == 'left' and current_column_index > 0:
            new_column_index -= 1
        return new_row_index, new_column_index
    
    def get_shortest_path(self):
        if self.is_terminal_state(self.start_x, self.start_y):
            return []
        else:
            current_row_index, current_column_index = self.start_x, self.start_y
            shortest_path = []
            shortest_path.append([current_row_index, current_column_index])
            while not self.is_terminal_state(current_row_index, current_column_index):
                action_index = self.get_next_action(current_row_index, current_column_index, 1.)

                current_row_index, current_column_index = self.get_next_location(current_row_index, current_column_index, action_index)
                shortest_path.append([current_row_index, current_column_index])
            return shortest_path
        
    def train(self):
        for episode in range(1000):
            row_index, column_index = self.start_x, self.start_y

            cummulative_reward = 0
            steps = 0
            while not self.is_terminal_state(row_index, column_index):
                action_index = self.get_next_action(row_index, column_index, self.epsilon)

                old_row_index, old_column_index = row_index, column_index
                row_index, column_index = self.get_next_location(row_index, column_index, action_index)
                
                reward = self.rewards[row_index, column_index]
                old_q_value = self.q_table[old_row_index, old_column_index, action_index]
                temporal_difference = reward + (self.discount_factor * np.max(self.q_table[row_index, column_index])) - old_q_value

                new_q_value = old_q_value + (self.learning_rate * temporal_difference)
                self.q_table[old_row_index, old_column_index, action_index] = new_q_value
                
                cummulative_reward += reward
                steps += 1

            aggr_ep_data['ep'].append(episode)
            aggr_ep_data['reward'].append(cummulative_reward)
            aggr_ep_data['steps'].append(steps)
            
            
        print('Training complete!')

if __name__ == "__main__":
    q_agent = QLearningAgent()
    q_agent.initialize_env()
    q_agent.train()
    print(q_agent.get_shortest_path())
    fig, axs = plt.subplots(2)
    fig.suptitle('Q-learning results')
    axs[0].plot(aggr_ep_data['ep'], aggr_ep_data['reward'], label="reward")
    axs[0].legend(loc=4)
    axs[1].bar(aggr_ep_data['ep'], aggr_ep_data['steps'], label="steps")
    axs[1].legend(loc=1)
    plt.show()
    