import math
import numpy as np
import matplotlib.pyplot as plt

class EnsembleQLearning():
    def __init__(self, n, init_state, final_state, agent, env, ep, gamma, plot=False):
        self.n = n
        self.init_state = init_state
        self.final_state = final_state
        self.agent = agent
        self.env = env
        self.ep = ep
        self.gamma = gamma
        self.plot = plot

        self.visit = np.ones((self.agent.get_state_count(), 1))
        Qs = []
        for i in range(self.n):
            Qs.append(np.zeros((self.agent.get_state_count(), self.agent.get_action_count())))
        self.Qs = np.array(Qs)

        if plot:
            self.fig = plt.figure(self.__class__.__name__, figsize=(2, 2))
            self.ax = self.fig.add_subplot(111)

    def clear(self):
        Qs = []
        for i in range(self.n):
            Qs.append(np.zeros((self.agent.get_state_count(), self.agent.get_action_count())))
        self.Qs = np.array(Qs)

    def clear_visit(self):
        self.visit = np.ones((self.agent.get_state_count(), 1))

    def plot_learning(self, path):
        self.ax.clear()
        self.ax.imshow(self.env.grid, cmap='gray', origin='lower')
        self.ax.plot(self.init_state[0], self.init_state[1], 'go')
        self.ax.plot(self.final_state[0], self.final_state[1], 'r*')
        self.ax.plot(path[:, 0], path[:, 1], 'b--')
        self.ax.plot(path[-1, 0], path[-1, 1], 'rx')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save(self, filename):
        np.save(filename, self.Qs)

    def load(self, filename):
        self.Qs = np.load(filename)

    def is_final(self, state):
        return state == self.final_state

    def is_terminal(self, state):
        if not self.env.is_valid(state[0:2]):
            return True
        if self.env.is_obstacle(state[0:2]):
            return True
        if self.is_final(state):
            return True
        return False

    def reset_agent(self):
        self.agent.set_state(self.init_state)
    
    def epsilon_greedy_policy(self, state):
        if np.random.uniform() < 0.15:#1 / math.sqrt(self.visit[self.agent.find_state_index(state)]):
            return self.agent.get_random_action()
        else:
            row = self.agent.find_state_index(state)
            argmaxQ = np.argmax(np.sum(self.Qs, axis=0)[row])
            return self.agent.get_action_by_index(argmaxQ)

    def convergence(self):
        return np.sum(np.max(self.Qs, axis=2), axis=0) / self.n

    def compute_reward(self, state, next_state):
        if not self.env.is_valid(next_state):
            return -1
        elif self.env.is_obstacle(next_state):
            return -1
        elif self.is_final(next_state):
            return 1
        elif state == next_state:
            return -1
        else:
            return 0
        
    def update_Q(self, state, next_state, action, r):
        row = self.agent.find_state_index(state)
        col = self.agent.find_action_index(action)
            
        pQ = np.random.randint(0, self.n)
        pmaxQ = np.random.randint(0, self.n)

        maxQ = 0
        if not self.is_terminal(next_state):
            row_next = self.agent.find_state_index(next_state)
            maxQ = self.Qs[pQ][row_next, np.argmax(self.Qs[pmaxQ][row_next])]

        alpha = 1 if self.visit[row] < 2 else 1 / math.log2(self.visit[row])
        self.Qs[pQ][row, col] = self.Qs[pQ][row, col] + alpha * (r + self.gamma * maxQ - self.Qs[pQ][row, col])

    def run_episode(self):
        self.reset_agent()
        path = []
        aseq = []
        er = 0

        state = self.agent.get_state()
            
        while not self.is_terminal(state):
            action = self.epsilon_greedy_policy(state)
            next_state, c = self.agent.transition_state(action)
            
            path.append(state)
            aseq.append(action)
            
            r = self.compute_reward(state, next_state) - (c + 1)/1000
            self.update_Q(state, next_state, action, r)
            self.visit[self.agent.find_state_index(state)] += 1

            state = next_state

            er = er + r

        return path, state, aseq, er
        
    def learn(self):
        if self.plot:
             self.fig.show()

        convg = []
        rew = []
        try:
            for i in range(self.ep):
                path, terminal_state, _, er = self.run_episode()

                convg.append(self.convergence()[self.agent.find_state_index(self.init_state)])
                rew.append(er)

                if self.is_final(terminal_state):
                    path.append(terminal_state)
                    #print("Goal reached!")

                if self.plot:
                    self.plot_learning(np.array(path))
                    
                if i % 1000 == 0:
                    print('Completed Episode ' + str(i + 1))
        except KeyboardInterrupt:
            print('stop learning ...')
            
        return np.array(convg), np.array(rew)

    def get_policy(self):
        self.reset_agent()
        path = []
        aseq = []

        state = self.agent.get_state() 
        
        while not self.is_final(state):
            row = self.agent.find_state_index(state)
            col = np.argmax(np.sum(self.Qs, axis=0)[row])
            action = self.agent.get_action_by_index(col)

            path.append(state)
            aseq.append(action)
            
            state, _ = self.agent.transition_state(action)

        path.append(state)
        
        return np.array(path), np.array(aseq)

class QLearning(EnsembleQLearning):
    def __init__(self, init_state, final_state, agent, env, ep, gamma, plot=False):
            super().__init__(1, init_state, final_state, agent, env, ep, gamma, plot)
    
class SARSA(EnsembleQLearning):
    def __init__(self, init_state, final_state, agent, env, ep, gamma, plot=False):
            super().__init__(1, init_state, final_state, agent, env, ep, gamma, plot)

    def update_Q(self, state, next_state, action, r):
        row = self.agent.find_state_index(state)
        col = self.agent.find_action_index(action)

        futureQ = 0
        if not self.is_terminal(next_state):
            row_next = self.agent.find_state_index(next_state)
            next_action = self.epsilon_greedy_policy(next_state)
            col_next = self.agent.find_action_index(next_action)
            futureQ = self.Qs[0][row_next, col_next]

        alpha = 1 if self.visit[row] < 2 else 1 / math.log2(self.visit[row])
        self.Qs[0][row, col] = self.Qs[0][row, col] + alpha * (r + self.gamma * futureQ - self.Qs[0][row, col])

    def run_episode(self):
        self.reset_agent()
        path = []
        aseq = []
        er = 0

        state = self.agent.get_state()
            
        while not self.is_terminal(state):
            action = self.epsilon_greedy_policy(state)
            next_state, c = self.agent.transition_state(action)
            
            path.append(state)
            aseq.append(action)
            
            r = self.compute_reward(state, next_state) - (c + 1)/1000
            self.update_Q(state, next_state, action, r)
            self.visit[self.agent.find_state_index(state)] += 1

            state = next_state

            er = er + r

        return path, state, aseq, er

class KNNQLearning(EnsembleQLearning):
    def __init__(self, n, kmax, beta, init_state, final_state, agent, env, ep, gamma, plot=False):
        super().__init__(n, init_state, final_state, agent, env, ep, gamma, plot)

        self.kmax = kmax
        self.beta = beta

    def update_Q(self, state, next_state, action, r):
        row = self.agent.find_state_index(state)
        col = self.agent.find_action_index(action)

        k = self.kmax if self.visit[row] == 0 else min(int((10 / self.visit[row])), self.kmax)
        indices, dist = self.agent.get_state_action_distance_indices(state, action)
        indices = indices[1:k + 1]
        weights = (1 / (dist + 0.00001))**self.beta
        weights = weights[indices]
        weights = weights / np.sum(weights)
        #print(dist[indices[:self.k]])
        #print(weights)

        maxQ = 0
        if not self.is_terminal(next_state):
            row_next = self.agent.find_state_index(next_state)
            maxQ = self.Qs[0][row_next, np.argmax(self.Qs[0][row_next])]

        alpha = 1 if self.visit[row] < 2 else 1 / math.log2(self.visit[row])
        self.Qs[0][row, col] = self.Qs[0][row, col] + alpha * (r + self.gamma * maxQ - self.Qs[0][row, col])
        
        for i in range(k):
            state_action = self.agent.get_state_action_by_index(indices[i])
            row = self.agent.find_state_index(state_action[:4])
            col = self.agent.find_action_index(state_action[4:])

            alpha = 1 if self.visit[row] < 2 else 1 / math.log2(self.visit[row])
            self.Qs[0][row, col] = self.Qs[0][row, col] + alpha * weights[i] * (r + self.gamma * maxQ - self.Qs[0][row, col])