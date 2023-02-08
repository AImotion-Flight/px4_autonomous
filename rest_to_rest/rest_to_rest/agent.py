import math
import numpy as np

def clip(x, lb, ub):
    return min(max(x, lb), ub)

class Agent:
    def __init__(self, vstates, vactions):
        self.vstates = vstates
        self.vactions = vactions
        state_count = np.shape(self.vstates)[0]
        action_count = np.shape(self.vactions)[0]
        self.vstates_actions = np.hstack((np.tile(self.vstates, (action_count, 1)),
                                          np.repeat(self.vactions, state_count, axis=0)))

class DynamicalSystem(Agent):
    def __init__(self, vstates, vactions, min_vel, max_vel):
        super().__init__(vstates, vactions)
        self.min_vel = min_vel
        self.max_vel = max_vel
        self.state = (0, 0, 0, 0)

    def set_state(self, state):
        self.state = state
        
    def get_state_count(self):
        return len(self.vstates)

    def get_action_count(self):
        return len(self.vactions)

    def get_state(self):
        return self.state

    def get_state_by_index(self, i):
        return self.vstates[i]

    def find_state_index(self, state):
        return np.where(np.all(self.vstates == state, axis=1))[0][0] 

    def get_random_action(self):
        return self.vactions[np.random.randint(len(self.vactions))]
    
    def get_action_by_index(self, i):
        return self.vactions[i]

    def find_action_index(self, action):
        return np.where(np.all(self.vactions == action, axis=1))[0][0]

    def get_state_action_distance_indices(self, state, action):
        dist = np.linalg.norm(self.vstates_actions - np.concatenate((state, action)), axis=1)
        return np.argsort(dist), dist

    def get_state_action_by_index(self, i):
        return self.vstates_actions[i]

    def is_valid_action(self, action):
        if self.state[2] + action[0] > self.max_vel:
            return False
        if self.state[2] + action[0] < self.min_vel:
            return False
        if self.state[3] + action[1] > self.max_vel:
            return False
        if self.state[3] + action[1] < self.min_vel:
            return False
        return True
        
    def transition_state(self, action):
        ss_vx = clip(self.state[2] + action[0], self.min_vel, self.max_vel)
        ss_vy = clip(self.state[3] + action[1], self.min_vel, self.max_vel)

        prev_state = self.state
        self.state = (self.state[0] + ss_vx, self.state[1] + ss_vy, ss_vx, ss_vy)
        
        return self.state, math.sqrt((self.state[0] - prev_state[0])**2 + (self.state[1] - prev_state[1])**2)
