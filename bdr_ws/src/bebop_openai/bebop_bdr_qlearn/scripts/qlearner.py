import numpy as np
import random
import pickle

class QLearner:
    def __init__(self,
                 num_states=100,
                 num_actions=4,
                 alpha=0.9,
                 gamma=0.2,
                 random_action_rate=0.5,
                 random_action_decay_rate=0.9,
                 load_last=False):
        self.num_states = num_states
        self.num_actions = num_actions
        self.alpha = alpha
        self.gamma = gamma
        self.random_action_rate = random_action_rate
        self.random_action_decay_rate = random_action_decay_rate
        self.state = 0
        self.action = 0
        
        if load_last is True:
            with open('/home/landonbentley/bdr/bdr_ws/src/bebop_openai/bebop_bdr_qlearn/training_results/qresults.pkl', 'rb') as qresults:
                self.qtable = pickle.load(qresults)
        else:
            self.qtable = np.random.uniform(low=-1, high=1, size=(num_states, num_actions))

    def set_initial_state(self, state):
        """
        @summary: Sets the initial state and returns an action
        @param state: The initial state
        @returns: The selected action
        """
        self.state = state
        self.action = self.qtable[state].argsort()[-1]
        return self.action

    def save(self):
        print(str(self.alpha), str(self.gamma), str(self.random_action_rate))
        with open('/home/landonbentley/bdr/bdr_ws/src/bebop_openai/bebop_bdr_qlearn/training_results/qresults.pkl', 'wb') as qresults:
            pickle.dump(self.qtable, qresults)

    def move(self, state_prime, reward):
        """
        @summary: Moves to the given state with given reward and returns action
        @param state_prime: The new state
        @param reward: The reward
        @returns: The selected action
        """
        alpha = self.alpha
        gamma = self.gamma
        state = self.state
        action = self.action
        qtable = self.qtable

        choose_random_action = (1 - self.random_action_rate) <= np.random.uniform(0, 1)

        if choose_random_action:
            action_prime = random.randint(0, self.num_actions - 1)
            print("random", str(action_prime))
        else:
            action_prime = self.qtable[state_prime].argsort()[-1]

        self.random_action_rate *= self.random_action_decay_rate

        qtable[state, action] = (1 - alpha) * qtable[state, action] + alpha * (reward + gamma * qtable[state_prime, action_prime])

        self.state = state_prime
        self.action = action_prime

        return self.action
