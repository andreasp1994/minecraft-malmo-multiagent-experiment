from collections import defaultdict
import random

class QLearningAgent(object):
    u""" An exploratory Q-learning agent. It avoids having to learn the transition
        model because the Q-value of a state can be related directly to those of
        its neighbors. [Figure 21.8]
    """
    def __init__(self, actions, gamma, Ne, Rplus, alpha=None,):
        self.gamma = gamma
        self.all_act = actions
        self.Ne = Ne  # iteration limit in exploration function
        self.Rplus = Rplus  # large value to assign before iteration limit
        self.Q = defaultdict(float)
        self.Nsa = defaultdict(float)
        self.state_actions = defaultdict(float)
        self.Nf = 9.0
        self.s = None
        self.a = None
        self.r = None

        if alpha:
            self.alpha = alpha
        else:
            self.alpha = lambda n: 1/(1+n)  # udacity video

    def f(self, u, n):
        u""" Exploration function. Returns fixed Rplus untill
        agent has visited state, action a Ne number of times.
        Same as ADP agent in book."""
        if n < self.Ne:
            return self.Rplus
        else:
            return u

    def g(self, nsa):
        u""" Used to create a decreasing learning rate but stops
        to decrease at 0.1."""
        if nsa < self.Nf:
            return nsa
        else:
            return self.Nf

    def actions_in_state(self, state):
        u""" Returns actions possible in given state.
            Useful for max and argmax. """
        if state in self.state_actions:
            return self.state_actions[state]
        else:
            return self.all_act

    def __call__(self, percept, training):
        s1, r1 = self.process_perception(percept)
        Q, Nsa, s, a, r = self.Q, self.Nsa, self.s, self.a, self.r
        alpha, gamma, actions_in_state = self.alpha, self.gamma, self.actions_in_state

        if s is not None and training:
            Nsa[s, a] += 1
            Q[s, a] += alpha(self.g(Nsa[s, a])) * (r + gamma * max(Q[s1, a1] for a1 in actions_in_state(s1))
                                             - Q[s, a])

        self.s, self.r = s1, r1
        self.a = max(self.shuffled(actions_in_state(s1)), key=lambda a1: self.f(Q[s1, a1], Nsa[s1, a1]))
        return self.a

    def finished(self, reward):
        self.Q[self.s, self.a] = reward

    def reset(self):
        self.Nsa = defaultdict(float)
        self.s = self.a = self.r = None

    def process_perception(self, percept):
        self.state_actions[percept['state']] = percept['state_actions']
        return (percept['state'], percept['reward'])

    def shuffled(self, iterable):
        "Randomly shuffle a copy of iterable."
        random.seed()
        items = list(iterable)
        random.shuffle(items)
        return items

    def all_utilities(self, s):
        return list(self.Q[s, a1] for a1 in self.actions_in_state(s));