import copy
import numpy as np
import random
import time

# Directions
UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3

OBSTACLES = [(1, 1)]
EXIT_STATE = (-1, -1)

MAX_ITER = 100
EPSILON = 0.0001

class Grid:
    def __init__(self):
        self.x_size = 4
        self.y_size = 3
        self.p = 0.8
        self.actions = [UP, DOWN, LEFT, RIGHT]
        self.rewards = {(3, 1): -100, (3, 2): 1}
        self.discount = 0.9

        # 这里的state 只是坐标
        self.states = list((x, y) for x in range(self.x_size) for y in range(self.y_size))
        self.states.append(EXIT_STATE)
        for obstacle in OBSTACLES:
            self.states.remove(obstacle)

    # (0,1)
    def attempt_move(self, s, a):
        """ Attempts to move the agent from state s via action a.

            Parameters:
                s: The current state.
                a: The *actual* action performed (as opposed to the chosen
                   action; i.e. you do not need to account for non-determinism
                   in this method).
            Returns: the state resulting from performing action a in state s.
        """
        x, y = s

        # Check absorbing state
        if s == EXIT_STATE:
            return s

        # Default: no movement
        result = s 

        # Check borders
        """
        TODO: Write code here to check if applying an action 
        keeps the agent with the boundary
        """
        if (a == UP and s[1] >= self.y_size-1) or (a == DOWN and s[1] <= 0) or (a == LEFT and s[0] <= 0) or (a == RIGHT and s[0] >= self.x_size-1):
            return result
        elif a == UP:
            result = (s[0], s[1]+1)
        elif a == DOWN:
            result = (s[0], s[1]-1)
        elif a == LEFT:
            result = (s[0]-1, s[1])
        elif a == RIGHT:
            result = (s[0]+1, s[1])

        # Check obstacle cells
        """
        TODO: Write code here to check if applying an action 
        moves the agent into an obstacle cell
        """
        if result in OBSTACLES:
            result = s

        return result

    # return: {UP: 0.8, LEFT: 0.1, RIGHT: 0.1}
    def stoch_action(self, a):
        """ Returns the probabilities with which each action will actually occur,
            given that action a was requested.

        Parameters:
            a: The action requested by the agent.

        Returns:
            The probability distribution over actual actions that may occur.
        """
        if a == RIGHT:
            return {RIGHT: self.p, UP: (1-self.p)/2, DOWN: (1-self.p)/2}
        elif a == UP:
            return {UP: self.p, LEFT: (1-self.p)/2, RIGHT: (1-self.p)/2}
        elif a == LEFT:
            return {LEFT: self.p, UP: (1-self.p)/2, DOWN: (1-self.p)/2}
        return {DOWN: self.p, LEFT: (1-self.p)/2, RIGHT: (1-self.p)/2}

    # {(0, 1): 0.8, (0, 0): 0.1, (1, 0): 0.1}
    def get_transition_probabilities(self, s, a):
        """ Calculates the probability distribution over next states given
            action a is taken in state s.

        Parameters:
            s: The state the agent is in
            a: The action requested

        Returns:
            A map from the reachable next states to the probabilities of reaching
            those state; i.e. each item in the returned dictionary is of form
            s' : P(s'|s,a)
        """
        """
            TODO: Create and return a dictionary mapping each possible next state to the
            probability that that state will be reached by doing a in s.
        """
        probability = dict()
        possible_actions = self.stoch_action(a)
        for action in possible_actions.keys():
            new_state = self.attempt_move(s, action)
            if new_state not in probability:
                probability[new_state] = possible_actions[action]
            else:
                probability[new_state] = probability[new_state] + possible_actions[action]
        return probability

    def get_reward(self, s):
        """ Returns the reward for being in state s. """
        if s == EXIT_STATE:
            return 0

        return self.rewards.get(s, 0)


class ValueIteration:
    def __init__(self, grid):
        self.grid = grid
        self.values = {state: 0 for state in self.grid.states}
        self.policy = {state: RIGHT for state in self.grid.states}

    def next_iteration(self):
        new_values = dict()
        new_policy = dict()
        """
        TODO: Write code here to implement the VI value update
        Iterate over self.grid.states and self.grid.actions
        Use stoch_action(a) and attempt_move(s,a)
        """
        # calculate all values of states
        for state in self.grid.states:
            # store values of all actions
            action_values = []
            action_corresponding_values = dict()
            # calculate all value for possible actions
            for action in self.grid.actions:
                value = 0
                """
                # return: {UP: 0.8, LEFT: 0.1, RIGHT: 0.1}
                self.grid.stoch_action(action)
                """
                # {(0, 1): 0.8, (0, 0): 0.1, (1, 0): 0.1}
                new_states = self.grid.get_transition_probabilities(state, action)

                # get the value for one action of the state
                for new_state, probability in new_states.items():
                    # p * (R + discount * V)
                    value += probability * (self.grid.get_reward(state) + self.grid.discount * self.values.get(new_state, 0))

                action_corresponding_values[action] = value
                action_values.append(value)
            new_values[state] = max(action_values)
            new_policy[state] = list(action_corresponding_values.keys())[list(action_corresponding_values.values()).index(max(action_values))]

        self.values = new_values
        self.policy = new_policy


    # print the value of each state
    def print_values(self):
        for state, value in self.values.items():
            print(state, value)


class PolicyIteration:
    def __init__(self, grid):
        self.grid = grid
        self.values = {state: 0 for state in self.grid.states}
        self.policy = {pi: RIGHT for pi in self.grid.states}
        self.r = [0 for s in self.grid.states]
        for idx, state in enumerate(self.grid.states): 
            if state in self.grid.rewards.keys(): 
                self.r[idx] = self.grid.rewards[state]
        print('r is ', self.r)
    
    def next_iteration(self):
        """ 
        TODO: Write code to orchestrate one iteration of PI here.
        """
        return

    def policy_evaluation(self):
        """ 
        TODO: Write code for the policy evaluation step of PI here. That is, update
        the current value estimates using the current policy estimate.
        """
        return

    def policy_improvement(self):
        """
        TODO: Write code to extract the best policy for a given value function here
        """ 
        return

    def convergence_check(self):
        """
        TODO: Write code to check if PI has converged here
        """   
        return
        
    def print_values(self):
        for state, value in self.values.items():
            print(state, value)
    
    def print_policy(self):
        for state, policy in self.policy.items():
            print(state, policy)


if __name__ == "__main__":
    grid = Grid()
    vi = ValueIteration(grid)

    start = time.time()
    print("Initial values:")
    vi.print_values()
    print()

    for i in range(2):
        vi.next_iteration()
        print("Values after iteration", i + 1)
        vi.print_values()
        print()
        """
        if vi.converged():
            break"""







    end = time.time()
    print("Time to complete", i + 1, "VI iterations")
    print(end - start)
    