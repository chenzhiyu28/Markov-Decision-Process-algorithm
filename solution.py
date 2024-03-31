import sys
import time
from constants import *
from environment import *
from state import State

"""
solution.py

This file is a template you should use to implement your solution.

You should implement each section below which contains a TODO comment.

COMP3702 2022 Assignment 2 Support Code

Last updated by njc 08/09/22
"""


class StateNode:
    def __init__(self, env, state, parent, action_from_parent, path_steps: int,
                 path_cost):
        """
        :param env: environment
        :param state: state belonging to this node
        :param parent: parent of this node
        :param action_from_parent: LEFT, RIGHT, UP, or DOWN
        """
        self.env = env
        self.state = state
        self.parent = parent
        self.action_from_parent = action_from_parent
        self.path_steps = path_steps
        self.path_cost = path_cost

    def get_path(self):
        """
        :return: A list of actions
        """
        path = []
        cur = self
        while cur.action_from_parent is not None:
            path.append(cur.action_from_parent)
            cur = cur.parent
        path.reverse()
        return path

    def get_successors(self):
        """
        :return: A list of successor StateNodes
        """
        successors = []

        for a in ROBOT_ACTIONS:
            # cost 是真实成本
            cost, next_state = self.env.apply_dynamics(self.state, a)
            if next_state != self.state:
                successors.append(StateNode(self.env, next_state, self, a,
                                            self.path_steps + 1,
                                            self.path_cost + cost))
        return successors

    def __lt__(self, other):
        return self.path_cost < other.path_cost


class Solver:

    def __init__(self, environment: Environment):
        # 环境， 收敛值， 折扣
        self.environment = environment
        self.EPSILON = environment.epsilon
        self.gamma = environment.gamma

        # 初始state， 完成stateNode集合， 所有state集合
        self.initial_state = self.environment.get_init_state()
        self.win_case, self.all_states = self.bfs(self.environment)

        # probs for stoch action
        cw = environment.drift_cw_probs
        ccw = environment.drift_ccw_probs
        double_move = environment.double_move_probs
        self.forward_prob = (1 - double_move[FORWARD]) * (
                    1 - cw[FORWARD] - ccw[FORWARD])
        self.double_forward_prob = (double_move[FORWARD]) * (
                    1 - cw[FORWARD] - ccw[FORWARD])
        self.cw_forward_prob = (1 - double_move[FORWARD]) * (cw[FORWARD])
        self.ccw_forward_prob = (1 - double_move[FORWARD]) * (ccw[FORWARD])
        self.cw_double_forward_prob = (double_move[FORWARD]) * (cw[FORWARD])
        self.ccw_double_forward_prob = (double_move[FORWARD]) * (ccw[FORWARD])

        self.reverse_prob = (1 - double_move[REVERSE]) * (
                    1 - cw[REVERSE] - ccw[REVERSE])
        self.double_reverse_prob = (double_move[REVERSE]) * (
                    1 - cw[REVERSE] - ccw[REVERSE])
        self.cw_reverse_prob = (1 - double_move[REVERSE]) * (cw[REVERSE])
        self.ccw_reverse_prob = (1 - double_move[REVERSE]) * (ccw[REVERSE])
        self.cw_double_reverse_prob = (double_move[REVERSE]) * (cw[REVERSE])
        self.ccw_double_reverse_prob = (double_move[REVERSE]) * (ccw[REVERSE])

        self.forward_stoch = {tuple([FORWARD]): self.forward_prob,
                              tuple([FORWARD, FORWARD]): self.double_forward_prob,
                              tuple([SPIN_RIGHT, FORWARD]): self.cw_forward_prob,
                              tuple([SPIN_LEFT, FORWARD]): self.ccw_forward_prob,
                              tuple([SPIN_RIGHT, FORWARD, FORWARD]): self.cw_double_forward_prob,
                              tuple([SPIN_LEFT, FORWARD, FORWARD]): self.ccw_double_forward_prob}

        self.reverse_stoch = {tuple([REVERSE]): self.reverse_prob,
                              tuple([REVERSE, REVERSE]): self.double_reverse_prob,
                              tuple([SPIN_RIGHT, REVERSE]): self.cw_reverse_prob,
                              tuple([SPIN_LEFT, REVERSE]): self.ccw_reverse_prob,
                              tuple([SPIN_RIGHT, REVERSE, REVERSE]): self.cw_double_reverse_prob,
                              tuple([SPIN_LEFT, REVERSE, REVERSE]): self.ccw_double_reverse_prob}

        # dimensions for obstacles
        obstacle_map = self.environment.obstacle_map
        self.obstacles = []
        for line_index in range(len(obstacle_map)):
            line = obstacle_map[line_index]
            for row_index in range(len(line)):
                row = line[row_index]
                if row == 1:
                    self.obstacles.append((line_index, row_index))

        # dimensions for hazard
        hazard_map = self.environment.hazard_map
        self.hazard = []
        for line_index in range(len(hazard_map)):
            line = hazard_map[line_index]
            for row_index in range(len(line)):
                row = line[row_index]
                if row == 1:
                    self.hazard.append((line_index, row_index))

        self.vi_initialise()
        self.pi_initialise()
        #
        # TODO: Define any class instance variables you require (e.g. dictionary mapping state to VI value) here.
        #

    def bfs(self, env):
        container = [StateNode(env, env.get_init_state(), None, None, 0, 0)]
        visited = set()
        # all success nodes
        success_nodes = set()

        n_expanded = 0
        solution = None
        while len(container) > 0:
            # expand node
            node = container.pop(0)

            # test for goal
            if env.is_solved(node.state):
                # node is a StateNode instance
                solution = node
                success_nodes.add(node)

            # add successors
            successors = node.get_successors()
            for s in successors:
                if s.state not in visited:
                    container.append(s)
                    visited.add(s.state)
            n_expanded += 1
            """
            if solution is not None:
                print(
                    f'Visited Nodes: {len(visited)},\t\tExpanded Nodes: {n_expanded},\t\t'
                    f'Nodes in Container: {len(container)}')
                print(f'Cost of Path (with Costly Moves): {solution.path_cost}')
            """
            # success_nodes: 所有通关stateNode 的集合,无序
            # visited: 所有生成state 的集合
        if solution is not None:
            return success_nodes, visited
        return None

    # === Value Iteration ==============================================================================================

    def vi_initialise(self):
        """
        Initialise any variables required before the start of Value Iteration.
        """
        self.values = {state_Node.state: 10 for state_Node in self.win_case}
        self.state_action = {}
        self.policy = {}
        self.converged = False
        self.differences = []
        #
        # TODO: Implement any initialisation for Value Iteration (e.g. building a list of states) here. You should not
        #  perform value iteration in this method.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #

    def vi_is_converged(self):
        """
        Check if Value Iteration has reached convergence.
        :return: True if converged, False otherwise
        """
        #
        # TODO: Implement code to check if Value Iteration has reached convergence here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        if (len(self.differences) != 0) and max(self.differences) <= self.EPSILON:
            return True
        return False

    def vi_iteration(self):
        """
        Perform a single iteration of Value Iteration (i.e. loop over the state space once).
        """
        #
        # TODO: Implement code to perform a single iteration of Value Iteration here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        self.differences.clear()
        # 排除 solved的 state
        for s in self.all_states:
            if not self.environment.is_solved(s):
                action_value = {}

                # 4次循环
                for action in ROBOT_ACTIONS:

                    # ① 不会有随机事件
                    if action == SPIN_LEFT or action == SPIN_RIGHT:
                        cost, new_state = self.environment.apply_dynamics(s, action)

                        # 写入action_value{action:value}
                        action_value[action] = (1 * cost) + (self.gamma * self.values.get(new_state, 0))

                    # ② 前进
                    elif action == FORWARD:
                        sum_value = 0
                        costs = []

                        # 返回包含组合 action 的 tuple ((FORWARD, FORWARD))
                        for stoch_action in self.forward_stoch.keys():
                            new_state = s
                            for single_action in stoch_action:
                                cost, new_state = self.environment.apply_dynamics(new_state, single_action)
                                costs.append(1 * cost)
                            cost = max(costs)
                            sum_value += self.forward_stoch[stoch_action] * (cost + self.gamma * self.values.get(new_state, 0))

                        # 写入action_value{action:value}
                        action_value[action] = sum_value

                    # ③ 后退
                    elif action == REVERSE:
                        sum_value = 0
                        costs = []

                        # 返回包含组合 action 的 tuple ((REVERSE, REVERSE))
                        for stoch_action in self.reverse_stoch.keys():
                            new_state = s
                            for single_action in stoch_action:
                                cost, new_state = self.environment.apply_dynamics(new_state, single_action)
                                costs.append(1 * cost)
                            cost = max(costs)
                            sum_value += self.reverse_stoch[stoch_action] * (cost + self.gamma * self.values.get(new_state, 0))
                        action_value[action] = sum_value

                # 一个 state 对应 4个 action 的 value 都在 action_value中
                actions = []
                values = []
                for action, value in action_value.items():
                    actions.append(action)
                    values.append(value)
                max_value = max(values)
                wise_action = actions[values.index(max(values))]

                self.differences.append(abs(max_value - self.values.get(s, 0)))
                self.values[s] = max_value
                self.state_action[s] = wise_action

    def vi_plan_offline(self):
        """
        Plan using Value Iteration.
        """
        # !!! In order to ensure compatibility with tester, you should not modify this method !!!
        self.vi_initialise()
        while not self.vi_is_converged():
            self.vi_iteration()

    def vi_get_state_value(self, state: State):
        """
        Retrieve V(s) for the given state.
        :param state: the current state
        :return: V(s)
        """
        #
        # TODO: Implement code to return the value V(s) for the given state (based on your stored VI values) here. If a
        #  value for V(s) has not yet been computed, this function should return 0.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        return self.values.get(state, 0)

    def vi_select_action(self, state: State):
        """
        Retrieve the optimal action for the given state (based on values computed by Value Iteration).
        :param state: the current state
        :return: optimal action for the given state (element of ROBOT_ACTIONS)
        """
        #
        # TODO: Implement code to return the optimal action for the given state (based on your stored VI values) here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        return self.state_action.get(state, FORWARD)

    # === Policy Iteration =============================================================================================

    def pi_initialise(self):
        """
        Initialise any variables required before the start of Policy Iteration.
        """
        #
        # TODO: Implement any initialisation for Policy Iteration (e.g. building a list of states) here. You should not
        #  perform policy iteration in this method. You should assume an initial policy of always move FORWARDS.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        self.pi_values = {state_Node.state: 30 for state_Node in self.win_case}
        self.pi_state_action = {}

        self.pi_policy = {state: SPIN_LEFT for state in self.all_states}

        self.pi_iter_times = 0
        self.pi_policy_record = {0: self.pi_policy}

    def pi_is_converged(self):
        """
        Check if Policy Iteration has reached convergence.
        :return: True if converged, False otherwise
        """
        #
        # TODO: Implement code to check if Policy Iteration has reached convergence here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        for i in range(len(self.pi_policy_record) - 1):
            if (self.pi_policy_record[i] == self.pi_policy_record[i+1]) and self.pi_iter_times >= 2:
                return True
        return False

    def pi_iteration(self):
        """
        Perform a single iteration of Policy Iteration (i.e. perform one step of policy evaluation and one step of
        policy improvement).
        """
        #
        # TODO: Implement code to perform a single iteration of Policy Iteration (evaluation + improvement) here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        # update policy
        self.pi_iter_times += 1

        value_converged = False

        while not value_converged:
            # new_values 还必须放到while里面来，神奇
            new_values = {state_Node.state: 30 for state_Node in self.win_case}
            # 排除 solved的 所有state
            for s in self.all_states:
                if not self.environment.is_solved(s):

                    # 只用计算一种action
                    action = self.pi_policy[s]

                    # ① 无随机事件的 左右转
                    if action == SPIN_LEFT or action == SPIN_RIGHT:
                        cost, new_state = self.environment.apply_dynamics(s, action)
                        # 写入 new_values {s:value}
                        new_values[s] = cost + (self.gamma * self.pi_values.get(new_state, 0))

                    # ② 前进
                    elif action == FORWARD:
                        sum_value = 0
                        costs = []

                        # 返回包含组合 action 的 tuple ((FORWARD, FORWARD))
                        for stoch_action in self.forward_stoch.keys():
                            new_state = s
                            for single_action in stoch_action:
                                cost, new_state = self.environment.apply_dynamics(new_state, single_action)
                                costs.append(cost)
                            cost = max(costs)
                            sum_value += self.forward_stoch[stoch_action] * (cost + self.gamma * self.pi_values.get(new_state, 0))

                        # 写入 new_values{s:value}
                        new_values[s] = sum_value


                    # ③ 后退
                    elif action == REVERSE:
                        sum_value = 0
                        costs = []

                        # 返回包含组合 action 的 tuple ((REVERSE, REVERSE))
                        for stoch_action in self.reverse_stoch.keys():
                            new_state = s
                            for single_action in stoch_action:
                                cost, new_state = self.environment.apply_dynamics(new_state, single_action)
                                costs.append(cost)
                            cost = max(costs)
                            sum_value += self.reverse_stoch[stoch_action] * (cost + self.gamma * self.pi_values.get(new_state, 0))
                        new_values[s] = sum_value

            differences = [abs(self.pi_values.get(s, 0) - new_values[s]) for s in self.all_states]

            if max(differences) < self.EPSILON * 5:
                value_converged = True

            self.pi_values = new_values

        self.pi_improvement()

    def pi_plan_offline(self):
        """
        Plan using Policy Iteration.
        """
        # !!! In order to ensure compatibility with tester, you should not modify this method !!!
        self.pi_initialise()
        while not self.pi_is_converged():
            self.pi_iteration()

    def pi_select_action(self, state: State):
        """
        Retrieve the optimal action for the given state (based on values computed by Value Iteration).
        :param state: the current state
        :return: optimal action for the given state (element of ROBOT_ACTIONS)
        """
        #
        # TODO: Implement code to return the optimal action for the given state (based on your stored PI policy) here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        return self.pi_policy.get(state, FORWARD)

    # 根据已有value, 更新action
    def pi_improvement(self):

        new_policy = {state_Node.state: FORWARD for state_Node in self.win_case}

        # 排除 solved的 所有state
        for s in self.all_states:
            if not self.environment.is_solved(s):
                action_value = {}
                state_policy = {}

                # 4次循环
                for action in ROBOT_ACTIONS:

                    # ① 不会有随机事件
                    if action == SPIN_LEFT or action == SPIN_RIGHT:
                        cost, new_state = self.environment.apply_dynamics(s, action)

                        # 写入action_value{action:value}
                        action_value[action] = cost + (self.gamma * self.pi_values.get(new_state, 0))

                    # ② 前进
                    elif action == FORWARD:
                        sum_value = 0
                        costs = []

                        # 返回包含组合 action 的 tuple ((FORWARD, FORWARD))
                        for stoch_action in self.forward_stoch.keys():
                            new_state = s
                            for single_action in stoch_action:
                                cost, new_state = self.environment.apply_dynamics(new_state, single_action)
                                costs.append(cost)
                            # cost实际为最高的那个
                            cost = max(costs)
                            sum_value += self.forward_stoch[stoch_action] * (cost + self.gamma * self.pi_values.get(new_state, 0))

                        # 写入action_value{action:value}
                        action_value[action] = sum_value

                    # ③ 后退
                    elif action == REVERSE:
                        sum_value = 0
                        costs = []

                        # 返回包含组合 action 的 tuple ((REVERSE, REVERSE))
                        for stoch_action in self.reverse_stoch.keys():
                            new_state = s
                            for single_action in stoch_action:
                                cost, new_state = self.environment.apply_dynamics(new_state, single_action)
                                costs.append(cost)
                            cost = max(costs)
                            sum_value += self.reverse_stoch[stoch_action] * (cost + self.gamma * self.pi_values.get(new_state, 0))
                        action_value[action] = sum_value

                # 一个 state 对应 4个 action 的 value 都在 action_value中
                actions = []
                values = []
                for action, value in action_value.items():
                    actions.append(action)
                    values.append(value)
                max_value = max(values)
                wise_action = actions[values.index(max(values))]

                new_policy[s] = wise_action

        self.pi_policy = new_policy
        self.pi_policy_record[self.pi_iter_times] = self.pi_policy



    # === Helper Methods ===============================================================================================
    #
    #
    # TODO: Add any additional methods here
    #
    #
