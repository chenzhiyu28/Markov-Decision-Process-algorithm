from environment import *
from constants import *
from state import *
import heapq

ACTIONS = [[FORWARD], [REVERSE], [SPIN_LEFT], [SPIN_RIGHT],
           [FORWARD, FORWARD], [SPIN_RIGHT, FORWARD],
           [SPIN_LEFT, FORWARD], [SPIN_RIGHT, FORWARD, FORWARD],
           [SPIN_RIGHT, FORWARD, FORWARD], [REVERSE, REVERSE],
           [SPIN_RIGHT, REVERSE], [SPIN_LEFT, REVERSE],
           [SPIN_RIGHT, REVERSE, REVERSE], [SPIN_LEFT, REVERSE, REVERSE]]


class StateNode:
    def __init__(self, env, state, parent, action_from_parent, path_steps: int, path_cost):
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
                successors.append(StateNode(self.env, next_state, self, a, self.path_steps + 1, self.path_cost + cost))
        return successors

    def __lt__(self, other):
        return self.path_cost < other.path_cost

# 返回所有状态，
def bfs(env):
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


env = Environment("ex1.txt")

# win_case: 所有通关stateNode 的集合,无序
# all_states: 所有生成state 的集合
win_case, all_states = bfs(env)
values = {state_Node.state: 10 for state_Node in win_case}

print(f'All possible states:{len(all_states)},\t\tAll win states:{len(win_case)}')
print(len(values))

a = float("inf")
"""
print(env.drift_cw_probs)
print(env.drift_ccw_probs)
print(env.double_move_probs)

{0: 0.05, 1: 0.025, 2: 0.0, 3: 0.0}
{0: 0.05, 1: 0.025, 2: 0.0, 3: 0.0}
{0: 0.25, 1: 0.1, 2: 0.0, 3: 0.0}

"""

forward_stoch = {}
reverse_stoch = {}


a = [1,2,3]
print(a[1])
