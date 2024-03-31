from environment import *
from constants import *
from solution import *

env = Environment("ex3.txt")

"""
print(env.widget_init_posits)
print(env.widget_init_orients)

# 收敛值
print(env.epsilon)
# discount
print(env.gamma)


# 漂移概率
print(env.drift_cw_probs)
print(env.drift_ccw_probs)
print(env.double_move_probs)

# 碰撞惩罚
print(env.collision_penalty)
print(env.hazard_penalty)


ACTIONS = [[FORWARD], [REVERSE], [SPIN_LEFT], [SPIN_RIGHT],
           [FORWARD, FORWARD], [SPIN_RIGHT, FORWARD],
           [SPIN_LEFT, FORWARD], [SPIN_RIGHT, FORWARD, FORWARD],
           [SPIN_RIGHT, FORWARD, FORWARD], [REVERSE, REVERSE],
           [SPIN_RIGHT, REVERSE], [SPIN_LEFT, REVERSE],
           [SPIN_RIGHT, REVERSE, REVERSE], [SPIN_LEFT, REVERSE, REVERSE]]

print('------------')
"""
s = Solver(env)

for i in range(1):
    s.pi_iteration()
"""

print(s.pi_policy)
print(s.pi_values)

print(len(s.pi_policy))
print(len(s.all_states))
"""