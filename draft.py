from environment import *
from constants import *

env1 = Environment("ex1.txt")
state1 = env1.get_init_state()

env9 = Environment("ex9.txt")
"""
返回movement
apply_action_noise(FORWARD)
"""
"""
2 返回 obstacles 坐标
print(env1.obstacle_map)
obstacle_map = env1.obstacle_map
obstacles = []
for line_index in range(len(obstacle_map)):
    line = map[line_index]
    for row_index in range(len(line)):
        row = line[row_index]
        if row == 1:
            obstacles.append((line_index, row_index))
print(obstacles)
"""

"""
3 返回 hazard 坐标
hazard_map = env9.hazard_map
hazard = []
for line_index in range(len(hazard_map)):
    line = hazard_map[line_index]
    for row_index in range(len(line)):
        row = line[row_index]
        if row == 1:
            hazard.append((line_index, row_index))
print(hazard_map)
print(hazard)
"""

"""
4 widget占用坐标
widget_cells = [
    widget_get_occupied_cells(self.widget_types[i], state.widget_centres[i],
                              state.widget_orients[i]) for i in range(self.n_widgets)]
"""


def widget_not_collide(widget_occupied_cells: list[tuple], environment: Environment, agent_x: int, agent_y: int) -> bool:
    for widget_occupied in widget_occupied_cells:
        x, y = widget_occupied
        if x >= environment.n_rows or y >= environment.n_cols:
            return False
        if environment.obstacle_map[x][y] or environment.hazard_map[x][y] or (x, y) == (agent_x, agent_y):
            return False
    return True


all_states = []
env = Environment("ex1.txt")
"""
find all possible states in the env
1. robot not collide with obstacle or hazard
2. widget cell not collide with obstacle or hazard
3. widget cell not collide with robot
4. widget cell inside boundary
"""

widget_type = env.widget_types[0]
widget_oris = {"3": WIDGET3_ORIENTATIONS, "4": WIDGET4_ORIENTATIONS, "5": WIDGET5_ORIENTATIONS}
# all conditions of robot position
for robot_x in range(env.n_rows):
    for robot_y in range(env.n_cols):
        if env.obstacle_map[robot_x][robot_y] or env.hazard_map[robot_x][robot_y]:
            continue  # 验证不在墙上,也不在danger里 print(robot_x ,robot_y)

        # all conditions of robot orientation
        for robot_orientation in ROBOT_ORIENTATIONS:

            # all possible conditions of widget center
            # assume only 1 widget in each map
            for widget_x in range(env.n_rows):
                for widget_y in range(env.n_cols):
                    if env.obstacle_map[widget_x][widget_y] or env.hazard_map[widget_x][widget_y] or (widget_x, widget_y) == (robot_x, robot_y):
                        continue

                    # each widget center, try different orientation
                    for widget_orientation in widget_oris[widget_type]:
                        widget_cells = widget_get_occupied_cells(widget_type, (widget_x, widget_y), widget_orientation)

                        if widget_not_collide(widget_cells, env, robot_x, robot_y):
                            new_state = State(env, (robot_x, robot_y), robot_orientation, [(widget_x, widget_y)], tuple([widget_orientation]))
                            if new_state not in all_states:
                                all_states.append(new_state)

print(len(all_states))


"""
self.environment = environment
self.robot_posit = robot_posit  # (row,col)
self.robot_orient = robot_orient  # ROBOT_UP
self.widget_centres = widget_centres  # (row,col)
self.widget_orients = widget_orients  # (VERTICAL)
self.force_valid = force_valid
"""














