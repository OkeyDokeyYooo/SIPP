import heapq
import random
from pathlib import Path


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]     # add wait direction
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def get_location(path, time):
    if len(path) == 0:
        raise BaseException('path empty')
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = [goal_node['loc']]
    curr = goal_node
    if curr['parent'] == None:
        return path

    while curr['parent'] is not None:
        for i in range(curr['wait_time']+1):
            path.append(curr['parent']['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def push_node(open_list, node):
    #print("generate node:")
    # print(node)
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node['safe_interval'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    #print("expand node:")
    # print(curr)
    return curr


def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.

    constraint_table = dict()
    temp_constraints = []

    # deep copy constraints
    for constraint in constraints:
        temp_constraints.append(constraint)

    for constraint in temp_constraints:
        # the constraint belong to the agent if the constraint is specific for
        if constraint['agent'] == agent:
            time_step = -1
            if type(constraint['timestep']) == type(1):
                time_step = constraint['timestep']
            # new time step
            if time_step not in constraint_table.keys():
                constraint_table[time_step] = []
            constraint_table[time_step].append(constraint)

    return constraint_table


# calculate safe interval
def get_safe_interval(curr_loc, cfg, timestep, constraint_table):
    safe_interval = []
    constraint_set = []
    infinite_constraint = []
    last_time = None

    for (time, constraints) in constraint_table.items():
        for constraint in constraints:
            # vertex constraint
            if len(constraint['loc']) == 1:
                if constraint['loc'][0] == cfg:
                    if time != -1:
                        if constraint['timestep'] not in constraint_set:
                            constraint_set.append(constraint['timestep'])
                    else:
                        infinite_constraint.append(constraint['timestep'])
                        last_time = constraint['timestep'][1]
            # edge constraint
            else:
                if curr_loc == constraint['loc'][0] and cfg == constraint['loc'][1] and constraint['timestep'] not in constraint_set:
                    constraint_set.append(constraint['timestep']-1)
                    constraint_set.append(constraint['timestep'])

    constraint_set.sort()
    constraint_set += infinite_constraint

    # no constraint
    if len(constraint_set) == 0:
        return [(timestep, -1)]

    for i in range(len(constraint_set)):
        # first meet point not no start loc, and time not past
        if i == 0 and type(constraint_set[i-1]) == type(1) and constraint_set[i] != 0 and constraint_set[i] > timestep:
            safe_interval.append((timestep, constraint_set[i]-1))

        # not the first meet point and obstacle not stay in meet point
        if i != 0 and type(constraint_set[i-1]) == type(1) and constraint_set[i-1]+1 != constraint_set[i]:
            # time not past
            if constraint_set[i-1]+1 <= timestep <= constraint_set[i]-1:
                safe_interval.append((timestep, constraint_set[i]-1))
            elif timestep < constraint_set[i]:
                safe_interval.append((constraint_set[i-1]+1, constraint_set[i]-1))

        if i == len(constraint_set)-1 and constraint_set[i] != last_time:
            if timestep > constraint_set[i]+1:
                safe_interval.append((timestep, -1))
            else:
                safe_interval.append((constraint_set[i]+1, -1))

    safe_interval.sort(key=lambda x: x[0])

    return safe_interval


# set time limit
def set_time_limit(my_map):
    possible_agent_number = 0
    for row in my_map:
        for col in row:
            if col != '@':
                possible_agent_number += 1
    time_limit = (possible_agent_number-1)*possible_agent_number

    return time_limit


# earliest arrival time at cfg during interval i with no collisions
def find_earliest_arrival(safe_interval, curr_time):
    # the safe interval is already past
    if curr_time > safe_interval[1] and safe_interval[1] != -1:
        return None
    # need to wait then move
    wait_time = safe_interval[0] - curr_time - 1
    if wait_time > 0:
        return wait_time + 1
    else:
        return 1


# expand node to get all valid successors
def get_successors(curr, my_map, h_values, constraint_table):
    successors = []

    for dir in range(4):
        cfg = move(curr['loc'], dir)

        # set boudary constraint
        if cfg[0] < 0 or cfg[1] < 0 or cfg[0] >= len(my_map) or cfg[1] >= len(my_map[0]):
            continue
        # encounter a block
        if my_map[cfg[0]][cfg[1]]:		# the position is True meaning '@': invalid movement
            continue

        start_t = curr['timestep'] + 1		# the m_time is alwasy 1
        end_t = curr['safe_interval'][1]
        if curr['safe_interval'][1] != -1:
            end_t = curr['safe_interval'][1] + 1

    # all safe inervals in cfg
        cfg_safe_intervals = get_safe_interval(curr['loc'], cfg, curr['timestep']+1, constraint_table)

        for each_SI in cfg_safe_intervals:
            if (each_SI[0] > end_t and end_t != -1) or (each_SI[1] < start_t and each_SI[1] != -1):
                continue
        # earliest arrival time at cfg during interval i with no collisions
            t = find_earliest_arrival(each_SI, curr['timestep'])
            if t is None:
                continue

            successor = {'loc': cfg, 'g_val': curr['g_val']+t, 'h_val': h_values[cfg], 'parent': curr,
                         'timestep': curr['timestep']+t, 'safe_interval': each_SI, 'wait_time': 0}

            if t > 1:
                successor['wait_time'] += t - 1
                '''
				print()
				print("> 1 is: ")
				print(curr)
				print("cfg: ", cfg)
				print()'''

            successors.append(successor)

    return successors


# safe interval A start search
def a_star_safe_interval(my_map, start_loc, goal_loc, h_values, agent, constraints):
    # open list and closed list
    open_list = []
    closed_list = dict()

    # the goal location is not connect to the start location: No solution!
    if start_loc not in h_values.keys():
        return None

    # initialize h value
    h_value = h_values[start_loc]

    # constraints table
    constraint_table = build_constraint_table(constraints, agent)

    # note root only choose the first interval of all save intervals
    root_safe_interval = get_safe_interval(start_loc, start_loc, 0, constraint_table)[0]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0, 'safe_interval': root_safe_interval, 'wait_time': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root_safe_interval)] = root

    max_time = 0
    if len(constraint_table.keys()) > 0:
        max_time = max(constraint_table.keys())

    # add time limit to avoid infinite loop
    time_limit = set_time_limit(my_map)

    while len(open_list) > 0 and time_limit > 0:
        curr = pop_node(open_list)

        # find solution
        if curr['loc'] == goal_loc and curr['timestep'] >= max_time:		# safe interval need to be -1 avoid obstacle hit agent after get goal
            '''print("find solution:")
            print()
            print(curr)'''
            return get_path(curr)

        # expand curr node, get all successors
        successors = get_successors(curr, my_map, h_values, constraint_table)

        for successor in successors:
            # expand the old(in closed list) child if with smaller f-val
            if (successor['loc'], successor['safe_interval']) in closed_list:
                existing_node = closed_list[(successor['loc'], successor['safe_interval'])]
                if compare_nodes(successor, existing_node):
                    closed_list[(successor['loc'], successor['safe_interval'])] = successor
                    print("changed")
                    push_node(open_list, successor)

            # expand the child if it isn't in closed list
            else:
                closed_list[(successor['loc'], successor['safe_interval'])] = successor
                push_node(open_list, successor)

        time_limit -= 1

    return None


'''
my_map = [[1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [0,0,0,0,0,0,0,0,0],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1],
		  [1,1,1,1,0,1,1,1,1]]
h_values = compute_heuristics(my_map, (8,4))
curr = {'loc': (3,4), 'g_val': 0, 'h_val': 5, 'parent': None, 'timestep': 0, 'safe_interval': [0,4]}
obstacles = [[(8,4), (7,4), (6,4), (5,4), (4,4), (3,4), (2,4), (1,4)], [(4,6), (4,5), (4,4), (4,3), (4,2)]]
print(a_star_safe_interval(my_map, curr['loc'], (8,4), h_values, obstacles))

#expand node:{'loc': (2, 4), 'g_val': 1, 'h_val': 6, 'parent': {'loc': (3, 4), 'g_val': 0, 'h_val': 5, 'parent': None, 'timestep': 0, 'safe_interval': (0, 4)}, 'timestep': 1, 'safe_interval': (1, 5)}
'''


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == "__main__":
    my_map, starts, goals = import_mapf_instance("instances/test_47.txt")

    h_values = compute_heuristics(my_map, goals[6])
    constraints = [{'agent': 0, 'loc': [(4, 3)], 'timestep': 5, 'positive': 0}, {'agent': 3, 'loc': [(5, 3), (5, 4)], 'timestep': 6, 'positive': 0}, {'agent': 4, 'loc': [(3, 4)], 'timestep': 3, 'positive': 0}, {'agent': 4, 'loc': [(4, 4), (3, 4)], 'timestep': 4, 'positive': 0}, {'agent': 4, 'loc': [(4, 4)], 'timestep': 4, 'positive': 0}, {'agent': 2, 'loc': [(4, 3)], 'timestep': 5, 'positive': 0}, {'agent': 2, 'loc': [(3, 3), (4, 3)], 'timestep': 6, 'positive': 0}, {'agent': 2, 'loc': [(1, 4)], 'timestep': 1, 'positive': 0}, {'agent': 1, 'loc': [(5, 3), (4, 3)], 'timestep': 5, 'positive': 0}, {
        'agent': 2, 'loc': [(4, 3)], 'timestep': 6, 'positive': 0}, {'agent': 2, 'loc': [(3, 3), (4, 3)], 'timestep': 7, 'positive': 0}, {'agent': 5, 'loc': [(5, 3)], 'timestep': 5, 'positive': 0}, {'agent': 5, 'loc': [(5, 3)], 'timestep': 6, 'positive': 0}, {'agent': 5, 'loc': [(4, 3)], 'timestep': 6, 'positive': 0}, {'agent': 1, 'loc': [(5, 1)], 'timestep': 2, 'positive': 0}, {'agent': 2, 'loc': [(4, 4)], 'timestep': 5, 'positive': 0}, {'agent': 4, 'loc': [(3, 3)], 'timestep': 4, 'positive': 0}, {'agent': 4, 'loc': [(3, 4)], 'timestep': 5, 'positive': 0}]
    print(a_star_safe_interval(my_map, starts[6], goals[6], h_values, 6, constraints))
