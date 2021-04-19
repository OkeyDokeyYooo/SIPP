
# combine two safe intervals
def combine_two(I1, I2):
	result = []

	for i in range(len(I1)):
		for j in range(len(I2)):
			# two interval doesn't have overlap, make sure -1 is infinite 
			if (I1[i][1] < I2[j][0] and I1[i][1] != -1) or (I1[i][0] > I2[j][1] and I2[j][1] != -1):
				continue
			# two interval have overlap
			else: 
				# I1[i] include I2[j]]
				if (I1[i][0] <= I2[j][0] and I1[i][1] >= I2[j][1] and I2[j][1] != -1) or\
				   (I1[i][0] <= I2[j][0] and I1[i][1] == -1 and I2[j][1] != -1) or\
				   (I1[i][0] <= I2[j][0] and I1[i][1] == -1 and I2[j][1] == -1):
					if I2[j] not in result:
						result.append(I2[j])
				# I2[j] include I1[i]
				elif (I2[j][0] <= I1[i][0] and I2[j][1] >= I1[i][1] and I1[i][1] != -1) or\
					 (I2[j][0] <= I1[i][0] and I2[j][1] == -1 and I1[i][1] != -1) or\
					 (I2[j][0] <= I1[i][0] and I2[j][1] == -1 and I1[i][1] == -1):
					if I1[i] not in result:
						result.append(I1[i])
				# overlap but not include
				else:
					if (I2[j][1] >= I1[i][1] >= I2[j][0] and I1[i][1] != -1) or\
					   (I2[j][1] == -1 and I1[i][1] >= I2[j][0]):
						if (I2[j][0], I1[i][1]) not in result:
							result.append((I2[j][0], I1[i][1]))
					elif (I1[i][1] >= I2[j][1] >= I1[i][0] and I2[j][1] != -1) or\
						 (I1[i][1] == -1 and I2[j][1] >= I1[i][0]):
						if (I1[i][0], I2[j][1]) not in result:
							result.append((I1[i][0], I2[j][1]))
	return result


# combine all the safe intervals
def combine_intervals(all_intervals):
	result = []

	# combine first two interval to get first result
	if len(all_intervals) >= 2:
		result = combine_two(all_intervals[0], all_intervals[1])
	else:
		if len(all_intervals) > 0:
			return all_intervals[0]

	# combine each interval with the result
	for i in range(2, len(all_intervals)):
		result = combine_two(result, all_intervals[i])

	return result


# calculate safe interval
def get_safe_interval(cfg, timestep, obstacles):
    each_safe_interval = []

    for path in obstacles:
        index = [] 		# list of index of obstacles that hits the cfg
        temp = []
        for i in range(len(path)):
            if path[i] == cfg:
                index.append(i)

        for i in range(len(index)):
        	# first meet point not no start loc, and time not past
            if index[i] != 0 and i == 0 and index[0] > timestep:
                temp.append((timestep, index[i]-1))

            # not the first meet point and obstacle not stay in meet point
            if i != 0 and index[i-1]+1 != index[i]:
            	# time not past 
            	if index[i-1]+1 <= timestep <= index[i]-1:
            		temp.append((timestep, index[i]-1))
            	elif timestep < index[i]:
            		temp.append((index[i-1]+1, index[i]-1))

            if i == len(index)-1 and index[i] != len(path)-1:
            	if timestep > index[i]+1:
            		temp.append((timestep, -1))
            	else:
            		temp.append((index[i]+1, -1))

        if len(index) != 0:
        	each_safe_interval.append(temp)

    result_interval = combine_intervals(each_safe_interval)
    result_interval.sort(key=lambda x:x[0])

    return result_interval


#all_intervals = [[(0,3), (8,-1)], [(0,0), (4,9), (12,-1)], [(1,7), (9,9), (13,15), (17,-1)]]
#cfg = (0,4)
#timestep = 111
#obstacles = [[(0,0), (0,1), (0,2), (0,3), (0,4), (0,5), (0,6), (0,5), (0,4), (0,3), (0,2), (0,1), (0,0)]]
#print(get_safe_interval(cfg, timestep, obstacles))


#cfg = (2,4)
#cfg = (0,4)
#obstacles = [[(0,0), (0,1), (0,2), (0,3), (0,4), (0,5), (0,6), (0,5), (0,4), (0,3)]]		# answer (0,3), (5,7), (9,-1)
#obstacles = [[(0,4), (0,5), (0,6), (0,5), (0,4), (0,3)]]			# answer (1,3), (5,-1)
#obstacles = [[(0,0), (0,1), (0,2), (0,3), (0,4), (0,4), (0,4), (0,5), (0,6), (0,5), (0,4)]]		# answer (0,3), (7,-1)
#obstacles = [[(0,0), (0,1), (0,2), (0,3), (0,4), (0,5), (0,4)]]
#obstacles = [[(0,4), (0,4), (0,5), (0,4), (0,4), (0,3), (0,4)]]		# answer (2, 2), (5, 5)
#obstacles = [[(8,4), (7,4), (6,4), (5,4), (4,4), (3,4), (2,4), (1,4)], [(4,6), (4,5), (4,4), (4,3), (4,2)]]
#print(get_safe_interval(cfg, 0, obstacles))

#----------------------------- 　find earileast time ------------------------------#
'''

def detect_collision(path1, path2):

    l1, l2 = len(path1), len(path2)
    max_path = max(l1, l2)

    # deep copy of path1 and path2
    temp_path1, temp_path2 = [], []
    for pos in path1: 
        temp_path1.append(pos)
    for pos in path2:
        temp_path2.append(pos)

    if l1 > l2:
        for i in range(l1-l2):
            temp_path2.append(temp_path2[l2-1])
    else:
        for i in range(l2-l1):
            temp_path1.append(temp_path1[l1-1])

    for i in range(max_path):
         # detect vertex collision
        if get_location(temp_path1, i) == get_location(temp_path2, i):
        	print("vertex collision: ", get_location(temp_path1, i), get_location(temp_path2, i))
        	return True

        # detect edge collisions     
        if i < max_path-1:
             if get_location(temp_path1, i) == get_location(temp_path2, i+1) and get_location(temp_path2, i) == get_location(temp_path1, i+1):
             	print("edge collision: ", get_location(temp_path1, i), get_location(temp_path1, i+1), get_location(temp_path2, i), get_location(temp_path2, i+1))
             	return True

    # no collision between two paths
    return False


def find_earliest_arrival(safe_interval, curr_time, curr_loc, next_loc, obstacles):
	# the safe interval is already past
	if curr_time > safe_interval[1]:
		return None
	# move immediately
	if  (safe_interval[0] <= curr_time+1 <= safe_interval[1]) or\
		(safe_interval[0] <= curr_time+1 and safe_interval[1] == -1):
		return 1
	# need to wait then move
	else:
		if safe_interval[1] != -1:
			for i in range(safe_interval[0], safe_interval[1]+1):
				if i - curr_time - 1 < 0:
					continue
				wait_time = i - curr_time
				agent_path = []
				for j in range(wait_time):
					agent_path.append(curr_loc)
				agent_path.append(next_loc)

				for each_path in obstacles:
					print(agent_path, each_path[:len(agent_path)])
					if detect_collision(agent_path, each_path[:len(agent_path)]) == True:
						continue

				return len(agent_path)-1
		else:


#print(find_earliest_arrival([3,3], 0, (3,4), (4,4), obstacles = [[(8,4), (7,4), (6,4), (5,4), (4,4), (3,4), (2,4), (1,4)], [(4,6), (4,5), (4,4), (4,3), (4,2)]]))

'''
#--------------------------------------------------------------------------------------#

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location

def find_earliest_arrival(safe_interval, curr_time, curr_loc, next_loc, obstacles):
	# the safe interval is already past
	if curr_time > safe_interval[1] and safe_interval[1] != -1:
		return None
	# need to wait then move
	wait_time = safe_interval[0] - curr_time - 1
	agent_path = [curr_loc]
	for j in range(wait_time):
		agent_path.append(curr_loc)
	agent_path.append(next_loc)

	for each_obstacle in obstacles:
		temp1 , temp2 = agent_path[:], each_obstacle[curr_time:]
		l1, l2 = len(each_obstacle), len(agent_path)

		'''if l1 > l2:
			for i in range(l1-l2):
				temp2.append(temp2[l2-1])
		else:
			for i in range(l2-l1):
				temp1.append(temp1[l1-1])'''

		print(temp1, temp2)
		# detect edge collision
		for i in range(max(l1, l2)):
			if i < max(l1, l2)-1:
				if get_location(temp1, i) == get_location(temp2, i+1) and get_location(temp2, i) == get_location(temp1, i+1):
					#print("edge collision: ", get_location(temp1, i), get_location(temp1, i+1), get_location(temp2, i), get_location(temp2, i+1))
					return None

	return len(agent_path)-1

print(find_earliest_arrival([3,3], 0, (3,4), (4,4), [[(8,4), (7,4), (6,4), (5,4), (4,4), (3,4), (2,4), (1,4)], [(4,6), (4,5), (4,4), (4,3), (4,2)]]))
#print(find_earliest_arrival((2, -1), 1, (4, 4), (4, 5), [[(8, 4), (7, 4), (6, 4), (5, 4), (4, 4), (3, 4), (2, 4), (1, 4)], [(4, 6), (4, 5), (4, 4), (4, 3), (4, 2)]]))

'''
def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()

    # the goal location is not connect to the start location: No solution!
    if start_loc not in h_values.keys():
        return None
    h_value = h_values[start_loc]   

    # Task 1.2 add constraint_table 
    constraint_table = build_constraint_table(constraints, agent)

    max_time = 0
    if len(constraint_table.keys()) > 0:
        max_time = max(constraint_table.keys())

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], 0)] = root

    # add time limit to avoid infinite loop
    possible_agent_number = 0
    for row in my_map:
        for col in row:
            if col != '@':
                possible_agent_number += 1
    time_limit = (possible_agent_number-1)*possible_agent_number
    # Prioritized planning time limit
    #time_limit = len(constraints) + len(my_map)*len(my_map[0]) 

    while len(open_list) > 0 and time_limit > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['timestep'] >= max_time:
            return get_path(curr)

        # expand curr node 
        for dir in range(5):                                # add wait direction to range 5
            child_loc = move(curr['loc'], dir)

            # set boudary constraint 
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]): 
                continue
            # encounter a block
            if my_map[child_loc[0]][child_loc[1]]:          # the position is True meaning '@': invalid movement
                continue

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}

            # Task 1.2 check constraint, if doesn't satisfied just prune it
            if not is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            # expand the old(in closed list) child if with smaller f-val        
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):                        
                    closed_list[(child['loc'], child['timestep'])] = child     
                    push_node(open_list, child) 

            # expand the child if it isn't in closed list                          
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

        time_limit -= 1

    return None  # Failed to find solutions
'''


