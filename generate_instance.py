import os
import random
import argparse
from pathlib import Path
from itertools import product
from datetime import datetime
from independent import IndependentSolver
import math

ROAD = '.'
WALL = '@'


def generate_start_end_pos(left_coord, num_agents, should_remove=True):

    starts = []
    ends = []
    for agent in range(num_agents):
        agent_coords = random.sample(left_coord, 2)

        if should_remove:
            left_coord.remove(agent_coords[0])
            left_coord.remove(agent_coords[1])

        starts.append(agent_coords[0])
        ends.append(agent_coords[1])

    return starts, ends, left_coord


def init_map(width, height, all_coord, num_walls, should_remove=True):
    # init the binary representation map, True and False
    map = []
    for i in range(0, height):
        row = []
        for j in range(0, width):
            row.append(False)
        map.append(row)

    # keep the number of walls is 10% to 15% of the whole map, hopefully it will create a fence
    walls = []
    for coord in random.sample(all_coord, num_walls):
        x = coord[0]
        y = coord[1]
        # insert WALL into map
        map[x][y] = True

        # remove wall coord from all
        if should_remove:
            all_coord.remove(coord)

    return map, all_coord


def is_valid_map(map, starts_pos, ends_pos):

    try:
        solver = IndependentSolver(map, starts_pos, ends_pos)
        paths = solver.find_solution()
    except BaseException:
        return False

    return True


def write_txt(num, size, map, starts_pos, ends_pos):
    # create new txt file
    with open(os.path.join(path, "test_{}.txt".format(num)), "w") as f:

        # write the size of the map
        f.write("{} {}\n".format(args.size, args.size))

        # write the body of the map
        for i in range(len(map)):
            for j in range(len(map[i])):
                insert = ROAD
                if map[i][j] == True:
                    insert = WALL
                if j == len(map[i]) - 1:
                    f.write(insert)
                else:
                    f.write(insert + " ")
            f.write('\n')

        # write the number of agents
        f.write(str(len(starts_pos)) + "\n")

        # write each start and end pos on the agent
        for i in range(len(starts_pos)):
            f.write("{} {} {} {}\n".format(
                starts_pos[i][0],
                starts_pos[i][1],
                ends_pos[i][0],
                ends_pos[i][1]
            ))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate instance for testing")
    parser.add_argument('--size', type=int, default=8, help='Create Size M x M map')
    parser.add_argument('--num', type=int, default=50, help='Number of Instances to be created')
    parser.add_argument('--wall', action='store_true', default=False, help="Only incease number of walls")
    parser.add_argument('--agent', action='store_true', default=False, help="Only incease number of agents")

    args = parser.parse_args()

    file_name = None
    if args.wall == True:
        assert args.agent == False
        file_name = "wall"
    if args.agent == True:
        assert args.wall == False
        file_name = "agent"

    path = os.path.join(os.getcwd(), "{}_instances_size_{}".format(args.size, file_name))

    Path(path).mkdir(parents=True, exist_ok=True)

    print("Start create test instance ...")

    if args.wall == True:
        all_coord = list(product(range(args.size), range(args.size)))

        num_agents = int(args.size * 0.25)
        starts_pos, ends_pos, left_coord = generate_start_end_pos(all_coord, num_agents)

        for num in range(1, args.num + 1):
            num_walls = int(args.size * args.size * (num - 1) * 0.01)
            instance, _ = init_map(args.size, args.size, left_coord, num_walls, should_remove=False)

            regenerate_map = True

            while regenerate_map:
                if is_valid_map(instance, starts_pos, ends_pos) == False:
                    regenerate_map = True
                    instance, _ = init_map(args.size, args.size, left_coord, num_walls, should_remove=False)
                else:
                    break

            write_txt(num, args.size, instance, starts_pos, ends_pos)

    elif args.agent == True:
        all_coord = list(product(range(args.size), range(args.size)))

        num_walls = int(args.size * args.size * 0.1)
        instance, left_coord = init_map(args.size, args.size, all_coord, num_walls)

        for num in range(1, args.num + 1):
            num_agents = math.ceil(args.size * num * 0.02)

            starts_pos, ends_pos, _ = generate_start_end_pos(left_coord, num_agents, should_remove=False)

            regenerate_map = True

            while regenerate_map:
                if is_valid_map(instance, starts_pos, ends_pos) == False:
                    regenerate_map = True
                    starts_pos, ends_pos, _ = generate_start_end_pos(left_coord, num_agents, should_remove=False)
                else:
                    break

            write_txt(num, args.size, instance, starts_pos, ends_pos)

    print("End processing ...")
