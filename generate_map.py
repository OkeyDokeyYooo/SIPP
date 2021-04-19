import os
import random
import argparse
from pathlib import Path
from itertools import product
from datetime import datetime
from independent import IndependentSolver

ROAD = '.'
WALL = '@'


def init_map(width, height):
    # init the binary representation map, True and False
    map = []
    for i in range(0, height):
        row = []
        for j in range(0, width):
            row.append(False)
        map.append(row)

    all_coord = list(product(range(height), range(width)))

    # keep the number of walls is 10% to 15% of the whole map, hopefully it will create a fence
    num_walls_min = int(width * height * 0.1)
    num_walls_max = int(width * height * 0.15)
    walls = []
    for coord in random.sample(all_coord, random.randint(num_walls_min, num_walls_max)):
        x = coord[0]
        y = coord[1]
        # insert WALL into map
        map[x][y] = True

        # remove wall coord from all
        all_coord.remove(coord)

    return map, all_coord


def is_valid_map(map, starts_pos, ends_pos):

    try:
        solver = IndependentSolver(map, starts_pos, ends_pos)
        paths = solver.find_solution()
    except BaseException:
        return False

    return True


def generate_start_end_pos(left_coord, size):
    # determine number of agent, keep it to 50% to 80% of the SIZE ?
    num_agent = random.randint(int(size * 0.5), int(size * 0.8))

    starts = []
    ends = []
    for agent in range(num_agent):
        agent_coords = random.sample(left_coord, 2)

        left_coord.remove(agent_coords[0])
        left_coord.remove(agent_coords[1])

        starts.append(agent_coords[0])
        ends.append(agent_coords[1])

    return starts, ends


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate instance for testing")
    parser.add_argument('--size', type=int, default=8, help='Create Size M x M map')
    parser.add_argument('--num', type=int, default=50, help='Number of Instances to be created')

    args = parser.parse_args()

    path = os.path.join(os.getcwd(), "instances_size_{}".format(args.size))

    Path(path).mkdir(parents=True, exist_ok=True)

    print("Start create test instance ...")
    # start writing the input file
    for num in range(1, args.num + 1):

        random.seed(datetime.now())

        # creating a new map
        map, left_coord = init_map(args.size, args.size)
        starts_pos, ends_pos = generate_start_end_pos(left_coord, args.size)

        # check if this is map is valid: each agent has valid path

        regenerate_map = True

        while regenerate_map:
            if is_valid_map(map, starts_pos, ends_pos) == False:
                regenerate_map = True
                map, left_coord = init_map(args.size, args.size)
                starts_pos, ends_pos = generate_start_end_pos(left_coord, args.size)
            else:
                break

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

    print("End processing ...")
