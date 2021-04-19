import os
import random
import argparse
from pathlib import Path
from itertools import product

ROAD = '.'
WALL = '@'


def init_maze(width, height):
    # init all the ROAD first, and then insert WALL
    maze = []
    for i in range(0, height):
        row = []
        for j in range(0, width):
            row.append(ROAD)
        maze.append(row)

    all_coord = list(product(range(height), range(width)))
    # keep the number of walls is 10% to 15% of the whole map, hopefully it will create a fence
    num_walls_min = int(width * height * 0.1)
    num_walls_max = int(width * height * 0.15)
    walls = []
    for coord in random.sample(all_coord, random.randint(num_walls_min, num_walls_max)):
        x = coord[0]
        y = coord[1]
        # insert WALL into maze
        maze[x][y] = WALL

        # remove wall coord from all
        all_coord.remove(coord)

    return maze, all_coord


def generate_start_end_pos(left_coord, size):
    # determine number of agent, keep it to 60% to 80% of the SIZE ?
    num_agent = random.randint(int(size * 0.5), int(size * 0.8))
    result = []
    for agent in range(num_agent):
        agent_coords = random.sample(left_coord, 2)

        left_coord.remove(agent_coords[0])
        left_coord.remove(agent_coords[1])

        result.append(agent_coords)

    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate instance for testing")
    parser.add_argument('--size', type=int, default=8, help='Create Size M x M map')
    parser.add_argument('--num', type=int, default=50, help='Number of Instances to be created')

    args = parser.parse_args()

    maze, left_coord = init_maze(args.size, args.size)

    all_pos = generate_start_end_pos(left_coord, args.size)

    path = os.path.join(os.getcwd(), "instance_size_{}".format(args.size))

    Path(path).mkdir(parents=True, exist_ok=True)

    # start writing the input file
    for num in range(1, args.num + 1):
        print("Start create test instance ...")

        # create new txt file
        with open(os.path.join(path, "test_{}.txt".format(num)), "w") as f:

            # write the size of the maze
            f.write("{} {}\n".format(args.size, args.size))

            # write the body of the maze
            for i in range(len(maze)):
                for j in range(len(maze[i])):
                    insert = maze[i][j]
                    if j == len(maze[i]) - 1:
                        f.write(insert)
                    else:
                        f.write(insert + " ")
                f.write('\n')

            # write the number of agents
            f.write(str(len(all_pos)) + "\n")

            # write each start and end pos on the agent
            for pos in all_pos:
                f.write("{} {} {} {}\n".format(
                    pos[0][0],
                    pos[0][1],
                    pos[1][0],
                    pos[1][1]
                ))

        print("End processing ...")
