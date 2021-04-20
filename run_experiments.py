#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from cbs_normal import CBSSolver as CBSSolver_normal
from visualize import Animation
from SIPP import get_sum_of_cost
import csv
import signal
from contextlib import contextmanager
import os
import time as timer

# learn setting time limit from https://stackoverflow.com/a/601168


class TimeoutException(Exception):
    pass


@contextmanager
def time_limit(seconds):
    def signal_handler(signum, frame):
        raise TimeoutException("Timed out!")
    signal.signal(signal.SIGALRM, signal_handler)
    signal.alarm(seconds)
    try:
        yield
    finally:
        signal.alarm(0)


SOLVER = "CBS"


def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--size', type=str, default=8,
                        help='The size of the map')

    args = parser.parse_args()

    with open(os.path.join(os.getcwd(), "result/{}_results_size_{}.csv".format(args.solver, args.size)), mode="w") as result_file:
        csv_write = csv.writer(result_file)
        csv_write.writerow(['Instance', 'Node Expanded', 'CPU_time'])

        for file in sorted(glob.glob(args.instance)):

            print("***Import an instance***")
            my_map, starts, goals = import_mapf_instance(file)
            print_mapf_instance(my_map, starts, goals)

            if args.solver == "CBS":
                print("***Run CBS***")
                try:
                    with time_limit(300):
                        start_time = timer.time()
                        cbs = CBSSolver(my_map, starts, goals)
                        paths = cbs.find_solution(args.disjoint)
                        time = timer.time() - start_time
                except TimeoutException as e:
                    time = float("inf")
            elif args.solver == "CBS_N":
                print("**Run CBS With Normal A*")
                try:
                    with time_limit(300):
                        start_time = timer.time()
                        cbs = CBSSolver_normal(my_map, starts, goals)
                        paths = cbs.find_solution(args.disjoint)
                        time = timer.time() - start_time
                except TimeoutException as e:
                    time = float("inf")
            else:
                raise RuntimeError("Unknown solver!")

            if paths is not None:
                cost = get_sum_of_cost(paths)
            else:
                cost = "None"

            csv_write.writerow([file, cost, "{:.5f}".format(time)])

            if not args.batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)
                # animation.save("output.mp4", 1.0)
                animation.show()
