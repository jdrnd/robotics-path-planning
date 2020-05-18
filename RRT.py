import numpy as np
import matplotlib.pyplot as plt
from read_pgm import read_pgm
import math
from math import pi
import random

positions = [
    [5, 5, pi/4],  # Starting pose
    [45, 50, 0],
    [9, 90, pi/3],
    [90, 10, pi/2],
    [90, 90, pi]
]
posns = [(pos[0], pos[1]) for pos in positions]


def dist(p1, p2):
    return math.sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2))

# Ensure point is not too close to an obstacle
def is_clear(point, pgm):
    return (
            pgm[point[1]][point[0] - 3] == 255 and
            pgm[point[1]][point[0] + 3] == 255 and
            pgm[point[1] + 3][point[0]] == 255 and
            pgm[point[1] - 3][point[0]] == 255 and
            # On corner use 2 squares instead of 3 as dist = sqrt(2)*squares
            pgm[point[1] + 2][point[0] - 2] == 255 and
            pgm[point[1] - 2][point[0] - 2] == 255 and
            pgm[point[1] + 2][point[0] + 2] == 255 and
            pgm[point[1] + 2][point[0] - 2] == 255
    )


def has_clear_path(p1, p2, pgm):
    # Check every 30 cm
    num_checks = math.floor(dist(p1, p2) / 3) + 1
    deltax = (p2[0] - p1[0])/num_checks
    deltay = (p2[1] - p1[1])/num_checks

    for i in range(1, num_checks):
        interior_point = (math.floor(p1[0]+i*deltax), math.floor(p1[1]+i*deltay))
        if not is_clear(interior_point, pgm):
            return False
    return True


def plot_tree(tree, pgm):
    plt.imshow(pgm, cmap='gray', vmin=0, vmax=255, origin='lower')
    for node in tree:
        if tree[node] is not None:
            plt.plot([node[0], tree[node][0]], [node[1], tree[node][1]], c='red')


def get_rrt_path(start, end, pgm):
    # Key: node, Value: parent node
    RRT = dict()
    RRT[start] = None
    last_added = None
    k=0
    while last_added != end:
        candidate = (random.randrange(3, 96, 1), random.randrange(3,  96, 1))  # Don't generate points near edges
        nearest = min(RRT, key=lambda p: dist(p, candidate))

        # Don't generate points too close together or in bad locations
        if (dist(nearest, candidate) < 3) or not is_clear(candidate, pgm) and candidate:
            continue

        prev = nearest

        # Add node to tree directly
        if has_clear_path(candidate, prev, pgm) and dist(candidate, prev) <= 5:
            RRT[candidate] = prev
            last_added = candidate
            if dist(candidate, end) < 5:
                RRT[end] = candidate
                last_added = end
        # Add intermediate nodes starting from parent
        else:
            # create vector from parent to candidate with magnitude 50 cm
            delta = (candidate[0]-nearest[0], candidate[1]-nearest[1])
            deltamag = math.sqrt(pow(delta[0], 2) + pow(delta[1], 2))
            delta = (5*delta[0]/deltamag, 5*delta[1]/deltamag)

            num_internal = math.floor(dist(nearest, candidate)/5) + 1
            for i in range(1, num_internal):
                internal = (math.floor(nearest[0]+i*delta[0]), math.floor(nearest[1]+i*delta[1]))

                if is_clear(internal, pgm) and internal not in RRT:
                    RRT[internal] = prev
                    prev = internal
                    last_added = internal

                    if last_added == end:
                        break

                    if 0 < dist(internal, end) < 5 and has_clear_path(internal, end, pgm):
                        RRT[end] = internal
                        last_added = end
                        break
                else:
                    break

    plt.figure()
    plot_tree(RRT, pgm)
    plt.scatter([start[0],end[0]], [start[1],end[1]], c='blue')
    plt.title('Generated RRT with Start and End Positions')

    path = []
    node = end
    while RRT[node] is not None:
        path.append(node)
        node = RRT[node]
    path.reverse()
    return path


def main():
    random.seed(42)

    pgm = read_pgm('sim_map.pgm')
    pgm = np.array(pgm)
    path = []
    for i in range(len(posns)-1):
        path += get_rrt_path(posns[i], posns[i+1], pgm)

    plt.figure()
    plt.imshow(pgm, cmap='gray', vmin=0, vmax=255, origin='lower')
    for i in range(len(path)-1):
        plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], c='red')
    plt.title('Overall PPath for a Set of Waypoints')

    plt.show()


if __name__ == '__main__':
    main()