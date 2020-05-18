import numpy as np
import matplotlib.pyplot as plt
from read_pgm import read_pgm
import math
from math import pi
import random

# Positions/poses in (x,y,theta) format
positions = [
    [5, 5, pi/4],  # Starting pose
    [60, 40, 3*pi/2],
    [85, 85, pi],
    [30, 95, pi/2],
    [5, 50, 0]
]

WHITE = 255
BLACK = 0


def dist(p1, p2):
    return math.sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2))


def is_clear(point, pgm):
    return (
            pgm[point[1]][point[0] - 3] == WHITE and
            pgm[point[1]][point[0] + 3] == WHITE and
            pgm[point[1] + 3][point[0]] == WHITE and
            pgm[point[1] - 3][point[0]] == WHITE and
            # On corner use 2 squares instead of 3 as dist = sqrt(2)*squares
            pgm[point[1] + 2][point[0] - 2] == WHITE and
            pgm[point[1] - 2][point[0] - 2] == WHITE and
            pgm[point[1] + 2][point[0] + 2] == WHITE and
            pgm[point[1] + 2][point[0] - 2] == WHITE
    )


# Check 10 points between points to ensure that the path does not collide with an obstabcle
def has_clear_path(p1, p2, pgm):
    deltax = (p2[0] - p1[0])/10.0
    deltay = (p2[1] - p1[1])/10.0

    for i in range(1, 10):
        interior_point = (math.floor(p1[0]+i*deltax), math.floor(p1[1]+i*deltay))
        if not is_clear(interior_point, pgm):
            return False
    return True


NO_PATH = 99999999
# A* algorithm, this implimentation uses distance to node + distance from node to end as heuristic
def shortest_path_a_star(start, end, graph):
    nodes = list(graph.keys())
    distances = dict((node, NO_PATH) for node in nodes)
    distances[start] = 0
    prev = dict((node, None) for node in nodes)
    end_distances = dict((node, dist(end, node)) for node in nodes)

    nodes_notvisited = set(nodes)
    nodes_visited = set()
    curr_node = start

    while curr_node:
        if curr_node == end:
            break

        # Update distance to all neighbors
        for n in graph[curr_node]:
            updated_value = distances[curr_node] + dist(curr_node, n)
            if updated_value < distances[n]:
                distances[n] = updated_value
                prev[n] = curr_node

        nodes_notvisited.remove(curr_node)
        nodes_visited.add(curr_node)

        best_nodes = sorted(nodes_notvisited, key=lambda no: (distances[no] + end_distances[no]))
        if not best_nodes:
            curr_node = None
        else:
            curr_node = best_nodes[0]

    # Plot explored tree on maps
    for node in prev:
        if prev[node] is not None:
            plt.plot([node[0], prev[node][0]], [node[1], prev[node][1]])

    # Create path by backtracking along prev pointers
    path = [end]
    n = end
    while prev[n] is not None:
        path.append(prev[n])
        n = prev[n]
    path.reverse()
    return path


def main():
    random.seed(42)

    pgm = read_pgm('sim_map.pgm')
    pgm = np.array(pgm)

    # Generate a large number of potential path points
    num_points_per_path = 1000
    points = set()
    for i in range(num_points_per_path):
        points.add((random.randrange(0, 100, 1), random.randrange(0, 100, 1)))

    plt.imshow(pgm, cmap='gray', vmin=BLACK, vmax=WHITE, origin='lower')

    # Filter out points inside obstacles
    points = set([p for p in points if pgm[p[1]][p[0]] == WHITE])

    # Add each robot target state as a point
    position_points = set([(p[0], p[1]) for p in positions])
    points |= position_points

    # Filter out points within 30cm of each other
    points_tooclose = set()
    for point in points:
        for point2 in points:
            x1, y1 = point[0], point[1]
            x2, y2 = point2[0], point2[1]

            if math.sqrt(pow(x2-x1, 2) + pow(y2-y1,2)) < 3.0 and point != point2 and point not in points_tooclose and point2 not in points_tooclose and point2 not in position_points:
                points_tooclose.add(point2)
    points -= points_tooclose

    # Filter out points within 40 cm of the map edge
    # 10cm thick obstacle around the map plus 30 cm for robot clearance
    points = set([p for p in points if (3 <= p[0] <= 96 and 3 <= p[1] <= 96)])

    # Filter out points within 30 cm of an obstacle
    points = set([p for p in points if is_clear(p, pgm)])

    # Plot allowable locations
    x = [elem[0] for elem in points]
    y = [elem[1] for elem in points]
    plt.scatter(x, y, c='blue')

    # Add goal locations to plot
    posex = [pos[0] for pos in positions]
    posey = [pos[1] for pos in positions]
    plt.scatter(posex, posey, c='red')
    plt.title('Randomly Sampled Map Positions')

    # Use a dictionary to represent the graph
    # Keys: a graph node
    # Values: set of nodes that node is connected to
    map_graph = dict()
    for p in points:
        map_graph[p] = set()

    # Construct graph from allowable edges
    for p1 in points:
        for p2 in points:
            if p1 == p2:
                continue
            if p1 in map_graph[p2] or p2 in map_graph[p1]:
                continue

            d = dist(p1, p2)
            # Set max edge length to 20 squares = 2.0m
            if d < 20 and has_clear_path(p1, p2, pgm):
                map_graph[p1].add(p2)
                map_graph[p2].add(p1)

    # Visualize entire graph
    plt.figure()
    plt.imshow(pgm, cmap='gray', vmin=BLACK, vmax=WHITE, origin='lower')
    for node in map_graph:
        for node2 in map_graph[node]:
            plt.plot([node[0], node2[0]], [node[1], node2[1]])
    plt.title('Position Graph')

    # Calls to shortest_path will show explored paths
    plt.figure()
    plt.imshow(pgm, cmap='gray', vmin=BLACK, vmax=WHITE, origin='lower')
    total_path = list()
    for i in range(len(positions)-1):
        total_path += shortest_path_a_star(tuple(positions[i][0:2]), tuple(positions[i+1][0:2]), map_graph)[:-1]
    total_path += [tuple(positions[-1][0:2])]
    plt.title("A* Visualization for a Set of Waypoints")


    # Show the final path of waypoints
    plt.figure()
    plt.imshow(pgm, cmap='gray', vmin=BLACK, vmax=WHITE, origin='lower')
    for i in range(len(total_path)-1):
        plt.plot([total_path[i][0], total_path[i+1][0]], [total_path[i][1], total_path[i+1][1]], c='red')
    plt.scatter([p[0] for p in positions], [p[1] for p in positions], c='black', s=70)
    plt.title("Final Path for a Set of Waypoints")

    # Show the final path of waypoints to plot path onto
    plt.figure()
    plt.imshow(pgm, cmap='gray', vmin=BLACK, vmax=WHITE, origin='lower')
    for i in range(len(total_path)-1):
        plt.plot([total_path[i][0], total_path[i+1][0]], [total_path[i][1], total_path[i+1][1]], c='red')

    # Run robot simulation
    r_state = [0.5, 0.5, math.pi/4-0.1]
    T = 0.1

    wgain = 3  # gain term for angular velocity controller
    max_speed = 0.2  # m/s

    pointsx = []
    pointsy = []
    headings = []
    for goal_pos in total_path[1:]:  # 1st "goal state" is just the starting state
        # Convert from pixels to meters
        goal_pos = (goal_pos[0] / 10, goal_pos[1] / 10)
        r_pos = (r_state[0], r_state[1])

        # Navigate to within 10 cm of each end goal
        while dist(r_pos, goal_pos) > 0.1:
            pointsx.append(r_state[0] * 10)
            pointsy.append(r_state[1] * 10)
            headings.append(r_state[2])

            target_heading = math.atan2(goal_pos[1]-r_pos[1], goal_pos[0]-r_pos[0])
            heading_error = target_heading - r_state[2]
            w = wgain*heading_error
            v = max_speed

            # Run state update using motion model
            r_state[0] = r_state[0] + v * math.cos(r_state[2]) * T
            r_state[1] = r_state[1] + v * math.sin(r_state[2]) * T
            r_state[2] = r_state[2] + w * T

            r_pos = (r_state[0], r_state[1])

    # Add to previous plot
    for i in range(len(pointsx)):
        if i % 45 == 0:
            plt.arrow(pointsx[i], pointsy[i], 10*math.cos(headings[i]), 10*math.sin(headings[i]), color='red', width=0.5)
    plt.scatter(pointsx, pointsy)
    plt.show()


if __name__ == '__main__':
    main()