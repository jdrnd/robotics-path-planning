import numpy as np
import matplotlib.pyplot as plt
from read_pgm import read_pgm
import math

# Use x,y convention
startpos = np.array((5,5))
endpos = np.array((95,95))

xlim = np.array((0.0, 10.0))
ylim = np.array((0.0, 10.0))

max_potential = 1000


# Uses an *approximate* method to find distance from a point to any obstacle up to a make distance
def get_distance_from_obstacle(pos, pgm, maxdist):
    x, y = pos

    if pgm[y][x] == 0:
        return 0

    for i in range(1, maxdist+1):
        try:
            if pgm[y][x+i] == 0 or \
                    pgm[y+i][x+i] == 0 or \
                    pgm[y-i][x+i] == 0 or \
                    pgm[y+i][x] == 0 or \
                    pgm[y-i][x] == 0 or \
                    pgm[y][x-i] == 0 or \
                    pgm[y+i][x-i] == 0 or \
                    pgm[y-i][x-i] == 0:
                return i
        except:
            pass

    # Distance to obstacle is more than max
    return -1


def main():
    # For simplicity going forward assume this will always be square
    pgm = read_pgm('sim_map.pgm')
    pgm = np.array(pgm)
    imsize = len(pgm)

    attr_potential = np.zeros_like(pgm)
    # Generate attractor potential field
    for x in range(imsize):
        for y in range(imsize):
            pos = np.array((x,y))
            dist = np.linalg.norm(pos-endpos)
            attr_potential[y][x] = 100*dist

    x = np.linspace(0, 10, 100)
    y = np.linspace(0, 10, 100)
    X, Y = np.meshgrid(x, y)

    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X,Y, attr_potential, cmap='viridis')
    plt.xlabel('x Position (m)')
    plt.ylabel('y Position (m)')
    ax.set_zlabel('Relative potential')
    plt.title('Attractive Potential Field Toward Destination')

    # Normalize attractor potential matrix
    maxv = np.max(attr_potential)
    attr_potential = attr_potential / maxv
    fattractor_y, fattractor_x = np.multiply(np.gradient(attr_potential), -1)

    # plt.figure()
    # ax = plt.axes()
    # plt.quiver(fattractor_x[::5, ::5], fattractor_y[::5, ::5])  # downsample to make plot nicer, shows force field from attractive potential
    # ax.set_yticklabels([])
    # ax.set_xticklabels([])
    # plt.xlabel('x Position')
    # plt.ylabel('y Position')

    # Generate attractor potential field
    frepulse_x = np.empty_like(pgm)
    frepulse_y = np.empty_like(pgm)

    rep_potential = np.zeros_like(pgm, dtype=np.float32)

    # Robot has diameter of 0.45 m - use "sphere of influence" around obstacles to be 100 cm
    max_dist = 10
    for x in range(0, imsize):
        for y in range(0, imsize):
                dist = get_distance_from_obstacle((x, y), pgm, max_dist)
                if dist > 0:
                    # Use a quadratic repulsive field
                    rep_potential[y][x] = math.pow((max_dist-dist)/max_dist, 2)
                    pass
                elif dist == 0:
                    rep_potential[y][x] = 1
                else:
                    rep_potential[y][x] = 0

    # Normalize the repulsive field
    maxv = np.max(rep_potential)
    rep_potential = rep_potential / maxv

    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, rep_potential, cmap='viridis')
    plt.title('Repulsive Potential Field from Obstacles')

    rep_potential = np.multiply(rep_potential, -1.0)
    frepulse_y,  frepulse_x = np.gradient(rep_potential)

    # Create total force field
    repgain = 0.2
    totalf_x = fattractor_x + repgain*frepulse_x
    totalf_y = fattractor_y + repgain*frepulse_y

    plt.figure()
    plt.quiver(totalf_x, totalf_y)
    plt.imshow(pgm, cmap='gray', vmin=0, vmax=255, origin='lower')
    plt.title('Overall Force Field and Robot Path')

    # Run robot simulation
    r_state = [0.5, 0.5, math.pi/4-0.1]
    end_state = [9.5, 9.5, math.pi/4]
    T = 0.1

    wgain = 3  # gain term for angular velocity controller
    max_speed = 0.2  # m/s

    pointsx = []
    pointsy = []
    headings = []
    while math.sqrt( math.pow(end_state[0]-r_state[0],2) + math.pow(end_state[1]-r_state[1], 2)) > 0.5:
        x, y = (math.floor(r_state[0]*10), math.floor(r_state[1]*10))
        # Store state for later display
        pointsx.append(r_state[0]*10)
        pointsy.append(r_state[1]*10)
        headings.append(r_state[2])

        v = max_speed
        # Use a proportional controller to match the robot's heading angle to the force vector's
        w = wgain*(math.atan2(totalf_y[y][x], totalf_x[y][x]) - r_state[2])

        # Run state update using motion model
        r_state[0] = r_state[0] + v*math.cos(r_state[2])*T
        r_state[1] = r_state[1] + v*math.sin(r_state[2]) * T
        r_state[2] = r_state[2] + w*T

    plt.scatter(pointsx, pointsy)
    plt.show()


if __name__=='__main__':
    main()
