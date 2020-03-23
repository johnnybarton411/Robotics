"""Optimized RRT* Algorithn Implementation

Author: Cody Dillinger - Cody.M.Dillinger@gmail.com

This .py script will utilize the optimized RRT algorithm, referred to as RRT*,
to generate a single path from starting point to destination. As the path
planning tree forms, it displays the tree to the user in a pygame window using
the pygame library.

In the algorithm:
    * V is the set of vertices created from all samples up to some point in
    time, here this is an array of point objects.
    * E is the set of edges created between all samples up to somepoint in
    time, here this is the pointer from one array element to another

Below is a summary of this RRT* algorithm:
    * Initialize a tree with no edges as a starting point, where the tree is
    an array of point objects
    * Then for some number n samples:
        * sampleFree() - Take a random sample in space.
        * nearest() - Find the nearest vertex in the tree to that random sample.
        * steer() - find point in the direction of random sample but within some radius of the nearest vertex.
        * collision() - If there is no obstacle on the edge between steer() point and nearest() vertex
        * near() - Check for other vertices in tree that are within some radius of the steer() point, where the radius is a function of the number of samples
        * Add the steer() point to the tree
        * cost() - get cost (total distance) of a vertex from starting point
        * add_edge() - add edge between two points, given two elements of tree function
        * erase_edge() - erase edge between two points, given two elements of tree function

See https://arxiv.org/abs/1105.1186 for more information regarding the analysis
of RRT* and related motion planning algorithms
"""

import random
import pygame
from pygame.locals import *
from math import *

LENGTH = 500                              # set constants for pygame window size
WIDTH = 700
MAX_VELOCITY = 10                          # max velocity of drone
RADIUS = 20				  # constant radius for steer() function
X_START = 450
Y_START = 50
X_END = 50
Y_END = 650

WHITE = 255, 255, 255
BLACK = 0, 0, 0    # RGB color values, black for obstacles
RED = 255, 0, 0
GREEN = 0, 255, 0  # green and red for destination and starting point respectively
BLUE = 0, 0, 255
PINK = 200, 20, 240

# initialize usage of pygame
pygame.init()
PY_WINDOW = pygame.display.set_mode((LENGTH, WIDTH))   # create pygame display

#######################################


class Point:
    """tree array will contain these point objects"""
    x = 0
    y = 0
    last = None
    cost = 0
    time = 0

    def __init__(self, x_val, y_val):
        self.x = x_val
        self.y = y_val


def distance(pt1: Point, pt2: Point):
    """return distance between two points from two point objects"""
    return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y))


def sample_free():
    """return pseudo random point object"""
    x_rand = Point(random.random()*LENGTH, random.random()*WIDTH)
    # TODO: see if adding collision check will eventually result in path
    # TODO: maybe add [seed] into random.random to increase randomness
    # TODO: add bias towards destination
    return x_rand


def nearest(x_rand1, tree1):
    """return index of the point in tree which is closest to random sampled point"""
    i_closest = 0				 # future - this loop makes this whole algorithm O(n^2) ? Shouldn't.
    for i in range(len(tree1)):
        if (distance(tree1[i], x_rand1) < distance(tree1[i_closest], x_rand1)):
            i_closest = i
    return i_closest


def steer(x_nearest1: Point, x_rand1: Point, radius1):
    """return point object in direction of random sample but within radius of nearest vertex"""
    if distance(x_nearest1, x_rand1) > radius1:
        delta_y = (x_rand1.y - x_nearest1.y)
        delta_x = (x_rand1.x - x_nearest1.x)
        m = delta_y / delta_x
        theta = atan(abs(m))			  # note: used abs(theta) due to inverted y axis

        if delta_x == 0:				  # for unlikely case of random sample having same x val
            x_new = x_nearest1.x
            if delta_y > 0:
                y_new = x_nearest1.y + radius1
            elif delta_y < 0:
                y_new = x_nearest1.y - radius1
        if delta_y == 0:				  # for unlikely case of random sample having same y val
            y_new = x_nearest1.y
            if delta_x > 0:
                x_new = x_nearest1.x + radius1
            elif delta_x < 0:
                x_new = x_nearest1.x - radius1

        if delta_x > 0:				  # 4 cases for 4 quadrants
            if delta_y > 0:
                x_new = x_nearest1.x + radius1 * cos(theta)
                y_new = x_nearest1.y + radius1 * sin(theta)
            else:
                x_new = x_nearest1.x + radius1 * cos(theta)
                y_new = x_nearest1.y - radius1 * sin(theta)
        else:
            if delta_y > 0:
                x_new = x_nearest1.x - radius1 * cos(theta)
                y_new = x_nearest1.y + radius1 * sin(theta)
            else:
                x_new = x_nearest1.x - radius1 * cos(theta)
                y_new = x_nearest1.y - radius1 * sin(theta)

        x_new1 = Point(x_new, y_new)
    else:
        x_new1 = x_rand1
    return x_new1


def collision(point, point_new):
    """checking for intersection, not made to easily scale to 3+ dimensions"""
    # if ptNew out of window then collision
    if (point_new.x <= 0 or point_new.x >= LENGTH or point_new.y <= 0 or point_new.y >= WIDTH):
        return 1
    x1 = 0
    x2 = 280
    y1 = 500
    y2 = 520   # obstacle 1 rectangular boundaries
    x3 = 220
    x4 = 600
    y3 = 250
    y4 = 270   # obstacle 2 rectangular boundaries
    delta_y = point_new.y - point.y
    delta_x = point_new.x - point.x
    m = delta_y / delta_x
    m_inv = 1 / m

    if (
            (point.y + m * (x1 - point.x) < y2 and point.y + m * (x1 - point.x) > y1) or
            (point.y + m * (x2 - point.x) < y2 and point.y + m * (x2 - point.x) > y1) or
            (point.x + m_inv * (y1 - point.y) < x2 and point.x + m_inv * (y1 - point.y) > x1) or
            (point.x + m_inv * (y2 - point.y) <
             x2 and point.x + m_inv * (y2 - point.y) > x1)
    ):
        return 1                       # if hit obstacle 1
    if (
            (point.y + m * (x3 - point.x) < y4 and point.y + m * (x3 - point.x) > y3) or
            (point.y + m * (x4 - point.x) < y4 and point.y + m * (x4 - point.x) > y3) or
            (point.x + m_inv * (y3 - point.y) < x4 and point.x + m_inv * (y3 - point.y) > x3) or
            (point.x + m_inv * (y4 - point.y) <
             x4 and point.x + m_inv * (y4 - point.y) > x3)
    ):
        return 1                       # if hit obstacle 2
    return 0


def near(rad, tree1, x_new1):
    """find vertices that are within radius of x_new1"""
    near_vertices = []
    for i in range(len(tree1)):
        if distance(tree1[i], x_new1) < rad:
            near_vertices.append(i)
    return near_vertices


def cost():
    """TODO: Write docstring"""
    return 0


def add_edge(pt1, pt2, tree1):
    """pts are just element nums. pt1 to pt2 - direction is important"""
    pygame.draw.line(PY_WINDOW, BLACK, [tree1[pt1].x, tree1[pt1].y], [
                     tree1[pt2].x, tree1[pt2].y])
    pygame.display.flip()
    return 0


def erase_edge():
    """TODO: Write docstring"""
    return 0

####################################################################################################
####################################################################################################


def main():
    """TODO: Write docstring"""
    # put these into initPygame() function
    # set background of pygame window to white
    PY_WINDOW.fill(WHITE)
    # display starting point with red circle
    pygame.draw.circle(PY_WINDOW, RED, (X_START, Y_START), 10, 0)
    # display destination point with green circle
    pygame.draw.circle(PY_WINDOW, GREEN, (X_END, Y_END), 10, 0)
    # display black rectangle obstacle
    pygame.draw.rect(PY_WINDOW, BLACK, (0, 500, 280, 20), 0)
    # display black rectangle obstacle
    pygame.draw.rect(PY_WINDOW, BLACK, (220, 250, 280, 20), 0)
    # update display with these new shapes
    pygame.display.flip()

    # adjustable number of tree vertices each time you run the code
    vertex_num = int(input('Enter number of random samples: '))

    tree = []
    # create new tree with starting vertex at 250, 150
    tree.append(Point(X_START, Y_START))
    # create destination target at 450, 600
    target = Point(X_END, Y_END)
    cost_min = 0
    num_vertices = 1
    for i in range(vertex_num):
        # get random sample within pygame window size, as a point object
        x_rand = sample_free()
        # get vertex in tree that is nearest to the random sample, as an index
        x_nearest = nearest(x_rand, tree)
        # get point in direction of xrand from xNearest (if it is too far away), as a point object
        x_new = steer(tree[x_nearest], x_rand, RADIUS)
        print(f'xNear = {floor(tree[x_nearest].x)} {floor(tree[x_nearest].y)}')
        print(f'xRand = {floor(x_rand.x)} {floor(x_rand.y)}')
        print(f'x_new1 = {x_new.x} {x_new.y}')

        # if no collision
        if collision(tree[x_nearest], x_new) == 0:
            # number of vertices or nodes, in the tree
            num_vertices = num_vertices + 1
            # total size of window minus size of obstacles
            total_space = (WIDTH * LENGTH) - (280 * 20 * 2)

            # TODO: Double check this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            unit_ball = pi

            # RRT* asymptotically stable if gamma* > (2*(1+1/d))^(1/d)*(μ(Xfree)/ζd)^(1/d)
            gamma = 2 * sqrt(3) * sqrt((total_space / unit_ball))
            near_radius = min(
                gamma * sqrt(log(num_vertices) / num_vertices), RADIUS)

            # return just indices of tree. radius here is function of number of existing samples
            near_vertices = near(near_radius, tree, x_new)

            # append this point element to array regardless of path
            tree.append(x_new)

            # variable for lowest cost point, initializing under assumption of xNearest
            x_min = x_nearest

            # variable for lowest cost. These may be changed for one of Near vertices
            # TODO: use cost function
            cost_min = 0  # cost(numVertices - 1, tree)

            # for j in near_vertices						     # connect along a minimum cost path by checking near vertices
            # costNear = cost(j, tree) + distance(tree(j),x_new)		     # cost of path through near vertex
            # if (collision(tree(j),x_new) == false and costNear < costMin):       # if near path is lower cost and no collision
            # xMin = j							     # update xMin for this shorter path
            # costMin = costNear						     # update costMin for this shorter path
            #tree(i).last = tree(xMin)

            # add edge on pygame visual AND update parent
            add_edge(x_min, num_vertices - 1, tree)

            # for j in near_vertices						     # rewire tree / update parents if xnear is not on its shortest path
            # newCost = cost(numVertices-1)+distance(x_new,tree(j))			     # potential new cost of xNear
            # if (collision(tree(j),x_new) == false and newCost < cost(tree(j))):
            # eraseEdge(tree(j),tree(j).last)				     # erase edge on pygame visual
            # addEdge(numVertices-1, j, tree)				     # add edge on pygame visual and update parent
        # time.sleep(.01)
    print(f'final numVertices = {num_vertices}')
    print(f'final tree size = {len(tree)}')
    return


##################################################################################################################################
if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
