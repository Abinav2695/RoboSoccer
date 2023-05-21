#!/usr/bin/python3

import math
import os
import sys
import random
import numpy as np
import rospy
import matplotlib.pyplot as plt



x_min=-1500
x_max=1500
y_min=-1200
y_max=1200


SIZE_FACTOR =1

class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self,
                 start=None,
                 goal=None,
                 obstacle_list=None,
                 randArea_x=[x_min,x_max], 
                 randArea_y=[y_min,y_max], 
                 expand_dis=3.0,
                 path_resolution=150,
                 goal_sample_rate=50,
                 max_iter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """

        if start is not None:
            self.start = self.Node(start[0], start[1])
        if goal is not None:
            self.end = self.Node(goal[0], goal[1])
        
        if obstacle_list is not None:
            self.obstacle_list = obstacle_list

        # self.start = self.Node(start[0], start[1])
        # self.end = self.Node(goal[0], goal[1])
        # self.obstacle_list = obstacle_list

        self.minrand_x = randArea_x[0]
        self.maxrand_x = randArea_x[1]
        self.minrand_y = randArea_y[0]
        self.maxrand_y = randArea_y[1]

        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        
        self.node_list = []
        self.sizeFactor=2.3

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            
            
            if self.check_collision(new_node, self.obstacle_list,self.sizeFactor):
                self.node_list.append(new_node)
            
            
            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list,self.sizeFactor):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.minrand_x, self.maxrand_x),
                random.uniform(self.minrand_y, self.maxrand_y))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for obs_data in self.obstacle_list:
            self.plot_circle(obs_data.x, obs_data.y, obs_data.radius)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.minrand_x, self.maxrand_x, self.minrand_y, self.maxrand_y])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacleList,sizeFactor):

        if node is None:
            return False

        for obstacle in obstacleList:
            dx_list = [obstacle.x - x for x in node.path_x]
            dy_list = [obstacle.y - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            final_dist = (min(d_list))**0.5
            if (final_dist < sizeFactor *(obstacle.radius)):
                return False  # collision

        return True  # safe

        # for (ox, oy, size) in obstacleList:
        #     dx = ox - node.x
        #     dy = oy - node.y
        #     d = math.sqrt(dx * dx + dy * dy)
        #     if d <= 2.5*size:
        #         return False

        # return True

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
