#!/usr/bin/python3

import math
import os
import sys
import random
import numpy as np
import rospy
import matplotlib.pyplot as plt
from utils_updated.geometry_functions.geometry_functions import Line,Vector2D
import time
# try:
#     from rrt import RRT
# except ImportError:
#     raise

show_animation = False

x_min=-1550
x_max=1550
y_min=-1250
y_max=1250
SIZE_FACTOR =2.2

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
            
            
            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)
            
            
            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
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

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

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
    def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            final_dist = (min(d_list))**0.5
            if (final_dist < SIZE_FACTOR*(size)):
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


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self,
                 start=None,
                 goal=None,
                 obstacle_list=None,
                 randArea_x=[x_min,x_max], 
                 randArea_y=[y_min,y_max], 
                 expand_dis=500.0,
                 path_resolution=150,
                 goal_sample_rate=10,
                 max_iter=300,
                 connect_circle_dist=50.0,
                 search_until_max_iter=False):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        # if start is not None:
        #     self.start = self.Node(start[0], start[1])
        # if goal is not None:
        #     self.end = self.Node(goal[0], goal[1])
        
        # if obstacle_list is not None:
        #     self.obstacle_list = obstacle_list


        super().__init__(start, goal, obstacle_list, randArea_x,randArea_y, expand_dis,
                         path_resolution, goal_sample_rate, max_iter)
        self.connect_circle_dist = connect_circle_dist

        # if goal is not None:
        #     self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter

    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        print('Planning RRTStar')
        self.node_list = [self.start]
        for i in range(self.max_iter):
            #print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x-near_node.x,
                           new_node.y-near_node.y)
            
            
           
            if self.check_collision(new_node, self.obstacle_list):
                
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)
            
                
            if animation:
                self.draw_graph(new_node)

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

       
        last_index = self.search_best_goal_node()
        
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.end)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def get_path_length(self,path):
        le = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            le += d

        return le


    def get_target_point(self,path, targetL):
        le = 0
        ti = 0
        lastPairLen = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            le += d
            if le >= targetL:
                ti = i - 1
                lastPairLen = d
                break

        partRatio = (le - targetL) / lastPairLen

        x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
        y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

        return [x, y, ti]


    def line_collision_check(self,first, second, obstacleList):
        # Line Equation

        x1 = first[0]
        y1 = first[1]
        x2 = second[0]
        y2 = second[1]

        try:
            a = y2 - y1
            b = -(x2 - x1)
            c = y2 * (x2 - x1) - x2 * (y2 - y1)
        except ZeroDivisionError:
            return False

        for (ox, oy, size) in obstacleList:
            d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
            if d <= SIZE_FACTOR*size:
                L1 = Line(point1 = Vector2D(x1,y1),point2 = Vector2D(x2,y2))
                L2 = Line(point1 = Vector2D(ox,oy),slope=-1/L1.slope) 
                pointOfIntersection = L1.intersection_with_line(L2)

                x_list=[x1,x2]
                x_list.sort()
                y_list = [y1,y2]
                y_list.sort()
                if(x_list[0] <= pointOfIntersection.x <=x_list[1] 
                        or y_list[0] <= pointOfIntersection.y <=y_list[1] ):
                    return False

        return True  # OK
    
    def line_collision_check1(self,first, second, obstacleList):
        # Line Equation

        x1 = first[0]
        y1 = first[1]
        x2 = second[0]
        y2 = second[1]

        try:
            a = y2 - y1
            b = -(x2 - x1)
            c = y2 * (x2 - x1) - x2 * (y2 - y1)
        except ZeroDivisionError:
            return False

        for (ox, oy, size) in obstacleList:
            d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
            if d <= SIZE_FACTOR*size:
                L1 = Line(point1 = Vector2D(x1,y1),point2 = Vector2D(x2,y2))
                L2 = Line(point1 = Vector2D(ox,oy),slope=-1/L1.slope) 
                pointOfIntersection = L1.intersection_with_line(L2)

                x_list=[x1,x2]
                x_list.sort()
                y_list = [y1,y2]
                y_list.sort()
                if(x_list[0] <= pointOfIntersection.x <=x_list[1] 
                        or y_list[0] <= pointOfIntersection.y <=y_list[1] ):

                    return False

        return True  # OK

    def pointInCollisionRange(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node[0]
            dy = oy - node[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d <= 2.5*size:
                return False

        return True

    def path_smoothing(self,path, max_iter, obstacle_list):
        le = self.get_path_length(path)

        for i in range(max_iter):
            # Sample two points
            pickPoints = [random.uniform(0, le), random.uniform(0, le)]
            pickPoints.sort()
            first = self.get_target_point(path, pickPoints[0])
            second = self.get_target_point(path, pickPoints[1])

            if first[2] <= 0 or second[2] <= 0:
                continue

            if (second[2] + 1) > len(path):
                continue

            if second[2] == first[2]:
                continue

            # collision check
            if not self.line_collision_check(first, second, obstacle_list):
                continue

            # Create New path
            newPath = []
            newPath.extend(path[:first[2] + 1])
            newPath.append([first[0], first[1]])
            newPath.append([second[0], second[1]])
            newPath.extend(path[second[2] + 1:])
            path = newPath
            le = self.get_path_length(path)

        return path

    def update(self,obstacleList, start, goal):

        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.obstacle_list = obstacleList

        path=self.planning(animation=False)

        if path is None:
            #print("Cannot find path")
            return None

        else:
            #print(path)
            maxIter = 500
            smoothedPath = self.path_smoothing(path, maxIter, obstacleList)
            
            smoothedPath_updated=[]
            

            for i in range (0,len(smoothedPath)):
                smoothedPath_x = math.floor(smoothedPath[i][0])
                smoothedPath_y = math.floor(smoothedPath[i][1])

                if [smoothedPath_x,smoothedPath_y] not in smoothedPath_updated:
                    smoothedPath_updated.append([smoothedPath_x,smoothedPath_y])


            smoothedPath_updated = smoothedPath_updated[::-1]
            smoothedPath_updated = smoothedPath_updated[1:]

            
            # Draw final path
            if False:
                plt.close('all')
                self.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
                plt.plot([x for (x, y) in smoothedPath], [
                    y for (x, y) in smoothedPath], 'b-')
                plt.grid(True)
                plt.pause(0.001)
                plt.show(block=False)
            
            print(smoothedPath_updated)
            return smoothedPath_updated


def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    robotRadius =100
   
    obstacleList = [
        (-1448.77,-35.669,robotRadius),
        (-5.85, 398.557,robotRadius),  
        (715.76, -281.83, robotRadius),
        (502.43, 469.524, robotRadius),
        (538.453, 48.357, robotRadius),
        (881.30,-790.8056,robotRadius),
        (-441.728,642.945,robotRadius)
    ]  # [x,y,size]

    # Set Initial parameters
    rrt_star = RRTStar(
        start=(1497.67, 1.66),goal=(-574,160),
        path_resolution=0.5,
        obstacle_list=obstacleList,
        expand_dis=150)
    path = rrt_star.planning(animation=False)

    if path is None:
        print("Cannot find path")

    else:
        #print(path)
        maxIter = 500
        smoothedPath = rrt_star.path_smoothing(path, maxIter, obstacleList)
        
        smoothedPath_updated=[]
            

        for i in range (0,len(smoothedPath)):
            smoothedPath_x = math.floor(smoothedPath[i][0])
            smoothedPath_y = math.floor(smoothedPath[i][1])

            if [smoothedPath_x,smoothedPath_y] not in smoothedPath_updated:
                smoothedPath_updated.append([smoothedPath_x,smoothedPath_y])

        smoothedPath_updated = smoothedPath_updated[::-1]
        smoothedPath_updated =smoothedPath_updated[1:]

        print(len(smoothedPath_updated))

        # Draw final path
        if True:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            plt.plot([x for (x, y) in smoothedPath], [
                y for (x, y) in smoothedPath], 'g')
            plt.grid(True)
            
            plt.pause(0.001)
            plt.show(block=False)
            time.sleep(3)
            plt.close('all')


if __name__ == '__main__':

    rospy.init_node('path_planner',anonymous=False)
    for i in range (0,1):
        main()
