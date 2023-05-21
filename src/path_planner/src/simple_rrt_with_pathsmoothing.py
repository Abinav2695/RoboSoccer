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
import copy

show_animation = False

COLLISION_FACTOR=2.5
x_min=-1550
x_max=1550
y_min=-1250
y_max=1250


class RRT():
    """
    Class for RRT Planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None

    def __init__(self, start=None, goal=None, obstacleList=None, 
                    randArea_x=[x_min,x_max], 
                    randArea_y=[y_min,y_max], 
                    expandDis=200, 
                    goalSampleRate=10, 
                    maxIter=300):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        if start is not None:
            self.start = Node(start[0], start[1])
        if goal is not None:
            self.end = Node(goal[0], goal[1])
        
        if obstacleList is not None:
            self.obstacleList = obstacleList
        self.minrand_x = randArea_x[0]
        self.maxrand_x = randArea_x[1]
        self.minrand_y = randArea_y[0]
        self.maxrand_y = randArea_y[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand_x, self.maxrand_x), random.uniform(
                    self.minrand_y, self.maxrand_y)]
            else:
                rnd = [self.end.x, self.end.y]

            nind = self.GetNearestListIndex(self.nodeList, rnd)

            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList):
                continue

            self.nodeList.append(newNode)

            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        #print(path)
        return path

    def DrawGraph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")
        for (x, y, size) in self.obstacleList:
            self.PlotCircle(x, y, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([self.minrand_x, self.maxrand_x, self.minrand_y, self.maxrand_y])
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= COLLISION_FACTOR*size:
                return False

        return True


    def GetPathLength(self,path):
        le = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx * dx + dy * dy)
            le += d

        return le

    def GetTargetPoint(self,path, targetL):
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

    def LineCollisionCheck(self,first, second, obstacleList):

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
            if d <= (2.3*size):
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

        return True


    def PathSmoothing(self,path, maxIter, obstacleList):

        le = self.GetPathLength(path)

        for i in range(maxIter):
            pickPoints = [random.uniform(0, le), random.uniform(0, le)]
            pickPoints.sort()
            first = self.GetTargetPoint(path, pickPoints[0])
            second = self.GetTargetPoint(path, pickPoints[1])

            if first[2] <= 0 or second[2] <= 0:
                continue

            if (second[2] + 1) > len(path):
                continue

            if second[2] == first[2]:
                continue

            if not self.LineCollisionCheck(first, second, obstacleList):
                continue

            newPath = []
            newPath.extend(path[:first[2] + 1])
            newPath.append([first[0], first[1]])
            newPath.append([second[0], second[1]])
            newPath.extend(path[second[2] + 1:])
            path = newPath
            le = self.GetPathLength(path)

        return path
    
    def pointInCollisionRange(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node[0]
            dy = oy - node[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d <= COLLISION_FACTOR*size:
                return False

        return True

    def update(self,obstacleList, start, goal):
        # rrt = RRT(start=start, goal=goal,obstacleList=obstacleList)
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.obstacleList = obstacleList

        
        path = self.Planning(animation=False)

        if path is None:
            print("Cannot find path")
            return None
        
        
        maxIter = 500
        smoothedPath = self.PathSmoothing(path, maxIter, obstacleList)

        smoothedPath_updated=[]
            

        for i in range (0,len(smoothedPath)):
            smoothedPath_x = math.floor(smoothedPath[i][0])
            smoothedPath_y = math.floor(smoothedPath[i][1])

            if [smoothedPath_x,smoothedPath_y] not in smoothedPath_updated:
                smoothedPath_updated.append([smoothedPath_x,smoothedPath_y])


        smoothedPath_updated = smoothedPath_updated[::-1]
        smoothedPath_updated = smoothedPath_updated[1:]
        #print("Smoothed path is :- ", smoothedPath)

        if False:
            plt.close('all')
            self.DrawGraph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            plt.plot([x for (x, y) in smoothedPath], [
                y for (x, y) in smoothedPath], 'b-')
            plt.grid(True)
            plt.pause(0.001)
            plt.show(block=False)

        print(smoothedPath_updated)
        return smoothedPath_updated


    def __del__(self):
        print('deleting instance')
    
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
    pathInstance = RRT()

    pathInstance.update(start = (0,0),goal=(0,0),obstacleList=obstacleList)


if __name__ == '__main__':

    rospy.init_node('path_planner',anonymous=False)
    for i in range (0,1):
        main()
