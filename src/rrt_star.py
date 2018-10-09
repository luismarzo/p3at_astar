#!/usr/bin/env python
#si quieres ejecutar los archivos python con rosrun debes de cambiarle los permisos al archivo.
import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64

show_animation = True
min_ejex = -17
max_ejex = 2
min_ejey = -2
max_ejey = 14


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 expandDis=0.5, goalSampleRate=20, maxIter=1000):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.__CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation:
                self.DrawGraph(rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steer(self, rnd, nind):

        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = nind
        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-y")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "sg", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([min_ejex, max_ejex, min_ejey, max_ejey])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt planning")
    print("Calculating the path, just wait a moment...")
    # ====Search Path with RRT====
    obstacleList = [
        (1, -1, 0.9),
        (1, 0, 0.9),
        (1, 1, 0.9),
        (1, 2, 0.9),
        (1, 3, 0.9),
        (1, 4, 0.9),
        (1, 5, 0.9),
        (1, 6, 0.9),
        (1, 7, 0.9),
        (1, 8, 0.9),
        (1, 9, 0.9),
        (1, 10, 0.9),
        (1, 11, 0.9),
        (1, 12, 0.9),
        (1, 13, 0.9),
        (-16, 0, 0.9),
        (-16, 1, 0.9),
        (-16, 2, 0.9),
        (-16, 3, 0.9),
        (-16, 4, 0.9),
        (-16, 5, 0.9),
        (-16, 6, 0.9),
        (-16, 7, 0.9),
        (-16, 8, 0.9),
        (-16, 9, 0.9),
        (-16, 10, 0.9),
        (-16, 11, 0.9),
        (-16, 12, 0.9),
        (-16, 13, 0.9),
        (0, -1, 0.9),
        (-1, -1, 0.9),
        (-2, -1, 0.9),
        (-3, -1, 0.9),
        (-4, -1, 0.9),
        (-5, -1, 0.9),
        (-6, -1, 0.9),
        (-7, -1, 0.9),
        (-8, -1, 0.9),
        (-9, -1, 0.9),
        (-10, -1, 0.9),
        (-11, -1, 0.9),
        (-12, -1, 0.9),
        (-13, -1, 0.9),
        (-14, -1, 0.9),
        (-15, -1, 0.9),
        (-16, -1, 0.9),
        (-1, 13, 0.9),
        (-2, 13, 0.9),
        (-3, 13, 0.9),
        (-4, 13, 0.9),
        (-5, 13, 0.9),
        (-6, 13, 0.9),
        (-7, 13, 0.9),
        (-8, 13, 0.9),
        (-9, 13, 0.9),
        (-10, 13, 0.9),
        (-11, 13, 0.9),
        (-12, 13, 0.9),
        (-13, 13, 0.9),
        (-14, 13, 0.9),
        (-15, 13, 0.9),
        (-16, 13, 0.9),
        (-7, 0, 0.9),
        (-7, 1, 0.9),
        (-7, 2, 0.9),
        (-7, 3, 0.9),
        (-7, 4, 0.9),
        (-7, 5, 0.9),
        (-7, 6, 0.9),
        (-7, 7, 0.9),
        (8, 8, 0.9),
        (-7, 8, 0.9),
        (-6, 8, 0.9),
        (-5, 8, 0.9),
        (-4, 8, 0.9),
        (-10, 8, 0.9),
        (-10, 9, 0.9),
        (-10, 10, 0.9),
        (-10, 11, 0.9),
        (-10, 12, 0.9),
        (-10, 8, 0.9),
        (-11, 8, 0.9),
        (-12, 8, 0.9),
        (0, 13, 0.9),
        (-7, 4, 0.9),
        (-8, 4, 0.9),
        (-9, 4, 0.9),
        (-10, 4, 0.9),
        (-11, 4, 0.9),
        (-12, 4, 0.9),
        (-13, 4, 0.9),
        (-14, 4, 0.9),
        (-11, 0, 0.9),
        (-11, 1, 0.9),
        (-3, 8, 0.9)]

    # Set Initial parameters
    rrt= RRT(start=[0, 0], goal=[-5, 10],randArea=[min_ejex, max_ejey], obstacleList = obstacleList)
    path= rrt.Planning(animation=False)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        print([x for (x, y) in path])
        print([y for (x, y) in path])
        plt.pause(0.01)  # Need for Mac
        publisher(path)
        plt.show()


def publisher(path):
    pub = rospy.Publisher('python_talker', Float64, queue_size=10)
    rospy.init_node('python_talker', anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)  # 10hz
    xx = ([x for (x, y) in path])
    yy = ([y for (x, y) in path])
    print(len(xx))
    msg = 0   #publico dos mensajes vacios porque en C no recibe los dos primeros mensajes
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()
    msg = 0
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()
    msg = len(xx)+len(yy)
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()

    # while not rospy.is_shutdown():
    for i in range(len(xx)):
        msg = xx[i]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
    for i in range(len(xx)):
        msg = yy[i]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
    rospy.signal_shutdown("lol")


if __name__ == '__main__':
	main()
