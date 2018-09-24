"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import random
import math
import copy

show_animation = True
min_ejex=-9
max_ejex=2
min_ejey=-2
max_ejey=9

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=0.5, goalSampleRate=10, maxIter=500):       #cuanto mas alto el goalSampleRate más directo va
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
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList):
                continue

            self.nodeList.append(newNode)
            print("nNodelist:", len(self.nodeList))

            # check goal
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

        return path

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

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
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        (1, -1, 0.8),
        (1, 0, 0.8),
        (1, 1, 0.8),
	(1, 2, 0.8),
	(1, 3, 0.8),
	(1, 4, 0.8),
	(1, 5, 0.8),
	(1, 6, 0.8),
	(1, 7, 0.8),
	(1, 8, 0.8),
	(1, 9, 0.8),
        (0, -1, 0.8),
	(-1, -1, 0.8),
	(-2, -1, 0.8),
	(-3, -1, 0.8),
	(-4, -1, 0.8),
	(-5, -1, 0.8),
	(-6, -1, 0.8),
	(-7, -1, 0.8),
	(-8, -1, 0.8),
	(-9, -1, 0.8),
	(-7, 0, 0.8),
	(-7, 1, 0.8),
	(-7, 2, 0.8),
	(-7, 3, 0.8),
	(-7, 4, 0.8),
	(-7, 5, 0.8),
	(-7, 6, 0.8),
	(-7, 7, 0.8),
	(8, 8, 0.8),
	(-7, 8, 0.8),
	(-6, 8, 0.8),
	(-5, 8, 0.8),
	(-4, 8, 0.8),
	(-3, 8, 0.8)

    ]  # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[0, 8],
              randArea=[min_ejex, max_ejey], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-m')
	print([x for (x, y) in path])
	print([y for (x, y) in path])
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()