import sys
import json
import numpy as np
import networkx as nx
import networkx.algorithms.approximation as naa
import networkx.algorithms.components.connected as nacc
from math import pi
from math import tan, cos, sin
import math

class BrutForce:
    def __init__(self, filename):
        self.filename = filename
        with open(self.filename) as problem_file:
            problem = json.load(problem_file)
        self.opponents = np.array(problem["opponents"])
        self.goals = np.array(problem["goals"])
        self.field_limits = np.array(problem["field_limits"])
        self.pos_step = problem["pos_step"]
        self.theta_step = problem["theta_step"]
        self.robot_radius = problem["robot_radius"]

        self.computingField()
        self.computingShoots()

        self.shotsOnTarget()
        print(len(self.Bc), len(self.Bnc))

        self.computingGraph()
        self.creatingGraph()

    def computingField(self):
        """
        Computes positions for defensors.
        """
        self.T = []
        for i in np.arange(self.field_limits[0][0], self.field_limits[0][1], self.pos_step):
            for j in np.arange(self.field_limits[1][0], self.field_limits[1][1], self.pos_step):
                self.T.append((i, j))


    def computingShoots(self):
        """
        Computes all shoots.
        """
        self.B = []
        for o in self.opponents:
            for a in np.arange(-pi, pi, self.theta_step):
                self.B.append((o, a))


    def lineIntersection(self, l1, l2):
        """
        Computes coordinates of the intersection of two lines.
        Input: l1 = (a, b, c) such that a*x + b*y + c = 0
               l2 = (a, b, c) such that a*x + b*y + c = 0
        Output:  None if lines are parallel
                 (x, y) the intersection
        """
        (a1, b1, c1) = l1
        (a2, b2, c2) = l2
        det = a1 * b2 - b1 * a2
        if (det == 0):
            return None
        else:
            x = (b2 * c1 - b1 * c2) / det
            y = (a1 * c2 - a2 * c1) / det
            return (x, y)


    def polToCart(self, l):
        """
        Computes the cartesian equation of a line.
        Input: l = ((x, y), alpha) the polar equation
        Output: (a, b, c) such that a*x + b*y + c = 0
                (d1, d2) the direction of the line
        """
        ((i, j), a) = l
        if ((a > -pi/2) and (a < pi/2)):
            d1 = 1
        elif ((a == -pi/2) or (a == pi/2)):
            d1 = 0
        else:
            d1 = -1

        if (a < 0):
            d2 = -1
        elif ((a == 0) or (a == pi)):
            d2 = 0
        else:
            d2 = 1

        if ((a != -pi/2) or (a != pi/2)):
            return ((1, tan(a), - i - j * tan(a)), (d1, d2))
        else:
            return ((1, 0, j), (d1, d2))


    def goalToCart(self, g):
        """
        Computes the equation of a line for a goal.
        Input: g = [[x1, y1], [x2, y2]]
        Output: (a, b, c) such that a*x + b*y + c = 0
        """
        if (g[1][0] - g[0][0] == 0):
            a = 1
            b = 0
            c = g[1][0]
        else:
            a = (g[1][1] - g[0][1]) / (g[1][0] - g[0][0])
            b = 1
            c = g[0][1] - a * g[0][0]
        return (a, b, c)


    def isInGoal(self, l, g, eps = 1e-4):
        """
        Checks if the shoot l is in the goal g.
        Input: l = ((x, y), alpha) the polar coordinates
               g = ([[x1, y1], [x2, y2]], direction)
        Output: True / False
        """
        # Computes intersection between goal line and shoot.
        (l_car, d) = self.polToCart(l)
        (gc, gd) = g
        inter = self.lineIntersection(self.goalToCart(gc), l_car)
        if (inter == None):
            return False
        (x, y) = inter
        # Checks if the intersection is in the goal.
        if ((x < min(gc[0][0], gc[1][0])-eps) or (x > max(gc[0][0], gc[1][0])+eps)):
            return False
        if ((y < min(gc[0][1], gc[1][1])-eps) or (y > max(gc[0][1], gc[1][1])+eps)):
            return False

        # Checks if the shoot is in the good direction.
        if (np.dot(gd, np.array(d)) > 0):
            return False
        return True


    def shotsOnTarget(self):
        """
        Checks if shots are on target (Bc) or not (Bnc).
        """
        self.Bc = []
        self.Bnc = []
        for (o, a) in self.B:
            for g in self.goals:
                (cart, direction) = self.polToCart((o, a))
                if (self.isInGoal((o, a), (g["posts"], g["direction"]))):
                    self.Bc.append((cart, o, direction))
                else:
                    self.Bnc.append((cart, o, direction))


    def isIntercepted(self, pos, shoot):
        """
        Checks if a position can intercept a shoot.
        Input: pos (x, y)
               shoot (a, b, c) such that a*x + b*y + c = 0
        Output: True / False
        """
        (x, y) = pos
        ((a, b, c), (ox, oy), (dx, dy)) = shoot
        d = (a * x + b * y + c) / math.sqrt(a**2 + b**2)
        if (d > self.robot_radius):
            return False
        if (dx == 0):
            k = (y - oy) / dy
        else:
            k = (x - ox) / dx
        return (k > 0)


    def isCollisionOpponent(self, pos):
        (x, y) = pos
        for (ox, oy) in self.opponents:
            d = math.sqrt((x - ox)**2 + (y - oy)**2)
            if (d <= 2 * self.robot_radius):
                return True
        return False


    def computingGraph(self):
        """
        Computes edges and nodes of the graph modeling the problem.
        """
        self.edges = []
        self.nodes = self.Bc.copy()
        k = 0
        for i in range(len(self.T)):
            if (self.isCollisionOpponent(self.T[i])):
                k = k + 1
                continue
            neighbour = False
            for j in range(len(self.Bc)):
                if (self.isIntercepted(self.T[i], self.Bc[j])):
                    neighbour = True
                    self.edges.append((len(self.Bc) + i - k, j))
            if (neighbour):
                self.nodes.append(self.T[i])
            else:
                k = k + 1


    def creatingGraph(self):
        """
        Creates a graph modeling the problem.
        """
        self.G = nx.Graph()
        self.G.add_nodes_from(range(len(self.Bc)), weight = 100)
        self.G.add_nodes_from(range(len(self.Bc), len(self.nodes)), weight = 1)
        self.G.add_edges_from(self.edges)
        sub = [self.G.subgraph(c).copy() for c in nx.connected_components(self.G)]
        #print(sorted(nx.connected_components(self.G), key=len, reverse=True))
        print(len(sub))
        print("lenBc", len(self.Bc))#self.Bc[11], self.Bc[12])
        #print("toto", naa.min_weighted_dominating_set(sub[1]))
        print(len(self.edges), len(self.nodes))
        G = nx.path_graph(4)
        print([n for n in G.neighbors(3)])
        print("brutforce", self.brutForce(self.G))

    def brutForce(self, G, k = 8):
        if (k < 0):
            return 9
        sets = []
        nodes = list(G.nodes)
        for i in nodes:
            if (i < len(self.Bc)):
                continue
            #print("t", i, k)
            H = G.copy()
            neighbors = list(H.neighbors(i))
            for j in neighbors:
                H.remove_node(j)
            H.remove_node(i)
            if (list(H.nodes)[0] >= len(self.Bc)):
                if (8-k) <= 4:
                    print("test")
                return 8 - k
            sets.append(self.brutForce(H, k - 1))
        return max(sets)


if __name__ == "__main__":
    filename =  sys.argv[1]
    BrutForce = BrutForce(filename)
    print("###########")
    G = nx.Graph()
    G.add_nodes_from(range(1, 6), weight = 1)
    G.add_nodes_from(range(6, 11), weight = 100)
    G.add_edges_from([(1, 6), (2, 6), (2, 7), (2, 8), (3, 8), (3, 9), (4, 9), (4, 10), (5, 6)])
    print(naa.min_weighted_dominating_set(G, 2))
    print(nx.is_dominating_set(G, naa.min_weighted_dominating_set(G, 2)))
