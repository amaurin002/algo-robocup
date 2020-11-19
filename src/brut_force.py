import sys
import json
import numpy as np
import networkx as nx
import networkx.algorithms.components.connected as nacc
from math import tan, cos, sin, pi
import math
import numpy as np
import matplotlib.pyplot as plt

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

        if ("goalkeeper_area" in problem):
            self.goalkeeper_area = problem["goalkeeper_area"]
        else:
            self.goalkeeper_area = None

        self.maxPlayers = 8

        self.computingField()
        self.computingShoots()

        self.shotsOnTarget()

        self.computingGraph()
        self.creatingGraph()

    def computingField(self):
        """
        Computes positions for defensors.
        """
        self.T = []
        self.TG = []
        for i in np.arange(self.field_limits[0][0], self.field_limits[0][1], self.pos_step):
            for j in np.arange(self.field_limits[1][0], self.field_limits[1][1], self.pos_step):
                if (self.goalkeeper_area == None):
                    self.T.append((i, j))
                else:
                    if (self.isInGoalArea((i, j))):
                        self.TG.append((i, j))
                    else:
                        self.T.append((i, j))


    def isInGoalArea(self, p, eps = 1e-4):
        """
        Checks if a point is in the goal area.
        Input: p = (x, y)
        Output: True / False
        """
        (x, y) = p
        if (x <= min(self.goalkeeper_area[0]) - eps):
            return False
        if (x >= max(self.goalkeeper_area[0]) + eps):
            return False
        if (y <= min(self.goalkeeper_area[1]) - eps):
            return False
        if (y >= max(self.goalkeeper_area[1]) + eps):
            return False
        return True


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
        c1 = -c1
        c2 = -c2
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
            return ((1, 0, -i), (d1, d2))


    def goalToCart(self, g):
        """
        Computes the equation of a line for a goal.
        Input: g = [[x1, y1], [x2, y2]]
        Output: (a, b, c) such that a*x + b*y + c = 0
        """
        if (g[1][0] - g[0][0] == 0):
            a = 1
            b = 0
            c = - g[1][0]
        else:
            a = (g[1][1] - g[0][1]) / (g[1][0] - g[0][0])
            b = 1
            c = - g[0][1] - a * g[0][0]
        return (a, b, c)


    def isInGoal(self, l, g, eps = 1e-6):
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
        if (np.dot(gd, np.array(d)) >= 0):
            return False
        self.INTER.append(inter)
        return True


    def shotsOnTarget(self):
        """
        Checks if shots are on target (Bc) or not (Bnc).
        """
        self.Bc = []
        self.Bnc = []
        self.INTER = []
        for (o, a) in self.B:
            for g in self.goals:
                (cart, direction) = self.polToCart((o, a))
                if (self.isInGoal((o, a), (g["posts"], g["direction"]))):
                    self.Bc.append((cart, o, direction))
                else:
                    self.Bnc.append((cart, o, direction))


    def isIntercepted(self, pos, shoot, index):
        """
        Checks if a position can intercept a shoot.
        Input: pos (x, y)
               shoot (a, b, c) such that a*x + b*y + c = 0
               inter index
        Output: True / False
        """
        (x, y) = pos
        ((a, b, c), (ox, oy), (dx, dy)) = shoot
        d = abs(a * x + b * y + c) / math.sqrt(a**2 + b**2)
        if (d > self.robot_radius):
            return False
        if (index < len(self.Bc)):
            (ix, iy) = self.INTER[index]
            d1 = math.sqrt((ox - ix)**2 + (oy - iy)**2)
            d2 = math.sqrt((x - ox)**2 + (y - oy)**2)
            if (d1 <= d2):
                return False
        if (dx == 0):
            k = (y - oy) / dy
        else:
            k = (x - ox) / dx
        return (k > 0)


    def isCollisionOpponent(self, pos):
        """
        Checks if the position can be used for a defensors.
        Input : pos (x, y)
        Output: True / False
        """
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
        POS = self.T + self.TG
        self.nbT = len(self.T)
        for i in range(len(POS)):
            if (len(self.T) == i):
                self.nbT = len(self.Bc) + i - k
            if (self.isCollisionOpponent(POS[i])):
                k = k + 1
                continue
            neighbour = False
            for j in range(len(self.Bc)):
                if (self.isIntercepted(POS[i], self.Bc[j], j)):
                    neighbour = True
                    self.edges.append((len(self.Bc) + i - k, j))
            if (neighbour):
                self.nodes.append(POS[i])
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
        subG = [self.G.subgraph(c).copy() for c in nx.connected_components(self.G)]
        #for (x, y) in self.nodes[len(self.Bc):]:
        #    plt.scatter(x, y)
        #plt.gca().set_xlim(-4.5, 4.5)
        #plt.gca().set_ylim(-3, 3)
        #plt.show()
        self.dominatingSet = []
        for i in subG:
            ret = []
            k = 1
            Bc = []
            area = (self.goalkeeper_area != None)
            for j in i.nodes:
                if (j < len(self.Bc)):
                    Bc.append(j)
            if (len(Bc) == 0):
                continue
            else:
                print("No solution for 0 defendors.")
            while ((ret == []) and (k <= self.maxPlayers)):
                ret = self.brutForce(i, k, Bc, [], area)
                if (ret == []):
                    print("No solution for", k, "defendors.")
                k = k + 1
            if (ret == []):
                raise Exception("This configuration can't be defended.")
            self.dominatingSet += ret
            if (len(self.dominatingSet) > self.maxPlayers):
                break
        if (len(self.dominatingSet) > self.maxPlayers):
            raise Exception("This configuration can't be defended.")


    def roundClosest(self, p):
        c = []
        for a in p:
            res = a % self.pos_step
            if (res >= self.pos_step / 2):
                a = a + self.pos_step
            a = a - res
            c.append(a)
        return c


    def saveJSon(self, filename):
        defense = []
        for i in self.dominatingSet:
            defense.append(self.roundClosest(self.nodes[i]))
        data = {}
        data["defenders"] = defense
        with open(filename + '.json', 'w') as outfile:
            json.dump(data, outfile, sort_keys = True, indent = 4,
                      separators = (',', ': '))


    def brutForce(self, G, k, Bc, D, area):
        if (k == 0):
            if (self.isDominated(G, Bc, D)):
                return D
            return []
        nodes = list(G.nodes)
        for i in nodes:
            if ((not area) and (i >= self.nbT)):
                continue
            if (i in D or i < len(self.Bc)):
                continue
            E = self.brutForce(G, k-1, Bc, D.copy() + [i],
                               i >= self.nbT)
            if (E != []):
                return E
        return []


    def isDominated(self, G, Bc, D):
        X = set()
        for i in D:
            N = list(G.neighbors(i))
            for j in N:
                X.add(j)
        return (len(Bc) == len(X))


if __name__ == "__main__":
    if (len(sys.argv) != 3):
        raise Exception("Usage : ./brut_force filename_problem filename_solution")
    filename =  sys.argv[1]
    BrutForce = BrutForce(filename)
    BrutForce.saveJSon(sys.argv[2])
