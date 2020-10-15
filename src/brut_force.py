import sys
import json
import numpy
import networkx as nx
from math import pi
from math import tan
import math

filename = sys.argv[1]
with open(filename) as problem_file:
    problem = json.load(problem_file)
opponents = numpy.array(problem["opponents"])
goals = numpy.array(problem["goals"])
field_limits = numpy.array(problem["field_limits"])
pos_step = problem["pos_step"]
theta_step = problem["theta_step"]

T = []
for i in numpy.arange(field_limits[0][0], field_limits[0][1], pos_step):
    for j in numpy.arange(field_limits[1][0], field_limits[1][1], pos_step):
        T.append((i, j))

B = []
for o in opponents:
    for a in numpy.arange(-pi, pi, theta_step):
        B.append((o, a))

def line_intersection(l1, l2):
    (a1, b1, c1) = l1
    (a2, b2, c2) = l2
    determinant = a1*b2 - b1 * a2
    if (determinant == 0):
        return None
    else:
        x = (c1*b2 - c2*b1) / determinant
        y = (a1 * c2 - a2 * c1) / determinant
        return (x, y)



def pol_to_cart(l):
    ((i, j), a) = l
    if (a != -pi/2 or a != pi/2):
        return (tan(theta), 1, j + i*tan(theta))
    else:
        return (1, 0, i)

def is_in_goal(l, g):
    y = (g[1][0] - g[0][0]) / (g[1][1] - g[0][1])
    x = 0

    return 0 == 0

#print(is_in_goal(B[0], goals[0]["posts"]))

print(len(B))
Beq = []
for (o, a) in B:
    (i, j) = o



G = nx.Graph()
