import sys
import json
import numpy
import networkx as nx
from math import *
from node import Node
from in_out import *

# Utilisation de notre propre objet node
def create_graph_v1 (elements):
    G = nx.Graph()
    for i in range(len(elements)):
        G.add_node(i, node=elements[i])
    return G

# Utilisation d'attributs via networkx 
def create_graph_v2 (elements):
    G = nx.Graph()
    for i in range(len(elements)):
        G.add_node(i, x=elements[i][0], y=elements[i][0], angle=theta_step)
    return G

# Parsing the configuration file
problem_path = sys.argv[1]
problem = getProblem(problem_path)

## Goals config
goal = problem["goals"][0]["posts"]
(a, b) = (abs(goal[0][0] - goal[1][0]), abs(goal[0][1] - goal[1][1]))
length_goal = sqrt(a*a+ b*b)
center_goal = [sum(x) / 2 for x in zip(*(goal[0], goal[1]))]

## Opponents
opponents = numpy.array(problem["opponents"])

## Radius, Theta_Step & Pos_Step
radius = problem["robot_radius"]
theta_step = problem["theta_step"]
theta_step = problem["theta_step"]

# Creation of graph_v1
opp = []
for i in range(len(opponents)):
    opp.append(Node(opponents[i], theta_step))
Gr = create_graph_v1(opponents)
print(Gr.nodes.data())

# Creation of graph_v2
Gr = create_graph_v2(opponents)
print(Gr.nodes.data())

''' Ici relier les noeuds avec des arêtes ... Ensuite resolution du prblm '''

## Put a robot in front of an opponents
offensers = []
for i in range(len(opponents)):
    opp_goal = center_goal - opponents[i]
    length = 2 * radius * numpy.linalg.norm(opp_goal) / length_goal
    unitary_opp_goal = [x / numpy.linalg.norm(opp_goal) for x in opp_goal]
    res = opponents[i] + [x * length for x in unitary_opp_goal]
    defense.append(res)

# Creation of result file
print(offensers)
graphToJson(offensers, 'result.json')