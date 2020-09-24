import sys
import json
import numpy
from math import *

problem_path = sys.argv[1]

with open(problem_path) as problem_file:
    problem = json.load(problem_file)

goal = problem["goals"][0]["posts"]
(a, b) = (abs(goal[0][0] - goal[1][0]), abs(goal[0][1] - goal[1][1]))
length_goal = sqrt(a*a+ b*b)
center_goal = [sum(x) / 2 for x in zip(*(goal[0], goal[1]))]

radius = problem["robot_radius"]

opponents = numpy.array(problem["opponents"])

defense = []
for i in range(len(opponents)):
    opp_goal = center_goal - opponents[i]
    length = 2 * radius * numpy.linalg.norm(opp_goal) / length_goal
    unitary_opp_goal = [x / numpy.linalg.norm(opp_goal) for x in opp_goal]
    res = opponents[i] + [x * length for x in unitary_opp_goal]
    print(res)
