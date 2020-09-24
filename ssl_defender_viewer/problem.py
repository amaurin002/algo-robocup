import numpy
import sys

from goal import *


class Problem:
    def __init__(self, data):
        # Checking cont
        mandatory_keys = ["field_limits", "robot_radius",
                          "opponents", "theta_step", "pos_step", "goals"]
        for key in mandatory_keys:
            if key not in data:
                raise ValueError("Cannot find '" + key + "'")
        # Reading field limits
        self.field_limits = numpy.array(data["field_limits"])
        if (self.field_limits.shape != (2, 2)):
            raise ValueError("Invalid shape for 'field_limits': "
                             + str(self.field_limits.shape) + " expecting (2, 2)")
        # Reading goals
        self.goals = []
        for goal_data in data["goals"]:
            self.goals.append(Goal(goal_data))
        if (len(self.goals) == 0):
            raise ValueError("No goal found")
        # Reading opponents
        self.opponents = numpy.array(data["opponents"]).transpose()
        if (self.opponents.shape[1] == 0):
            raise ValueError("No opponent found")
        if (self.opponents.shape[0] != 2):
            raise ValueError("Invalid data for opponents")
        # Reading other parameters
        self.robot_radius = data["robot_radius"]
        self.theta_step = data["theta_step"]
        self.pos_step = data["pos_step"]
        # Reading optional parameters
        self.defenders = None
        if "defenders" in data:
            self.defenders = numpy.array(data["defenders"]).transpose()
        # Reading optional minimal distance
        self.min_dist = None
        if "min_dist" in data:
            self.min_dist = data["min_dist"]
        # Reading optional goal area
        self.goalkeeper_area = None
        if "goalkeeper_area" in data:
            self.goalkeeper_area = numpy.array(data["goalkeeper_area"])

    def getFieldCenter(self):
        """ Return the position of the center of the field """
        return (self.field_limits[:, 1] + self.field_limits[:, 0]) / 2

    """ Width of the playing field [m]"""

    def getFieldWidth(self):
        return self.field_limits[0, 1] - self.field_limits[0, 0]

    """ Height of the playing field [m]"""

    def getFieldHeight(self):
        return self.field_limits[1, 1] - self.field_limits[1, 0]

    def getNbOpponents(self):
        return self.opponents.shape[1]

    def getOpponent(self, opp_id):
        return self.opponents[:, opp_id]

    def getNbDefenders(self):
        if (self.defenders is None):
            return 0
        return self.defenders.shape[1]

    def getDefender(self, def_id):
        return self.defenders[:, def_id]
