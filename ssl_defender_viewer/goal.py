import math
import numpy
import sys

from geometry import *


class Goal:
    def __init__(self, data):
        mandatory_keys = ["posts", "direction"]
        for key in mandatory_keys:
            if key not in data:
                raise ValueError("Cannot find '" + key + "'")
        self.posts = numpy.array(data["posts"]).transpose()
        self.direction = numpy.array(data["direction"])
        if (self.posts.shape != (2, 2)):
            raise ValueError("Invalid shape for 'posts': "
                             + str(self.field_limits.shape) + " expecting (2, 2)")
        if (self.direction.shape != (2,)):
            raise ValueError("Invalid shape for 'direction': "
                             + str(self.direction.shape) + " expecting (2,)")

    def kickResult(self, pos, theta):
        """ Return None if kick cannot score a goal, otherwise return intersection with the goal """
        # Checking that ball is not coming from behind the goal
        kick_dir = numpy.array([math.cos(theta), math.sin(theta)])
        if (kick_dir.dot(self.direction) <= 0):
            return None
        return segmentLineIntersection(self.posts[:, 0], self.posts[:, 1],
                                       pos, pos + kick_dir)
