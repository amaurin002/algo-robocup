import numpy
import sys


class Solution:
    def __init__(self, data):
        # Checking cont
        mandatory_keys = ["defenders"]
        for key in mandatory_keys:
            if key not in data:
                raise ValueError("Cannot find '" + key + "'")
        # Reading defenders
        self.defenders = numpy.array(data["defenders"]).transpose()
        if (self.defenders.shape[1] == 0):
            raise ValueError("No opponent found")
        if (self.defenders.shape[0] != 2):
            raise ValueError("Invalid data for defenders")

    def getNbDefenders(self):
        return self.defenders.shape[1]

    def getDefender(self, def_id):
        return self.defenders[:, def_id]
