class Node(object):

    def __init__(self, pos, angle=None):
        self.pos = pos
        self.angle = angle

    def isDef(self):
        return self.angle is None

    def isOpp(self):
        return self.angle is not None

    def __eq__(self, node):
        if self.isOff() and node.isOff():
            return self.angle == node.angle and self.pos[0] == node.pos[0] and self.pos[1] == node.pos[1]
        
        if self.isDef() and node.isDef():
            return self.pos[0] == node.pos[0] and self.pos[1] == node.pos[1]
        return False

    def __str__(self):
        return "Node (pos: %s, angle: %s)" % (self.pos, self.angle)

    __repr__ = __str__
        
    def __key(self):
        return self.pos[0], self.pos[1], self.angle

    def __hash__(self):
        return hash(self.__key())