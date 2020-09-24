import math
import numpy as np


def segmentLineIntersection(seg_start, seg_end, line_p1, line_p2):
    """
    Return None if there is no intersection between the line and the segment or
    if the lines are coincident, otherwise, return the intersection point
    """
    # Line-segment intersection from points
    # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    x1 = seg_start[0]
    y1 = seg_start[1]
    x2 = seg_end[0]
    y2 = seg_end[1]
    x3 = line_p1[0]
    y3 = line_p1[1]
    x4 = line_p2[0]
    y4 = line_p2[1]
    num = (x1-x3)*(y3-y4)-(y1-y3)*(x3-x4)
    den = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
    if (den == 0):
        # Line + segment are parallel or coincident
        return None
    t = num / den
    if t < 0 or t > 1:
        return None
    return np.array([x1+t*(x2-x1), y1+t*(y2-y1)])


def segmentCircleIntersection(seg_start, seg_end, circle_center, circle_radius):
    """
    Return None if there is no intersection between the segment and the
    circle. If there is an interesection, return the first intersection with the
    circle.
    """
    # First intersect segment with the normal line passing through center of the circle
    seg_dir = seg_end - seg_start
    seg_dir = seg_dir / np.linalg.norm(seg_dir)
    normal_line_dir = np.array([-seg_dir[1], seg_dir[0]])
    normal_intersection = segmentLineIntersection(seg_start, seg_end,
                                                  circle_center, circle_center + normal_line_dir)
    # Intersection is outside of segment or too far from circle_center
    if normal_intersection is None:
        return None
    dist = np.linalg.norm(circle_center - normal_intersection)
    if dist > circle_radius:
        return None
    # Intersection is inside the circle, now going to the border in opposite direction of seg_dir
    offset_length = math.sqrt(circle_radius ** 2 - dist ** 2)  # Pythagore
    return normal_intersection - offset_length * seg_dir
