from __future__ import division

import numpy
import cv2

from tf import transformations

def make_dest(width, height):
    return numpy.zeros((height, width, 4), dtype=numpy.uint8)

class Renderer(object):
    def __init__(self, dest, proj_mat=numpy.identity(3)):
        assert proj_mat.shape[0] == 3
        self._dest = dest
        self._proj_mat = proj_mat
    
    def draw_polygon(self, corners, color):
        self.draw_polygon_homo(
            [numpy.concatenate([corner, [1]])
                for corner in corners],
            color,
        )
    
    def draw_polygon_homo(self, corners, color):
        def wdivide((x, y, w)):
            return int(round(x/w)), int(round(y/w))
        cv2.fillPoly(
            self._dest,
            numpy.array([[wdivide(self._proj_mat.dot(corner))
                for corner in corners]]),
            color,
        )
    
    def prepend_transform(self, mat):
        return Renderer(self._dest, self._proj_mat.dot(mat))

def look_at_mat(rel_pos):
    forward = transformations.unit_vector(rel_pos)
    left = transformations.unit_vector(numpy.cross([0, 0, 1], forward))
    up = numpy.cross(forward, left)
    mat = numpy.array([
        -left,
        -up,
        forward,
    ])
    return numpy.vstack([
        numpy.hstack([mat, [[0], [0], [0]]]),
        [[0, 0, 0, 1]],
    ])
