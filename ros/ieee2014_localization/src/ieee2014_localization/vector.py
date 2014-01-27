from __future__ import division

import math
import random

class V(tuple):
    __slots__ = []
    
    def __neg__(self):
        return V(-a for a in self)
    
    def __add__(self, other):
        assert isinstance(other, V)
        assert len(self) == len(other)
        return V(a + b for a, b in zip(self, other))
    def __radd__(self, other):
        return self + other
    
    def __mul__(self, other):
        if isinstance(other, V):
            assert len(self) == len(other)
            return sum(a * b for a, b in zip(self, other))
        elif isinstance(other, (int, float, long)):
            return V(a * other for a in self)
        else:
            assert False
    def __rmul__(self, other):
        return self * other
    
    def __sub__(self, other):
        return self + -other
    
    def __mod__(self, other):
        assert isinstance(other, V)
        if len(self) == 3 and len(other) == 3:
            (x, y, z), (X, Y, Z) = self, other
            return V([y*Z-z*Y, z*X-x*Z, x*Y-y*X])
        elif len(self) == 4 and len(other) == 4:
            return V((
                self[0] * other[0] - self[1] * other[1] - self[2] * other[2] -  self[3] * other[3],
                self[0] * other[1] + self[1] * other[0] + self[2] * other[3] -  self[3] * other[2],
                self[0] * other[2] - self[1] * other[3] + self[2] * other[0] +  self[3] * other[1],
                self[0] * other[3] + self[1] * other[2] - self[2] * other[1] +  self[3] * other[0],
            ))
        else:
            raise TypeError()
    def __rmod__(self, other):
        assert isinstance(other, V)
        return self.__mod__.im_func(other, self)
    
    def __repr__(self):
        return 'v' + tuple.__repr__(self)
    
    def norm(self):
        return math.sqrt(self*self)
    
    def unit(self):
        n = self.norm()
        if n == 0:
            self = V(random.gauss(0, 1) for _ in self)
            n = self.norm()
        return self*(1/self.norm())
    
    def conj(self):
        return V(-x if i else x for i, x in enumerate(self))
    
    def quat_to_matrix3(self):
        return m(
            (1 - 2*self[2]*self[2] - 2*self[3]*self[3],     2*self[1]*self[2] - 2*self[3]*self[0],     2*self[1]*self[3] + 2*self[2]*self[0]),
            (    2*self[1]*self[2] + 2*self[3]*self[0], 1 - 2*self[1]*self[1] - 2*self[3]*self[3],     2*self[2]*self[3] - 2*self[1]*self[0]),
            (    2*self[1]*self[3] - 2*self[2]*self[0],     2*self[2]*self[3] + 2*self[1]*self[0], 1 - 2*self[1]*self[1] - 2*self[2]*self[2]),
        )
    def quat_to_matrix4(self, pos=(0, 0, 0)):
        return M(tuple(a) + (b,) for a, b in zip(tuple(self.quat_to_matrix3()) + ((0, 0, 0),), (pos[0], pos[1], pos[2], 1)))
    
    def quat_to_axisangle(self):
        self = self.unit()
        if self[0] < 0:
            self = -self
        return V(self[1:]).unit(), math.acos(self[0]) * 2
    
    def quat_to_scaledaxis(self):
        axis, angle = self.quat_to_axisangle()
        return axis * angle
    
    def quat_rot(self, v):
        return V((self % V([0] + list(v)) % self.conj())[1:])

def axisangle_to_quat(axis, angle):
    return V([math.cos(angle/2)] + list(math.sin(angle/2) * V(axis).unit()))

def scaledaxis_to_quat(scaledaxis):
    scaledaxis = V(scaledaxis)
    return axisangle_to_quat(scaledaxis, scaledaxis.norm())

def v(*args):
    return V(args)

class M(tuple):
    @classmethod
    def I(self, size):
        return M([[1 if x == y else 0 for x in xrange(size)] for y in xrange(size)])
    
    def __neg__(self):
        return M([[-item for item in row] for row in self])
    
    def __add__(self, other):
        assert isinstance(other, M)
        return M([[a + b for a, b in zip(self_row, other_row)] for self_row, other_row in zip(self, other)])
    def __radd__(self, other):
        return self + other
    
    def __sub__(self, other):
        return self + -other
    def __rsub__(self, other):
        return -self + other
    
    def __mul__(self, other):
        if isinstance(other, M):
            return M([[V(row)*V(col) for col in other.T] for row in self])
        elif isinstance(other, (float, int, long)):
            return M([[item*other for item in row] for row in self])
        else:
            assert False, other
    def __rmul__(self, other):
        if isinstance(other, M):
            return self.__mul__.im_func(other, self)
        elif isinstance(other, (float, int, long)):
            return M([[item*other for item in row] for row in self])
        else:
            assert False, other
    
    @property
    def det(self):
        if len(self) == 3 and len(self[0]) == 3:
            return V(self[0]) % V(self[1]) * V(self[2])
        else:
            raise NotImplementedError()
    
    @property
    def T(self):
        return M(zip(*self))
    
    @property
    def trace(self):
        return sum(item for y, row in enumerate(self) for x, item in enumerate(row) if x == y)
    
    def __str__(self):
        return '(\n    ' + '\n    '.join(' '.join(str(x).rjust(20) for x in row) for row in self) + '\n)'

def m(*rows):
    return M(rows)
