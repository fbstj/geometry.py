from __future__ import division
import math
import operator

class point(tuple):
    """A multi-dimentional tuple with vector space maths helpers"""
    def __new__(cls, *args):
        return tuple.__new__(cls, args)

    def __neg__(self):
        """-(a,b,c,...) = (-a,-b,-c,...)"""
        return self.__class__(*map(operator.neg, self))

    def __abs__(self):
        """distance to origin"""
        return math.sqrt(sum(x*x for x in self))
    @property
    def norm(self):
        """normalised point"""
        mag = abs(self)
        return self.__class__(*(x / mag for x in self))

    def __invert__(self):
        """angle(s) subtended"""
        if len(self) == 2:
            return math.atan2(self[1],self[0])
        if len(self) == 3:
            x, y, z = self
            return dict(#TODO: fix these to be correct
                xy=math.atan2(y, x), xz=math.atan2(z, x),
                yz=math.atan2(z, y), yx=math.atan2(x, y),
                zx=math.atan2(x, z), zy=math.atan2(y, z),
                length=abs(self)
                )
        raise NotImplementedError()

    def __add__(self, other):
        assert len(self) == len(other), "dimentions differ"
        return self.__class__(*map(operator.add, self, other))
    def __radd__(self, other):
        assert len(self) == len(other), "dimentions differ"
        return self.__class__(*map(operator.add, other, self))
    def __sub__(self, other):
        assert len(self) == len(other), "dimentions differ"
        return self.__class__(*map(operator.sub, self, other))
    def __rsub__(self, other):
        assert len(self) == len(other), "dimentions differ"
        return self.__class__(*map(operator.sub, other, self))

    def __rmul__(self, scalar):
        """scalar product"""
        return self.__class__(*(x * scalar for x in self))
    def __mul__(self, other):
        """dot product"""
        assert len(self) == len(other), "dimentions differ"
        return sum(x * y for (x, y) in zip(self, other))

    def __xor__(self, other):
        """cross product"""
        assert len(self) == len(other) == 3, "only works for 3D vectors"
        (x1, y1, z1), (x2, y2, z2) = self, other
        return self.__class__(
            y1 * z2 - y2 * z1,
            z1 * x2 - z2 * x1,
            x1 * y2 - x2 * y1
            )

    def __rxor__(self, other):
        """reverse cross product"""
        return self.__class__(*other) ^ self

    @property
    def cylindrical(self):
        """cylindrical coordinates"""
        assert len(self) == 3, "only for 3D vectors"
        x, y, z = self
        return (math.hypot(x, y), math.atan2(y, x), z)
    @property
    def spherical(self):
        """spherical coordinates"""
        r, a, z = self.cylindrical
        return (abs(self), math.atan2(z, r), a)

def origin(length=1):
    """the origin in passed dimentions"""
    return point(*(0,)*length)

def from_xy_xz(xy, xz, length=1):
    """generates a point from two angles
        xy and xz are the angles of rotation around
            z and y respectively, relative to the x axis
    """
    tanA = math.tan(xy)
    tanB = math.tan(xz)
    assert tanA != 0 and tanB != 0, "neither tan should be 0"
    tanA2, tanB2 = tanA**2, tanB**2
    norm = tanA2 * tanB2 + tanA2 + tanB2
    assert norm != 0, "cannot be zero"
    norm = math.sqrt(norm)
    p = point(
        length * tanA * tanB / norm,
        length * tanB / norm,
        length * tanA / norm
        )
    assert abs(p) == length, "length incorrect"
    return p

def rotate(point, r=0, o=origin(2)):
    x, y = point
    x, y = (x - o[0], y - o[1])
    cos, sin = math.cos(r), math.sin(r)
    x, y = (x * cos - y * sin, x * sin + y * cos)
    return (x + o[0], y + o[1])

if __name__ == "__main__":
    p = point(1,2,3)
    print p, p == point(1,2,3), p == (1,2,3), (1,2,3) == p
    print 5 * p, p * p, p * (1, 1, 1), (1, 2, 3) * p
    print p ^ p, p ^ (1, 1, 1), (1, 1, 1) ^ p
    print
    print p + (4,4,5), (4,4,5) + p
    print p - (4,4,5), (4,4,5) - p
    print
    print -p, abs(point(3,4)), math.degrees(~point(-1,1))
    print
    print p.norm, p.norm.norm
    print
    print p.cylindrical, p.spherical
    print origin(1), origin(2), origin(3)
    print
    q = point(1,1,1)
    qa = ~q
    print q
    print qa
    print from_xy_xz(qa['yx'], qa['zx'], length=abs(q))
    print
