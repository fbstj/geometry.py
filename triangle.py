"""
Triangle: a class for representing three vertexes and the angles between them
various triangle solvers from wikipedia: Solution of Triangles
    solver for three 2D points
a generator for solving all unique triangles from a subset of lengths/angles

includes _eq and _eqa for imprecise equals
sine_rule, cosine_rule, acosine_rule helper functions

"""
from __future__ import division
import math
import fuz

def sine_rule(a, A, B):
    """work out length using oposite angle and another matching pair"""
    return a * math.sin(B) / math.sin(A)

def cosine_rule(a, b, C):
    """work out length using other lengths and oposite angle"""
    return math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(C))

def acosine_rule(a, b, c):
    """work out angle using three sides"""
    return math.acos((b**2 + c**2 - a**2) / (2 * b * c))

def SSS(a, b, c):
    """generate angles from three sides"""
    (a,b,c) = sorted((a,b,c))
    if abs(a + b - c) < 1e-15:
        return None # straight line
    A1 = acosine_rule(a, b, c)
    A2 = acosine_rule(a, c, b)
    assert A1 == A2, "abc != acb"
    B1 = acosine_rule(b, c, a)
    B2 = acosine_rule(b, a, c)
    assert B1 == B2, "bca != bac"
    C1 = acosine_rule(c, a, b)
    C2 = acosine_rule(c, b, a)
    assert C1 == C2, "cab != cba"
    return Triangle((a, b, c), (A1, B1, C1))

def SAS(a, B, c):
    """generate side and angles from two sides and interior angle"""
    b1 = cosine_rule(a, c, B)
    b2 = cosine_rule(c, a, B)
    assert b1 == b2, "acb != cab"
    return SSS(a, b1, c)

def SSA(a, b, A):
    """generate side and angles from two sides and exterior angle"""
    if fuz.pi == A:
        return None# straight line
    sinB = math.sin(A) * b / a
    assert sinB <= 1, "sin(B) cannot be greater than 1"
    if sinB == 1:
        # right angle triangle
        B = math.pi / 2
        c = math.hypot(a, b)
        C = B - A
        return Triangle((a, b, c), (A, B, C))
    B = math.asin(sinB)
    C = math.pi - A - B
    c = sine_rule(a, A, C)
    if a < b:
        # B may be either accute or obtuse
        value = dict(acute = Triangle((a, b, c), (A, B, C)))
        B = math.pi - B
        C = math.pi - A - B
        c = sine_rule(a, A, C)
        value.update(obtuse = Triangle((a, b, c), (A, B, C)))
        return value
    return Triangle((a, b, c), (A, B, C))

def ASA(A, b, C):
    """generate sides and angle from side and two angles"""
    B = math.pi - A - C
    a = sine_rule(b, B, A)
    c = sine_rule(b, B, C)
    return Triangle((a, b, c), (A, B, C))

def AAA(A, B, C, perimeter=1):
    """generate unit side lengtsh from sides and two angles"""
    # the sides will add up to perimeter
    assert fuz.pi == A + B + C, str(angles) + " should sum to pi"
    a = 1.0
    b = sine_rule(a, A, B)
    c = sine_rule(a, A, C)
    abc = (a + b + c) / perimeter
    return Triangle((a / abc, b / abc, c / abc), (A, B, C))

def PPP(p1, p2, p3):
    """generates Triangle from three points"""
    assert len(p1) == len(p2) == len(p3), "points need to be in same space"
    if len(p1) == 2:
        x1,y1 = p1
        x2,y2 = p2
        x3,y3 = p3
        assert x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2) != 0, "points are colinear"
        a = math.hypot(x1 - x2, y1 - y2)
        b = math.hypot(x2 - x3, y2 - y3)
        c = math.hypot(x3 - x1, y3 - y1)
        t = SSS(a, b, c)
        return t
    raise NotImplementedError()

class Triangle:
    def __init__(self, sides, angles):
        """"""
        pairs = zip(sides, angles)
        pairs = sorted(pairs, key=lambda (side,angle): angle)
        sides,angles = zip(*pairs)
        a,b,c = sides
        A,B,C = angles
        assert C < math.pi, "the largest angle cannot be larger than pi"
        assert fuz.pi == sum(angles), str(angles) + " should sum to pi"
        sinrule = (math.sin(A) / a, math.sin(B) / b, math.sin(C) / c)
        assert fuz.fuz(max(sinrule)) ==  min(sinrule), "sine rule fail"
        # properties
        self.a = fuz.fuz(a)
        self.b = fuz.fuz(b)
        self.c = fuz.fuz(c)
        self.A = fuz.fuz(A)
        self.B = fuz.fuz(B)
        self.C = fuz.fuz(C)
        # group properties
        self.sides = sides
        self.angles = angles

    def __repr__(self):
        return str((self.sides, map(math.degrees, self.angles)))

    def __hash__(self):
        return 1 # causes set() to use the __eq__ method

    def similar(self, other):
        """triangles have equal angles"""
        assert isinstance(other, Triangle), "Triangles required"
        return self.A == other.A and self.B == other.B and self.C == other.C

    def __eq__(self, other):
        assert isinstance(other, Triangle), "Triangle required"
        return self.similar(other) and self.a == other.a

    def __contains__(self, other):
        #
        assert isinstance(other, Triangle), "Triangle required"
        raise NotImplementedError()

    @property
    def area(self):
        """the area of the triangle"""
        s = self.perimeter / 2.0
        return math.sqrt(s * (s - self.a) * (s - self.b) * (s - self.c))

    @property
    def perimeter(self):
        return sum(self.sides)

    @property
    def right_angled(self):
        """triangle is right angled"""
        return self.C == math.pi/2

    @property
    def equilateral(self):
        """triangle is right angled"""
        return self.A == self.B == self.C == math.pi/3

    def scale(self, area=1, perimeter=1):
        """scale the triangle by the passed amount"""
        if permieter != 1:
            assert area == 1, "cannot change perimeter and area at same time"
            self.a *= permieter
            self.b *= permieter
            self.c *= permieter
            return
        raise NotImplementedError()

    def points(self, flip=False, origin=(0,0), angle=0):
        """returns three coordinates
with no parameters return (0,0), (c, 0) and the calculated thrid point
the origin parameter translates all three points after generation
if flip is True, point two becomes (0, c) and the thrid
the angle parameter will rotate all three points relative to (0,0)
TODO: work out which order of changes is prefered
"""
        def translate(p, o=origin):
            return (o[0] + p[0], o[1] + p[1])
        def rotate(point, r=angle, o=(0,0)):
            x,y = point
            x,y = (x - o[0], y - o[1])
            cos,sin = math.cos(r), math.sin(r)
            x,y = (x * cos - y * sin, x * sin + y * cos)
            return (x + o[0], y + o[1])
        flip = -1 if flip else 1
        pA = (0, 0)
        pB = (self.c, 0)[::flip]
        pC = rotate((self.b, 0), r=self.A)[::flip]
        return map(rotate, map(translate, (pA, pB, pC)))

def triangles(a=None, b=None, c=None, A=None, B=None, C=None):
    has_a = a is not None
    has_b = b is not None
    has_c = c is not None
    has_A = A is not None
    has_B = B is not None
    has_C = C is not None
    def _all():
        if has_a and has_b and has_c and has_A and has_B and has_C:
            yield Triangle((a, b, c), (A, B, C))
        if has_a and has_b and has_c:
            yield SSS(a, b, c)
            yield SSS(a, c, b)
            yield SSS(b, a, c)
            yield SSS(b, c, a)
            yield SSS(c, a, b)
            yield SSS(c, b, a)
        if has_a and has_B and has_c:
            yield SAS(a, B, c)
            yield SAS(c, B, a)
        if has_A and has_b and has_c:
            yield SAS(b, A, c)
            yield SAS(c, A, b)
        if has_a and has_b and has_C:
            yield SAS(a, C, b)
            yield SAS(b, C, a)
        if has_a and has_A and has_b:
            ssa =  SSA(a, b, A)
            if isinstance(ssa, dict):
                yield ssa['acute']
                yield ssa['obtuse']
            else:
                yield ssa
    return set(_all())

if __name__ == "__main__":

    ppp = PPP([1,1],[5,3],[1,3])
    print ppp
    print PPP(*ppp.points())
    assert ppp == PPP(*ppp.points()), "PPP should equal PPP of itself"
    print PPP(*ppp.points(flip=True))
    assert ppp == PPP(*ppp.points(flip=True)), "reflection broke"
    print PPP(*ppp.points(angle=math.pi))
    assert ppp == PPP(*ppp.points(angle=math.pi)), "rotating broke"
    print PPP(*ppp.points(origin=(1,1)))
    assert ppp == PPP(*ppp.points(origin=(1,1))), "translation broke"

    print SSA(5, 6, math.pi/4)

    def meta(t):
        print t
        if t is None:
            return ''
        print t.similar(t), t.right_angled, t.equilateral, t.area
        return ''

    print meta(SSS(1, 1, 1))
    print meta(SSS(3, 1, 2))
    print meta(SSS(2, 4, 3))
    print meta(AAA(2 * math.pi / 6, math.pi / 6, math.pi / 2))
    print meta(AAA(math.pi / 4, math.pi / 4, math.pi / 2, perimeter=5))
    print meta(Triangle((1, math.sqrt(3), 2),\
                        (math.pi / 6, 2 * math.pi / 6, math.pi / 2)))

    print SSS(1, 1, 1).similar(SSS(2, 2, 2)),\
          SSS(2, 1, 2).similar(SSS(1, 1, 1))

    ts = triangles(1, math.sqrt(3), 2, math.pi/6, 2*math.pi/6, math.pi/2)
    for t in ts:
        print t
