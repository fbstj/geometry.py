import math

class fuz(float):
    precision = 1e-10

    def __eq__(self, other):
        return abs(self - other) < self.precision

pi = fuz(math.pi)
