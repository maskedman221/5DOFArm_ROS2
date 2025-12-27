import math

PI = 3.14159265359
HALF_PI = 1.5707963268
DOUBLE_PI = 6.28318530718
DEGREE_STEP = 0.01745329251
FREE_ANGLE = 999.9


class Link:
    def __init__(self):
        self._length = 0.0
        self._angleLow = 0.0
        self._angleHigh = 0.0
        self._angle = 0.0

    def init(self, length, angle_low_limit, angle_high_limit):
        self._length = length
        self._angleLow = angle_low_limit
        self._angleHigh = angle_high_limit

    def inRange(self, angle):
        return self._angleLow <= angle <= self._angleHigh

    def getLength(self):
        return self._length

    def getAngle(self):
        return self._angle

    def setAngle(self, angle):
        self._angle = angle


class Inverse:
    def __init__(self):
        self._L0 = Link()
        self._L1 = Link()
        self._L2 = Link()
        self._L3 = Link()
        self._currentPhi = -DOUBLE_PI

    def attach(self, shoulder, upperarm, forearm, hand):
        self._L0 = shoulder
        self._L1 = upperarm
        self._L2 = forearm
        self._L3 = hand

    def _cosrule(self, opposite, adjacent1, adjacent2):
        delta = 2 * adjacent1 * adjacent2
        if delta == 0:
            return None
        cos_val = (adjacent1**2 + adjacent2**2 - opposite**2) / delta
        if cos_val > 1 or cos_val < -1:
            return None
        return math.acos(cos_val)

    def _solve_fixed_phi(self, x, y, phi):
        # wrist coordinates
        xw = x + self._L3.getLength() * math.cos(phi)
        yw = y + self._L3.getLength() * math.sin(phi)

        R = math.sqrt(xw**2 + yw**2)

        beta = self._cosrule(self._L2.getLength(), R, self._L1.getLength())
        if beta is None:
            return None
        gamma = self._cosrule(R, self._L1.getLength(), self._L2.getLength())
        if gamma is None:
            return None

        alpha = math.atan2(yw, xw)
        shoulder = alpha + beta
        elbow = gamma
        wrist = PI - shoulder - elbow + phi

        return shoulder, elbow, wrist

    def _solve_free_phi(self, x, y):
        for phi in frange(-DOUBLE_PI, DOUBLE_PI, DEGREE_STEP):
            result = self._solve_fixed_phi(x, y, phi)
            if result is not None:
                self._currentPhi = phi
                return result
        return None

    def solve(self, x, y, z, phi=FREE_ANGLE):
        _r = math.sqrt(x**2 + y**2)
        _base = math.atan2(y, x)

        if not self._L0.inRange(_base):
            _base += PI if _base < 0 else -PI
            _r *= -1
            if phi != FREE_ANGLE:
                phi = PI - phi

        if phi == FREE_ANGLE:
            result = self._solve_free_phi(_r, z - self._L0.getLength())
        else:
            result = self._solve_fixed_phi(_r, z - self._L0.getLength(), phi)

        if result is None:
            return None

        shoulder, elbow, wrist = result
        return _base, shoulder, elbow, wrist


def frange(start, stop, step):
    while start < stop:
        yield start
        start += step


# Conversion helpers
def b2a(b):
    return b / 180.0 * PI

def a2b(a):
    return a * 180.0 / PI

# Example usage inside your ROS2 node
if __name__ == "__main__":
    base = Link()
    upperarm = Link()
    forearm = Link()
    hand = Link()

    base.init(130, b2a(0.0), b2a(180.0))
    upperarm.init(130, b2a(-45.0), b2a(165.0))
    forearm.init(130, b2a(45.0), b2a(300.0))
    hand.init(100, b2a(90.0), b2a(270.0))

    inv = Inverse()
    inv.attach(base, upperarm, forearm, hand)

    result = inv.solve(120, 0, 100, b2a(90.0))
    if result:
        a0, a1, a2, a3 = result
        print(a2b(a0), a2b(a1), a2b(a2), a2b(a3))
    else:
        print("No solution found!")
