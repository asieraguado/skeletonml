import math


def angularRelativeDistance(a, b):
    return (abs(a % math.pi - b % math.pi)) / math.pi


def angularDistance(a, b):
    return (abs(a % math.pi - b % math.pi))


# in radians
def geodesicDistance(a0, a1, b0, b1):
    try:
        return abs(math.acos(math.sin(a0) * math.sin(b0) + math.cos(a0) * math.cos(b0) * math.cos(angularDistance(a1, b1))))
    except ValueError:
        print "[Warning]: Invalid values in geodesicDistance"
        return 0


# in rad/s
def angularSpeed(a, b, tw):
    FPS = 15.0
    sec = tw / FPS
    return angularDistance(a, b) / sec