import math

#Eclidean distance
#def heuristic(a, b):
#    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

#Manhattan distance
def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])