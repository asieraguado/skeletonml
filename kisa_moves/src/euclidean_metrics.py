import math

def euclideanDistance(A, B): # A and B are vectors of the same size
    sum = 0
    for i in range(0, len(A)):
        sum += (A[i]-B[i])**2
    distance = math.sqrt(sum)
    return distance