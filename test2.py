class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

total = rigid + clearance

def check_polygon(polygon, point, clearance, rigid):
    A = []
    B = []
    C = []
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i+1)% len(polygon)]
        
        a = -(p2.y - p1.y)
        b = p2.x - p1.x
        c = -(a * p1.x + b * p1.y)

        A.append(a)
        B.append(b)
        C.append(c)

    D = []
    for i in range(len(A)):
        d = A[i] * point.x + B[i] * point.y + C[i]
        D.append(d)

    t1 = all(d >= 0 for d in D)
    t2 = all(d <= 0 for d in D)
    return t1 or t2

if __name__ == '__main__':
    polygon = [Point(0, 0), Point(5, 0), Point(6, 7), Point(2, 3), Point(0, 4)]
    p1 = Point(1, 1)
    print (check_polygon(polygon, p1)) # returns True
    p2 = Point(6, 2)
    print (check_polygon(polygon, p2)) # returns False

    
    