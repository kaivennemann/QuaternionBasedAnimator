import math

class Vector:

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    @staticmethod
    def distance(p1, p2):
        return Vector.difference(p1, p2).magnitude()
    
    @staticmethod
    def difference(p1, p2):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        return Vector(x, y, z)
    
    @staticmethod
    def sum(p1, p2):
        x = p1.x + p2.x
        y = p1.y + p2.y
        z = p1.z + p2.z
        return Vector(x, y, z)
    
    @staticmethod
    def dot(p1, p2):
        return (p1.x * p2.x) + (p1.y * p2.y) + (p1.z * p2.z)
    
    def as_list(self):
        return [self.x, self.y, self.z]

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def normalized(self):
        m = self.magnitude()
        return Vector(self.x / m, self.y / m, self.z / m)
    
    def distance_from(self, p_other):
        return Vector.difference(self, p_other).magnitude()
    
    def __str__(self):
        return f"Vector({self.x}, {self.y}, {self.z})"
    