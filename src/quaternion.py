import math

from .vector import Vector

class Quaternion:

    def __init__(self, coefficients : list):
        self.real = coefficients[0]
        self.ci = coefficients[1]
        self.cj = coefficients[2]
        self.ck = coefficients[3]

    @staticmethod
    def from_vector(vector : Vector):
        return Quaternion([0, vector.x, vector.y, vector.z])

    @staticmethod
    def create_rotation(rotation_axis : Vector, theta : float):
        n = rotation_axis.normalized()  # normalize rotation axis vector
        sin, cos = math.sin(theta/2), math.cos(theta/2)
        return Quaternion([cos, sin * n.x, sin * n.y, sin * n.z])  # Qr = cos(theta/2) + sin(theta/2)(n)
    
    @staticmethod
    def create_translation(translation : Vector):
        return Quaternion([0, translation.x / 2, translation.y / 2, translation.z / 2])  # Qt = 0.5 (0, t)

    @staticmethod
    def sum(q1, q2):
        real = q1.real + q2.real
        ci = q1.ci + q2.ci
        cj = q1.cj + q2.cj
        ck = q1.ck + q2.ck
        return Quaternion([real, ci, cj, ck])
    
    @staticmethod
    def difference(q1, q2):
        real = q1.real - q2.real
        ci = q1.ci - q2.ci
        cj = q1.cj - q2.cj
        ck = q1.ck - q2.ck
        return Quaternion([real, ci, cj, ck])
    
    @staticmethod
    def product(q1, q2):
        real = q1.real * q2.real - (q1.ci * q2.ci + q1.cj * q2.cj + q1.ck * q2.ck)
        ci = q1.real * q2.ci + q2.real * q1.ci + q1.cj * q2.ck - q1.ck * q2.cj
        cj = q1.real * q2.cj + q2.real * q1.cj + q1.ck * q2.ci - q1.ci * q2.ck
        ck = q1.real * q2.ck + q2.real * q1.ck + q1.ci * q2.cj - q1.cj * q2.ci
        result = Quaternion([real, ci, cj, ck])
        return result

    def get_conjugate(self):
        return Quaternion([self.real, -self.ci, - self.cj, -self.ck])
    
    def get_magnitude(self):
        return math.sqrt(self.real ** 2 + self.ci ** 2 + self.cj ** 2 + self.ck ** 2)
    
    def get_scaled(self, s):
        return Quaternion([s * self.real, s * self.ci, s * self.cj, s * self.ck])
    
    def as_vector(self):
        return Vector(self.ci, self.cj, self.ck)
    
    def as_list(self):
        return [self.real, self.ci, self.cj, self.ck]
    
    def __str__(self):
        return f"({self.real}, {self.ci}, {self.cj}, {self.ck})"
    
