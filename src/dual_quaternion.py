import math

class DualQuaternion:

    def __init__(self, q_real, q_dual):
        self.q_real = q_real
        self.q_dual = q_dual

    @staticmethod
    def from_components(q_real_components, q_dual_components):
        return DualQuaternion(Quaternion(q_real_components), Quaternion(q_dual_components))
    
    @staticmethod
    def from_transform(translation_vector, rotation_axis_vector, theta):
        q_real = Quaternion.create_rotation(rotation_axis_vector, theta)  # Qr = r
        qt = Quaternion.create_translation(translation_vector)
        q_dual = Quaternion.product(qt, q_real)  # Qd = 0.5 (0, t) r
        return DualQuaternion(q_real, q_dual)
    
    @staticmethod
    def from_vector(vector):
        return DualQuaternion.from_components([1, 0, 0, 0], [0, vector.x, vector.y, vector.z])
    
    @staticmethod
    def difference(q1, q2):
        q_real = Quaternion.difference(q1.q_real, q2.q_real)
        q_dual = Quaternion.difference(q1.q_dual, q2.q_dual)
        return DualQuaternion(q_real, q_dual)
    
    @staticmethod
    def product(q1, q2):
        q_real = Quaternion.product(q1.q_real, q2.q_real)
        product1 = Quaternion.product(q1.q_real, q2.q_dual)
        product2 = Quaternion.product(q1.q_dual, q2.q_real)
        q_dual = Quaternion.sum(product1, product2)
        return DualQuaternion(q_real, q_dual)
    
    @staticmethod
    def sum(q1, q2):
        q_real = Quaternion.sum(q1.q_real, q2.q_real)
        q_dual = Quaternion.sum(q1.q_dual, q2.q_dual)
        return DualQuaternion(q_real, q_dual)

    def get_conjugate(self):
        return DualQuaternion(self.q_real.get_conjugate(), self.q_dual.get_conjugate().get_scaled(-1))

    def get_scaled(self, s):
        q_real = self.q_real.get_scaled(s)
        q_dual = self.q_dual.get_scaled(s)
        return DualQuaternion(q_real, q_dual)
    
    def get_magnitude(self):
        prod = DualQuaternion.product(self, self.get_conjugate())
        return math.sqrt(prod.q_real.real)
    
    def get_rotation_quaternion(self):
        return self.q_real
    
    def get_rotation_dual_quaternion(self):
        return DualQuaternion(self.q_real, Quaternion([0, 0, 0, 0]))
    
    def get_translation_quaternion(self):
        return Quaternion.product(self.q_dual, self.q_real.get_conjugate()).get_scaled(2)  # t = 2 Qd Qr*
    
    def get_translation_dual_quaternion(self):
        return DualQuaternion(Quaternion([1, 0, 0, 0]), self.get_translation_quaternion())
    
    def apply_to_point(self, point):

        # Convert p to DualQuaternion, i.e. 1 + ɛp
        p = DualQuaternion.from_vector(point)  

        # Apply formula: 1 + ɛp' = q * p * q_conjugate
        qp = DualQuaternion.product(self, p)
        result = DualQuaternion.product(qp, self.get_conjugate())
        
        # Extract new point p' from 1 + ɛp'
        return result.q_dual.as_vector()


    def __str__(self):
        return f"{self.q_real} + {self.q_dual}ɛ"



class Quaternion:

    def __init__(self, coefficients):
        self.real = coefficients[0]
        self.ci = coefficients[1]
        self.cj = coefficients[2]
        self.ck = coefficients[3]

    @staticmethod
    def from_vector(vector):
        return Quaternion([0, vector.x, vector.y, vector.z])

    @staticmethod
    def create_rotation(rotation_axis_vector, theta):
        magnitude = math.sqrt(sum([xi * xi for xi in rotation_axis_vector]))
        nx, ny, nz = [xi / magnitude for xi in rotation_axis_vector]  # normalize rotation axis vector
        sin, cos = math.sin(theta/2), math.cos(theta/2)
        return Quaternion([cos, sin * nx, sin * ny, sin * nz])
    
    @staticmethod
    def create_translation(translation_vector):
        tx, ty, tz = translation_vector
        return Quaternion([0, tx/2, ty/2, tz/2])  # Qt = 0.5 (0, t)

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
    
    def __str__(self):
        return f"({self.real}, {self.ci}, {self.cj}, {self.ck})"
    

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
    
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def distance_from(self, p_other):
        return Vector.difference(self, p_other).magnitude()
    