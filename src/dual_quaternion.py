import math

from .vector import Vector
from .quaternion import Quaternion

class DualQuaternion:

    def __init__(self, q_real : Quaternion, q_dual : Quaternion):
        self.q_real = q_real
        self.q_dual = q_dual

    @staticmethod
    def from_components(q_real_components : list, q_dual_components : list):
        return DualQuaternion(Quaternion(q_real_components), Quaternion(q_dual_components))
    
    @staticmethod
    def from_transform(translation : Vector, rotation_axis : Vector, theta : float):
        q_real = Quaternion.create_rotation(rotation_axis, theta)  # Qr = r
        qt = Quaternion.create_translation(translation)
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
