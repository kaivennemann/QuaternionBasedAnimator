import math
import random
from .dual_quaternion import DualQuaternion, Quaternion, Vector

class TestDualQuaternion:

    def test_init(self):
        pass

    def test_apply_to_point_1(self):
        # Arrange
        a = random.random() * 10000 - 5000  # random values between -5000 and 4999
        b = random.random() * 10000 - 5000
        point = Vector(a, 0, 0)
        translation = [0, 0, b]
        axis = [1, 1, 1]
        theta = 2 * math.pi / 3

        # Act
        transform = DualQuaternion.from_transform(
            translation_vector = translation,
            rotation_axis_vector = axis,
            theta = theta
        )
        transformed_point = transform.apply_to_point(point)

        # Assert
        reference_point = Vector(0, a, b)
        assert Vector.distance(reference_point, transformed_point) < 0.000001
        
