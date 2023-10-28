import math
import random

from .vector import Vector
from .quaternion import Quaternion
from .dual_quaternion import DualQuaternion

class TestDualQuaternion:

    def test_init(self):
        # Arrange
        translation = Vector(10, -20, 30)
        axis = Vector(-1, -1, -1)
        theta = 0

        # Assert
        q = DualQuaternion.from_transform(translation, axis, theta)

        # Act
        assert q.q_real.as_list() == Quaternion.create_rotation(axis, theta).as_list()
        assert q.get_translation_quaternion().as_list() == Quaternion.create_translation(translation).get_scaled(2).as_list()

    def test_apply_to_point_1(self):
        # Arrange
        a = random.random() * 10000 - 5000  # random values between -5000 and 4999
        b = random.random() * 10000 - 5000
        point = Vector(a, 0, 0)
        translation = Vector(0, 0, b)
        axis = Vector(1, 1, 1)
        theta = 2 * math.pi / 3

        # Act
        transform = DualQuaternion.from_transform(
            translation = translation,
            rotation_axis = axis,
            theta = theta
        )
        transformed_point = transform.apply_to_point(point)

        # Assert
        reference_point = Vector(0, a, b)
        assert Vector.distance(reference_point, transformed_point) < 0.000001
