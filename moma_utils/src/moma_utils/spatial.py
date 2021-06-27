import numpy as np
import scipy.spatial.transform
from moveit_commander.conversions import list_to_pose_stamped


class Rotation(scipy.spatial.transform.Rotation):
    @classmethod
    def identity(cls):
        return cls.from_quat([0.0, 0.0, 0.0, 1.0])


class Transform(object):
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = np.asarray(translation, np.double)

    @classmethod
    def from_matrix(cls, m):
        rotation = Rotation.from_dcm(m[:3, :3])
        translation = m[:3, 3]
        return cls(rotation, translation)

    @classmethod
    def from_list(cls, l):
        return cls(Rotation.from_quat(l[:4]), l[4:])

    def as_matrix(self):
        return np.vstack(
            (np.c_[self.rotation.as_dcm(), self.translation], [0.0, 0.0, 0.0, 1.0])
        )

    def to_list(self):
        return self.translation.tolist() + self.rotation.as_quat().tolist()

    def __mul__(self, other):
        rotation = self.rotation * other.rotation
        translation = self.rotation.apply(other.translation) + self.translation
        return self.__class__(rotation, translation)

    def inverse(self):
        rotation = self.rotation.inv()
        translation = -rotation.apply(self.translation)
        return self.__class__(rotation, translation)

    @classmethod
    def identity(cls):
        rotation = Rotation.from_quat([0.0, 0.0, 0.0, 1.0])
        translation = np.array([0.0, 0.0, 0.0])
        return cls(rotation, translation)

    @classmethod
    def translation(cls, translation):
        rotation = Rotation.identity()
        return cls(rotation, translation)

    @classmethod
    def rotation(cls, rotation):
        translation = np.zeros(3)
        return cls(rotation, translation)
