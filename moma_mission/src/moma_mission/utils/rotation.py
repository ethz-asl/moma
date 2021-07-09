import platform
from scipy.spatial.transform import Rotation


def is_python3():
    return platform.python_version()[0] == '3'


if is_python3():
    class CompatibleRotation(Rotation):
        def __init__(self, *args, **kwargs):
            Rotation.__init__(self, *args, **kwargs)

        def as_dcm(self):
            return self.as_matrix()

        @classmethod
        def from_dcm(cls, *args, **kwargs):
            return cls.from_matrix(*args, **kwargs)

else:
    class CompatibleRotation(Rotation):
        def __init__(self, *args, **kwargs):
            Rotation.__init__(self, *args, **kwargs)

        def as_matrix(self):
            return self.as_dcm()

        @classmethod
        def from_matrix(cls, *args, **kwargs):
            return cls.from_dcm(*args, **kwargs)

if __name__ == "__main__":
    import numpy as np

    r = CompatibleRotation.from_dcm(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))
    print(r.as_dcm())
