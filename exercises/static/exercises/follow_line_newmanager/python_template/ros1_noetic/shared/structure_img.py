from ctypes import Structure, c_int32, c_int64


class MD(Structure):
    _fields_ = [
        ('shape_0', c_int32),
        ('shape_1', c_int32),
        ('shape_2', c_int32),
        ('size', c_int64)
    ]

    def __str__(self):
        return f"---- MD -----\n" \
               f"  shape_0: {self.shape_0}\n" \
               f"  shape_1: {self.shape_1}\n" \
               f"  shape_2: {self.shape_2}\n" \
               f"  size: {self.size}\n" \
               f"-------------"
