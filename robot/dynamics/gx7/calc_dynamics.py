import numpy as np
import ctypes
import os

abspath = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.join(abspath, "libregressor.so")


class CalcDynamics:
    def __init__(self, lib_path=lib_path):
        self.lib = ctypes.CDLL(lib_path)
        self.base_idxs = [
            5,
            11,
            12,
            13,
            14,
            15,
            16,
            18,
            19,
            20,
            21,
            25,
            26,
            27,
            28,
            29,
            30,
            32,
            33,
            34,
            35,
            38,
            39,
            40,
            41,
            42,
            43,
            44,
            46,
            47,
            48,
            49,
            52,
            53,
            54,
            55,
            56,
            57,
            58,
            60,
            61,
            62,
            63,
            66,
            67,
            68,
            69,
            70,
            71,
            72,
            74,
            75,
            76,
            77,
            80,
            81,
            82,
            83,
            84,
            85,
            86,
            88,
            89,
            90,
            91,
            94,
            95,
            96,
            97,
        ]

        # Define the function signature
        # void H_func(double* regressor, const double* q, const double* dq, const double* ddq)
        self.lib.H_func.argtypes = [
            ctypes.POINTER(ctypes.c_double),  # regressor array
            ctypes.POINTER(ctypes.c_double),  # q array
            ctypes.POINTER(ctypes.c_double),  # dq array
            ctypes.POINTER(ctypes.c_double),  # ddq array
        ]
        self.lib.H_func.restype = None

    def calc(self, q, dq, ddq):
        q = np.array(q, dtype=np.float64)
        dq = np.array(dq, dtype=np.float64)
        ddq = np.array(ddq, dtype=np.float64)

        regressor = np.zeros(686, dtype=np.float64)

        # Get pointers to the arrays
        q_ptr = q.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        dq_ptr = dq.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        ddq_ptr = ddq.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        regressor_ptr = regressor.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

        # Call the C function
        self.lib.H_func(regressor_ptr, q_ptr, dq_ptr, ddq_ptr)
        regressor = regressor.reshape(len(q), -1)[:, self.base_idxs]

        return regressor


if __name__ == "__main__":

    calc = CalcDynamics("./libregressor.so")

    regressor = calc.calc(np.zeros(7), np.zeros(7), np.zeros(7))

    print("regressor shape:", regressor.shape)
