import numpy as np
import ctypes
from ctypes import c_double, POINTER
import os
import subprocess
from pathlib import Path

def compile_dynamics_library(c_source_path, output_name='dynamics'):
    """Compile the C source code into a shared library"""
    # Determine the output path based on the system
    if os.name == 'nt':  # Windows
        lib_path = f'{output_name}.dll'
    elif os.name == 'posix':  # Linux/Mac
        lib_path = f'lib{output_name}.so'
    else:
        raise OSError("Unsupported operating system")
    
    # Compile command
    compile_cmd = [
        'gcc',
        '-shared', '-fPIC',
        '-o', lib_path,
        c_source_path,
        '-lm',  # Link math library
    ]
    
    # Run the compilation
    try:
        subprocess.run(compile_cmd, check=True)
        print(f"Successfully compiled {lib_path}")
        return lib_path
    except subprocess.CalledProcessError as e:
        print(f"Compilation failed: {e}")
        return None
    except FileNotFoundError:
        print("gcc compiler not found. Please install gcc.")
        return None

def load_dynamics_library(lib_path):
    """Load the compiled dynamics library"""
    try:
        lib = ctypes.CDLL(lib_path)
        print(f"Successfully loaded {lib_path}")
        return lib
    except Exception as e:
        print(f"Failed to load library: {e}")
        return None

def setup_dynamics_function(lib):
    """Set up the H_func function from the library"""
    # Define the argument and return types
    lib.H_func.argtypes = [
        POINTER(c_double),  # regressor (output)
        POINTER(c_double),  # q
        POINTER(c_double),  # dq
        POINTER(c_double),  # ddq
    ]
    lib.H_func.restype = None  # void function
    
    def h_func_py(q, dq, ddq):
        """Python wrapper for H_func"""
        # Convert inputs to contiguous arrays if they aren't already
        q_arr = np.ascontiguousarray(q, dtype=np.float64)
        dq_arr = np.ascontiguousarray(dq, dtype=np.float64)
        ddq_arr = np.ascontiguousarray(ddq, dtype=np.float64)
        
        # Fixed size of the regressor output based on the C code
        # The C code shows indices up to 503, so we need 504 elements
        regressor_size = 504
        regressor = np.zeros(regressor_size, dtype=np.float64)
        
        # Get pointers to the numpy arrays
        regressor_ptr = regressor.ctypes.data_as(POINTER(c_double))
        q_ptr = q_arr.ctypes.data_as(POINTER(c_double))
        dq_ptr = dq_arr.ctypes.data_as(POINTER(c_double))
        ddq_ptr = ddq_arr.ctypes.data_as(POINTER(c_double))
        
        # Call the C function
        lib.H_func(regressor_ptr, q_ptr, dq_ptr, ddq_ptr)
        
        return regressor
    
    return h_func_py

class Dynamics:
    def __init__(self):
        abs_path = os.path.abspath(__file__)
        lib_path = os.path.join(
                    os.path.dirname(abs_path), f"libdynamics.so"
                )
        
        lib = ctypes.CDLL(lib_path)
        
        self.h_func = setup_dynamics_function(lib)
        
        self.beta = np.load(os.path.join(os.path.dirname(abs_path), 'beta_ols.npy'))
        
        self.base_idxs = np.array([5, 11, 12, 13, 14, 15, 16, 18, 19, 20, 
        21, 25, 26, 27, 28, 29, 30, 32, 33, 34, 
        35, 38, 39, 40, 41, 42, 43, 44, 46, 47, 
        48, 49, 52, 53, 54, 55, 56, 57, 58, 60, 
        61, 62, 63, 66, 67, 68, 69, 70, 71, 72, 
        74, 75, 76, 77, 80, 81, 82, 83])
        
    def calc(self, q, dq, ddq):
        regressor =  self.h_func(q, dq, ddq)
        
        regressor = regressor.reshape(6, -1)[:, self.base_idxs]
        
        tau = regressor @ self.beta
        
        return tau
        


# Example usage
if __name__ == "__main__":
    import ctypes
    # Path to your C source file
    c_source_file = "gx7_dyna_regreessor.c"
    
    # 1. Compile the library
    lib_path = compile_dynamics_library(c_source_file)
    if lib_path is None:
        exit(1)
    
    # 2. Load the compiled library
    abs_path = os.path.abspath(__file__)
    lib_path = os.path.join(
                os.path.dirname(abs_path), f"libdynamics.so"
            )
    
    # 3. Set up the Python-callable function
    try:
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f"Library file {lib_path} not found")
        lib = ctypes.CDLL(lib_path)
        print(f"Successfully loaded {lib_path}")
    
        # Get the Python wrapper function
        h_func = setup_dynamics_function(lib)
    
        # 4. Test the function with some example data
        n = 6  # GX7 has 6 axes
        q = np.random.rand(n)
        dq = np.random.rand(n)
        ddq = np.random.rand(n)
    
        print("Input q:", q)
        print("Input dq:", dq)
        print("Input ddq:", ddq)
    
        # Call the Python wrapper function
        regressor = h_func(q, dq, ddq)
    
        print("Output regressor shape:", regressor.shape)
        print("Output regressor sample:", regressor)
    except FileNotFoundError as e:
        print(f"File error: {e}")
        exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        exit(1)
        
        print("Output regressor:", regressor)
        
    except Exception as e:
        print(f"Error: {e}")