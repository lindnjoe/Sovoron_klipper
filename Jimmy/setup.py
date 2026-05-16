from setuptools import setup
from Cython.Build import cythonize

# for local test
# import numpy as np
# include_dirs=[np.get_include()]
include_dirs=[]

setup(
    ext_modules=cythonize("flow_calculator.pyx"),
    include_dirs=include_dirs,
    zip_safe=False,
)
