# CUDA_AVAILABLE = True
from .np_kernel import compute_normals, compute_normals_quad
from .np_kernel import inverse_map as cuda_inverse_map

# interp
# try: 
#     from .cuda_kernel import cuda_interp
# except:
CUDA_AVAILABLE = False
from .np_kernel import interp as cuda_interp