from calib import calib
import env
from keypoint.harris import harris
from measurement import slice,interpolation


calib.calib(env.mkv1_file_env,env.mkv2_file_env)
# harris(env.reconstructed_file_env)
# slice.test_slice(env.reconstructed_file_env,1,0.1)

# interpolation.compute_interpolation(env.reconstructed_file_env, env.standard_ply_file_env, 1.6, 0.2)
