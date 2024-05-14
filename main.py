from calib import calib
from calib.calib_k4a import merge_data
import env
from keypoint.harris import harris
from measurement import slice,interpolation


calib.calib(env.mkv1_file_env,env.mkv2_file_env,env.concrete_path,False,True)
merge_data(env.concrete_path)
# harris(env.reconstructed_file_env)
# harris(r"D:\Knowledge\Graduation_Design\WorkSpace\Resources\ply\input.ply")
# slice.test_slice(r"D:\Knowledge\Graduation_Design\WorkSpace\Resources\ply\input.ply",0.1)

# interpolation.compute_interpolation(env.reconstructed_file_env, env.standard_ply_file_env, 1.6, 0.2)
