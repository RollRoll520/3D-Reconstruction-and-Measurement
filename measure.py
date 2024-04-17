import open3d as o3d
import torch
import env
import measurement.contour as ct
import measurement.slice as slice

if env.REMOTE:
    torch.cuda.synchronize()

filename = env.reconstructed_file_env
point_cloud = o3d.io.read_point_cloud(filename)
max_height = ct.get_max_height(point_cloud)
height = 0
thickness = 0.05
radius = 0.2
volume = 0
if not env.REMOTE:
    vis = o3d.visualization.Visualizer()
    vis.create_window()

while height < max_height:
    _, _, _, reconstructed_intersection_points = slice.slice_ply(filename, height, thickness)
    
    contour, temp = ct.contour_compute(reconstructed_intersection_points, thickness, radius)
    volume += temp
    height += thickness

    if not env.REMOTE:
        # 创建Open3D点云对象
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points=o3d.utility.Vector3dVector(reconstructed_intersection_points)
        # 创建Open3D线集对象表示轮廓
        line =  o3d.geometry.PointCloud()
        line.points =o3d.utility.Vector3dVector(contour)
    
        vis.add_geometry(point_cloud)
        vis.add_geometry(line)
    
        vis.poll_events()
        vis.update_renderer()

if not env.REMOTE:
    vis.run()
    vis.destroy_window()

print(f"the volume of {filename} is {volume}")