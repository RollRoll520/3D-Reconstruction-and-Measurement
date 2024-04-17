from matplotlib import pyplot as plt
import open3d as o3d
import numpy as np
import measurement.slice as slice

# 将切片平面内的点转换为极坐标
def cartesian_to_polar(points):
    polar_points = []
    for point in points:
        x, y, _ = point
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        polar_points.append((r, theta))
    return np.array(polar_points)

# 计算两个极坐标点之间的距离
def polar_distance(point1, point2):
    r1, theta1 = point1
    r2, theta2 = point2
    dr = r2 - r1
    dtheta = np.abs(theta2 - theta1)
    return np.sqrt(dr**2 + (r1*dtheta)**2)

# 计算重建点云切片中每一个点到标准模型点云切片中最近点的距离
def compute_distances(reconstructed_points_polar, standard_points_polar):
    distances = []
    for point in reconstructed_points_polar:
        distance = np.min([polar_distance(point, sp) for sp in standard_points_polar])
        distances.append(distance)
    return distances


def compute_interpolation(filename, standard_filename, slice_height, slice_thickness):

    # 切片点云获取
    _, _, _, reconstructed_intersection_points = slice.slice_ply(filename, slice_height, slice_thickness)
    _, _, _, standard_intersection_points = slice.slice_ply(standard_filename, slice_height+0.1, slice_thickness)

    # 将点云转换为笛卡尔坐标系
    reconstructed_points_cartesian = np.asarray(reconstructed_intersection_points)
    standard_points_cartesian = np.asarray(standard_intersection_points)

    # 将笛卡尔坐标系转换为极坐标系
    reconstructed_points_polar = cartesian_to_polar(reconstructed_points_cartesian)
    standard_points_polar = cartesian_to_polar(standard_points_cartesian)

    # 计算重建点云切片中每一个点到标准模型切片点云最近点的距离
    distances = compute_distances(reconstructed_points_polar, standard_points_polar)

    # 使用灰度色彩映射将距离值转换为RGB颜色
    colormap = plt.cm.get_cmap("jet")  # 使用jet颜色映射，可根据需要修改
    colors = colormap(distances)[:, :3]  # 取RGB三个通道

    # 创建重建点云对象
    reconstructed_interp_cloud = o3d.geometry.PointCloud()
    reconstructed_interp_cloud.points = o3d.utility.Vector3dVector(reconstructed_points_cartesian)
    print(f"reconstructed count: {len(reconstructed_intersection_points)}")
    print(f"standard count: {len(standard_intersection_points)}")
    print(f"last count: {len(reconstructed_interp_cloud.points)}")
    reconstructed_interp_cloud.colors = o3d.utility.Vector3dVector(colors)

    # 可视化重建点云切片
    o3d.visualization.draw_geometries([reconstructed_interp_cloud])

#调用代码
# reconstruction_ply = reconstructed_file_env
# standard_ply =standard_ply_file_env
# compute_interpolation(reconstruction_ply, standard_ply, 1.0, 0.2)