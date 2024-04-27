from matplotlib import pyplot as plt
import open3d as o3d
import numpy as np
import measurement.slice as slice
import open3d.visualization as vis

# 将切片平面内的点转换为极坐标
def cartesian_to_polar(points):
    polar_points = []
    for point in points:
        x, y, _ = point
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        polar_points.append((r, theta))
    return np.array(polar_points)

# 将插值后的点转换回笛卡尔坐标系
def polar_to_cartesian(polar_points,slice_height):
    cartesian_points = []
    for point in polar_points:
        r, theta = point
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        cartesian_points.append((x, y, slice_height))
    return np.array(cartesian_points)

# 使用牛顿插值方法进行插值
def newton_interpolation(x, y, x_interp):
    coeffs = np.zeros_like(x)
    i = 0
    # 计算差商
    while i < len(x):
        j = len(x)-1
        while j > i:
            if i == 0:
                coeffs[j]=(y[j]-y[j-1])/(x[j]-x[j-1])
            else:
                    coeffs[j] = (coeffs[j]-coeffs[j-1])/(x[j]-x[j-1-i])
            j-=1
        i+=1 

    # 插值计算
    interp_vals = np.zeros_like(x_interp)
    for i in range(len(x_interp)):
        interp_vals[i] = coeffs[0]
        for j in range(1, len(coeffs)):
            interp_vals[i] = interp_vals[i] * (x_interp[i] - x[j-1]) + coeffs[j]
        print(f"{x_interp[i]},{interp_vals[i]}")

    return interp_vals

# 计算两个极坐标点之间的距离
def polar_distance(point1, point2):
    r1, theta1 = point1
    r2, theta2 = point2
    dr = r2 - r1
    dtheta = np.abs(theta2 - theta1)
    if np.abs(dtheta) < 1e-6:  # 处理接近零的情况
        dtheta = 1e-6
    return np.sqrt(dr**2 + np.power(r1*dtheta, 2, dtype=np.float64))

# 计算重建点云切片中每一个点到标准模型点云切片中最近点的距离
def compute_distances(reconstructed_points_polar, standard_points_polar):
    distances = []
    for point in reconstructed_points_polar:
        distances_to_sp = [polar_distance(point, sp) for sp in standard_points_polar]
        if len(distances_to_sp) > 0:  # 处理空列表的情况
            distance = np.min(distances_to_sp)
            distances.append(distance)
    return distances

def compute_interpolation(filename, standard_filename, slice_height, slice_thickness):

    # 切片点云获取
    _, _, _, reconstructed_intersection_points = slice.slice_ply(filename, slice_height, slice_thickness)
    _, _, _, standard_intersection_points = slice.slice_ply(standard_filename, slice_height+0.1, slice_thickness)

    # 将点云转换为笛卡尔坐标系
    reconstructed_points_cartesian = np.asarray(reconstructed_intersection_points)
    standard_points_cartesian = np.asarray(standard_intersection_points)

        # 计算 x、y、z 的最小值
    min_x = np.min(reconstructed_points_cartesian[:, 0])
    min_y = np.min(reconstructed_points_cartesian[:, 1])
    min_z = np.min(reconstructed_points_cartesian[:, 2])

    # 平移操作，将最小值作为原点
    reconstructed_points_cartesian[:, 0] -= min_x
    reconstructed_points_cartesian[:, 1] -= min_y
    reconstructed_points_cartesian[:, 2] -= min_z
    standard_points_cartesian[:, 0] -= min_x
    standard_points_cartesian[:, 1] -= min_y
    standard_points_cartesian[:, 2] -= min_z

    #计算笛卡尔插值
    x_interp = np.linspace(np.min(reconstructed_points_cartesian[:, 0]), np.max(reconstructed_points_cartesian[:, 0]), num=50)
    y_interp = newton_interpolation(reconstructed_points_cartesian[:, 0], reconstructed_points_cartesian[:, 1], x_interp)
    reconstructed_interp_cartesian_points = np.column_stack((x_interp, y_interp, np.full_like(x_interp, slice_height-min_z)))

    # 将笛卡尔坐标系转换为极坐标系
    reconstructed_points_polar = cartesian_to_polar(reconstructed_points_cartesian)
    standard_points_polar = cartesian_to_polar(standard_points_cartesian)

    #计算极坐标插值
    x_interp = np.linspace(np.min(reconstructed_points_polar[:, 0]), np.max(reconstructed_points_polar[:, 0]), num=50)
    y_interp = newton_interpolation(reconstructed_points_polar[:, 0], reconstructed_points_polar[:, 1], x_interp)
    reconstructed_interp_points = np.column_stack((x_interp, y_interp))
    # 在原来的点云上进行插值
    reconstructed_combine_points = np.vstack((reconstructed_points_polar, reconstructed_interp_points))
    reconstructed_points_combined_cartesian = polar_to_cartesian(reconstructed_combine_points,slice_height-min_z)

    print(f"reconstructed count: {len(reconstructed_intersection_points)}")
    print(f"standard count: {len(standard_intersection_points)}")
    print(f"interp count: {len(reconstructed_interp_points)}")
    print(f"combine count: {len(reconstructed_combine_points)}")
    # 计算重建点云切片中每一个点到标准模型切片点云最近点的距离
    distances = compute_distances(reconstructed_combine_points, standard_points_polar)

    # 使用灰度色彩映射将距离值转换为RGB颜色
    colormap = plt.cm.get_cmap("jet")  # 使用jet颜色映射，可根据需要修改
    colors = colormap(distances)[:, :3]  # 取RGB三个通道



    # 创建重建点云对象
    reconstructed_combine_cloud = o3d.geometry.PointCloud()
    reconstructed_combine_cloud.points = o3d.utility.Vector3dVector(reconstructed_points_combined_cartesian)
    reconstructed_combine_cloud.colors = o3d.utility.Vector3dVector(colors)

    # 可视化重建点云切片
    # o3d.visualization.draw_geometries([reconstructed_combine_cloud])

    # 测试插值代码
    # 测试时需将上面的可视化代码注释！
    origin_point_cloud = o3d.geometry.PointCloud()
    interp_point_cloud = o3d.geometry.PointCloud()
    origin_point_cloud.points = o3d.utility.Vector3dVector(reconstructed_points_cartesian)
    interp_point_cartesian = polar_to_cartesian(reconstructed_interp_points,slice_height-min_z)
    interp_point_cloud.points = o3d.utility.Vector3dVector(interp_point_cartesian)
    # interp_point_cloud.points = o3d.utility.Vector3dVector(reconstructed_interp_cartesian_points)
    origin_point_cloud.paint_uniform_color([0.0, 0.0, 1.0])  # 自定义插值点的颜色
    interp_point_cloud.paint_uniform_color([1.0, 0.0, 0.0])

    # for point in origin_point_cloud.points:
    #     print(point)
    # for point in interp_point_cloud.points:
    #     print(point)

    visualizer = vis.Visualizer()
    visualizer.create_window()
    visualizer.add_geometry(interp_point_cloud)
    visualizer.add_geometry(origin_point_cloud)

    # 设置点的大小（半径）
    render_option = visualizer.get_render_option()
    render_option.point_size = 5.0
    # 显示可视化结果
    visualizer.run()
    visualizer.destroy_window()

#调用代码
# reconstruction_ply = reconstructed_file_env
# standard_ply =standard_ply_file_env
# compute_interpolation(reconstruction_ply, standard_ply, 1.0, 0.2)