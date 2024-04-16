import open3d as o3d
import numpy as np

def slice_point_cloud(point_cloud,slice_height, slice_thickness):
    # 获取点云的最小边界框
    aabb = point_cloud.get_axis_aligned_bounding_box()
    min_bound = aabb.get_min_bound()
    max_bound = aabb.get_max_bound()
    min_point = np.mean(min_bound, axis=0)
    max_point = np.mean(max_bound, axis=0)

    points = np.asarray(point_cloud.points)
    sliced_cloud_left = o3d.geometry.PointCloud()
    sliced_cloud_right = o3d.geometry.PointCloud()


    # 计算切片平面的左右平移距离
    delta = slice_thickness / 2.0
    points_left = (points[:, 2] >= slice_height-delta) & (points[:, 2] < slice_height)
    points_right = (points[:, 2] >= slice_height) & (points[:, 2] < slice_height+delta)
    sliced_cloud_left.points = o3d.utility.Vector3dVector(points[points_left])
    sliced_cloud_right.points = o3d.utility.Vector3dVector(points[points_right])

    return sliced_cloud_left, sliced_cloud_right

def find_nearest_point(point, point_cloud):
    distances = np.linalg.norm(np.asarray(point_cloud.points) - point, axis=1)
    nearest_index = np.argmin(distances)
    nearest_point = point_cloud.points[nearest_index]
    return nearest_point

def find_matching_point(p_left, p_right, point_cloud):
    left_indices = np.where(np.all(np.isclose(np.asarray(point_cloud.points), p_left), axis=1))[0]
    right_points = np.asarray(point_cloud.points)[left_indices]
    distances = np.linalg.norm(right_points - p_right, axis=1)
    if len(distances) == 0:
        return None, None
    matching_index = np.argmin(distances)
    matching_point_index = left_indices[matching_index]
    matching_point = point_cloud.points[matching_point_index]
    return matching_point_index, matching_point

def slice_data_calculation(sliced_cloud_left, sliced_cloud_right,slice_thickness):
    sliced_points_left = np.asarray(sliced_cloud_left.points)
    sliced_points_right = np.asarray(sliced_cloud_right.points)
    sliced_indices_left = np.arange(len(sliced_points_left))
    sliced_indices_right = np.arange(len(sliced_points_right))

    validated_indices_left = []
    validated_indices_right = []

    while len(sliced_indices_left) > 0:
        p_left_index = sliced_indices_left[0]
        p_left = sliced_points_left[p_left_index]
        p_right = find_nearest_point(p_left, sliced_cloud_right)

        c = slice_thickness
        radius = c
        matching_point_index = None
        matching_point = None

        while matching_point_index is None:
            indices_within_radius = np.where(np.linalg.norm(sliced_points_right - p_right, axis=1) <= radius)[0]
            if len(indices_within_radius) > 0:
                matching_point_index, matching_point = find_matching_point(p_left, p_right, sliced_cloud_right)
            else:
                radius += c

        if matching_point_index is not None:
            validated_indices_left.append(p_left_index)
            validated_indices_right.append(matching_point_index)

            # 删除验证的点对
            sliced_indices_left = np.delete(sliced_indices_left, 0)
            sliced_indices_right = np.delete(sliced_indices_right, np.where(sliced_indices_right == matching_point_index))

    validated_points_left = sliced_points_left[validated_indices_left]
    validated_points_right = sliced_points_right[validated_indices_right]

    return validated_points_left, validated_points_right

def connect_and_intersect_points(validated_points_left, validated_points_right, slice_thickness):
    # 创建连接线段
    lines = []
    for i in range(len(validated_points_left)):
        p_left = validated_points_left[i]
        p_right = validated_points_right[i]
        line = o3d.geometry.LineSet.create_from_point_cloud_correspondences(
            o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector([p_left])),
            o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector([p_right])),
            [[0, 0]]
        )
        lines.append(line)

    # 将连接线段合并为一个LineSet
    merged_lines = o3d.geometry.LineSet()
    for line in lines:
        merged_lines += line

    # 求解交点
    plane_normal = np.array([0, 0, 1])  # 切片平面的法向量，这里假设为z轴正方向
    plane_origin = np.array([0, 0, 0])  # 切片平面的原点，可根据具体需求进行设置
    intersection_points = []

    for i in range(len(validated_points_left)):
        p_left = validated_points_left[i]
        p_right = validated_points_right[i]

        # 构建直线方程
        line_direction = p_right - p_left
        line_origin = p_left

        # 求解直线与平面的交点
        t = np.dot(plane_normal, plane_origin - line_origin) / np.dot(plane_normal, line_direction)
        intersection_point = line_origin + t * line_direction
        intersection_points.append(intersection_point)

    return merged_lines, intersection_points

# 读取PLY文件
point_cloud = o3d.io.read_point_cloud(r'D:\Knowledge\Graduation_Design\WorkSpace\Py_WorkPlace\reconstruct\fr1_cam\mesh.ply')

# 设置切片厚高度、厚度、阈值
slice_height = 1.0
slice_thickness = 0.2  # 切片厚度

# 进行点云分层切片
sliced_cloud_left, sliced_cloud_right = slice_point_cloud(point_cloud, slice_height,slice_thickness)

# 进行切片数据计算
validated_points_left, validated_points_right = slice_data_calculation(sliced_cloud_left, sliced_cloud_right,slice_thickness)

# 连接切片点并求解交点
merged_lines, intersection_points=connect_and_intersect_points(validated_points_left, validated_points_right,slice_thickness)

# 可视化切片后的点云
intersection_cloud = o3d.geometry.PointCloud()
intersection_cloud.points = o3d.utility.Vector3dVector(intersection_points)
sliced_cloud_left.points = o3d.utility.Vector3dVector(validated_points_left)
sliced_cloud_right.points = o3d.utility.Vector3dVector(validated_points_right)

intersection_cloud_size = len(intersection_points)
left_cloud_size = len(validated_points_left)
right_cloud_size = len(validated_points_right)

print("Intersection Cloud size:", intersection_cloud_size)
print("Left Cloud size:", left_cloud_size)
print("Right Cloud size:", right_cloud_size)

sliced_cloud_left.paint_uniform_color([0, 1, 0]) 
sliced_cloud_right.paint_uniform_color([0, 0, 1]) 
intersection_cloud.paint_uniform_color([1, 0, 0])  
o3d.visualization.draw_geometries([sliced_cloud_left, sliced_cloud_right])
# o3d.visualization.draw_geometries([sliced_cloud_left, sliced_cloud_right, intersection_cloud])