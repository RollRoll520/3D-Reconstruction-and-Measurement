import open3d as o3d
import numpy as np
import torch

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


def slice_data_calculation(sliced_cloud_left, sliced_cloud_right, slice_thickness):
    sliced_points_left = np.asarray(sliced_cloud_left.points)
    sliced_points_right = np.asarray(sliced_cloud_right.points)
    sliced_indices_left = np.arange(len(sliced_points_left))
    sliced_indices_right = np.arange(len(sliced_points_right))

    validated_indices_left = []
    validated_indices_right = []

    while len(sliced_indices_left) > 0:
        p_left_index = sliced_indices_left[0]
        p_left = sliced_points_left[p_left_index]

        c = slice_thickness
        radius = 0
        matching_point_index = 0
        matching_point = None
        print(f"left to compute is: {len(sliced_indices_left)}")
        while matching_point_index == 0:
            # print("matching")
            radius += c
            distances = np.linalg.norm(sliced_points_right - p_left, axis=1)
            print(f"{radius}")
            indices_within_radius = np.where(distances <= radius)[0]
            if len(indices_within_radius) > 0:
                closest_index = indices_within_radius[np.argmin(distances[indices_within_radius])]
                closest_distance = np.min(distances[indices_within_radius])
                print(f"{closest_index}/{closest_distance}")
                # if closest_distance < np.min(np.linalg.norm(sliced_points_left[sliced_indices_left != p_left_index] - sliced_points_right[closest_index], axis=1)):
                if len(np.where(np.linalg.norm(sliced_points_left - sliced_points_right[closest_index], axis=1) <closest_distance)[0]) == 0:
                    matching_point_index = closest_index
                    validated_indices_left.append(p_left_index)
                    validated_indices_right.append(matching_point_index)                    

                # 删除验证的点对
                print("delete")
                if len(sliced_indices_left) > 0:
                    sliced_indices_left = np.delete(sliced_indices_left, [0])
                sliced_indices_right = np.delete(sliced_indices_right, np.where(sliced_indices_right == closest_index))

                # sliced_points_left = np.delete(sliced_points_left, 0, axis=0)
                sliced_points_right = np.delete(sliced_points_right, np.where(sliced_indices_right == closest_index), axis=0)
                break
            else :
                print("delete")
                if len(sliced_indices_left) > 0:
                    sliced_indices_left = np.delete(sliced_indices_left, [0])
                # sliced_points_left = np.delete(sliced_points_left, 0, axis=0)
                break
    
    validated_points_left = sliced_points_left[validated_indices_left]
    validated_points_right = sliced_points_right[validated_indices_right]

    return validated_points_left, validated_points_right

def connect_and_intersect_points(validated_points_left, validated_points_right, slice_height):
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
    plane_origin = np.array([0, 0, slice_height])  # 切片平面的原点，设置为指定的切片高度
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



def slice_ply(filename,slice_height,slice_thickness):

    # 读取PLY文件
    point_cloud = o3d.io.read_point_cloud(filename)

    # 进行点云分层切片
    sliced_cloud_left, sliced_cloud_right = slice_point_cloud(point_cloud, slice_height,slice_thickness)

    # 进行切片数据计算
    validated_points_left, validated_points_right = slice_data_calculation(sliced_cloud_left, sliced_cloud_right,slice_thickness)

    # 连接切片点并求解交点
    merged_lines, intersection_points=connect_and_intersect_points(validated_points_left, validated_points_right,slice_height)

    return validated_points_left,validated_points_right,merged_lines,intersection_points


# #接口样例
# # torch.cuda.synchronize()
# validated_points_left,validated_points_right,merged_lines,intersection_points = slice_ply(r'D:\Knowledge\Graduation_Design\WorkSpace\Py_WorkPlace\reconstruct\fr1_cam\mesh.ply',1.0,0.2)

# # 可视化切片后的点云
# intersection_cloud = o3d.geometry.PointCloud()
# validated_cloud_left = o3d.geometry.PointCloud()
# validated_cloud_right = o3d.geometry.PointCloud()
# intersection_cloud.points = o3d.utility.Vector3dVector(intersection_points)
# validated_cloud_left.points = o3d.utility.Vector3dVector(validated_points_left)
# validated_cloud_right.points = o3d.utility.Vector3dVector(validated_points_right)

# intersection_cloud_size = len(intersection_points)
# left_cloud_size = len(validated_points_left)
# right_cloud_size = len(validated_points_right)

# print("Intersection Cloud size:", intersection_cloud_size)
# print("Left Cloud size:", left_cloud_size)
# print("Right Cloud size:", right_cloud_size)

# spheres = o3d.geometry.TriangleMesh()
# for point in intersection_cloud.points:
#     sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
#     sphere.translate(point)
#     spheres += sphere
# validated_cloud_left.paint_uniform_color([1, 1, 1]) 
# validated_cloud_right.paint_uniform_color([1, 1, 1]) 
# intersection_cloud.paint_uniform_color([0, 0, 0])  
# merged_lines.paint_uniform_color([0,0.5,0])
# # o3d.visualization.draw_geometries([spheres,merged_lines,validated_cloud_left,validated_cloud_right])
# o3d.visualization.draw_geometries([spheres])
