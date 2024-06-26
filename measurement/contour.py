import numpy as np
import open3d as o3d


# 距离计算
def calculate_distance(point1, point2):
    return np.sqrt(np.sum((point1 - point2) ** 2))


# 双向最近点搜索法进行轮廓提取
def extract_contour(points, radius):
    contour = []
    starting_point = points[0]  # 假设起点为第一个点
    current_point = starting_point
    max_iterations = len(points)  # 设置最大迭代次数，避免死循环
    iterations = 0  # 迭代计数器

    while iterations < max_iterations:
        min_distance = float('inf')
        nearest_point = None

        # 在给定半径内搜索最近点
        for point in points:
            if not any(np.array_equal(point, c) for c in contour):  # 使用np.array_equal进行比较
                distance = calculate_distance(current_point, point)
                if distance < min_distance:
                    min_distance = distance
                    nearest_point = point

        if np.array_equal(nearest_point, starting_point):
            break

        new_starting_point = None
        min_distance_to_start = float('inf')

        # 判断是否将该点作为新的起点
        for point in points:
            if not any(np.array_equal(point, c) for c in contour):  # 使用np.array_equal进行比较
                distance = calculate_distance(starting_point, point)
                if distance < min_distance_to_start:
                    min_distance_to_start = distance
                    new_starting_point = point

        if min_distance_to_start < min_distance:
            contour.append(starting_point)
            starting_point = new_starting_point
            current_point = starting_point
        else:
            contour.append(nearest_point)
            current_point = nearest_point

    contour.append(starting_point)  # 连接起点和终点形成闭合轮廓
    return contour

# 计算多边形面积
def calculate_polygon_area(points):
    n = len(points)
    area = 0.0
    for i in range(n):
        x1, y1,_ = points[i]
        x2, y2,_ = points[(i + 1) % n]
        area += (x1 * y2 - x2 * y1)
    return abs(area) / 2.0

# 计算点云模型体积
def calculate_volume(contours, thickness):
    volume = 0.0
    for contour in contours:
        area = calculate_polygon_area(contour)
        volume += area * thickness
    return volume

# 创建Open3D点云对象
def create_point_cloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

# 创建Open3D线集对象表示轮廓
def create_line_set(contour):
    lines = [[i, (i + 1) % len(contour)] for i in range(len(contour))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(contour)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    return line_set

def contour_compute(points,slice_thickness=0.1,compute_radius=1):
    # 提取点云轮廓
    contour = extract_contour(points, compute_radius)

    # 计算点云体积
    volume = calculate_volume([contour], slice_thickness)

    return contour,volume

def get_max_height(points):
    np_points = np.asarray(points.points)  # Convert Open3D PointCloud to NumPy array
    max_height = np.max(np_points[:, 2])  # Get the maximum height from the Z coordinate
    return max_height
