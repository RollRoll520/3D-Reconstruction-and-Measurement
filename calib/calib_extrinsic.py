from ast import IsNot
import json
import math
from typing import List
import cv2
import numpy as np
import open3d as o3d
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation
from cv2 import aruco
import os

from sympy import true

#外参类型
class CameraExtrinsics:
    def __init__(self):
        self.translation = np.zeros(3)
        self.rotation = np.zeros(4)

#内参类型
class CameraIntrinsics:
    def __init__(self):
        self.Width = 0
        self.Height = 0
        self.cx = 0.0
        self.cy = 0.0
        self.fx = 0.0
        self.fy = 0.0
        self.k = np.zeros(6)
        self.codx = 0.0
        self.cody = 0.0
        self.p1 = 0.0
        self.p2 = 0.0

#标定参数类型
class CameraCalibration:
    def __init__(self):
        #相机的内参
        self.Color = CameraIntrinsics()
        self.Depth = CameraIntrinsics()
        #外参：从三维深度相机点到相对于彩色相机的三维点的外部变换
        self.RotationFromDepth = np.zeros((3, 3))
        self.TranslationFromDepth = np.zeros(3)


class FrameInfo:
    def __init__(self):
        self.Calibration = CameraCalibration()
        self.Accelerometer = np.zeros(3)
        self.TimeStamp = 0
        self.ColorImage = []
        self.ColorWidth = 0
        self.ColorHeight = 0
        self.ColorStride = 0
        self.DepthImage = []
        self.DepthWidth = 0
        self.DepthHeight = 0
        self.DepthStride = 0
        self.PointCloudData = []
        self.CameraIndex = 0
        self.FrameNumber = 0
        self.filename = ""


#对齐变换
class AlignmentTransform:
    def __init__(self):
        self.Transform = np.zeros(16)
        self.Identity = True

    #将输入的变换矩阵设置为 Transform 数组
    def set_from_matrix(self, src):
        self.Identity = np.allclose(src, np.identity(4))
        self.Transform = src.ravel().tolist()

    #根据 Identity 的值将 Transform 数组的值设置到输出的目标矩阵中
    def set_to_matrix(self, dest):
        if self.Identity:
            dest[:] = np.identity(4).ravel()
        else:
            dest[:] = np.reshape(self.Transform, (4, 4))


class ExtrinsicsCalibration:
    
    def __init__(self):       
        self.tag_detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        size = [5,7]
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.board = aruco.CharucoBoard(size,0.04,0.02,self.dictionary,None)
        self.params = aruco.DetectorParameters()
        self.full_cloud = o3d.geometry.PointCloud()
    

    def GenerateFullCloudFromFrames(self,frame):
        idx = frame.CameraIndex
        if not frame.PointCloudData:
            return False

        print("Generating Cloud From Frames")
        self.full_cloud[idx] = o3d.geometry.PointCloud()
        coord_stride = 1

        self.full_cloud[idx].points = o3d.utility.Vector3dVector()
        self.full_cloud[idx].colors = o3d.utility.Vector3dVector([])

        for y in range(0, frame.ColorHeight, coord_stride):
            for x in range(0, frame.ColorWidth, coord_stride):
                q_idx = (y * frame.ColorWidth + x) * 3
                q = np.array([
                    frame.PointCloudData[q_idx] / 1000.0,
                    frame.PointCloudData[q_idx + 1] / 1000.0,
                    frame.PointCloudData[q_idx + 2] / 1000.0
                ])
                if q[2] == 0:
                    continue
                 
                # RGBA -> RGB
                color_rgba = frame.ColorImage[y, x]
                color_rgb = color_rgba[:3][::-1] / 255.0

                if np.all(color_rgb == 0):
                    continue

                self.full_cloud[idx].points.append(q)
                self.full_cloud[idx].colors.append(color_rgb.tolist())

        self.full_cloud[idx].colors = o3d.utility.Vector3dVector(self.full_cloud[idx].colors)

        print("Constructed Open3D PointCloud for Camera ID:", idx)
        return True

    #todo:
    def refine_calculate(self):

        return 
    
    #todo:
    def refine_extrinsics(self):

        return 
    

    def down_sample(self, cloud, voxel_size, idx):
        # 对点云进行下采样
        voxel_down_pcd = self.full_cloud[idx].voxel_down_sample(voxel_size)
        if voxel_down_pcd is None:
            print("VoxelDownSample Failure")
            return False     
        # 使用完整分辨率的点云估计法线
        normal_radius = voxel_size * 2.0
        normals_params = o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30)
        fast_normal_computation = True
        voxel_down_pcd.estimate_normals(normals_params, fast_normal_computation)     
        # 假设法线应朝向相机位置
        voxel_down_pcd.orient_normals_towards_camera_location(np.array([0, 0, 0]))
        print("------VoxelDownSample Finished------")   
        return True


    def calculate_extrinsics(self, frames):
        print("------Start Calculate Extrinsics------")
        output:List[AlignmentTransform] = []
        for i in range(len(frames)):
            temp = AlignmentTransform()
            output.append(temp)

        if len(frames) == 0:
            print("No images provided to registration")
            return output

        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

        camera_count = len(frames)
        self.full_cloud = [None] * camera_count
        
        tag_poses = [None] * camera_count
        current_transform = [None] * camera_count
        l = 0.38

        for camera_index in range(camera_count):
            color_image = frames[camera_index].ColorImage


            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            intrinsics = frames[camera_index].Calibration.Color
            tag_size = 0.1
            detections = self.tag_detector.detect(gray,true,[intrinsics.fx,intrinsics.fy,intrinsics.cx,intrinsics.cx],tag_size)

            print("Detected", len(detections), "fiducial markers")
            found = False
            calibration = frames[camera_index].Calibration

            #apriltag
            if len(detections) != 0:
                detection = detections[0]
                print("Camera", camera_index, "detected marker ID:", detection.tag_id)
                print("pose_R:\n",detection.pose_R)
                print("pose_t:\n",detection.pose_t)

                if detection.tag_id == 1:  #todo:添加纠正
                    detection.pose_R = detection.pose_R
                    detection.pose_t = detection.pose_t
                elif detection.tag_id == 2:
                    detection.pose_R = detection.pose_R
                    detection.pose_t = detection.pose_t
                elif detection.tag_id == 3:
                    detection.pose_R = detection.pose_R
                    detection.pose_t = detection.pose_t

                transform = np.identity(4, dtype=np.float32)  # 创建单位矩阵
                for row in range(3):                          # 将旋转矩阵的值复制到transform矩阵中
                    for col in range(3):
                        transform[row, col] = detection.pose_R[row, col]                
                for row in range(3):                          # 将平移向量的值复制到transform矩阵中
                    transform[row, 3] = detection.pose_t[row]
                print("Pose:\n",transform)
                tag_poses[camera_index] = transform
                tag_pose = np.linalg.inv(tag_poses[0]) @ tag_poses[camera_index]  # 计算相对于第一个相机的标记姿态变换矩阵
                current_transform[camera_index] = tag_pose.astype(np.float64)  # 转换为双精度矩阵
                found = true

            if not found:
                print("No AprilTag detected, trying ChArUco board")
                markerIds = []  # 存储检测到的标记ID
                markerCorners = []  # 存储每个标记的角点坐标
                cv2.aruco.detectMarkers(gray,self.dictionary,markerCorners)
                if len(markerIds) > 0:  # 如果检测到标记
                    charucoCorners = []
                    charucoIds = []
                    # 相机内参矩阵
                    camMatrix = np.array([[calibration.Color.fx, 0, calibration.Color.cx],
                                          [0, calibration.Color.fy, calibration.Color.cy],
                                          [0, 0, 1]], dtype=np.float32)
                    # 相机畸变系数
                    distCoeffs = np.array([[calibration.Color.k[0]], [calibration.Color.k[1]], [calibration.Color.p1],
                                           [calibration.Color.p2], [calibration.Color.k[2]], [calibration.Color.k[3]],
                                           [calibration.Color.k[4]], [calibration.Color.k[5]]], dtype=np.float32)  
                    aruco.interpolateCornersCharuco(markerCorners, markerIds, gray, None, charucoCorners, charucoIds, camMatrix, distCoeffs)

                    if len(charucoIds) > 0:  # 如果检测到Charuco角点
                        rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, self.board, camMatrix, distCoeffs)
                        if rvec is not None and tvec is not None:  # 如果估计的旋转向量和平移向量可用
                            rmat, _ = cv2.Rodrigues(rvec)  # 旋转向量转换为旋转矩阵
                            transform = np.identity(4, dtype=np.float32)  # 创建单位矩阵

                            # 将旋转矩阵的值复制到transform矩阵中
                            for row in range(3):
                                for col in range(3):
                                    transform[row, col] = rmat[row, col]

                            # 将平移向量的值复制到transform矩阵中
                            for row in range(3):
                                transform[row, 3] = tvec[row]

                            print("Pose:")
                            print(transform)

                            tag_poses[camera_index] = transform  # 存储标记的姿态变换矩阵
                            tag_pose = np.linalg.inv(tag_poses[0]) @ tag_poses[camera_index]  # 计算相对于第一个相机的标记姿态变换矩阵
                            current_transform[camera_index] = tag_pose.astype(np.float64)  # 转换为双精度矩阵
                            found = True  # 设置标志表示找到了姿态

            if not found:
                print(f"Camera {camera_index} did not observe the fiducial marker - Waiting for the next frame")
                return output
            

            M_PI_FLOAT = 3.14159265
            
            pose0 = tag_poses[0]
            # Extract yaw angle from pose0
            rotation = Rotation.from_matrix(pose0[:3, :3])
            
            euler0 = rotation.as_euler('xyz', degrees=True)
            yaw = np.radians(euler0[2])
            print("Detected marker yaw =", np.degrees(yaw), "degrees")
            
            # 创建绕 y 轴旋转的角度轴
            yaw_rot = np.array([[math.cos(-yaw), 0, math.sin(-yaw)],
                    [0, 1, 0],
                    [-math.sin(-yaw), 0, math.cos(-yaw)]])
            # 创建 yaw_transform 仿射变换矩阵
            yaw_transform = np.eye(4)
            yaw_transform[:3, :3] = yaw_rot
            
            # 将场景以标记为中心
            marker_offset_0 = pose0[:3, 3]
            
            # 根据加速度计校正相机的倾斜
            tilt_transform = np.eye(4)
            
            # 使用第一个相机作为参考
            accel = frames[0].Accelerometer
            if np.all(accel == 0):
                print("IMU acceleration reading not available for tilt correction")
            else:
                print("Correcting tilt of primary camera using gravity down-vector", accel)
                # 调整加速度计和点云的坐标系
                gravity_vector = np.array([accel[1], accel[2], accel[0]])
                tilt_r, _ = cv2.Rodrigues(np.array([0, -1, 0]) * gravity_vector)
                tilt_transform = np.eye(4)
                tilt_transform[:3, :3] = tilt_r.T

                # 通过 tilt_r 矩阵的逆变换对marker_offset_0进行校正
                marker_offset_0 = np.linalg.inv(tilt_r) @ marker_offset_0
            
            # 创建平移变换矩阵
            translation_transform = np.eye(4)
            translation_transform[:3, 3] = -marker_offset_0
            
            # 创建 center_transform 变换矩阵
            center_transform = np.matmul(np.matmul(yaw_transform, translation_transform), tilt_transform)
            print("center_transform:")
            print(center_transform)
            print("===========================================================")
            print("!!! Starting extrinsics calibration for", camera_count, "cameras...")
            
            output[0].set_from_matrix(center_transform)

            for camera_index in range(camera_count):
                if not self.GenerateFullCloudFromFrames(frames[camera_index]):
                    print("GenerateCloudFromFrames failed for i =", camera_index)
                    return False
                
                cloud_i = o3d.geometry.PointCloud()
                if not self.down_sample(cloud_i, 0.01, camera_index):
                    print("DownSample failed for i =", camera_index)
                    return False
                
                cloud_i.transform(center_transform.astype(np.float64) @ current_transform[camera_index])
                filename = "cloud_" + str(camera_index) + ".ply"
                o3d.io.write_point_cloud(filename, cloud_i, write_ascii=True)


            voxel_radius = [0.04, 0.02, 0.01]  # 每个阶段的体素半径
            max_iter = [50, 30, 14]  # 每个阶段的最大迭代次数

            for stage in range(3):
                radius = voxel_radius[stage]  # 当前阶段的体素半径
                iter = max_iter[stage]  # 当前阶段的最大迭代次数
                print("voxel radius:", voxel_radius[stage], "Max iterations:", max_iter[stage])

                cloud_0 = o3d.geometry.PointCloud()  # 创建点云对象 cloud_0
                if not self.down_sample(cloud_0, radius, 0):  # 对 cloud_0 进行下采样
                    print("DownSample failed for i=0")
                    return False

                for camera_index in range(1, camera_count):  # 遍历每个相机索引
                    cloud_i = o3d.geometry.PointCloud()  # 创建点云对象 cloud_i
                    if not self.down_sample(cloud_i, radius, camera_index):  # 对 cloud_i 进行下采样
                        print("DownSample failed for i =", camera_index)
                        return False

                    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(1e-6, 1e-6, iter)  # ICP 收敛准则
                    lambda_geometric = 0.968  # 几何与颜色之间的权重系数
                    transform_estimate = o3d.pipelines.registration.TransformationEstimationForColoredICP(lambda_geometric)  # 变换估计器

                    result = o3d.pipelines.registration.registration_colored_icp(
                        cloud_i,
                        cloud_0,
                        voxel_radius[stage],
                        current_transform[camera_index],
                        transform_estimate,
                        criteria
                    )  # 进行有颜色信息的 ICP 配准

                    current_transform[camera_index] = result.transformation_.cast(np.double)  # 更新当前相机的变换矩阵

                    print("===========================================================")
                    print("Color ICP refinement for", camera_index, "-> 0")

                    output[camera_index].set_from_matrix(center_transform * current_transform[camera_index].cast(float))  # 计算输出结果
            
            for camera_index in range(camera_count):
                cloud_i = o3d.geometry.PointCloud()
                if not self.down_sample(cloud_i, 0.01, camera_index):
                    print("DownSample failed for i =", camera_index)
                    return False

                cloud_transform = center_transform @ current_transform[camera_index].astype(np.float32)
                cloud_i.transform(cloud_transform.astype(np.float64))

                filename = "icp_cloud_" + str(camera_index) + ".ply"
                o3d.io.write_point_cloud(filename, cloud_i, write_ascii=True)

                print("Point cloud saved")

                t = np.eye(4, dtype=np.float32)
                t[:3, :3] = cloud_transform[:3, :3]

                # Transform to Depth camera coordinate
                calibration = frames[camera_index].Calibration
                r_depth = np.array(calibration.RotationFromDepth, dtype=np.float32).reshape(3, 3).T
                t_depth = np.array(calibration.TranslationFromDepth, dtype=np.float32).reshape(3, 1) / 1000.0  # in meters

                t[:3, :3] = np.matmul(t[:3, :3], r_depth)
                t[:3, 3] = t[:3, 3] + np.matmul(r_depth, t_depth).flatten()


                # Flip y and z for OpenGL coordinate system
                yz_transform = np.eye(4, dtype=np.float32)
                yz_transform[1, 1] = -1.0
                yz_transform[2, 2] = -1.0
                t = yz_transform @ t @ yz_transform

                # Convert rotation matrix to quaternion
                r = Rotation.from_matrix(t[:3, :3])
                rotation_quat = r.as_quat()

                extrinsics_mat = {
                    "timeStamp":frames[camera_index].TimeStamp,
                    "translation": t[:3, 3].tolist(),
                    "rotation": rotation_quat.tolist()
                }

                # Save JSON file
                path = os.path.join(os.path.dirname(frames[camera_index].filename), "cn0" + str(camera_count - camera_index) + ".json")
                with open(path, "w") as file:
                    json.dump(extrinsics_mat, file)

            print("===========================================================")
                        
        return output
    


    def save_to_file( output,file_path):
        with open(file_path, "w") as file:
            for out in output:
                file.write(str(out) + "\n")
        print("Extrinsics calibration saved to file:", file_path)


# 示例用法
# colorFrame = FrameInfo()
# depthFrame = FrameInfo()
# irFrame = FrameInfo()
# frames = [colorFrame,depthFrame,irFrame]  # 假设有三个FrameInfo对象
# extrinsics_calib = ExtrinsicsCalibration()
# extrinsics = extrinsics_calib.calculate_extrinsics()
# print(extrinsics)