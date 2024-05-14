import os
import cv2
import numpy as np
import env

def process_color_images():
    input_path  = os.path.join(env.concrete_path, 'color')
    output_path = os.path.join(env.concrete_path, 'color_filtered')
    ksize = env.ksize

    # 创建输出文件夹
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # 遍历输入文件夹中的所有png图像文件
    for filename in os.listdir(input_path):
        if filename.endswith(".png"):
            input_file = os.path.join(input_path, filename)
            output_file = os.path.join(output_path, filename)

            print("Reading:", input_file)

            # 读取彩色图像
            image = cv2.imread(input_file)
            if image is None:
                print("Error: Could not open or find the image:", input_file)
                continue

            # 分割彩色图像为B, G, R通道
            b, g, r = cv2.split(image)

            # 对每个通道应用中值滤波
            filtered_b = cv2.medianBlur(b, ksize)
            filtered_g = cv2.medianBlur(g, ksize)
            filtered_r = cv2.medianBlur(r, ksize)

            # 合并滤波后的通道以创建彩色图像
            filtered_image = cv2.merge([filtered_b, filtered_g, filtered_r])

            # 保存处理后的图像到输出文件夹
            cv2.imwrite(output_file, filtered_image)

            print("Processed:", filename, "and saved as", output_file)

def process_depth_images():
    input_path  = os.path.join(env.concrete_path, 'depth')
    output_path = os.path.join(env.concrete_path, 'depth_filtered')
    output_32_path = os.path.join(env.concrete_path, 'depth_32')
    d = env.d
    sigma_color = env.sigma_color
    sigma_space = env.sigma_space

    if not os.path.exists(output_path):
        os.makedirs(output_path)
    if not os.path.exists(output_32_path):
        os.makedirs(output_32_path)

    # 遍历输入文件夹中的所有png图像文件
    for filename in os.listdir(input_path):
        if filename.endswith(".png"):
            input_file = os.path.join(input_path, filename)
            output_file = os.path.join(output_path, filename)
            output_32_file = os.path.join(output_32_path, filename)


            print("Reading:", input_file)

            # 读取深度图像
            depth_image = cv2.imread(input_file, cv2.IMREAD_ANYDEPTH)
            if depth_image is None:
                print("Error: Could not open or find the image:", input_file)
                continue

            # 将深度图像转换为32位浮点数图像
            depth_image_float = depth_image.astype(np.float32)

            # 双边滤波
            filtered_image = cv2.bilateralFilter(depth_image_float, d, sigma_color, sigma_space)

            # 保存处理后的图像到输出文件夹
            # 转换为16位整数图像
            # filtered_image = filtered_image.astype(np.uint16)

            # 保存处理后的图像到输出文件夹
            cv2.imwrite(output_32_file, depth_image_float)
            cv2.imwrite(output_file, filtered_image)

            print("Processed:", filename, "and saved as", output_file)


# 处理彩色图像
# process_color_images()
process_depth_images()