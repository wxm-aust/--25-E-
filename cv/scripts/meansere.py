import cv2
import numpy as np
from itertools import combinations
from yjm_pnp import DEFAULT_OBJ_POINTS, DEFAULT_CAMERA_MATRIX, DEFAULT_DIST_COEFFS,DEFAULT_OBJ_POINTS

def calculate(dots,world_points=DEFAULT_OBJ_POINTS):
    if len(dots) == 4:
        H_dots = np.array(dots, dtype=np.float32)
        H, mask = cv2.findHomography(H_dots, world_points, cv2.RANSAC, 5.0)
        success, rotation_vector, translation_vector = cv2.solvePnP(world_points, H_dots,DEFAULT_CAMERA_MATRIX , DEFAULT_DIST_COEFFS)
        return H, rotation_vector, translation_vector
    else:
        return None, None, None

###单应性矩阵法###
def decompose_homography(H, K):
    """从单应性矩阵和相机内参分解R和t"""
    # 将单应性矩阵转换到相机坐标系
    H = np.matmul(np.linalg.inv(K), H)
    
    # 计算尺度因子
    lambda_ = 1.0 / np.linalg.norm(H[:, 0])
    
    # 提取旋转矩阵的列
    r1 = lambda_ * H[:, 0]
    r2 = lambda_ * H[:, 1]
    r3 = np.cross(r1, r2)
    
    # 构建旋转矩阵
    R = np.column_stack((r1, r2, r3))
    
    # 执行SVD确保R是正交矩阵
    U, _, Vt = np.linalg.svd(R)
    R = np.matmul(U, Vt)
    
    # 提取平移向量
    t = lambda_ * H[:, 2]
    
    return R, t

def get_distance(H):
    camera_height = 75
    K = DEFAULT_CAMERA_MATRIX 
    R, t_norm = decompose_homography(H, K)  
    t_world_dir = R.T @ t_norm
    tz_world = t_world_dir[2]
    scale = camera_height / tz_world
    camera_position_world = scale * t_world_dir  # [Cx, Cy, Cz]
    horizontal_distance_mm = np.sqrt(camera_position_world[0]**2 + camera_position_world[1]**2)

    print(f"估计的水平距离: {horizontal_distance_mm:.2f} mm")
    return horizontal_distance_mm


def wxm_get_distance(H):
    K = DEFAULT_CAMERA_MATRIX 
    H = H.astype(np.float64)
    K = K.astype(np.float64)
    K_inv = np.linalg.inv(K)
    A = K_inv @ H
    r1 = A[:, 0]
    r2 = A[:, 1]
    t = A[:, 2]
    lambda_r1 = np.linalg.norm(r1)
    lambda_r2 = np.linalg.norm(r2)
    lambda_ = (lambda_r1 + lambda_r2) / 2.0
    r1 = r1 / lambda_
    r2 = r2 / lambda_
    t = t / lambda_
    r3 = np.cross(r1, r2)
    R = np.column_stack((r1, r2, r3))
    U, _, Vt = np.linalg.svd(R)
    R_corrected = U @ Vt
    r3_corrected = R_corrected[:, 2]
    distance_perpendicular = np.abs(np.dot(t, r3_corrected))
    print(f"\n估计的相机到目标平面的垂直距离: {distance_perpendicular:.2f} 单位")
    return distance_perpendicular

def get_world_point(dots,H):
    new_image_point = np.array(dots, dtype=np.float32)
    new_world_point = cv2.perspectiveTransform(new_image_point.reshape(-1, 1, 2), H)
    return new_world_point

def get_square_size(dots,H):
    # 确保有4个点
    if len(dots) != 4:
        raise ValueError("正方形必须有4个顶点")
    # 转换为世界坐标
    world_points = get_world_point(dots, H)
    
    # 假设点的顺序是：左上、右上、左下、右下
    side_lengths = [
        np.linalg.norm(world_points[1] - world_points[0]),  # 上边
        np.linalg.norm(world_points[3] - world_points[1]),  # 右边
        np.linalg.norm(world_points[2] - world_points[0]),  # 左边
        np.linalg.norm(world_points[3] - world_points[2])   # 下边
    ]
    
    avg_side_length = np.mean(side_lengths)
    
    print(f"正方形四条边长度: {[f'{s:.2f}' for s in side_lengths]} cm")
    print(f"正方形平均边长: {avg_side_length:.2f} cm")
    
    return avg_side_length

def get_equilateral_triangle_size(dots,H):
    # 确保有3个点
    if len(dots) != 3:
        raise ValueError("三角形必须有3个顶点")
    
    # 转换为世界坐标
    world_points = get_world_point(dots, H)
    
    # 计算三条边的长度
    side_lengths = [
        np.linalg.norm(world_points[1] - world_points[0]),  # 边1-2
        np.linalg.norm(world_points[2] - world_points[1]),  # 边2-3
        np.linalg.norm(world_points[0] - world_points[2])   # 边3-1
    ]
    
    avg_side_length = np.mean(side_lengths)
    
    print(f"三角形三条边长度: {[f'{s:.2f}' for s in side_lengths]} cm")
    print(f"三角形平均边长: {avg_side_length:.2f} cm")
    
    return avg_side_length

def get_circle_size(dots,H):
    
    if len(circle_data) != 3:
        raise ValueError("圆的数据格式应为 [x, y, r]")
    
    x, y, r = dots

    # 转换圆心到世界坐标
    center_world = get_world_point([[x, y],[x+r,y],[x,y+r]], H)
    R_length = [
        np.linalg.norm(center_world[1] - center_world[0]),
        np.linalg.norm(center_world[2] - center_world[1]),
        np.linalg.norm(center_world[0] - center_world[2])
    ]
    avg_R_length = np.mean(R_length)
    print(f"圆心世界坐标: ({center_world[0][0]:.2f}, {center_world[0][1]:.2f}) cm")
    print(f"圆的实际半径: {avg_R_length:.2f} cm")
    return avg_R_length






######PNP法######
def calculate_world_coordinates(image_points_array, rotation_vector, translation_vector, camera_matrix, dist_coeffs):
    """
    将图像点转换为世界坐标点
    
    参数:
    image_points_array: 图像点坐标
    rotation_vector: 旋转向量
    translation_vector: 平移向量
    camera_matrix: 相机内参矩阵
    dist_coeffs: 畸变系数
    
    返回:
    world_points: 世界坐标点
    """
    
    # 确保输入是正确的形状
    img_points = np.array(image_points_array, dtype=np.float32)
    if len(img_points.shape) == 1:
        img_points = img_points.reshape(-1, 1, 2)
    elif len(img_points.shape) == 2 and img_points.shape[1] == 2:
        img_points = img_points.reshape(-1, 1, 2)
    
    # 去畸变处理
    undistorted_points = cv2.undistortPoints(img_points, camera_matrix, dist_coeffs, P=camera_matrix)
    undistorted_points = undistorted_points.reshape(-1, 2)
    
    # 将相机内参转换为逆矩阵
    inv_camera_matrix = np.linalg.inv(camera_matrix)
    
    # 将图像点转换为归一化相机坐标系中的光线方向
    rays = []
    for point in undistorted_points:
        point_homo = np.array([point[0], point[1], 1.0])
        ray = inv_camera_matrix @ point_homo
        rays.append(ray)
    
    rays = np.array(rays)
    
    # 转换旋转矩阵
    R, _ = cv2.Rodrigues(rotation_vector)
    R_inv = R.T
    
    # 计算每个光线与参考平面(Z=0)的交点
    world_points = []
    ref_z = translation_vector[2]
    
    for ray in rays:
        ray_world = R_inv @ ray
        origin_world = translation_vector.flatten()
        
        if abs(ray_world[2]) > 1e-10:
            t = (ref_z - origin_world[2]) / ray_world[2]
            intersection = origin_world + t * ray_world
            world_points.append(intersection[:2])  # 只取X,Y坐标
        else:
            world_points.append(origin_world[:2])
    
    return np.array(world_points)
def calculate_a4_size(square_image_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs):
    """
    计算正方形的实际边长
    """
    # 确保有4个点
    if len(square_image_points) != 4:
        raise ValueError("正方形必须有4个顶点")
    
    # 转换为世界坐标
    world_points = calculate_world_coordinates(
        square_image_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs
    )
    
    # 假设点的顺序是：左上、右上、左下、右下
    side_lengths_a = [
        np.linalg.norm(world_points[1] - world_points[0]),  # 上边
        np.linalg.norm(world_points[3] - world_points[2])   # 下边
    ]
    side_lengths_b = [
        np.linalg.norm(world_points[3] - world_points[1]),  # 右边
        np.linalg.norm(world_points[2] - world_points[0]),  # 左边
    ]

    avg_side_length_a = np.mean(side_lengths_a)
    avg_side_length_b = np.mean(side_lengths_b)
    print(f"a4短边长度: {[f'{s:.2f}' for s in side_lengths_a]} cm")
    print(f"a4长边长度: {[f'{s:.2f}' for s in side_lengths_b]} cm")
    print(f"短边平均边长: {avg_side_length_a:.2f} cm")
    print(f"长边平均边长: {avg_side_length_b:.2f} cm")

    return avg_side_length_a, avg_side_length_b

def calculate_square_size(square_image_points, rotation_vector, translation_vector, camera_matrix=DEFAULT_CAMERA_MATRIX, dist_coeffs=DEFAULT_DIST_COEFFS):
    """
    计算正方形的实际边长
    """
    # 确保有4个点
    if len(square_image_points) != 4:
        raise ValueError("正方形必须有4个顶点")
    
    # 转换为世界坐标
    world_points = calculate_world_coordinates(
        square_image_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs
    )
    
    # 假设点的顺序是：左上、右上、左下、右下
    side_lengths = [
        np.linalg.norm(world_points[1] - world_points[0]),  # 上边
        np.linalg.norm(world_points[3] - world_points[1]),  # 右边
        np.linalg.norm(world_points[2] - world_points[0]),  # 左边
        np.linalg.norm(world_points[3] - world_points[2])   # 下边
    ]
    
    avg_side_length = np.mean(side_lengths)
    
    print(f"正方形四条边长度: {[f'{s:.2f}' for s in side_lengths]} cm")
    print(f"正方形平均边长: {avg_side_length:.2f} cm")
    
    return avg_side_length

def calculate_equilateral_triangle_size(triangle_image_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs):
    """
    计算等边三角形的实际边长
    """
    # 确保有3个点
    if len(triangle_image_points) != 3:
        raise ValueError("三角形必须有3个顶点")
    
    # 转换为世界坐标
    world_points = calculate_world_coordinates(
        triangle_image_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs
    )
    
    # 计算三条边的长度
    side_lengths = [
        np.linalg.norm(world_points[1] - world_points[0]),  # 边1-2
        np.linalg.norm(world_points[2] - world_points[1]),  # 边2-3
        np.linalg.norm(world_points[0] - world_points[2])   # 边3-1
    ]
    
    avg_side_length = np.mean(side_lengths)
    
    print(f"三角形三条边长度: {[f'{s:.2f}' for s in side_lengths]} cm")
    print(f"三角形平均边长: {avg_side_length:.2f} cm")
    
    return avg_side_length

def calculate_circle_size(circle_data, rotation_vector, translation_vector, camera_matrix, dist_coeffs):
    """
    计算圆的实际半径
    
    参数:
    circle_data: [x, y, r] 格式，其中x,y是圆心坐标，r是图像半径
    rotation_vector: 旋转向量
    translation_vector: 平移向量
    camera_matrix: 相机内参矩阵
    dist_coeffs: 畸变系数
    
    返回:
    real_radius: 实际半径
    center_world: 圆心世界坐标
    """
    
    if len(circle_data) != 3:
        raise ValueError("圆的数据格式应为 [x, y, r]")
    
    x, y, r = circle_data
    
    # 转换圆心到世界坐标
    center_world = calculate_world_coordinates(
        [[x, y]], rotation_vector, translation_vector, camera_matrix, dist_coeffs
    )[0]
    
    # 计算比例因子：使用A4纸的实际尺寸作为参考
    # 获取A4纸在世界坐标中的实际尺寸
    a4_world_points = calculate_world_coordinates(
        image_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs
    )
    
    # 计算A4纸的实际宽度（右上角到左上角）
    a4_real_width = np.linalg.norm(a4_world_points[1] - a4_world_points[0])
    # A4纸的图像宽度
    a4_image_width = np.linalg.norm(np.array([598, 4]) - np.array([10, 10]))
    
    # 计算像素到实际距离的比例
    if a4_image_width > 0:
        pixel_to_real_ratio = a4_real_width / a4_image_width
    else:
        # 备用方法：使用相机参数计算
        pixel_to_real_ratio = 19.8 / (598 - 10)  # A4纸实际宽度19.8cm
    
    # 计算圆的实际半径
    real_radius = r * pixel_to_real_ratio
    
    print(f"圆心图像坐标: ({x:.2f}, {y:.2f})")
    print(f"圆心世界坐标: ({center_world[0]:.2f}, {center_world[1]:.2f}) cm")
    print(f"圆的图像半径: {r:.2f} pixels")
    print(f"圆的实际半径: {real_radius:.2f} cm")
    
    return real_radius, center_world