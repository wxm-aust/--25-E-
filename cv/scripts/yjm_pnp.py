import cv2
import numpy as np
import math

DEFAULT_CAMERA_MATRIX = np.array([
    [1.55707449e+03, 0.00000000e+00, 2.41420974e+02],
    [0.00000000e+00, 1.48826397e+03, 3.22752129e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)


DEFAULT_DIST_COEFFS = np.array([ 5.16466178e-01,  1.34248913e+01,  1.69922516e-02 ,
     6.84656058e-03,  -5.73229139e+02], dtype=np.float32)


paper_width = 0.297 - 0.04
paper_height = 210 - 0.04

DEFAULT_OBJ_POINTS = np.array([
    [-paper_height*0.5, paper_width*0.5, 0],        # 左上角
    [paper_height*0.5, paper_width*0.5, 0],     # 右上角 
    [-paper_height*0.5, -paper_width*0.5, 0],       # 左下角 
    [paper_height*0.5, -paper_width*0.5, 0]     # 右下角
], dtype=np.float32)


def radians_to_degrees(radian_value):
    """
    将弧度转换为角度。
    
    参数:
    radian_value -- 弧度值(float)。
    
    返回:
    转换后的角度值(float)。
    """
    return math.degrees(radian_value)

# 示例：将π弧度转换为角度
radians = math.pi
degrees = radians_to_degrees(radians)

def perform_pnp( image_points,object_points = DEFAULT_OBJ_POINTS, camera_matrix=DEFAULT_CAMERA_MATRIX, dist_coeffs=DEFAULT_DIST_COEFFS):
    """
    使用OpenCV执行PnP操作。
    
    参数:
    - object_points: 一个包含4个3D点坐标的列表或numpy数组。
    - image_points: 一个包含4个对应2D点坐标的列表或numpy数组。
    - camera_matrix: (可选) 相机内参矩阵，默认使用预定义的矩阵。
    - dist_coeffs: (可选) 畸变系数，默认使用预定义的系数。
    
    返回:
    - 成功标志。
    - 旋转向量。
    - 平移向量。
    """
    # 确保输入是numpy数组
    object_points = np.array(object_points, dtype=np.float32)
    im_points = np.array(image_points, dtype=np.float32)

    # 使用solvePnP进行PnP求解
    success, rvec, tvec = cv2.solvePnP(object_points, im_points, camera_matrix, dist_coeffs)
    distance = np.linalg.norm(tvec)
    # print(rvec)
    # print(tvec)
    # print(distance)
    # print(tvec[0],tvec[1],tvec[2])
    # angle1 = radians_to_degrees(math.atan2(tvec[1],tvec[2]))
    # angle2 = radians_to_degrees(math.atan2(tvec[0],tvec[2]))
    # print(angle1,angle2)
    return round(distance,4)

def caculate_length(pix_length,known_length=0.21,F = 1130.95):
    return pix_length*F/known_length
