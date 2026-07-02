import cv2
import numpy as np
from Get_rect import *
import math
from digital import *

import cv2
import numpy as np

def transform_to_face(img, dots, res_width, res_height):
    """
    将图像中指定四边形区域透视变换为指定尺寸的矩形图像
    
    参数:
        img: 输入图像 (numpy数组)
        dots: 四边形四个点坐标，顺序为[左上, 右上, 左下, 右下]
        res_width: 输出图像宽度
        res_height: 输出图像高度
        
    返回:
        transformed: 变换后的ROI图像
    """
    # 确保输入点顺序正确
    if len(dots) != 4:
        raise ValueError("必须提供4个点坐标")
    
    # 解包四个点
    tl, tr, bl, br = dots  # top-left, top-right, bottom-left, bottom-right
    
    # 源四边形点坐标
    src_pts = np.array([tl, tr, br, bl], dtype="float32")
    
    # 目标矩形点坐标 (按相同顺序)
    dst_pts = np.array([
        [0, 0],                     # 左上角
        [res_width - 1, 0],          # 右上角
        [res_width - 1, res_height - 1],  # 右下角
        [0, res_height - 1]          # 左下角
    ], dtype="float32")
    
    # 计算透视变换矩阵
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    
    # 应用透视变换
    transformed = cv2.warpPerspective(
        img, 
        M, 
        (res_width, res_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE
    )
    
    return transformed

def draw_points_on_image(img, points, color=(0, 0, 255), radius=5, thickness=-1):
    """
    在图像上绘制点
    
    参数:
        img: 输入的OpenCV图像 (numpy数组)
        points: 要绘制的点列表，格式为 [(x1, y1), (x2, y2), ...]
        color: 点的颜色 (BGR格式)，默认为红色 (0, 0, 255)
        radius: 点的半径，默认为5像素
        thickness: 线条粗细，-1表示实心填充，默认为-1
        
    返回:
        绘制了点的新图像 (原始图像会被修改)
    """
    # 创建图像的副本，避免修改原始图像
    result_img = img.copy()
    
    # 检查points是否为空
    if not points:
        return result_img
    
    # 遍历所有点并绘制
    for i, point in enumerate(points):
        # 确保点是整数坐标
        x, y = int(point[0]), int(point[1])
        
        # 绘制圆点
        cv2.circle(result_img, (x, y), radius, color, thickness)
        
    
    return result_img

def get_multi_square_contour(img,img_ori,show_msgs):
    """
    输入：二值化图像，原始图像，显示处理结果标志位
    返回：多个矩形返回最小的矩形的四个角点坐标
    顺序：
        RA______RB
         |______|
        RC      RD
    """
    dots = []
    smallest = np.array([])
    shortest = np.array([])
    minArea = 11451419
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        # cv2.drawContours(img_ori, cnt, -1, (255, 0, 0), 1)#用于检查轮廓状态
        area = cv2.contourArea(cnt)     
        # print(area)  
        if area > 200:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)#多边形逼近获取顶点数量
            # print(len(approx))
            if len(approx) == 4:
                points = approx.reshape(4, 2)

                if area < minArea :
                    smallest = approx
                    minArea = area
                    shortest = cnt
    if show_msgs == 1:
        cv2.drawContours(img_ori, shortest, -1, (255, 0, 0), 1)  # 绘制边界
        cv2.drawContours(img_ori, smallest, -1, (0, 0, 255), 5)  # 绘制角点
 
    if len(smallest) != 0:
        # 将角点按顺序存储到RA, RB, RC, RD列表中
        # 首先找到最左边的两个点
        leftmost = sorted([(tuple(point[0]), point[0]) for point in smallest], key=lambda x: x[0][0])[:2]
        # 然后找出这两个点中较高的点作为RA
        RA = min(leftmost, key=lambda x: x[0][1])[0]
        # 剩下的两个点中，找到最低的那个点作为RD
        rightmost = sorted([(tuple(point[0]), point[0]) for point in smallest], key=lambda x: x[0][0], reverse=True)[:2]
        RD = max(rightmost, key=lambda x: x[0][1])[0]
        # 最后剩下的两个点中，找到较低的那个点作为RB
        remaining_points = [tuple(point[0]) for point in smallest if tuple(point[0]) not in [RA, RD]]
        RB = min(remaining_points, key=lambda x: x[1])
        RC = [point for point in remaining_points if point != RB][0]
        # print(RA,RB,RC,RD)
        dots = [RA,RB,RC,RD]

    return dots

def get_multi_number_square_contour(img,img_ori,show_msgs):
    """
    输入：二值化图像，原始图像，显示处理结果标志位
    返回：多个矩形与其对应的数字识别结果
    识别结果为数字的字典，键为数字，值为四个角点坐标
    轮廓的四个角点顺序为：
    顺序：
        RA______RB
         |______|
        RC      RD
    """
    dots = []
    dict_square = {}
    smallest = np.array([])
    shortest = np.array([])
    minArea = 11451419
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        # print("con_num:",len(contours))
        cv2.drawContours(img_ori, cnt, -1, (255, 0, 0), 1)#用于检查轮廓状态
        area = cv2.contourArea(cnt)     
        # print(area)  
        if area > 200:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)#多边形逼近获取顶点数量
            # print(len(approx))
            if len(approx) == 4:
                points = approx.reshape(4, 2)

                # 将角点按顺序存储到RA, RB, RC, RD列表中
                # 首先找到最左边的两个点
                leftmost = sorted([(tuple(point[0]), point[0]) for point in approx], key=lambda x: x[0][0])[:2]
                # 然后找出这两个点中较高的点作为RA
                RA = min(leftmost, key=lambda x: x[0][1])[0]
                # 剩下的两个点中，找到最低的那个点作为RD
                rightmost = sorted([(tuple(point[0]), point[0]) for point in approx], key=lambda x: x[0][0], reverse=True)[:2]
                RD = max(rightmost, key=lambda x: x[0][1])[0]
                # 最后剩下的两个点中，找到较低的那个点作为RB
                remaining_points = [tuple(point[0]) for point in approx if tuple(point[0]) not in [RA, RD]]
                RB = min(remaining_points, key=lambda x: x[1])
                RC = [point for point in remaining_points if point != RB][0]
                # print(RA,RB,RC,RD)
                dots.append([RA,RB,RC,RD])

    # print(dots)
    # if show_msgs == 1:
        
        # cv2.drawContours(img_ori, shortest, -1, (255, 0, 0), 1)  # 绘制边界
        # cv2.drawContours(img_ori, smallest, -1, (0, 0, 255), 5)  # 绘制角点
    for dot in dots:
        # print (dot)
        recognition_result = recognize_image_from_mat(crop_perspective(img_ori,[dot[0],dot[1],dot[2]]))
        if recognition_result is not None:
            dict_square[recognition_result] = dot
        
        # print(dict_square)
        # draw_points_on_image(img_ori,dot)
    return dict_square

def get_fold_contour(img, img_ori, show_msgs):
    """
    输入：二值化图像，原始图像，显示处理结果标志位
    返回：最小内接圆的圆心坐标和半径
    轮廓的四个角点顺序为：
    顺序：
        RA______RB
         |______|
        RC      RD
    """
    dots = []
    biggest = np.array([])
    longest = np.array([])
    maxArea = 0
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    for cnt in contours:
        cv2.drawContours(img_ori, cnt, -1, (255, 0, 0), 3)  # 用于检查轮廓状态
        area = cv2.contourArea(cnt)
        if area > 1000:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)  # 多边形逼近获取顶点数量
            
            if len(approx) > 4:
                # 计算轮廓方向 (顺时针/逆时针)
                area_approx = cv2.contourArea(approx, oriented=True)
                is_ccw = area_approx > 0  # 逆时针为True
                
                n = len(approx)
                # 辅助函数：计算边的归一化直线方程
                def get_normalized_line(pt0, pt1, is_ccw):
                    dx = pt1[0] - pt0[0]
                    dy = pt1[1] - pt0[1]
                    length = math.sqrt(dx**2 + dy**2)
                    if length < 1e-5:  # 边太短，忽略
                        return (None, None, None)
                    if is_ccw:  # 逆时针轮廓，内法向量为(-dy, dx)
                        A = -dy / length
                        B = dx / length
                    else:       # 顺时针轮廓，内法向量为(dy, -dx)
                        A = dy / length
                        B = -dx / length
                    C = -(A * pt0[0] + B * pt0[1])
                    return (A, B, C)
                
                # 遍历每组连续的三条边
                for i in range(n):
                    # 获取三条边的端点
                    pt0_0 = approx[(i-1) % n][0]  # 边1起点
                    pt0_1 = approx[i][0]            # 边1终点
                    pt1_0 = approx[i][0]            # 边2起点
                    pt1_1 = approx[(i+1) % n][0]    # 边2终点
                    pt2_0 = approx[(i+1) % n][0]    # 边3起点
                    pt2_1 = approx[(i+2) % n][0]    # 边3终点
                    
                    # 计算三条边的归一化直线方程
                    line0 = get_normalized_line(pt0_0, pt0_1, is_ccw)
                    line1 = get_normalized_line(pt1_0, pt1_1, is_ccw)
                    line2 = get_normalized_line(pt2_0, pt2_1, is_ccw)
                    
                    # 跳过无效边
                    if None in line0 or None in line1 or None in line2:
                        continue
                    
                    A0, B0, C0 = line0
                    A1, B1, C1 = line1
                    A2, B2, C2 = line2
                    
                    # 构建线性方程组: 
                    #   A0*x + B0*y - r = -C0
                    #   A1*x + B1*y - r = -C1
                    #   A2*x + B2*y - r = -C2
                    M = np.array([
                        [A0, B0, -1],
                        [A1, B1, -1],
                        [A2, B2, -1]
                    ])
                    v = np.array([-C0, -C1, -C2])
                    
                    try:
                        # 求解圆心(x,y)和半径r
                        solution = np.linalg.solve(M, v)
                        x, y, r = solution
                        if r > 0:  # 仅保留正半径的有效解
                            dots.append((x, y, r))
                    except np.linalg.LinAlgError:
                        # 矩阵奇异或无解，跳过
                        continue

    min_circle = None
    if dots:  # 确保列表非空
        # 找到半径最小的圆
        min_circle = min(dots, key=lambda circle: circle[2])
        x, y, r = min_circle
        
        # 在原始图像上绘制最小圆
        cv2.circle(img_ori, (int(x), int(y)), int(r), (0, 255, 0), 3)  # 绿色圆
        cv2.circle(img_ori, (int(x), int(y)), 5, (0, 0, 255), -1)      # 红色圆心点
        
        # 可选：显示信息
        # if show_msgs:
        #     cv2.putText(img_ori, f"Min r: {r:.1f}", (int(x), int(y - r - 10)), 
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # 返回最小圆（如果存在）或None
    return min_circle

def get_multi_square(img,show_msgs = 0):
    img_ori = img.copy()
    img = preProcessing_OTSU(img)
    img = add_white_border_inward(img)
    # contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    dots = get_multi_square_contour(img,img_ori,show_msgs)
    # cv2.imshow("bin",img)
    # cv2.imshow("grey",img_ori)
    return img_ori,dots

def get_multi_number_square(img,show_msgs = 0):
    img_ori = img.copy()
    img = preProcessing_OTSU(img)
    img = add_white_border_inward(img)
    # contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    dots = get_multi_number_square_contour(img,img_ori,show_msgs)
    # print("res:",dots)
    # cv2.imshow("bin",img)
    # cv2.imshow("grey",img_ori)
    return img_ori,dots


def get_fold_square(img,show_msgs = 1):
    img_ori = img.copy()
    img = preProcessing_OTSU(img)
    img = add_white_border_inward(img)
    min_circle = get_fold_contour(img,img_ori,show_msgs)#直接返回最大内接圆
    return img_ori,min_circle