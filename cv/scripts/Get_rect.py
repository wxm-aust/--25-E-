import cv2
import numpy as np


def draw_quadrant_crosshair(image, color=(0, 0, 255), thickness=2):
    """
    在图像上绘制四宫格十字线（两条线：水平中线和垂直中线）
    
    参数:
    image: 输入的OpenCV图像 (numpy array)
    color: 十字线颜色，BGR格式，默认为红色 (0, 0, 255)
    thickness: 线条粗细，默认为2
    
    返回:
    绘制了十字线的图像副本
    """
    # 创建图像副本，避免修改原图
    # result = image.copy()
    
    # 获取图像尺寸
    height, width = image.shape[:2]
    
    # 计算中心点坐标
    center_x = width // 2
    center_y = height // 2
    
    # 绘制垂直线（从上到下，穿过中心）
    cv2.line(image, (center_x, 0), (center_x, height), color, thickness)
    
    # 绘制水平线（从左到右，穿过中心）
    cv2.line(image, (0, center_y), (width, center_y), color, thickness)
    
    # return image

def add_white_border_inward(image, border_ratio = 0.02):

    # 创建一个新的图像副本，避免直接修改原始图像
    bordered_image = image.copy()

    # 获取图像的高度和宽度
    height, width = image.shape[:2]

    # 计算边框的像素宽度 (上/下 和 左/右)
    top_bottom_border = int(height * border_ratio)
    left_right_border = int(width * border_ratio)

    # 在图像内部添加白色边框
    # 上边框
    bordered_image[:top_bottom_border, :] = 0
    # 下边框
    bordered_image[-top_bottom_border:, :] = 0
    # 左边框
    bordered_image[:, :left_right_border] = 0
    # 右边框
    bordered_image[:, -left_right_border:] = 0

    return bordered_image

def rect_shrink(dots,shrink_p = 0.05):
    """
    输入：检测到的矩形框四个点坐标（最大框）
    返回：缩小后的矩形框（沿着对角线两两靠近）
        顺序：
   RA______RB
    |______|
   RC      RD
    """
    RA = dots[0]
    RB = dots[1]
    RC = dots[2]
    RD = dots[3]

    _RA = (int(RA[0]+(RD[0]-RA[0])*shrink_p),int(RA[1]+(RD[1]-RA[1])*shrink_p))
    _RD = (int(RD[0]-(RD[0]-RA[0])*shrink_p),int(RD[1]-(RD[1]-RA[1])*shrink_p))
    
    _RB = (int(RB[0]-(RB[0]-RC[0])*shrink_p),int(RB[1]+(RC[1]-RB[1])*shrink_p))
    _RC = (int(RC[0]+(RB[0]-RC[0])*shrink_p),int(RC[1]-(RC[1]-RB[1])*shrink_p))

    return [_RA,_RB,_RC,_RD]


def preProcessing_grey(img):
    #灰度预处理，缺点：无法运用色彩信息
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
    imgCanny = cv2.Canny(imgBlur, 200, 200)
    kernel = np.ones((5, 5))
    imgDial = cv2.dilate(imgCanny, kernel, iterations=2)
    imgThres = cv2.erode(imgDial, kernel, iterations=1)
    return imgThres
 
def preProcessing_color(image, lower_green=np.array([35, 43, 46]), upper_green=np.array([77, 255, 255])):
    # 彩色预处理，能够获取色彩信息

    # 将图像从 BGR 转换为 HSV 颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 创建掩膜
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # 使用掩膜对原图进行“与”操作，只保留绿色部分
    green_only = cv2.bitwise_and(image, image, mask=mask)

    kernel = np.ones((5, 5))
    mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.erode(mask, kernel, iterations=1)
    # return green_only
    return mask

#外框专属的预处理，二值化不取反色
def preProcessing_OTSU_outer(img):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _,imgBin = cv2.threshold(imgGray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # imgBin = cv2.bitwise_not(imgBin)
    return imgBin

def preProcessing_OTSU(img):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _,imgBin = cv2.threshold(imgGray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    imgBin = cv2.bitwise_not(imgBin)
    return imgBin
def getContours(img,img_ori,show_msgs):
    """
    输入：二值化图像，原始图像，显示处理结果标志位
    返回：排序后的四个点[(xa,ya),(xb,yb),(xc,yc),(xd,yd)]
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
        cv2.drawContours(img_ori, cnt, -1, (255, 0, 0), 3)#用于检查轮廓状态
        area = cv2.contourArea(cnt)       
        if area > 1000:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)#多边形逼近获取顶点数量
            if area > maxArea and len(approx) == 4:
                biggest = approx
                maxArea = area
                longest = cnt
    if show_msgs == 1:
        cv2.drawContours(img_ori, longest, -1, (255, 0, 0), 1)  # 绘制边界
        cv2.drawContours(img_ori, biggest, -1, (0, 0, 255), 3)  # 绘制角点
 
    if len(biggest) != 0:
        # 将角点按顺序存储到RA, RB, RC, RD列表中
        # 首先找到最左边的两个点
        leftmost = sorted([(tuple(point[0]), point[0]) for point in biggest], key=lambda x: x[0][0])[:2]
        # 然后找出这两个点中较高的点作为RA
        RA = min(leftmost, key=lambda x: x[0][1])[0]
        # 剩下的两个点中，找到最低的那个点作为RD
        rightmost = sorted([(tuple(point[0]), point[0]) for point in biggest], key=lambda x: x[0][0], reverse=True)[:2]
        RD = max(rightmost, key=lambda x: x[0][1])[0]
        # 最后剩下的两个点中，找到较低的那个点作为RB
        remaining_points = [tuple(point[0]) for point in biggest if tuple(point[0]) not in [RA, RD]]
        RB = min(remaining_points, key=lambda x: x[1])
        RC = [point for point in remaining_points if point != RB][0]
        # print(RA,RB,RC,RD)
        dots = [RA,RB,RC,RD]

    return dots

def get_square_contour(img,img_ori,show_msgs):
    """
    输入：二值化图像，原始图像，显示处理结果标志位
    返回：排序后正方形的四个点[(xa,ya),(xb,yb),(xc,yc),(xd,yd)]
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
        cv2.drawContours(img_ori, cnt, -1, (255, 0, 0), 3)#用于检查轮廓状态
        area = cv2.contourArea(cnt)       
        if area > 1000:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)#多边形逼近获取顶点数量

            if len(approx) == 4:
                points = approx.reshape(4, 2)
                
                # 计算四条边的长度
                side1 = np.linalg.norm(points[0] - points[1])
                side2 = np.linalg.norm(points[1] - points[2])
                side3 = np.linalg.norm(points[2] - points[3])
                side4 = np.linalg.norm(points[3] - points[0])
                # print(side1,side2,side3,side4)
                # 计算长边和短边
                max_side = max(side1, side2, side3, side4)
                min_side = min(side1, side2, side3, side4)


                if area > maxArea :
                    biggest = approx
                    maxArea = area
                    longest = cnt
    if show_msgs == 1:
        cv2.drawContours(img_ori, longest, -1, (255, 0, 0), 3)  # 绘制边界
        cv2.drawContours(img_ori, biggest, -1, (0, 0, 255), 20)  # 绘制角点
 
    if len(biggest) != 0:
        # 将角点按顺序存储到RA, RB, RC, RD列表中
        # 首先找到最左边的两个点
        leftmost = sorted([(tuple(point[0]), point[0]) for point in biggest], key=lambda x: x[0][0])[:2]
        # 然后找出这两个点中较高的点作为RA
        RA = min(leftmost, key=lambda x: x[0][1])[0]
        # 剩下的两个点中，找到最低的那个点作为RD
        rightmost = sorted([(tuple(point[0]), point[0]) for point in biggest], key=lambda x: x[0][0], reverse=True)[:2]
        RD = max(rightmost, key=lambda x: x[0][1])[0]
        # 最后剩下的两个点中，找到较低的那个点作为RB
        remaining_points = [tuple(point[0]) for point in biggest if tuple(point[0]) not in [RA, RD]]
        RB = min(remaining_points, key=lambda x: x[1])
        RC = [point for point in remaining_points if point != RB][0]
        # print(RA,RB,RC,RD)
        dots = [RA,RB,RC,RD]

    return dots

def get_triangle_contour(img,img_ori,show_msgs):
    """
    输入：二值化图像，原始图像，显示处理结果标志位
    返回：排序后的四个点[(xa,ya),(xb,yb),(xc,yc)]
    顺序：
          RA
          /\
         /  \
     RB /____\RC
    """
    dots = []
    biggest = np.array([])
    longest = np.array([])
    maxArea = 0
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        # cv2.drawContours(img_ori, cnt, -1, (255, 0, 0), 3)#用于检查轮廓状态
        area = cv2.contourArea(cnt)       
        
        if area > 200:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)#多边形逼近获取顶点数量

            if len(approx) == 3:
                
                
                # 计算四条边的长度



                if area > maxArea :
                    biggest = approx
                    maxArea = area
                    longest = cnt

    if show_msgs == 1:
        cv2.drawContours(img_ori, longest, -1, (255, 0, 0), 3)  # 绘制边界
        # cv2.drawContours(img_ori, biggest, -1, (0, 0, 255), 20)  # 绘制角点
    
    if len(biggest)  != 0:
        # points = approx.reshape(3, 2)
        # dots = [points[0],points[1],points[2]]
        # print("triangle",biggest)
        # print("points",biggest[0][0])
        radius = 3
        cv2.circle(img_ori, (biggest[0][0][0], biggest[0][0][1]), radius, (0, 255, 0), 2)
        cv2.circle(img_ori, (biggest[1][0][0], biggest[1][0][1]), radius, (0, 255, 0), 2)
        cv2.circle(img_ori, (biggest[2][0][0], biggest[2][0][1]), radius, (0, 255, 0), 2)
        dots = [biggest[0][0],biggest[1][0],biggest[2][0]]

        # print("dots",dots)
    return dots

def detect_circles_by_contour(img,img_ori,show_msgs):
    """
    输入：二值化图像，原始图像，显示处理结果标志位
    返回：圆形坐标，半径[x,y,r]
    """
    ball = []
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_circle = None
    max_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100:  # 面积阈值可以根据实际情况调整
            (x, y), radius = cv2.minEnclosingCircle(contour)
            circle_area = np.pi * radius**2
            # 判断是否接近圆形
            if abs(1 - area / circle_area) < 0.2:  # 圆形度阈值也可以调整
                if area > max_area:
                    max_area = area
                    max_circle = (int(x), int(y), int(radius))

    if max_circle is not None:
        x, y, radius = max_circle
        ball = [x, y, radius]
        if show_msgs == 1:
            cv2.circle(img_ori, (x, y), radius, (0, 255, 0), 2)
            # cv2
            # print(area)
        

    return max_circle


def get_rect_grey2(img,show_msgs = 0):
    """
    描述：使用灰度图像canny获得矩形框
    输入：图像
    返回：标记处理结果的图像，以及四边形框四个点（已排序）;若没找到四边形，则返回控数组
        [(xa,ya),(xb,yb),(xc,yc),(xd,yd)]
    """
    img_ori = img.copy()
    img = preProcessing_grey(img)
    dots,dots1 = getContours2(img,img_ori,show_msgs)
    # cv2.imshow("grey",img_ori)
    return img_ori,dots,dots1

def get_rect_color(img,show_msgs = 0):
    """
    描述：使用hsv图像的二值化结果canny获得矩形框
    输入：图像
    返回：标记处理结果的图像，以及四边形框四个点（已排序）;若没找到四边形，则返回控数组
        [(xa,ya),(xb,yb),(xc,yc),(xd,yd)]
    """
    img_ori = img.copy()
    # img = preProcessing(img)
    # getContours(img,img_ori,show_msgs)
    img = preProcessing_color(img)
    # contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img_ori, contours, -1, (255, 0, 0), 2)
    dots = getContours(img,img_ori,show_msgs)

    # print("dots:",len(dots))
    # print(dots)
    return img_ori,dots

def get_rect_grey(img,show_msgs = 0):
    """
    描述：使用灰度图像canny获得矩形框
    输入：图像
    返回：标记处理结果的图像，以及四边形框四个点（已排序）;若没找到四边形，则返回控数组
        [(xa,ya),(xb,yb),(xc,yc),(xd,yd)]
    """
    img_ori = img.copy()
    img = preProcessing_OTSU_outer(img)#不能取反色
    # cv2.imshow("grey",img)
    dots = getContours(img,img_ori,show_msgs)
    # cv2.imshow("ori",img)
    # cv2.imshow("ori2",img_ori)
    return img_ori,dots

def get_rect_square(img,show_msgs = 0):
    img_ori = img.copy()
    img = preProcessing_OTSU(img)
    img = add_white_border_inward(img)
    # contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    dots = get_square_contour(img,img_ori,show_msgs)
    # cv2.imshow("bin",img)
    # cv2.imshow("grey",img_ori)
    return img_ori,dots

def get_triangle(img,show_msgs = 0):
    img_ori = img.copy()
    img = preProcessing_OTSU(img)
    # img = add_white_border_inward(img)
    dots = get_triangle_contour(img,img_ori,show_msgs)
    # cv2.imshow("bin",img)
    # cv2.imshow("grey",img_ori)
    return img_ori,dots

def get_circle(img,show_msgs = 0):
    img_ori = img.copy()
    img = preProcessing_OTSU(img)
    img = add_white_border_inward(img)
    ball = detect_circles_by_contour(img,img_ori,show_msgs)
    # cv2.circle(result_img, (ball[0], ball[1]), 1, color, -1)
    # cv2.imshow("red",img_ori)
    # print(ball)
    return img_ori,ball