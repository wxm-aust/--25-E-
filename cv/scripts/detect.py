#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float32,UInt32
import cv2
from Get_rect import *
from Get_cir import *
from std_msgs.msg import Int32MultiArray
import yaml
import os
from dynamic_reconfigure.server import Server
from cv.cfg import PIDConfigConfig
from meansere import *
import time
button_number = 0
from yjm_pnp import *
import cv2
import numpy as np
import ddddocr
from digital import *
world_points = np.array([
    [0, 0], 
    [21.0, 0],
    [0, 29.7],
    [21.0, 29.7]
], dtype=np.float32)

def offset_points(points, offset):
    """
    将点列表中的每个点与偏移量相加
    
    参数:
        points: 点列表，格式为 [(x1,y1), (x2,y2), ...]
        offset: 基准点偏移量，格式为 (x0, y0)
        
    返回:
        新的点列表：[(x1+x0, y1+y0), (x2+x0, y2+y0), ...]
    """
    x0, y0 = offset
    return [(x + x0, y + y0) for x, y in points]
def get_roi_image_four_points(image, points):
    """
    根据提供的四个点坐标（顺序应为左上、右上、左下、右下），从图像中提取出最大的内接矩形ROI。
    函数通过透视变换将四边形区域映射为矩形。

    参数:
    - image: 输入图像(numpy数组)
    - points: 四个角点的位置列表或numpy数组，形状为 (4, 2)。顺序应为：[左上角, 右上角, 左下角, 右下角]

    返回:
    - roi_image: 提取出的矩形ROI区域图像
    - None: 如果输入点数量不为4或变换失败则返回None
    """
    # 输入验证
    if len(points) != 4:
        print("错误：必须提供恰好四个点。")
        return None

    # 将点转换为 numpy 数组并确保是浮点型 (cv2.getPerspectiveTransform 需要 float32)
    src_pts = np.array(points, dtype=np.float32)

    # 计算目标矩形的宽度和高度
    width1 = np.linalg.norm(src_pts[0] - src_pts[1]) # 上边
    width2 = np.linalg.norm(src_pts[2] - src_pts[3]) # 下边
    height1 = np.linalg.norm(src_pts[0] - src_pts[2]) # 左边
    height2 = np.linalg.norm(src_pts[1] - src_pts[3]) # 右边
    
    max_width = int(max(width1, width2)) # 通常取最大值以包含更多区域
    max_height = int(max(height1, height2)) # 通常取最大值以包含更多区域

    # 定义目标矩形的四个角点 (左上, 右上, 左下, 右下)
    dst_pts = np.array([
        [0, 0],
        [max_width - 1, 0],
        [0, max_height - 1],
        [max_width - 1, max_height - 1]
    ], dtype=np.float32)

    # 计算透视变换矩阵
    try:
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    except cv2.error as e:
        print(f"计算透视变换矩阵时出错: {e}")
        return None

    # 应用透视变换
    roi_image = cv2.warpPerspective(image, M, (max_width, max_height))

    return roi_image
def get_roi_image(image, top_left, bottom_right):
    """
    根据提供的左上角和右下角坐标，从图像中提取出感兴趣的区域(ROI)。
    
    参数:
    - image: 输入图像(numpy数组)
    - top_left: 左上角点的位置 (x, y)
    - bottom_right: 右下角点的位置 (x, y)
    
    返回:
    - roi_image: 提取出的ROI区域图像
    """
    roi_image = image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
    return roi_image


def calculate_perimeter(dots):
    """
    计算给定矩形四个顶点的四条边长度之和。
    
    参数:
    dots -- 矩形四个点[(xa,ya),(xb,yb),(xc,yc),(xd,yd)]，分别为左上右上左下右下
    
    返回:
    矩形四条边边长之和
    """
    # 解构输入的坐标点
    (xa, ya), (xb, yb), (xc, yc), (xd, yd) = dots
    
    # 计算各边长度
    side1 = math.sqrt((xb - xa)**2 + (yb - ya)**2) # 上边
    side2 = math.sqrt((xd - xc)**2 + (yd - yc)**2) # 下边
    side3 = math.sqrt((xc - xa)**2 + (yc - ya)**2) # 左边
    side4 = math.sqrt((xd - xb)**2 + (yd - yb)**2) # 右边
    
    # 返回四条边长之和
    return side1 + side2 + side3 + side4

save_pic = 0
choose_num = 11

def button_callback(data):
    global button_number,save_pic,choose_num
    button_number = data.data
    if button_number == 18:
        save_pic = 1
    if button_number == 10:
        choose_num = 0
    if button_number >=1 and button_number <= 9:
        choose_num = button_number

    print(f"Received button number: {button_number}")
    print("choose_num",choose_num)

def serial_data_callback(data):
    """订阅串口数据的回调函数"""
    received_value = data.data
    voltage_value = received_value / 1000.0  # 将毫伏转换回伏特
    # print(f"Received Value: {received_value}")
    print(f"Voltage: {voltage_value:.3f} V")
    print(f"Power: {voltage_value*5:.3f} W")

Ori_Width = 1920
Ori_Height= 1080

def D_publisher(num_float):
    D_send = Float32()
    D_send.data = num_float*100
    pub_D.publish(D_send)

def X_publisher(num_float):
    X_send = Float32()
    X_send.data = num_float
    pub_X.publish(X_send)

def dot_length(dot_1,dot_2):
    x1, y1 = dot_1
    x2, y2 = dot_2
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

def cac_D_from_sum_slide(x):
    y = -0.7*x + 1584.86
    return y 

Width = 640
Height = 480




if __name__ == '__main__':
    
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber('button_clicks', Int32, button_callback)
    rospy.Subscriber('received_data', UInt32, serial_data_callback)
    pub = rospy.Publisher('dx_dy', Int32MultiArray, queue_size=10)
    pub_D = rospy.Publisher('d_value', Float32, queue_size=10)
    pub_X = rospy.Publisher('x_value', Float32, queue_size=10)
    # 打开默认摄像头（一般为0，如果有多个摄像头，可以尝试1, 2等）
    cap = cv2.VideoCapture('/dev/video0') 
    cap.set(3, Ori_Width)
    cap.set(4, Ori_Height)
    if not cap.isOpened():
        # print("无法打开摄像头")
        exit()
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
    # cap.set(cv2.CAP_PROP_AUTO_WB,0)
    # cap.set(cv2.CAP_PROP_EXPOSURE, -8)
    
    
    dx = 0
    dy = 0
    
    cv2.namedWindow("ROI")
    cv2.moveWindow('ROI', 10, 10)
    cv2.namedWindow("img")
    cv2.moveWindow('img', 10, 10)


    
    X = 0.0
    D = 0.0
    paper_height_pix = 0.0
    paper_width_pix = 0.0
    x_pix = 0.0

    while  not rospy.is_shutdown():
        try:
            dots0 = None
            dots1 = None
            dots2 = None
            dots3 = None
            dots4 = None
            dots5 = None
            dots6 = None
            dots7 = None
            dots8 = None
            dots9 = None

            # 捕获一帧视频
            ret, img = cap.read()

            # 定义裁剪区域：居中截取 480x480
            start_x = (Ori_Width - 640) // 2  # 计算左侧起点
            # 因为高度已经是 480，所以不需要改变 y 轴坐标
            end_x = start_x + 480
            start_y = (Ori_Height - 480) // 2
            end_y = start_y + 480
            # start_y = 0
            # end_y = 480
            # img = img[0:480, start_x:start_x+480]
            img = img[start_y:end_y, start_x:end_x]
            

            # 如果读取帧正确ret为True
            if not ret:
                # print("无法接收帧（可能是摄像头断开了？）。退出中 ...")
                break
            # cv2.imshow('ori', img)
            ########################################################################

            # _,ball = get_cir(img,show_msgs = 1)

            img_rect,dots, = get_rect_grey(img,show_msgs = 1)        
            # print(f"检测到的点: {dots}")
            # H, rotation_vector, translation_vector = calculate(dots)

            if len(dots) > 0:
                
                # print(dots)
                
                # print(dots[1][0]-dots[0][0])

                side_sum = calculate_perimeter(dots)
                r2 = 3
                # print(dots)
                cv2.circle(img, dots[0], r2, (0,0,255), -1)
                cv2.circle(img, dots[1], r2, (0,0,255), -1)
                cv2.circle(img, dots[2], r2, (0,0,255), -1)
                cv2.circle(img, dots[3], r2, (0,0,255), -1)

                # print("distance",l)
                # dots = rect_shrink(dots,shrink_p = 0.12)
                # img_roi1 = get_roi_image_four_points(img,dots)
                img_roi1 = get_roi_image(img,dots[0],dots[3])

                


                if button_number == 11:
                    ## 正方形[(x,y), (x,y), (x,y), (x,y)]
                    img_square,dots0 = get_rect_square(img_roi1,show_msgs = 1)
                    cv2.imshow('ROI', img_square)
                    
                    if(dots0 is not None):
                        # dots0 = offset_points(dots0,dots[0])
                        # img = draw_points_on_image(img,dots0)
                        if(len(dots)==4):
                        # print()
                            try:
                                x_pix = (dot_length(dots0[0],dots0[1])
                                    + dot_length(dots0[1],dots0[2])
                                    + dot_length(dots0[2],dots0[3])
                                    + dot_length(dots0[3],dots0[0]))*1.0/4
                            except:
                                continue
                if button_number == 12:
                    ##三角形[(x,y), (x,y), (x,y)]
                    img_triangle,dots1 = get_triangle(img_roi1,show_msgs = 1)
                    cv2.imshow('ROI', img_triangle)
                    
                    
                    if(dots1 is not None):
                        # print(dot_length(dots1[0],dots1[1]))
                        # print(dot_length(dots1[2],dots1[1]))
                        # print(dot_length(dots1[0],dots1[2]))
                        # x_pix = dot_length(dots1[0],dots1[1])
                    #     # dots1 = offset_points(dots1,dots[0])
                    #     # img = draw_points_on_image(img,dots1)
                        try:
                            x_pix = (dot_length(dots1[0],dots1[1])
                                    + dot_length(dots1[1],dots1[2])
                                    + dot_length(dots1[2],dots1[0]))*1.0/3

                            # print("triangle",dots1)
                        except:
                            continue
                        
                    
                
                if button_number == 13:
                    # ##圆形[x,y,r]
                    img_circle,dots2 = get_circle(img_roi1,show_msgs = 1)
                    cv2.imshow('ROI', img_circle)
                    # circle_radius_real, circle_center_world = calculate_circle_size(
                    # dot2, rotation_vector, translation_vector, 
                    # camera_matrix, dist_coeffs
                    # )
                    if(dots2 is not None):
                        # dots2[0] = 
                        # dots2[1] = 
                        # dots2 = (dots2[0] +dots[0][0],dots2[1] +dots[0][1])
                        # print(dots2)
                        # img = draw_points_on_image(img,[dots2])
                        try:
                            x_pix = dots2[2]*2
                        except:
                            continue

                if button_number == 14:
                    ##多个正方形,返回最小的
                    img_multi_square,dots3 = get_multi_square(img_roi1,show_msgs = 1)
                    # print(meansere_P(img_roi1.shape,dots3))
                    cv2.imshow('ROI', img_multi_square)
                    if(dots3 is not None):
                        # dots3 = offset_points(dots0,dots[0])
                        # img = draw_points_on_image(img,dots3)
                        try:

                            x_pix = (dot_length(dots3[0],dots3[1])
                            + dot_length(dots3[1],dots3[2])
                            + dot_length(dots3[2],dots3[3])
                            + dot_length(dots3[3],dots3[0]))*1.0/4
                        except:
                            continue


                if button_number == 15:
                    ##带有重叠的正方形,返回最小的
                    img_fold_square, dots4 = get_fold_square(img_roi1,show_msgs = 1)
                    cv2.imshow('ROI', img_fold_square)
                    if dots4 is not None:
                        try:
                            x_pix = dots4[2]*2
                        except:
                            continue

                if button_number == 16:
                    #多个带数字的正方形以及其对应的数字
                    #一定要先选数字再按nsquare
                    #[[(x,y), (x,y), (x,y),(x,y)],[(x,y), (x,y), (x,y),(x,y)],...],[1,5,...]
                    img_num_rect,dots5 = get_multi_number_square(img_roi1,show_msgs = 1)
                    if dots5 is not None:#dots5是一个字典，{“1”：dot}
                        cv2.imshow('ROI', img_num_rect)

                        if choose_num != 11:
                            # print(dots5)
                            rect_choose = dots5.get(str(choose_num))
                            # print("my_choose_num:",str(choose_num))
                            # print(rect_choose)
                            if rect_choose is not None:
                                # try:
                                x_pix = (dot_length(rect_choose[0],rect_choose[1])
                                    + dot_length(rect_choose[1],rect_choose[2])
                                    + dot_length(rect_choose[2],rect_choose[3])
                                    + dot_length(rect_choose[3],rect_choose[0]))*1.0/4
                                # print(x_pix)
                                # except:
                                #     continue
                        # for idx, rectangle in enumerate(dots5):
                            # print(dots5)
                            # print(f"矩形 {idx + 1}:")
                            # rectangle = offset_points(rectangle,dots[0])
                            # get_square_size(rectangle,H)
                            ####参数需要输入图片，临时写了函数作为参数
                            # recognition_result = recognize_image_from_mat(crop_perspective(img,[(257, 177), (348, 173), (261, 278)]))
                            # img = draw_points_on_image(im`g, rectangle)

                if button_number == 17 or button_number == 18 or button_number == 19 or button_number == 20:
                    dot_mid_up = (int((dots[0][0]+dots[1][0])*0.5),int((dots[0][1]+dots[1][1])*0.5))
                    dot_mid_down = (int((dots[2][0]+dots[3][0])*0.5),int((dots[2][1]+dots[3][1])*0.5))

                   
                    side_sum =  dot_length(dot_mid_up,dot_mid_down) * 1014/297  #将中线的边长映射为周长

                    cv2.circle(img, dot_mid_up, 3, (0, 0, 255), -1)
                    cv2.circle(img, dot_mid_down, 3, (0, 0, 255), -1)
                    # print(side_sum)
                    transformed_img = transform_to_face(img,dots,int(dot_length(dot_mid_up,dot_mid_down) * 0.70707),int(dot_length(dot_mid_up,dot_mid_down)))
                    # cv2.imshow("transed",transformed_img)

                    if button_number == 17:###旋转+单正方形
                        img_square_a1,dots6 = get_rect_square(transformed_img,show_msgs = 1)
                        
                        
                        
                        if(dots6 is not None):
                            # dots0 = offset_points(dots0,dots[0])
                            # img = draw_points_on_image(img,dots0)
                            if(len(dots6)==4):
                            # print()
                                try:
                                    x_pix = (dot_length(dots6[0],dots6[1])
                                        + dot_length(dots6[1],dots6[2])
                                        + dot_length(dots6[2],dots6[3])
                                        + dot_length(dots6[3],dots6[0]))*1.0/4
                                except:
                                    continue
                    elif button_number == 18:###旋转+重叠正方形
                        # print("inside!!!!")
                        # cv2.imshow('original', transformed_img)
                        img_square_a2,dots7 = get_fold_square(transformed_img,show_msgs = 1)
                        cv2.imshow('ROI', img_square_a2)

                        if dots7 is not None:
                            # try:
                            x_pix = dots7[2]*2
                            # except:
                            #     continue
                    elif button_number == 19:###旋转+数字正方形
                        img_square_a3,dots8 = get_multi_number_square(transformed_img,show_msgs = 1)
                        if dots8 is not None:#dots5是一个字典，{“1”：dot}
                            cv2.imshow('ROI', img_square_a3)

                            if choose_num != 11:
                                # print(dots5)
                                rect_choose = dots8.get(str(choose_num))
                                # print("my_choose_num:",str(choose_num))
                                # print(rect_choose)
                                if rect_choose is not None:
                                    # try:
                                    x_pix = (dot_length(rect_choose[0],rect_choose[1])
                                        + dot_length(rect_choose[1],rect_choose[2])
                                        + dot_length(rect_choose[2],rect_choose[3])
                                        + dot_length(rect_choose[3],rect_choose[0]))*1.0/4



                paper_height_pix = abs(dots[1][0]-dots[0][0])
                # D = cac_D_from_sum_slide(side_sum)
                if side_sum != 0:
                    if button_number == 17:
                        D = 91251.92 * 0.01/side_sum   ##倾斜测距
                    else:
                        D = 88845.79 * 0.01/side_sum   ##直面测距

                    if x_pix != 0 and D != 0:
                        # print("x_pix",x_pix, x_pix * D*0.08080)#矩形
                        # print("x_pix",x_pix, x_pix * D*0.099,x_pix * D*0.08080 )#三角形和圆形
                        if button_number == 12 or button_number == 13 or button_number == 15 or button_number == 18: 
                            X = x_pix * D *  0.099 
                        else:
                            X = x_pix * D * 0.08080 
                
                # if paper_height_pix != 0:
                #     D = 1.0*225/abs(dots[1][0]-dots[0][0])
                #     X = x_pix * 0.21 / paper_height_pix
            D_publisher(D)
            X_publisher(X)

                
                

                # ##测试：提取正方形ROI区域
                # img_roi_num = get_roi_image_four_points(img_roi1,dots3)
                # img_roi_num = cv2.cvtColor(img_roi_num, cv2.COLOR_BGR2GRAY)
                # _,img_roi_num = cv2.threshold(img_roi_num, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                # # img_roi_num = preProcessing_OTSU(img_roi_num)
                # cv2.imshow('num', img_roi_num)
                # if save_pic == 1:
                #     save_pic = 0
                #     cv2.imwrite("/home/orangepi/save5.png", img_roi_num)
                #     print("pic saved")


                # print("res:",dots4)


            position = (10, 420)  # 文本左下角的坐标

            # 使用 putText 方法添加文本
            font = cv2.FONT_HERSHEY_SIMPLEX  # 字体类型
            font_scale = 1  # 字体大小
            color = (255, 0, 255)  # BGR 颜色（此例中为红色）
            thickness = 2  # 线条粗细
            line_type = cv2.LINE_AA  # 抗锯齿线型
            # if len(ball)>0 :
            text = f'X= {X:.2f}, D= {D:.2f}'
            cv2.putText(img, text, position, font, font_scale, color, thickness, line_type)

            draw_quadrant_crosshair(img)
            cv2.imshow('img', img)
            
            # if save_pic == 1:
            #     save_pic = 0
            #     filename = f"image_{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time()*1000)%1000}.jpg"
            #     cv2.imwrite(f"/home/orangepi/biaoding_pic/{filename}", img)
            #     print("save_success\n")


            
           
            ########################################################################
            # 显示捕获到的帧
            # 缩放图像到 400x300（宽x高）
            # img = cv2.resize(img, (300, 300))
            # cv2.imshow('cam', img)
            # cv2.resizeWindow('cam', 300, 300)
            
            # 如果按下键盘上的q就退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # except:
        except Exception  as e:
            # continue
            print("发生了一个错误：", e)

            

    # 完成所有操作后，释放捕获器和关闭所有OpenCV窗口
    cap.release()
    cv2.destroyAllWindows()
