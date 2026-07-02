import ddddocr
import cv2
import numpy as np
ocr = ddddocr.DdddOcr(show_ad=False)
def crop_perspective(img, points):
    # 确保输入点数正确
    if len(points) != 3:
        raise ValueError("需要三个点: 左上, 右上, 左下")

    # 定义源点（即输入的三点加上推测出的第四点）
    src_pts = np.array(points, dtype=np.float32)
    
    # 计算右下角的坐标
    fourth_point = src_pts[1] + src_pts[2] - src_pts[0]
    src_pts = np.vstack([src_pts, fourth_point])

    # 获取最小外接矩形的宽度和高度
    width_top = np.linalg.norm(src_pts[0] - src_pts[1])
    width_bottom = np.linalg.norm(src_pts[2] - fourth_point)
    max_width = int(max(width_top, width_bottom))
    
    height_left = np.linalg.norm(src_pts[0] - src_pts[2])
    height_right = np.linalg.norm(src_pts[1] - fourth_point)
    max_height = int(max(height_left, height_right))

    # 定义目标图像的尺寸
    dst_pts = np.array([
        [0, 0],
        [max_width - 1, 0],
        [0, max_height - 1],
        [max_width - 1, max_height - 1]
    ], dtype=np.float32)

    # 获取透视变换矩阵
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    
    # 应用透视变换
    cropped_img = cv2.warpPerspective(img, M, (max_width, max_height))
    
    return cropped_img
def mat_to_bytes(image, format='.png'):
    """
    将 Mat 图像转换为字节流。
    
    参数:
        image (Mat): 输入的图像
        format (str): 转换格式，默认为 png
        
    返回:
        bytes: 图像的字节数据
    """
    # 利用 OpenCV 的 imencode 方法将 Mat 转换成字节流
    _, buffer = cv2.imencode(format, image)
    img_bytes = buffer.tobytes()
    return img_bytes

def recognize_image_from_mat(image_mat):
    """
    使用 ddddocr 识别从相机获取的图像内容。
    
    参数:
        image_mat (Mat): 从相机获取的图像，以 Mat 类型表示
        
    返回:
        str: 识别结果文本，如果识别失败则返回 None
    """
    # cv2.imshow("Image for OCR", image_mat)
    try:
        # 将 Mat 转换为字节流
        img_bytes = mat_to_bytes(image_mat)
        # 使用 OCR 进行识别
        result = ocr.classification(img_bytes)
        # print(f"识别结果: {result}")
        if len(result) == 1 and result.isdigit():
            return result
        else:
            return None
    except Exception as e:
        print(f"识别过程中发生错误: {e}")
        return None