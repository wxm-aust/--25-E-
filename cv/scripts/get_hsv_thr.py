
import cv2
import numpy as np

# 回调函数，用于滑块更新时不做任何操作，占位用
def nothing(x):
    pass

# 创建窗口并添加滑块
cv2.namedWindow('HSV Threshold Adjuster')

# 添加HSV范围的滑块
cv2.createTrackbar('H Lower', 'HSV Threshold Adjuster', 0, 179, nothing)
cv2.createTrackbar('H Upper', 'HSV Threshold Adjuster', 179, 179, nothing)
cv2.createTrackbar('S Lower', 'HSV Threshold Adjuster', 0, 255, nothing)
cv2.createTrackbar('S Upper', 'HSV Threshold Adjuster', 255, 255, nothing)
cv2.createTrackbar('V Lower', 'HSV Threshold Adjuster', 0, 255, nothing)
cv2.createTrackbar('V Upper', 'HSV Threshold Adjuster', 255, 255, nothing)

# 启动摄像头（也可以改为读取视频文件）
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 获取当前滑块位置
    h_l = cv2.getTrackbarPos('H Lower', 'HSV Threshold Adjuster')
    h_u = cv2.getTrackbarPos('H Upper', 'HSV Threshold Adjuster')
    s_l = cv2.getTrackbarPos('S Lower', 'HSV Threshold Adjuster')
    s_u = cv2.getTrackbarPos('S Upper', 'HSV Threshold Adjuster')
    v_l = cv2.getTrackbarPos('V Lower', 'HSV Threshold Adjuster')
    v_u = cv2.getTrackbarPos('V Upper', 'HSV Threshold Adjuster')

    # 定义HSV阈值范围
    lower_bound = np.array([h_l, s_l, v_l])
    upper_bound = np.array([h_u, s_u, v_u])

    # 创建掩膜
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # 应用掩膜
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # 显示图像
    # cv2.imshow('Original', frame)
    cv2.imshow('Masked Result', result)

    # 按下 q 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()