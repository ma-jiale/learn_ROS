import cv2
import numpy as np
import os
import math

# 水果的颜色hsv范围
h_range = (0, 25)
s_range = (200, 255)
v_range = (200, 255)


def calculate_distance(point1, point2):
    """
    计算两个像素点之间的欧几里得距离

    参数:
    - point1: tuple，第一个像素点的坐标 (x1, y1)
    - point2: tuple，第二个像素点的坐标 (x2, y2)

    返回:
    - float，两个点之间的距离
    """
    x1, y1 = point1
    x2, y2 = point2

    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance

def load_contour_centers(filename="contour_centers.npy"):
    """
    从 NumPy .npy 文件加载轮廓中心点。

    Args:
        filename: 文件名，默认为 "contour_centers.npy"。

    Returns:
        如果成功加载中心点，则返回中心点列表。
        如果文件不存在或发生其他错误，则返回 None。
    """
    try:
        if os.path.exists(filename):
          centers = np.load(filename)
          print(f"Contour centers loaded from {filename}")
          return centers
        else:
          print(f"Error: File not exists {filename}")
          return None
    except Exception as e:
        print(f"Error loading contour centers: {e}")
        return None

def calculate_contour_center(contour):
    """计算单个轮廓的中心点。"""
    if contour is None or len(contour) == 0:
        return None

    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None

    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return cX, cY

def find_contour_centers(img, h_range, s_range, v_range, output_filename="contour_centers.npy"):
    """
    查找图像中指定 HSV 范围内的轮廓的中心点。

    Args:
        img: 输入图像 (NumPy 数组)。
        h_range: 色调 (Hue) 范围。
        s_range: 饱和度 (Saturation) 范围。
        v_range: 明度 (Value) 范围。
        output_filename: 输出文件名，默认为 "contour_centers.npy"。

    Returns:
        如果成功查找中心点，则返回 True。
        如果发生错误，则返回 None。
    """
    if img is None:
        print(f"Error: Could not open or read image: {img}")
        return False

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 创建掩码
    lower_bound = np.array([h_range[0], s_range[0], v_range[0]])
    upper_bound = np.array([h_range[1], s_range[1], v_range[1]])
    mask = cv2.inRange(img_hsv, lower_bound, upper_bound)

    conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if conts:
        centers = []
        for contour in conts:
            center = calculate_contour_center(contour)
            if center:
                centers.append(center)
        # print(centers)
        if centers: #如果找到至少一个中心点
            return centers
        else:
            print("No valid contour centers found.")
            return None

    else:
        print("No contours found.")
        return None

def sort_rec_center(points):
    # 第一步：按 y 坐标排序（先按行排序）
    points = list(points)
    points.sort(key=lambda p: p[1])

    # 第二步：分成三行，再分别按 x 坐标排序
    row1 = sorted(points[:3], key=lambda p: p[0])  # 第一行
    row2 = sorted(points[3:6], key=lambda p: p[0])  # 第二行
    row3 = sorted(points[6:9], key=lambda p: p[0])  # 第三行

    # 第三步：将排序后的点存入字典
    sorted_points = row1 + row2 + row3
    points_dict = {i + 1: point for i, point in enumerate(sorted_points)}
    return  points_dict

def have_fruit(sorted_centers, fruit_centers):
    result = []
    side_length = sorted_centers[2][0] - sorted_centers[1][0]
    for fruit in fruit_centers:
        for i in range(1, 10):
            if calculate_distance(fruit, sorted_centers[i]) < side_length / 2:
                result.append(i)
    result = list(set(result))
    return result



if __name__ == "__main__":
    centers = load_contour_centers("contour_centers.npy")
    sorted_centers = sort_rec_center(centers)

    # 打开摄像头
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取帧")
            break
        cv2.imshow("Camera (press Enter to capture image, q to exit)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 13:  # Enter 键的 ASCII 码是 13
            captured_frame = frame.copy()
            fruit_centers = find_contour_centers(captured_frame, h_range, s_range, v_range)
            result = have_fruit(sorted_centers, fruit_centers)
            print(result)
        elif key == ord('q'):  # q 键退出
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()