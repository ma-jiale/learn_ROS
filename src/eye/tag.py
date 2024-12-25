import cv2
import numpy as np
import os

# 蓝色格子的hsv取值范围
h_range = (80, 120)
s_range = (150, 255)
v_range = (180, 255)

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

def save_contour_centers(img, h_range, s_range, v_range, output_filename="contour_centers.npy"):
    """
    查找并保存图像中指定 HSV 范围内的轮廓的中心点到 NumPy .npy 文件。

    Args:
        img: 输入图像 (NumPy 数组)。
        h_range: 色调 (Hue) 范围。
        s_range: 饱和度 (Saturation) 范围。
        v_range: 明度 (Value) 范围。
        output_filename: 输出文件名，默认为 "contour_centers.npy"。

    Returns:
        如果成功保存中心点，则返回 True。
        如果发生错误，则返回 False。
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
        if len(centers) == 9: #如果找到所有的格子中心点
            try:
                np.save(output_filename, np.array(centers)) #保存为numpy数组
                print(f"Contour centers saved to {output_filename}")
                return centers
            except Exception as e:
                print(f"Error saving contour centers: {e}")
                return None
        else:
            print("No valid contour centers found.")
            return None

    else:
        print("No contours found.")
        return None



def draw_points(img, points, color=(0, 0, 255), radius=3, thickness=-1):
    """
    在图像上绘制点。

    Args:
        img: 要在其上绘制点的图像 (NumPy 数组)。
        points: 包含坐标的数组或列表。每个元素应为 (x, y) 元组或列表。
        color: 点的颜色，默认为红色 (BGR 格式)。
        radius: 点的半径，默认为 3。
        thickness: 点的粗细。如果为 -1，则填充圆点，默认为 -1。

    Returns:
        绘制了点的图像 (NumPy 数组)。如果输入图像为空或点列表为空，则返回原始图像或None。
    """
    if img is None:
        print("Error: Input image is None.")
        return None

    if points is None or len(points) == 0:
        print("Warning: No points to draw.")
        return img

    img_copy = img.copy() # 在图像副本上绘制，防止修改原图

    for point in points:
        try:
            x, y = point
            x = int(x)
            y = int(y)
            cv2.circle(img_copy, (x, y), radius, color, thickness)
        except (TypeError, ValueError):
            print(f"Warning: Invalid point format: {point}. Skipping.")
        except IndexError:
            print(f"Warning: Point index error: {point}. Skipping.")
    return img_copy

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


if __name__ == "__main__":
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
        captured_frame = frame.copy()
        if frame is not None:
            centers = find_contour_centers(frame, h_range, s_range, v_range)
            if centers:
                frame = draw_points(frame, centers)
        cv2.imshow("Camera (press Enter to capture image, q to exit)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 13:  # Enter 键的 ASCII 码是 13
            if captured_frame is not None:
                centers = save_contour_centers(captured_frame, h_range, s_range, v_range)
                if centers:
                    cv2.imshow("with_centers", draw_points(captured_frame, centers))
        elif key == ord('q'):  # q 键退出
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
