# coding:utf-8
import cv2
import numpy as np

# 参数设置
input_file = "GlobalMap.pgm"     # 输入的PGM图像文件名
output_file = "map_processed.pgm"  # 处理后的输出图像文件名
kernel_size = 5  # 闭运算的内核大小
apply_filter = True    # 是否应用中值滤波
median_kernel_size = 3  # 中值滤波的窗口大小

# 读取PGM图像
img = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise ValueError(f"无法读取图像: {input_file}")

# 创建结构元素（内核）
kernel = np.ones((kernel_size, kernel_size), np.uint8)

# 如果墙体是黑色(0)，背景是白色(255)，需要先反转图像
img_inv = cv2.bitwise_not(img)

# 应用闭运算（先膨胀后腐蚀）来连接断开的边缘
closed = cv2.morphologyEx(img_inv, cv2.MORPH_CLOSE, kernel)

# 根据参数决定是否应用滤波
if apply_filter:
    processed = cv2.medianBlur(closed, median_kernel_size)  # 使用中值滤波
else:
    processed = closed

# 再次反转回来，使墙体恢复为黑色
result = cv2.bitwise_not(processed)

# 创建一个2x2的显示窗口来对比所有处理步骤
# 调整图像大小以便显示
height, width = img.shape
display_height = 300  # 设置显示高度
scale = display_height / height
display_width = int(width * scale)

def add_border_and_title(image, title):
    # 转换为RGB以添加彩色边框
    if len(image.shape) == 2:
        image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    else:
        image_rgb = image.copy()
    
    # 添加蓝色边框（不覆盖图像内容）
    border_thickness = 2
    h, w = image_rgb.shape[:2]
    # 画四条边
    cv2.line(image_rgb, (0, 0), (w-1, 0), (255, 0, 0), border_thickness)  # 上
    cv2.line(image_rgb, (0, 0), (0, h-1), (255, 0, 0), border_thickness)  # 左
    cv2.line(image_rgb, (w-1, 0), (w-1, h-1), (255, 0, 0), border_thickness)  # 右
    cv2.line(image_rgb, (0, h-1), (w-1, h-1), (255, 0, 0), border_thickness)  # 下
    
    # 使用OpenCV直接添加英文标题
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    thickness = 2
    text_size = cv2.getTextSize(title, font, font_scale, thickness)[0]
    x = (w - text_size[0]) // 2
    cv2.putText(image_rgb, title, (x, 30), font, font_scale, (255, 255, 255), thickness)
    
    return image_rgb

# 处理每张图片
img_display = cv2.resize(img, (display_width, display_height))
closed_display = cv2.resize(cv2.bitwise_not(closed), (display_width, display_height))
processed_display = cv2.resize(cv2.bitwise_not(processed), (display_width, display_height))
result_display = cv2.resize(result, (display_width, display_height))

# 添加边框和标题（使用英文）
img_display = add_border_and_title(img_display, "Original")
closed_display = add_border_and_title(closed_display, "After Closing")
processed_display = add_border_and_title(processed_display, "After Filter")
result_display = add_border_and_title(result_display, "Final Result")

# 创建2x2网格显示
padding = 10  # 图像之间的间距
grid_height = display_height * 2 + padding
grid_width = display_width * 2 + padding
grid = np.zeros((grid_height, grid_width, 3), dtype=np.uint8)

# 放置图像到网格中
grid[:display_height, :display_width] = img_display
grid[:display_height, display_width+padding:] = closed_display
grid[display_height+padding:, :display_width] = processed_display
grid[display_height+padding:, display_width+padding:] = result_display

# 显示对比图
cv2.imshow("Image Processing Steps", grid)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存处理后的图像
cv2.imwrite(output_file, result)