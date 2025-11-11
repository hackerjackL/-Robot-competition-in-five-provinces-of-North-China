#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append("/home/pi/TonyPi/") 
import cv2
import time
import math
import threading
import numpy as np
import hiwonder.Sonar as Sonar
import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
import RPi.GPIO as GPIO
import os
# 巡线
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__target_color = ('black',)
# 设置检测颜色
def setLineTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, (), 'SetVisualPatrolColor')


lab_data = None
servo_data = None


def load_config():
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


load_config()

# 面积过滤配置
min_area = 1      # 最小面积阈值，用于过滤噪声
max_area = 100    # 最大面积阈值，用于过滤大面积干扰

# 初始位置
def initMove():
    Board.setPWMServoPulse(1, 1100, 500)
    Board.setPWMServoPulse(2, 1394, 500)


line_centerx = -1


# 变量重置
def reset():
    global line_centerx
    global __target_color

    line_centerx = -1
    __target_color = ()


def setBuzzer(sleeptime):
    GPIO.setup(31, GPIO.OUT)  # 设置引脚为输出模式,BOARD编码31对应BCM编码6
    GPIO.output(31, 1)  # 设置引脚输出高电平
    time.sleep(sleeptime)  # 设置延时
    GPIO.output(31, 0)


# app初始化调用
def init():
    print("VisualPatrol Init")
    load_config()
    initMove()


__isRunning = False


# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")


# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("VisualPatrol Stop")


# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup("stand_low")
    print("VisualPatrol Exit")


# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 100:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓


distance = 99999
img_centerx = 320
j = 0


frame_count = 0  # 初始化帧计数器

def move():
    global distance, line_centerx, center_x_value, j
    for i in range(5):
        AGC.runActionGroup("left_move_fast")         # 向左移动
    while True:
        if distance != 99999:
            if j == 0 and distance <= 350:    # 如果距离小于350，则向右转向
                for i in range(5):
                    AGC.runActionGroup("right22")   # 向右转向
                for i in range(3):
                    AGC.runActionGroup("turn_right_fast")   # 向左转向
                AGC.runActionGroup("turn_left_fast")     # 向右转向    这个是个矫正，csdn有写，左右来回矫正防止找不到线
                j = j + 1
            elif j == 1 and distance <= 400:        # 如果距离小于400，则向左转向
                for i in range(8):
                    AGC.runActionGroup("left11")      # 向左转向
                for i in range(1):
                    AGC.runActionGroup("turn_left_fast")      # 向右转向
                    AGC.runActionGroup("turn_right_fast")            # 向左转向
                j = j + 1
            elif j == 2 and distance <= 300:
                for i in range(9):
                    AGC.runActionGroup("right_move_fast")      # 向右转向
                for i in range(2):
                    AGC.runActionGroup("turn_right_fast")        # 向左转向
            else:
                if __isRunning:
                    if line_centerx == -1:
                        time.sleep(0.01)
                    else:
                        if abs(line_centerx - img_centerx) <= 70:    
                            AGC.runActionGroup("go_forward_fast")                # 向前移动
                        elif line_centerx - img_centerx > 70:
                            AGC.runActionGroup("turn_right_fast")   
                        elif line_centerx - img_centerx < -70:
                            AGC.runActionGroup("turn_left_fast")
                else:
                    time.sleep(0.01)
        else:
            time.sleep(0.01)
            




# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

roi = [  # [ROI, weight]
    (240, 280, 0, 640, 0.1),
    (340, 380, 0, 640, 0.3),
    (440, 480, 0, 640, 0.6)
]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

size = (640, 480)

f = 1


# 注意：确保在调用run函数之前，所有必要的全局变量和函数都已经被正确定义和初始化。
def run(img):
    global line_centerx
    global __target_color
    global center_x_value
    global distance
    global f
    
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning or __target_color == ():
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)

    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0
    q = 50              # 定义一个变量q，值为50,偏差值，用于计算中心点

    # 将图像分割成上中下三个部分，这样处理速度会更快，更精确
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        area_max = 0
        areaMaxContour = 0
        for i in lab_data:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  # 对原图像和掩模进行位运算
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀

        cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找出所有轮廓
        # 统一应用面积范围过滤：只保留面积在 [min_area, max_area] 范围内的轮廓
        valid_cnts = [cnt for cnt in cnts if min_area <= cv2.contourArea(cnt) <= max_area]
        
        if len(valid_cnts) > 0:
            # 如果有多个符合条件的轮廓，选择面积最大的那个
            cnt_large, area = max([(cnt, cv2.contourArea(cnt)) for cnt in valid_cnts], key=lambda x: x[1])
        else:
            # 没有符合条件的轮廓
            cnt_large = None
            area = 0

        if cnt_large is not None:  # 如果轮廓不为空
            rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形
            box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1) * roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (255, 0, 0, 255), 2)  # 画出四个点组成的矩形
            # 获取矩形的对角点
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
            # 获取两条直线的斜率
            if box[1, 0] != box[2, 0]:
                k1 = (box[1, 1] - box[2, 1]) / (box[1, 0] - box[2, 0])
                if k1 != 0:
                    k2 = -1 / k1
                else:
                    k2 = None
            else:
                k2 = 0
            if k2 is not None:
                x1 = int(center_x + math.sqrt(q ** 2 / (k2 ** 2 + 1)))
                x2 = int(center_x - math.sqrt(q ** 2 / (k2 ** 2 + 1)))
                y1 = int(k2 * x1 + center_y - k2 * center_x)
                y2 = int(k2 * x2 + center_y - k2 * center_x)
            else:
                # 直线是竖直的，所以交点的 x 坐标就是直线的 x 坐标
                x = int(center_x)
                # 解出 y 坐标
                y1 = int(int(center_y) + q)
                y2 = int(int(center_y) - q)
                x1 = x2 = x
            # 绘制出新的坐标点
            cv2.circle(img, (x1, y1), 5, (255, 0, 0), -1)
            cv2.circle(img, (x2, y2), 5, (255, 255, 0), -1)
            # 按权重不同对上中下三个形心坐标点进行求和
            weight_sum += r[4]
            if distance > 400:
                centroid_x_sum += center_x * r[4]
            else:
                if center_x < 320 and x1 < x2:
                    centroid_x_sum += x2 * r[4]
                elif center_x < 320 and x1 > x2:
                    centroid_x_sum += x1 * r[4]
                elif center_x > 320 and x1 > x2:
                    centroid_x_sum += x2 * r[4]
                elif center_x > 320 and x1 < x2:
                    centroid_x_sum += x1 * r[4]

    if weight_sum != 0:
        # 绘制出这个最终坐标点
        cv2.circle(img, (line_centerx, int(center_y)), 10, (255, 0, 255), -1)
        # 求最终得到的中心点(这个点将会是机器人的轨迹点)
        line_centerx = int(centroid_x_sum / weight_sum)
        center_x_value = center_x
    else:
        line_centerx = -1
        center_x_value = -1
    
    
    return img
    

if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *

    # 加载参数
    param_data = np.load(calibration_param_path + '.npz')

    # 获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    init()
    start()
    __target_color = ('white',)
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()
    distance_list = []
    s = Sonar.Sonar()
    s.startSymphony()
    AGC.runActionGroup('stand')
    while True:
        ret, img = my_camera.read()
        if ret:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)  #刷新帧
            if key == 27:
                break
          
        else:
            time.sleep(0.01)

        distance_list.append(s.getDistance())

        if len(distance_list) >= 6:
            distance = int(round(np.mean(np.array(distance_list))))
            print(distance, 'mm')
            distance_list = []

        time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()



