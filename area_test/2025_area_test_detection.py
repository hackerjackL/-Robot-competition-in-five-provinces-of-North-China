#!/usr/bin/python3
# coding=utf8
"""
面积检测测试脚本
基于2025巡线.py，只做最大面积检测，将结果保存到JSON文件
"""
import sys
import os
import cv2
import time
import math
import json
import numpy as np
import hiwonder.Misc as Misc
import hiwonder.Camera as Camera
import hiwonder.yaml_handle as yaml_handle
from datetime import datetime

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

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

__isRunning = False

def start():
    global __isRunning
    __isRunning = True
    print("Area Test Detection Start")

def stop():
    global __isRunning
    __isRunning = False
    print("Area Test Detection Stop")

# ROI区域配置
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

def detect_max_area(img):
    """
    检测图像中的最大面积轮廓
    返回每个ROI区域的最大面积
    """
    global __target_color
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or __target_color == ():
        return None
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    
    roi_areas = []  # 存储每个ROI区域的最大面积
    n = 0
    
    # 将图像分割成上中下三个部分
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        
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
        
        # 计算所有轮廓的面积
        areas = [cv2.contourArea(cnt) for cnt in cnts if cv2.contourArea(cnt) > 0]
        
        if len(areas) > 0:
            max_area = max(areas)
            roi_areas.append({
                'roi_index': n - 1,
                'max_area': float(max_area),
                'contour_count': len(areas)
            })
        else:
            roi_areas.append({
                'roi_index': n - 1,
                'max_area': 0.0,
                'contour_count': 0
            })
    
    # 计算所有ROI区域的最大面积
    all_max_area = max([r['max_area'] for r in roi_areas]) if roi_areas else 0.0
    
    return {
        'roi_areas': roi_areas,
        'global_max_area': float(all_max_area),
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    }

def save_area_data(area_data, output_dir='area_test'):
    """
    将面积数据保存到JSON文件，使用时间戳命名
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 使用时间戳作为文件名（精确到毫秒）
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    filename = os.path.join(output_dir, f'2025_area_data_{timestamp}.json')
    
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(area_data, f, indent=2, ensure_ascii=False)
    
    print(f"Area data saved to: {filename}")
    return filename

if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *
    
    # 加载参数
    param_data = np.load(calibration_param_path + '.npz')
    
    # 获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    start()
    __target_color = ('white',)
    
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()
    
    frame_count = 0
    save_interval = 30  # 每30帧保存一次数据
    
    print("Area detection started. Press 's' to save current frame, 'q' to quit")
    
    while True:
        ret, img = my_camera.read()
        if ret:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            
            # 检测最大面积
            area_data = detect_max_area(frame)
            
            if area_data:
                # 在图像上显示面积信息
                cv2.putText(frame, f"Max Area: {area_data['global_max_area']:.2f}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                for i, roi_area in enumerate(area_data['roi_areas']):
                    cv2.putText(frame, f"ROI{i}: {roi_area['max_area']:.2f}", 
                               (10, 60 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.imshow('Area Detection', frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                # 手动保存当前帧的数据
                if area_data:
                    save_area_data(area_data)
            
            # 自动保存（每30帧）
            frame_count += 1
            if frame_count >= save_interval:
                if area_data:
                    save_area_data(area_data)
                frame_count = 0
        else:
            time.sleep(0.01)
    
    my_camera.camera_close()
    cv2.destroyAllWindows()
    stop()

