#!/usr/bin/env python3

# -*- coding: utf-8 -*- # 한글 주석쓰려면 이거 해야함
import rosbag
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from sklearn.linear_model import RANSACRegressor

bridge = CvBridge()

# 샤프닝 필터 커널
sharpening_kernel = np.array([[-1, -1, -1],
                              [-1,  9, -1],
                              [-1, -1, -1]])

# 샤프닝 필터 적용 함수
def apply_sharpening(image):
    return cv2.filter2D(image, -1, sharpening_kernel)

# 히스토그램 평활화 적용 함수
def apply_histogram_equalization(image):
    img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
    return cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

# CLAHE 적용 함수 (개선된 히스토그램 평활화)
def apply_clahe(image):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    img_yuv[:, :, 0] = clahe.apply(img_yuv[:, :, 0])
    return cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

# 가우시안 블러 적용 함수
def apply_gaussian_blur(image, kernel_size=(5, 5)):
    return cv2.GaussianBlur(image, kernel_size, 0)

# 밝기 및 대비 조절 함수
def adjust_brightness_contrast(image, brightness=0, contrast=0):
    beta = brightness
    alpha = contrast / 127 + 1
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

# 이진화 적용 함수
def apply_binarization(image, threshold=128):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray_image, threshold, 255, cv2.THRESH_BINARY)
    return cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)

# # 5단계 밝기 양자화 적용 함수
# def apply_brightness_quantization(image, levels=5):
#     gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     max_value = 255
#     step = max_value // levels
#     quantized_image = (gray_image // step) * step
#     return cv2.cvtColor(quantized_image, cv2.COLOR_GRAY2BGR)

def apply_brightness_thresholds(image, thresholds):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    quantized_image = np.zeros_like(gray_image)
    
    # Ensure thresholds are sorted
    thresholds = sorted(thresholds)
    
    # Apply thresholds
    for i, thresh in enumerate(thresholds):
        quantized_image[gray_image >= thresh] = thresh
    
    return cv2.cvtColor(quantized_image, cv2.COLOR_GRAY2BGR)

# 감마 조정 함수
def adjust_gamma(image, gamma=1.0):
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(256)]).astype("uint8")
    return cv2.LUT(image, table)


##############################################################################3

def grayscale(img): # 흑백이미지로 변환
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img, low_threshold, high_threshold): # Canny 알고리즘
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size): # 가우시안 필터
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def roi(image): # ROI 셋팅1 (영역 2개)
    x = int(image.shape[1])
    y = int(image.shape[0])

    # 한 붓 그리기
    # https://moon-coco.tistory.com/entry/OpenCV%EC%B0%A8%EC%84%A0-%EC%9D%B8%EC%8B%9D
    _shape = np.array(
        
        [[int(0*x), int(1*y)], 
         [int(0*x), int(0.1*y)], 
         [int(0.3*x), int(0.1*y)], 
         [int(0.3*x), int(1*y)], 
         [int(0.6*x), int(1*y)], 
         [int(0.6*x), int(0.1*y)],
         [int(0.99*x), int(0.1*y)], 
         [int(0.99*x), int(1*y)], 
         [int(0*x), int(1*y)]])

    mask = np.zeros_like(image)

    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)

    return masked_image

def region_of_interest(img, vertices, color3=(255,255,255), color1=255): # ROI 셋팅2 (영역 1개)

    mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
    
    if len(img.shape) > 2: # Color 이미지(3채널)라면 :
        color = color3
    else: # 흑백 이미지(1채널)라면 :
        color = color1
        
    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
    cv2.fillPoly(mask, vertices, color)
    
    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

def draw_lines(img, lines, color=[0, 0, 255], thickness=5): # 선 그리기
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    return lines

def weighted_img(img, initial_img, α=1, β=1., λ=0.): # 두 이미지 operlap 하기
    return cv2.addWeighted(initial_img, α, img, β, λ)


def bev_transform(image, vertices=None): # BEV (Bird's Eye View) 변환 수행
    h, w = image.shape[:2]
    src = np.float32([
            [10, 400],
            [w -20, 400],
            [0, h],
            [w, h]
        ])

    dst = np.float32([
            [0, 0],
            [w, 0],
            [0, h],
            [w, h]
        ])
    
    M = cv2.getPerspectiveTransform(src, dst)
    bev_image = cv2.warpPerspective(image, M, (w, h))
    return bev_image


def get_fitline(img, f_lines): # 대표선 구하기   
    lines = np.squeeze(f_lines)
    lines = lines.reshape(lines.shape[0]*2,2)
    rows,cols = img.shape[:2]
    output = cv2.fitLine(lines,cv2.DIST_L2,0, 0.01, 0.01)
    vx, vy, x, y = output[0], output[1], output[2], output[3]
    x1, y1 = int(((img.shape[0]-1)-y)/vy*vx + x) , img.shape[0]-1
    x2, y2 = int(((img.shape[0]/2+100)-y)/vy*vx + x) , int(img.shape[0]/2+100)
    
    result = [x1,y1,x2,y2]
    return result

def draw_fit_line(img, lines, color=[255, 0, 0], thickness=5): # 대표선 그리기
        cv2.line(img, (lines[0], lines[1]), (lines[2], lines[3]), color, thickness)

########################################################################3
# RANSAC을 이용한 선형 회귀
# RANSAC을 이용한 선형 회귀
def fit_line_ransac(x, y, min_samples=2):
    if len(x) < min_samples or len(y) < min_samples:
        return None
    ransac = RANSACRegressor(min_samples=min_samples)
    ransac.fit(x[:, np.newaxis], y)
    slope = ransac.estimator_.coef_[0]
    intercept = ransac.estimator_.intercept_
    if np.isinf(slope) or np.isinf(intercept) or np.isnan(slope) or np.isnan(intercept):
        return None
    return slope, intercept

def make_line_points(y1, y2, line):
    if line is None:
        return None
    slope, intercept = line
    if np.isinf(slope) or np.isinf(intercept) or np.isnan(slope) or np.isnan(intercept):
        return None
    x1 = int((y1 - intercept) / (slope +0.0001))
    x2 = int((y2 - intercept) / (slope +0.0001))
    y1 = int(y1)
    y2 = int(y2)
    return ((x1, y1), (x2, y2))

def draw_lane_lines_ransac(image, lines, color=[0, 0, 255], thickness=5):
    left_x, left_y, right_x, right_y = [], [], [], []
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1)
            if slope < 0:
                left_x.extend([x1, x2])
                left_y.extend([y1, y2])
            else:
                right_x.extend([x1, x2])
                right_y.extend([y1, y2])
    
    left_line = fit_line_ransac(np.array(left_x), np.array(left_y))
    right_line = fit_line_ransac(np.array(right_x), np.array(right_y))
    
    y1 = image.shape[0]
    y2 = int(y1 * 0.4)
    left_line_points = make_line_points(y1, y2, left_line)
    right_line_points = make_line_points(y1, y2, right_line)
    if left_line_points:
        cv2.line(image, left_line_points[0], left_line_points[1], color, thickness)
    if right_line_points:
        cv2.line(image, right_line_points[0], right_line_points[1], color, thickness)
        
#################################################33333
# 이동 중앙값 계산 함수
def moving_median(data, window_size):
    sorted_data = np.sort(data[-window_size:], axis=0)
    return np.median(sorted_data, axis=0)

def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# 직선 데이터 저장용 큐
class LineData:
    def __init__(self, max_length=15):
        self.max_length = max_length
        self.data = []

    def add_data(self, new_data):
        self.data.append(new_data)
        if len(self.data) > self.max_length:
            self.data.pop(0)

    def get_median(self):
        return moving_median(np.array(self.data), min(len(self.data), self.max_length))
    
    def get_average(self):
        if len(self.data) < self.max_length:
            return np.mean(self.data, axis=0)
        return moving_average(np.array(self.data), self.max_length)[-1]

# 좌우 직선 데이터 저장용 객체 생성
left_line_data = LineData()
right_line_data = LineData()

##########################################################3

def calculate_distance_from_center(line, image_center_x):
    x1, y1, x2, y2 = line
    line_center_x = (x1 + x2) / 2
    return abs(line_center_x - image_center_x)

def calculate_line_length(line):
    x1, y1, x2, y2 = line
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def select_representative_line(lines, image_center_x):
    if len(lines) < 2:
        return lines[0] if len(lines) == 1 else None
    
    # Calculate distance from the center for each line
    distances = [calculate_distance_from_center(line.reshape(-1), image_center_x) for line in lines]
    
    # Find the indices of the two closest lines to the center
    closest_indices = np.argsort(distances)[:2]
    
    # Select the two closest lines
    closest_lines = [lines[idx] for idx in closest_indices]
    
    # Calculate lengths of the two closest lines
    lengths = [calculate_line_length(line.reshape(-1)) for line in closest_lines]
    
    # Select the longer line
    representative_line = closest_lines[np.argmax(lengths)]
    return representative_line