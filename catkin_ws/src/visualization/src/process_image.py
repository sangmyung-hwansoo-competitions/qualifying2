import rosbag
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import numpy as np

# 원본 bag 파일 경로
input_bag_path = '/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data.bag'
# 새로운 bag 파일 경로
output_bag_path = '/home/ubuntu/catkin_ws/src/integration/bag/usb_cam_recorded_data_process.bag'

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

# # 이진화 적용 함수
# def apply_binarization(image, threshold=128):
#     gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     _, binary_image = cv2.threshold(gray_image, threshold, 255, cv2.THRESH_BINARY)
#     return cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)

# 5단계 밝기 양자화 적용 함수
def apply_brightness_quantization(image, levels=5):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    max_value = 255
    step = max_value // levels
    quantized_image = (gray_image // step) * step
    return cv2.cvtColor(quantized_image, cv2.COLOR_GRAY2BGR)


# 감마 조정 함수
def adjust_gamma(image, gamma=1.0):
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(256)]).astype("uint8")
    return cv2.LUT(image, table)


# 새로운 bag 파일을 열기
output_bag = rosbag.Bag(output_bag_path, 'w')

first_frame = True

# 원본 bag 파일을 읽기
with rosbag.Bag(input_bag_path, 'r') as input_bag:
    for topic, msg, t in input_bag.read_messages(topics=['/usb_cam/image_raw']):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 전처리 함수들을 차례대로 적용
        # processed_image = apply_sharpening(cv_image)
        # processed_image = apply_histogram_equalization(processed_image)
        # processed_image = apply_clahe(processed_image)
        # processed_image = apply_gaussian_blur(processed_image)
        # processed_image = adjust_brightness_contrast(processed_image, brightness=10, contrast=30)
        
        
        ## 전처리 1안 ##
        processed_image = apply_gaussian_blur(cv_image)
        processed_image = apply_sharpening(processed_image)
        processed_image = adjust_brightness_contrast(processed_image, brightness=-20, contrast=10)
        ####################################3
        processed_image = apply_clahe(processed_image)
        
        
        #processed_image = adjust_gamma(cv_image, gamma=2.0)
        
        
        
        
        
        
        ###################시각화########################
        # # 첫 번째 프레임을 시각화하고 특정 키 입력 대기 (RGB)
        # if first_frame:
        #     cv2.imshow('First Frame', processed_image)
        #     while True:
        #         key = cv2.waitKey(0)
        #         if key == ord('q'):  # 'q' 키가 눌리면 루프 종료
        #             break
        #     cv_image = cv2.destroyAllWindows()
        #     first_frame = False
        
        
        # # 첫 번째 프레임을 시각화하고 특정 키 입력 대기 (Gray Scale)
        # if first_frame:
        #     gray_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)
        #     cv2.imshow('First Frame (Gray)', gray_image)
        #     while True:
        #         key = cv2.waitKey(0)
        #         if key == ord('q'):  # 'q' 키가 눌리면 루프 종료
        #             break
        #     cv2.destroyAllWindows()
        #     first_frame = False

        # 새로운 메시지를 생성
        new_msg = bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
        new_msg.header.stamp = t

        # 새로운 메시지를 bag 파일에 기록
        output_bag.write('/usb_cam/image_raw', new_msg, t)

# 새로운 bag 파일 닫기
output_bag.close()
