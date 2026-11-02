#yolo

import sys
import os
from collections import deque, Counter

pwd="/ros2_ws/q_pt_files/traffic/"
print(pwd)
sys.path.append(pwd)
from typing import Tuple, Union
from rclpy.qos import qos_profile_sensor_data, QoSProfile

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image#, CompressedImage, CameraInfo
from cv_bridge import CvBridge

from utils.visualizer import draw_detections
from models.experimental import attempt_load
from utils.general import non_max_suppression

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose


def adjust_white_balance_adaptive(image_bgr, target_temp=4000, threshold=100):
    """
    화이트 밸런스를 조정하되, 현재 화이트 밸런스를 검사하여 기준치 이상이면 강제로 보정하고,
    기준치 미만이면 현재 밸런스의 1/10배만큼만 조정합니다.

    Parameters:
        image_bgr (numpy.ndarray): 입력 BGR 이미지.
        target_temp (int): 목표 색 온도 (기본값: 4000K).
        threshold (float): 화이트 밸런스 편차의 임계치. 임계치를 초과하면 전체 보정, 미만이면 부분 보정.

    Returns:
        numpy.ndarray: 보정된 BGR 이미지.
    """
    # BGR 이미지를 YCrCb로 변환
    image_ycrcb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2YCrCb)
    
    # 채널 분리
    y_channel, cr_channel, cb_channel = cv2.split(image_ycrcb)

    # Cr과 Cb 채널의 평균값 계산
    cr_mean = np.mean(cr_channel)
    cb_mean = np.mean(cb_channel)

    # 화이트 밸런스 편차 계산 (Cr과 Cb의 평균값이 128에서 얼마나 벗어났는지)
    cr_deviation = cr_mean - 128
    cb_deviation = cb_mean - 128
    total_deviation = (cr_deviation*abs(cr_deviation) + cb_deviation*abs(cb_deviation))

    # print(f"Cr 평균: {cr_mean:.2f}, Cb 평균: {cb_mean:.2f}, 편차: {total_deviation:.2f}")

    # 임계치 기준으로 보정 강도 결정
    if total_deviation < threshold:
        # print("임계치를 초과하여 전체 이미지에 화이트 밸런스 강제 보정 적용.")
        # 전체 보정
        adjustment_factor = target_temp / 4000  # 여기서는 target_temp가 4000K로 고정되어 있어 1
    else:
        # print("임계치 미만으로 현재 밸런스의 1/10배만큼 보정 적용.")
        # 부분 보정 (1/10배)
        adjustment_factor = (target_temp / 4000) * 0.1

    # Cr과 Cb 채널을 float32로 변환하여 보정 연산 수행
    cr_float = cr_channel.astype(np.float32)
    cb_float = cb_channel.astype(np.float32)

    # Cr과 Cb 채널 보정
    cr_adjust = cr_float - cr_deviation * adjustment_factor
    cb_adjust = cb_float - cb_deviation * adjustment_factor

    # 클리핑하여 값이 0~255 사이로 유지되도록 함
    cr_adjust = np.clip(cr_adjust, 0, 255).astype(np.uint8)
    cb_adjust = np.clip(cb_adjust, 0, 255).astype(np.uint8)

    # 보정된 Cr, Cb 채널을 다시 Y 채널과 합침
    adjusted_ycrcb = cv2.merge([y_channel, cr_adjust, cb_adjust])

    # YCrCb 이미지를 다시 BGR로 변환
    adjusted_bgr = cv2.cvtColor(adjusted_ycrcb, cv2.COLOR_YCrCb2BGR)

    return adjusted_bgr

class ObjectDetection(Node):
    def __init__(self, img_size: Union[Tuple[int, int], None] = (640, 640)):
        super().__init__("ObjectDetection")
        # Parameters
        self.declare_parameter("weights", pwd+"v9_rare_aug_e.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("classes_path", pwd+"traffic.txt", ParameterDescriptor(description="Class labels file"))
        self.declare_parameter("conf_thres", 0.25, ParameterDescriptor(description="Confidence threshold"))
        self.declare_parameter("iou_thres", 0.25, ParameterDescriptor(description="IOU threshold"))
        self.declare_parameter("img_size", 640, ParameterDescriptor(description="Image size"))
       
        self.weights = self.get_parameter("weights").get_parameter_value().string_value
        self.classes_path = self.get_parameter("classes_path").get_parameter_value().string_value
        self.conf_thres = self.get_parameter("conf_thres").get_parameter_value().double_value
        self.iou_thres = self.get_parameter("iou_thres").get_parameter_value().double_value
        self.img_size = self.get_parameter("img_size").get_parameter_value().integer_value

        self.image = None
        self.video = None
        self.img_size = img_size
        self.sub_img_flag = False

        self.camera_matrix = np.array([[496.425026737739, 0, 336.172743100203],
                                       [0, 486.698010854496, 256.673958587427],
                                       [0, 0, 1]], dtype=np.float64)
        self.distortion_coeff = np.array([-0.354715335015581, 0.133876681524426, 0, 0], dtype=np.float32)

        if self.classes_path:
            if not os.path.isfile(self.classes_path):
                raise FileExistsError(f"classes file not found at {self.classes_path}")
            self.classes = self.parse_classes_file(self.classes_path)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using {self.device} device")

        # 클러스터링을 위한 큐 추가
        self.class_queue = deque(maxlen=40)  # 최대 40개의 클래스 정보를 유지하는 큐

        if self.classes_path:
            if not os.path.isfile(self.classes_path):
                raise FileExistsError(f"classes file not found at {self.classes_path}")
            self.classes = self.parse_classes_file(self.classes_path)

        # Timer callback
        self.frequency = 30   # Hz : 1초에 30번 타이머 콜백 실행
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        # Publishers for result image
        best_effort_qos = QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        reliable_qos = QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=1
        )

        self.visualization_publisher = self.create_publisher(Image, "/traffic_result", best_effort_qos)
        self.vis_msg = Image()

        # Publishers for detection
        self.detect_publisher = self.create_publisher(Detection2DArray, "/traffic_detect", best_effort_qos)

        # Realsense package
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/video4', self.img_callback, best_effort_qos)
        self.stamp_time = None

        self.model = attempt_load(self.weights, map_location_=self.device)

    def parse_classes_file(self, path):
        classes = []
        with open(path, "r") as f:
            for line in f:
                line = line.replace("\n", "")
                classes.append(line)
        return classes

    def inference(self, img: torch.Tensor):
        img = img.unsqueeze(0)

        with torch.no_grad():
            pred_results = self.model(img)[0]

        detections = non_max_suppression(
            pred_results, conf_thres=self.conf_thres, iou_thres=self.iou_thres
        )
        if detections:
            detections = detections[0]
        return detections
    
    def rescale(self, ori_shape: Tuple[int, int], boxes: Union[torch.Tensor, np.ndarray],
                target_shape: Tuple[int, int]):
        
        xscale = target_shape[1] / ori_shape[1]
        yscale = target_shape[0] / ori_shape[0]

        boxes[:, [0, 2]] *= xscale
        boxes[:, [1, 3]] *= yscale

        return boxes

    def YOLOv7_detect(self):
        print("성공!")
        np_img_orig = self.video.copy()

        # 이미지 채널수를 3채널로 바꿔주는 과정
        if len(np_img_orig.shape) == 2:
            np_img_orig = np.stack([np_img_orig] * 3, axis=2)
        
        h_orig, w_orig, c = np_img_orig.shape

        if c == 1:
            np_img_orig = np.concatenate([np_img_orig] * 3, axis=2)
            c = 3
        
        # 이미지 사이즈를 640*640으로 바꿔준다.
        w_scaled, h_scaled = self.img_size

        np_img_resized = cv2.resize(np_img_orig, (w_scaled, h_scaled))

        # torch tensor로 변환하는 과정
        img = np_img_resized.transpose((2, 0, 1))[::-1] #bgr 이미지를 rgb 순서로 바꿔준다.
        img = torch.from_numpy(np.ascontiguousarray(img))
        img = img.float()
        img /= 255 # 이미지에서 텐서로 바꾼 뒤 그 값들을 0~1 사이 값으로 바꿔준다.
        img = img.to(self.device) # 여기까지 모델에 넣어주기 전까지의 과정. 

        detections = self.inference(img)
        detections[:, :4] = self.rescale(
            [h_scaled, w_scaled], detections[:, :4], [h_orig, w_orig])
        detections[:, :4] = detections[:, :4].round()

        if len(detections) == 0:
            print("nothing_detect")
        else:
            box_info = [[float(x1), float(y1), float(x2), float(y2), float(conf), int(c)]
                        for x1, y1, x2, y2, conf, c in detections[:, :].tolist()]

            my_bbox = Detection2DArray()

            classes = []
            self.class_queue=deque(maxlen=40)
            for bbox_coords in box_info:
                self.class_queue.append(str(bbox_coords[5]))

                # 빈도수가 가장 높은 클래스 결정
            most_common_class = Counter(self.class_queue).most_common(1)[0][0]
            print(f"Most common class: {most_common_class}")

            # 가장 빈도수가 높은 클래스의 바운딩 박스만 퍼블리시하기 위한 필터링
            filtered_detections = [bbox_coords for bbox_coords in box_info if str(bbox_coords[5]) == most_common_class]

            my_bbox = Detection2DArray()

            for bbox_coords in filtered_detections:
                detection = Detection2D()
                bbox = BoundingBox2D()
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(bbox_coords[5])
                hypothesis.hypothesis.score = bbox_coords[4]

                bbox.center.position.x = bbox_coords[0]
                bbox.center.position.y = bbox_coords[1]
                bbox.size_x = bbox_coords[2]
                bbox.size_y = bbox_coords[3]

                detection.results.append(hypothesis)
                detection.bbox = bbox
                my_bbox.detections.append(detection)

            # 필터링된 바운딩 박스들만 퍼블리시
            my_bbox.header.stamp = self.stamp_time
            self.detect_publisher.publish(my_bbox)

        if self.visualization_publisher:
            bboxes = [[int(x1), int(y1), int(x2), int(y2)]
                        for x1, y1, x2, y2 in detections[:, :4].tolist()]
            classes = [int(c) for c in detections[:, 5].tolist()]
            conf = [float(c) for c in detections[:, 4].tolist()]
            
            vis_img = draw_detections(np_img_orig, bboxes, classes, self.classes, conf)
            # vis_img = draw_dts(np_img_orig, bboxes, classes, self.classes, conf)
            self.vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            self.visualization_publisher.publish(self.vis_msg)
            # print("띄웠음")

            cv2.imshow('traffic', vis_img)
            cv2.waitKey(1)


    def timer_callback(self):
        if self.sub_img_flag:
            # print("YOLO node 시도중",end=" ")
            self.YOLOv7_detect()

    def img_callback(self, data):
        self.stamp_time = data.header.stamp
        self.image = self.bridge.imgmsg_to_cv2(data)
        if self.image.ndim == 2 or self.image.shape[2] == 1:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BayerBG2BGR)
        self.video = cv2.resize(self.image, (640, 480))
        # self.video = cv2.undistort(video, self.camera_matrix, self.distortion_coeff)
        # self.video = adjust_white_balance_adaptive(video,4000,0)
        self.sub_img_flag = True
        

def main(args=None):
    """Run the main function."""
    rclpy.init(args=args)
    with torch.no_grad():
        node = ObjectDetection()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()