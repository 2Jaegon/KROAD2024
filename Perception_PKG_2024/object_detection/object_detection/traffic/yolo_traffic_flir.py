import sys
import os
from collections import deque, Counter

# 기존 코드 유지
pwd_="/ros2_ws/q_pt_files/traffic"
print(pwd_)
sys.path.append(pwd_)

from typing import Tuple, Union
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from utils.visualizer import draw_detections
from models.experimental import attempt_load
from utils.general import non_max_suppression
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose


class ObjectDetection(Node):
    def __init__(self, img_size: Union[Tuple[int, int], None] = (640, 640)):
        super().__init__("ObjectDetection")

        # 기존 파라미터 설정 유지
        self.declare_parameter("weights", pwd_+"/v9_rare_aug_e.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("classes_path", pwd_+"/traffic.txt", ParameterDescriptor(description="Class labels file"))
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

        # 카메라 매트릭스 설정 유지
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
        self.class_queue = deque(maxlen=80)  # 최대 40개의 클래스 정보를 유지하는 큐

        # 타이머 콜백 유지
        self.frequency = 30   # Hz
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        # 퍼블리셔 설정 유지
        best_effort_qos = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        reliable_qos = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=1
        )

        self.visualization_publisher = self.create_publisher(Image, "/yolo_result", best_effort_qos)
        self.vis_msg = Image()
        self.detect_publisher = self.create_publisher(Detection2DArray, "/traffic_detect", best_effort_qos)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/flir_camera/image_raw', self.img_callback, best_effort_qos)
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

        np_img_orig = self.video.copy()

        # 이미지 채널수를 3채널로 바꿔주는 과정
        if len(np_img_orig.shape) == 2:
            np_img_orig = np.stack([np_img_orig] * 3, axis=2)
        
        h_orig, w_orig, c = np_img_orig.shape

        if c == 1:
            np_img_orig = np.concatenate([np_img_orig] * 3, axis=2)
            c = 3
        
        w_scaled, h_scaled = self.img_size

        np_img_resized = cv2.resize(np_img_orig, (w_scaled, h_scaled))

        img = np_img_resized.transpose((2, 0, 1))[::-1]
        img = torch.from_numpy(np.ascontiguousarray(img))
        img = img.float()
        img /= 255
        img = img.to(self.device)

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
            self.vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            self.visualization_publisher.publish(self.vis_msg)

            cv2.imshow('frame', vis_img)
            cv2.waitKey(1)

    def timer_callback(self):
        if self.sub_img_flag:
            self.YOLOv7_detect()

    def img_callback(self, data):
        self.stamp_time = data.header.stamp
        self.image = self.bridge.imgmsg_to_cv2(data)
        if self.image.ndim == 2 or self.image.shape[2] == 1:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BayerBG2BGR)
        video = cv2.resize(self.image, (640, 480))
        self.video = cv2.undistort(video, self.camera_matrix, self.distortion_coeff)
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
