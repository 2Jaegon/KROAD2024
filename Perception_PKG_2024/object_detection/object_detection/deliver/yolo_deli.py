import sys
import os

pwd_ = "/ros2_ws/q_pt_files/delivery"
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

from models.experimental import attempt_load
from utils.general import non_max_suppression

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Int16
from std_msgs.msg import Float64MultiArray

class ObjectDetection(Node):
    def __init__(self, img_size: Union[Tuple[int, int], None] = (640, 640)):
        super().__init__("ObjectDetection")
        self.planning_flag = 0  
        # Parameters
        self.declare_parameter("weights", pwd_ + "/bestofbest.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("classes_path", pwd_ + "/bestofbest.txt", ParameterDescriptor(description="Class labels file"))
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
        self.previous_classes = None
        self.camera_matrix = np.array([[512.850163068393,0,310.623520250903],
                                       [0,512.928940964189,257.780113020875],
                                       [0, 0, 1]], dtype=np.float64)
        self.distortion_coeff = np.array([0.0817915141024021,-0.176399735400575, 0, 0], dtype=np.float32)

        if self.classes_path:
            if not os.path.isfile(self.classes_path):
                raise FileExistsError(f"classes file not found at {self.classes_path}")
            self.classes = self.parse_classes_file(self.classes_path)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using {self.device} device")

        # Timer callback
        self.frequency = 30  # Hz : 1초에 30번 타이머 콜백 실행
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)

        # Publishers for result image
        best_effort_qos = QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1
            )
        
        reliable_qos = QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=1
            )
        self.visualization_publisher = self.create_publisher(Image, "/deli_result", best_effort_qos)
        self.vis_msg = Image()

        self.planning_mission_sub = self.create_subscription(Int16, "/Planning/mission", self.mission_cb, best_effort_qos)

        # Publishers for detection
        self.detect_publisher = self.create_publisher(Detection2DArray, "/deli_detect", best_effort_qos)
        self.deli_sign = self.create_publisher(Float64MultiArray, "/deli_signs", best_effort_qos)

        # Realsense package
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/video3', self.img_callback, best_effort_qos)
        self.stamp_time = None

        self.model = attempt_load(self.weights, map_location_=self.device)

        # Subscribe to cluster count topic
        self.cluster_count_sub = self.create_subscription(
            Int16, "/cluster_count", self.cluster_count_callback, best_effort_qos)

        self.cluster_count = 0  # Initialize cluster count

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



    def check_aspect_ratio(self, bbox_coords):
        """가로세로 비율을 확인하는 함수"""
        width = bbox_coords[2] - bbox_coords[0]
        height = bbox_coords[3] - bbox_coords[1]
        if height == 0:
            return False
        aspect_ratio = width / height
        return 0.67 <= aspect_ratio <= 1.32


    def mission_cb(self, msgs):
        self.planning_flag = msgs.data










    def cluster_count_callback(self, msg: Int16):
        self.cluster_count = msg.data  # Save the received cluster count

    def YOLOv7_detect(self):
        
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
        img = np_img_resized.transpose((2, 0, 1))[::-1]  # bgr 이미지를 rgb 순서로 바꿔준다.
        img = torch.from_numpy(np.ascontiguousarray(img))
        img = img.float()
        img /= 255  # 이미지에서 텐서로 바꾼 뒤 그 값들을 0~1 사이 값으로 바꿔준다.
        img = img.to(self.device)  # 여기까지 모델에 넣어주기 전까지의 과정.
        box_info = []
        classes = []
        detections = self.inference(img)
        detections[:, :4] = self.rescale([h_scaled, w_scaled], detections[:, :4], [h_orig, w_orig])
        detections[:, :4] = detections[:, :4].round()

        if len(detections) == 0:
            print("🥶🥶yolo=OK, nothing_detect🥶🥶")
        else:
            print("**************yolo=OK, detected****************")
            #### detection2darray로 메세지 보내기 ####
            box_info = [[float(x1), float(y1), float(x2), float(y2), float(conf), int(c)]
                        for x1, y1, x2, y2, conf, c in detections[:, :].tolist()
                        if not (self.planning_flag == 9 and int(c) in [1, 2, 0])]

            box_info.sort(key=lambda x: (x[2] - x[0]) * (x[3] - x[1]), reverse=True)
            valid_boxes = [box for box in box_info if self.check_aspect_ratio(box)]
            # 이미지에 바운딩 박스 그리기
            for bbox_coords in valid_boxes:
                x1, y1, x2, y2 = int(bbox_coords[0]), int(bbox_coords[1]), int(bbox_coords[2]), int(bbox_coords[3])
                label = f'{int(bbox_coords[5])+1}'
                cv2.rectangle(np_img_orig, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.putText(np_img_orig, label, (x1-10, y1+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # YOLO 감지 결과 이미지를 /yolo_result로 퍼블리시
            vis_msg = self.bridge.cv2_to_imgmsg(np_img_orig, encoding="bgr8")
            self.visualization_publisher.publish(vis_msg)

            # 클러스터 개수에 따른 처리
            
            if self.cluster_count == 1 and len(valid_boxes) >= 1:
                # 클러스터링 1개일 때: 가장 큰 YOLO 감지 결과 하나만 퍼블리시
                self.publish_deli_sign([valid_boxes[0]])
            elif self.cluster_count == 2 and len(valid_boxes) >= 2:
                # 클러스터링 2개일 때: 가장 큰 것과 두 번째 YOLO 감지 결과 퍼블리시
                self.publish_deli_sign([valid_boxes[0], valid_boxes[1]])
            elif self.cluster_count >= 3:
                # 클러스터링 3개 이상일 때: 기존 로직대로 모든 YOLO 감지 결과 퍼블리시
                self.publish_deli_sign(valid_boxes)
            elif self.cluster_count == 2 and len(valid_boxes) == 1:
                # psm7특별예외처리
                self.publish_deli_sign(valid_boxes)
                

            #### /yolo_detect로 퍼블리시 ####
            detection_array = Detection2DArray()
            detection_array.header.stamp = self.get_clock().now().to_msg()

            for bbox_coords in box_info:
                detection = Detection2D()
                bbox = BoundingBox2D()
                bbox.center.position.x = (bbox_coords[0] + bbox_coords[2]) / 2  # x 중심
                bbox.center.position.y = (bbox_coords[1] + bbox_coords[3]) / 2  # y 중심
                bbox.size_x = bbox_coords[2] - bbox_coords[0]  # 너비
                bbox.size_y = bbox_coords[3] - bbox_coords[1]  # 높이

                detection.bbox = bbox

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(bbox_coords[5]))  # 클래스 ID
                hypothesis.hypothesis.score = bbox_coords[4]  # confidence score

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

            # YOLO 감지 결과를 퍼블리시
            self.detect_publisher.publish(detection_array)
           

    def publish_deli_sign(self, box_info):
        """클러스터링 개수에 따라 YOLO 감지 결과를 `/deli_signs`로 퍼블리시하는 함수"""
        classes = []
        class_labels = ["A1", "A2", "A3", "B1", "B2", "B3"]

        for bbox_coords in box_info:
            class_id = bbox_coords[5]
            if class_id < len(class_labels):
                classes.append(float(class_id + 1))  # YOLO 클래스 정보를 저장

        # 클래스 정보 퍼블리시
        if len(classes) <= 3:
            float_array_msg = Float64MultiArray()
            float_array_msg.data = classes
            self.deli_sign.publish(float_array_msg)
            print("🔷YOLO🔷:", classes)

    def timer_callback(self):
        if self.sub_img_flag:
            self.YOLOv7_detect()

    def img_callback(self, data):
        self.stamp_time = data.header.stamp
        self.image = self.bridge.imgmsg_to_cv2(data)
        if self.image.ndim == 2 or self.image.shape[2] == 1:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BayerBG2BGR)
        video = cv2.resize(self.image, (640, 480))
        self.video =cv2.undistort(video, self.camera_matrix, self.distortion_coeff)
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