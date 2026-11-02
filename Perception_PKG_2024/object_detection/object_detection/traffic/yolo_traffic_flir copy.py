# 배달 미션에서 사용하는 detector
# blackflys 를 subsribe 받아서 detection을 수행한다.

import sys
import os


# pwd_=os.getcwd()+"/src/version-up-2024/object_detection/object_detection"
pwd_="/ros2_ws/src/perception_ws/object_detection/object_detection"
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
from sensor_msgs.msg import Image#, CompressedImage, CameraInfo
from cv_bridge import CvBridge

from utils.visualizer import draw_detections
from models.experimental import attempt_load
from utils.general import non_max_suppression

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose

class ObjectDetection(Node):
    def __init__(self, img_size: Union[Tuple[int, int], None] = (640, 640)):
        super().__init__("ObjectDetection")
        # Parameters
        self.declare_parameter("weights", pwd_+"/bestofbest.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("classes_path", pwd_+"/bestofbest.txt", ParameterDescriptor(description="Class labels file"))
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
        self.mission_status = 0  # mission 상태를 저장하는 변수 추가

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

        self.visualization_publisher = self.create_publisher(Image, "/yolo_result", best_effort_qos)
        self.vis_msg = Image()

        # Publishers for detection
        self.detect_publisher = self.create_publisher(Detection2DArray, "/yolo_detect", best_effort_qos)

        # Realsense package
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/video4', self.img_callback, best_effort_qos)
        self.stamp_time = None

        # Mission 상태를 구독하는 subscriber 추가
        self.status_subscriber_ = self.create_subscription(
            std_msgs.msg.Int16, "/Planning/mission", 
            self.status_callback, best_effort_qos
        )

        self.model = attempt_load(self.weights, map_location_=self.device)

    def status_callback(self, msg):
        # 미션 상태 값을 업데이트
        self.mission_status = msg.data

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

        # 미션 상태가 777일 경우 이미지의 우측 4분의 1 크롭
        if self.mission_status == 777:
            height, width, _ = np_img_orig.shape
            np_img_orig = np_img_orig[:, width // 2:, :]  # 우측 절반을 크롭

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
            #### detection2darray로 메세지 보내기 ####
            box_info = [[float(x1), float(y1), float(x2), float(y2), float(conf), int(c)]
                        for x1, y1, x2, y2, conf, c in detections[:, :].tolist()]

            my_bbox = Detection2DArray()

            classes = []
            for bbox_coords in box_info:
                detection = Detection2D()
                bbox = BoundingBox2D()
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(bbox_coords[5])
                hypothesis.hypothesis.score = bbox_coords[4]

                classes.append(hypothesis.hypothesis.class_id)
                
                bbox.center.position.x = bbox_coords[0]
                bbox.center.position.y = bbox_coords[1]
                bbox.size_x = bbox_coords[2]
                bbox.size_y = bbox_coords[3]

                detection.results.append(hypothesis)
                detection.bbox = bbox
                my_bbox.detections.append(detection)

            print(classes)
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
            # print("띄웠음")

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

    def __init__(self, img_size: Union[Tuple[int, int], None] = (640, 640)):
        super().__init__("ObjectDetection")
        # Parameters
        self.declare_parameter("weights", pwd_+"/bestofbest.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("classes_path", pwd_+"/bestofbest.txt", ParameterDescriptor(description="Class labels file"))
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


        self.visualization_publisher = self.create_publisher(Image, "/yolo_result", best_effort_qos)
        self.vis_msg = Image()

        # Publishers for detection
        self.detect_publisher = self.create_publisher(Detection2DArray, "/yolo_detect", best_effort_qos)

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
            #### detection2darray로 메세지 보내기 ####
            box_info = [[float(x1), float(y1), float(x2), float(y2), float(conf), int(c)]
                        for x1, y1, x2, y2, conf, c in detections[:, :].tolist()]

            my_bbox = Detection2DArray()

            classes = []
            for bbox_coords in box_info:
                detection = Detection2D()
                bbox = BoundingBox2D()
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(bbox_coords[5])
                hypothesis.hypothesis.score = bbox_coords[4]

                classes.append(hypothesis.hypothesis.class_id)
                
                bbox.center.position.x = bbox_coords[0]
                bbox.center.position.y = bbox_coords[1]
                bbox.size_x = bbox_coords[2]
                bbox.size_y = bbox_coords[3]

                detection.results.append(hypothesis)
                detection.bbox = bbox
                my_bbox.detections.append(detection)

            print(classes)
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