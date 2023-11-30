import torch
import numpy as np
import cv2
import time
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov5.models.experimental import attempt_load
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Bool, Float64
from math import atan, degrees
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qconjugate
import os


class ObjectDetection:
    """
    Class implements Yolo5 model to make inferences on a video using OpenCV.
    """
    
    def __init__(self):

        self.model = self.load_model()
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("\n\nDevice Used:",self.device)



    def load_model(self):
        """
        Loads Yolo5 pretrained model.
        :return: Trained Pytorch model.
        """
        my_path = os.path.expanduser('~/vrx_ws/src/aquabot_theboys/resource/detectallobjects.pt')
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=my_path,force_reload=True)
        return model


    def score_frame(self, frame):
        """
        Takes a single frame as input, and scores the frame using yolo5 model.
        :param frame: input frame in numpy/list/tuple format.
        :return: Labels and Coordinates of objects detected by model in the frame.
        """
        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
     
        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord


    def class_to_label(self, x):
        """
        For a given label value, return corresponding string label.
        :param x: numeric label
        :return: corresponding string label
        """
        return self.classes[int(x)]


    def plot_boxes(self, results, frame):
        """
        Takes a frame and its results as input, and plots the bounding boxes and label on to the frame.
        :param results: contains labels and coordinates predicted by model on the given frame.
        :param frame: Frame which has been scored.
        :return: Frame with bounding boxes and labels ploted on it.
        """
        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.2:
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

        return frame




class ObjectDetectionNode:
    def __init__(self):
        self.node = rclpy.create_node('object_detection_node')
        self.detection = ObjectDetection()
        self.bridge = CvBridge()
        self.current_orientation = 0.0  # Initial orientation
        self.object_detected_pub = self.node.create_publisher(Bool, 'object_detected', 10)
        self.object_position_pub = self.node.create_publisher(Float64, 'object_position', 10)
        self.node.create_subscription(Image, "/wamv/sensors/cameras/main_camera_sensor/image_raw", self.image_callback, 10)
        self.node.create_subscription(Imu, "/wamv/sensors/imu/imu/data", self.imu_callback, 10)

    def imu_callback(self, msg):
        quaternion = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )
        _, _, self.current_orientation = quat2euler(qconjugate(quaternion))

        #transform orientation from quaternion to euler

    def image_callback(self, msg):
        start_time = time.perf_counter()
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.detection.score_frame(frame)
        frame = self.detection.plot_boxes(results, frame)
        end_time = time.perf_counter()
        fps = 1 / np.round(end_time - start_time, 3)
        cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
        cv2.imshow("img", frame)
        cv2.waitKey(1)

        is_detected_msg = Bool()
        x_position_msg = Float64()

        # Convert classes dictionary to list
        classes_list = list(self.detection.classes.values())

        # Check if any objects are detected
        if len(results[0]) > 0:
            for i in range(len(results[0])):
                # Check if the detected object is a 'red boat'
                if int(results[0][i]) == classes_list.index('red boat'):
                    is_detected_msg.data = True
                    # Calculate center of the box
                    x1, y1, x2, y2 = results[1][i][:4]
                    x_center_norm = (x1 + x2) / 2.0
                    # Convert in pixels
                    x_center_pixel = x_center_norm * frame.shape[1]
                    # Calculate difference from the frame center
                    diff = x_center_pixel - frame.shape[1] / 2
                    # Convert difference to angle
                    angle = atan(diff / frame.shape[1])
                    x_position_msg.data = angle  
                    break
            else:
                is_detected_msg.data = False
                x_position_msg.data = 0.0  # No angle to calculate
        else:
            is_detected_msg.data = False
            x_position_msg.data = 0.0  # No angle to calculate

        self.object_detected_pub.publish(is_detected_msg)
        self.object_position_pub.publish(x_position_msg)




rclpy.init()
node = ObjectDetectionNode()


rclpy.spin(node.node)

# Free ressources
cv2.destroyAllWindows()
node.node.destroy_node()
rclpy.shutdown()