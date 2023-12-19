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

    
    def __init__(self):

        self.model = self.load_model()
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'



    def load_model(self):
        """
        Charge notre modèle pré-entrainé.
        """

        my_path = os.path.expanduser('~/vrx_ws/src/aquabot_theboys/resource/detectallobjects.pt')
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=my_path,force_reload=True)
        return model


    def score_frame(self, frame):
        """
        Prend une seule image en entrée, et évalue l'image en utilisant le modèle.
        """

        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
     
        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord


    def class_to_label(self, x):
        """
        Retourne les différentes classes des objets.
        """

        return self.classes[int(x)]


    def plot_boxes(self, results, frame):
        """
        Cette fonction dessine des boites englobantes autour des objets détectés.
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
        #Interface entre ROS et OpenCV
        self.bridge = CvBridge()
        self.current_orientation = 0.0  
        self.object_detected_pub = self.node.create_publisher(Bool, 'object_detected', 10)
        self.object_position_pub = self.node.create_publisher(Float64, 'object_position', 10)
        self.node.create_subscription(Image, "/wamv/sensors/cameras/main_camera_sensor/image_raw", self.image_callback, 10)
        self.node.create_subscription(Imu, "/wamv/sensors/imu/imu/data", self.imu_callback, 10)

    def imu_callback(self, msg):
        """
        Transforme l'orientation de quaternion à euler.
        """
        quaternion = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ) 
        _, _, self.current_orientation = quat2euler(qconjugate(quaternion))

       

    def image_callback(self, msg):
        """
        On rentre dans ce callback chaque fois qu'une image est reçue depuis la caméra.
        """
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

        classes_list = list(self.detection.classes.values())

        # Vérifie si un objet est détecté
        if len(results[0]) > 0:
            for i in range(len(results[0])):
                # Vérifie si l'objet est un 'red boat'
                if int(results[0][i]) == classes_list.index('red boat'):
                    is_detected_msg.data = True
                    # Calcule le centre de la boite englobante
                    x1, y1, x2, y2 = results[1][i][:4]
                    x_center_norm = (x1 + x2) / 2.0
                    # Converti en pixels
                    x_center_pixel = x_center_norm * frame.shape[1]
                    # Calcule la différence depuis le centre de l'image
                    diff = x_center_pixel - frame.shape[1] / 2
                    # Converti la différence en angle
                    angle = atan(diff / frame.shape[1])
                    x_position_msg.data = angle  
                    break
            else:
                is_detected_msg.data = False
                # Pas d'angle à calculer
                x_position_msg.data = 0.0  
        else:
            is_detected_msg.data = False
            # Pas d'angle à calculer
            x_position_msg.data = 0.0  

        self.object_detected_pub.publish(is_detected_msg)
        self.object_position_pub.publish(x_position_msg)




rclpy.init()
node = ObjectDetectionNode()


rclpy.spin(node.node)

cv2.destroyAllWindows()
node.node.destroy_node()
rclpy.shutdown()