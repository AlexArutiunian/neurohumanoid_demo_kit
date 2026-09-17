import rclpy
from rclpy.node import Node
from time import sleep

from sensor_msgs.msg import Image,CameraInfo

import cv2

import message_filters
import os
from pathlib import Path
import cv2
from cv_bridge import CvBridge
class ObjectDetector(Node):
    

    def __init__(self):
        super().__init__('object_detector')
        self.bridge=CvBridge()
        self.start()
    def start(self,num=0):
        self.num=num
        self.subscription_depth = self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.callback_color,
            10)
        self.subscription_color = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.callback_depth,
            10)
        
        
    def callback_color(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(f"data/image/current_frame_{self.num}.png", cv_image)
   
    def callback_depth(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite(f'data/depth/depth_{self.num}.png', cv_image)
def main(args=None):
    rclpy.init(args=args)
    obj = ObjectDetector()
    for i in range(10):
        obj.start()
        sleep(1)

if __name__ == '__main__':
    main()
