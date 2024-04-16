import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

class BoxTracking(Node):
    def __init__(self):
        super().__init__('box_tracking')
        self.get_logger().info('Box Tracking Node Started')
        self.create_subscription(Image, '/celconv_camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()  # This will convert ROS image messages to OpenCV formats
        self.yolo_model = YOLO("yolov8x.pt")

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image message to cv2 image: %s' % str(e))
            return
        
        results = self.yolo_model.track(cv_image, persist=True) # Perform object detection

        for det_box in results[0].boxes.data.tolist():
            confidence = det_box[5]

            if float(confidence) < 0.5:
                continue

            # x1, y1, x2, y2 = int(det_box[0]), int(det_box[1]), int(det_box[2]), int(det_box[3])
            # cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.putText(cv_image, str(det_box[4]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


        
        # Display the image using OpenCV
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)  # You need this line to allow OpenCV to process the window events

def main(args=None):
    rclpy.init(args=args)
    box_tracking = BoxTracking()
    rclpy.spin(box_tracking)
    box_tracking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
