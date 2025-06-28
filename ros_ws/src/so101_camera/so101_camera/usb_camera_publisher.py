import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class USBCameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera_publisher')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frequency', 30.0)
        
        camera_id = self.get_parameter('camera_id').value
        frequency = self.get_parameter('frequency').value
        
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(1.0/frequency, self.timer_callback)
        
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera with id {camera_id}")
            raise SystemExit
            
        self.bridge = CvBridge()
        self.get_logger().info(f"USB Camera {camera_id} started, publishing at {frequency} Hz.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(img_msg)
        else:
            self.get_logger().warn("Failed to capture frame from USB camera.")

def main(args=None):
    rclpy.init(args=args)
    usb_camera_publisher = USBCameraPublisher()
    rclpy.spin(usb_camera_publisher)
    usb_camera_publisher.cap.release()
    usb_camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 