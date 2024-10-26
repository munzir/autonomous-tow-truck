import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

class QrCodeDetectionNode(Node):
    # Constructor
    def __init__(self):
        super().__init__('qr_code_detection_node')
        # Subscription to the camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        # Publisher for the pose of the QR code
        self.publisher_ = self.create_publisher(Pose, '/qr_code/pose', 10)
        self.bridge = CvBridge()
    # Callback function for the camera image
    def image_callback(self, msg):
        # Convert the ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        distance, rotation_vector, translation_vector = self.detect_and_estimate_qr_code(cv_image)
        # rotation vector in euler angles
        if rotation_vector is not None:
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
            euler_angles = cv2.RQDecomp3x3(rotation_matrix)[0]
            self.get_logger().info(f"Rotation vector in Euler angles: {euler_angles}")
        if distance is not None:
            self.get_logger().info(f"Distance: {distance} meters")
            self.get_logger().info(f"Rotation vector: {rotation_vector}")
            self.get_logger().info(f"Translation vector: {translation_vector}")
            
        if translation_vector is not None:
            pose_msg = Pose()
            pose_msg.position.x = float(translation_vector[0])
            pose_msg.position.y = float(translation_vector[1])
            pose_msg.position.z = float(translation_vector[2])
            quaternion = self.rotation_vector_to_quaternion(rotation_vector)
            # self.get_logger().info(f"Rotation vector: {self.quaternion_to_rotation_vector(self.rotation_vector_to_quaternion(rotation_vector))}")
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]
            # Publishing the pose message
            self.publisher_.publish(pose_msg)
        # resetting the vectors to prevent the previous values from being published
        translation_vector, rotation_vector = None, None


    def detect_and_estimate_qr_code(self, image):
        # Detect QR code in the image
        qr_decoder = cv2.QRCodeDetector()
        data, bbox, _ = qr_decoder.detectAndDecode(image)
        if bbox is not None and data:
            self.get_logger().info(f"QR Code detected: {data}")
            bbox = np.int32(bbox)
            # Draw bounding box around the QR code
            cv2.polylines(image, [bbox], True, (0, 255, 0), 2)
            distance, rotation_vector, translation_vector = self.estimate_pose(bbox)
            return distance, rotation_vector, translation_vector
        return None, None, None

    def estimate_pose(self, bbox):
        # 3D object points of the QR code where the qr code is 0.16x16 meters
        object_points = np.array([
            [0.0, 0.0, 0.0],
            [0.16, 0.0, 0.0],
            [0.16, 0.16, 0.0],
            [0.0, 0.16, 0.0]
        ], dtype=np.float32)
        # 2D image points of the QR code
        image_points = bbox.reshape((4, 2)).astype(np.float32)
        calibration_data = np.load('/home/sarim/ros2_ws/src/qr_code_detection_pkg/qr_code_detection_pkg/camera_calibration.npz')
        camera_matrix = calibration_data['camera_matrix']
        dist_coeffs = calibration_data['dist_coeffs']
        # Estimate pose of the QR code
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points, image_points, camera_matrix, dist_coeffs)
        if success:
            distance = np.linalg.norm(translation_vector)
            return distance, rotation_vector, translation_vector
        return None, None, None

    def rotation_vector_to_quaternion(self, rotation_vector):
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        return quaternion
    def quaternion_to_rotation_vector(self, quaternion):
        rotation = R.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()
        rotation_vector, _ = cv2.Rodrigues(rotation_matrix)
        return rotation_vector

def main(args=None):
    # Initialize]ing the ROS 2 node
    rclpy.init(args=args)
    node = QrCodeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
