import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

class Camera(Node):
    def __init__(self):
        super().__init__('kamera_node')

        self.subscription=self.create_subscription(
            Image,
            '/kamera/goruntu',
            self.goruntu_callback,
            10
        )

        self.bridge=CvBridge()
        self.get_logger().info("camera ok")

    def goruntu_callback(self,msg):
        try:
            img=self.bridge.imgmsg_to_cv2(msg,"bgr")
            cv.imshow('camera',img)
            cv.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Hata: {e}")



def main(args=None):
    rclpy=rclpy.init(args=args)
    node=Camera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()

    
if __name__=='__main__':
    main()
        