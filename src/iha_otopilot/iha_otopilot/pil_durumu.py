import rclpy
from rclpy.node import Node

class PilDurumu(Node):
    def __init__(self):
        super().__init__('pil_durumu_node')


        self.sayac=0
        self.get_logger().info('pil durumu kontrolü ')

        self.timer=self.create_timer(1.0, self.timer_callback)

    
    def timer_callback(self):
        self.sayac += 1
        self.get_logger().info(f"pil durumu: , tekrar sayısı {self.sayac}")


def main(args=None):
    rclpy.init(args=args)
    node=PilDurumu()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()