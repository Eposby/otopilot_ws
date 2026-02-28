import rclpy
from rclpy.node import Node
from iha_messages.msg import HedefBilgi

class HedefDurum(Node):
    def __init__(self):
        super().__init__('hedef_durum')

        self.publisher=self.create_publisher(HedefBilgi, 'kamera/hedef_bilgi',10)
        self.timer=self.create_timer(1.0, self.veri_gönder)
        self.subscription=self.create_subscription(
            HedefBilgi,
            'kamera/hedef_bilgi',
            self.veri_alindi,
            10
        )

    def veri_alindi(self,msg):
        if msg.hedef_takip:
            self.get_logger().info(
                f"hedef x: {msg.center_x}, hedef y: {msg.center_y}"
            )
        else:
            self.get_logger().info("bulunamadi")

    def veri_gönder(self):
        msg=HedefBilgi()
        msg.hedef_takip=True
        msg.hedef_renk="siyah"
        msg.center_x=240
        msg.center_y=240
        msg.uzaklik=20.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    control = HedefDurum()
    rclpy.spin(control)
    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()