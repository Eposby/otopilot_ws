import rclpy
from rclpy.node import Node

from iha_messages.srv import TakeCommand

class Emir(Node):
    def __init__(self):
        super().__init__('emir_sunucusu')

        self.service=self.create_service(TakeCommand, 'emir_hatti',self.emir_sunucusu_callback)

        self.get_logger().info("sunucu bekliyor")

    def emir_sunucusu_callback(self, request, response):

        self.get_logger().info(f"client say: {request.komut_adi}, parametre: {request.parametre}")

        if request.komut_adi=="offboard":
            if request.parametre>0:
                response.basari_durumu=True
                response.mesaj=f"ucagin yeni durumu {request.parametre}"
            
            else:
                response.basari_durumu=False
                response.mesaj="Hata"

        
        elif request.komut_adi=="landing":
            response.basari_durumu=True
            response.mesaj=f"ucagin yeni durumu {request.parametre}"

        else:
            response.basari_durumu==False
            response.mesaj="yanlis komut"

        return response
    

def main(args=None):
    rclpy.init(args=args)
    node=Emir()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== '__main__':
    main()