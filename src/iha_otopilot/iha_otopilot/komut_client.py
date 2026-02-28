import rclpy 
from rclpy.node import Node
import sys
from iha_messages.srv import TakeCommand

# bu kodda bir server denemesi olduğu için bir server yok server kodun taammı aslında 
class Emir(Node):
    def __init__(self):
        super().__init__("emir_ver")

        self.client=self.create_client(TakeCommand, 'emir_hatti')
        self.count=0

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.count+=1
            self.get_logger().info(f"Servis bulunamadi, bekleniyor {self.count}")
        
        self.request=TakeCommand.Request()

    def istek_gönder(self,komut,parametre_değer):
        self.request.komut_adi=komut
        self.request.parametre=parametre_değer

        self.future=self.client.call_async(self.request)
        return self.future
    
def main(args=None):
    rclpy.init(args=args)
    node=Emir()
    komut=sys.argv[1] if len(sys.argv)> 1 else "komut_adi"
    parametre=float(sys.argv[2]) if len(sys.argv)>2 else "parametre_değer"

    node.get_logger().info(f"request datas: {komut}, {parametre}")

    future=node.istek_gönder(komut,parametre)
    rclpy.spin_until_future_complete(node, future)

    try:
        response=future.result()
        if response.basari_durumu:
            node.get_logger().info(f"Basarili: {response.mesaj}")

        else:
            node.get_logger().info(f"Basarisiz: {response.mesaj}")

    except Exception as e:
        node.get_logger().error(f"Serviste hata var: {e}")


    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()