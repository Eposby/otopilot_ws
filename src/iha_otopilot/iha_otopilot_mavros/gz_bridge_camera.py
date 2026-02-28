#!/usr/bin/env python3
"""
Gazebo Harmonic -> ROS 2 Kamera Köprüsü (Direct Binding)
Bu script, standart bridge yerine doğrudan Gazebo kütüphanesini kullanır.
Gereksinimler: python3-gz-transport13, python3-gz-msgs10
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
import threading

# Gazebo kütüphanelerini güvenli import etme
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
except ImportError:
    print("HATA: Gazebo Python kütüphaneleri bulunamadı!")
    print("Lütfen şu paketleri yükleyin (Ubuntu sürümüne göre değişebilir):")
    print("sudo apt install python3-gz-transport13 python3-gz-msgs10")
    sys.exit(1)

class GzDirectCameraBridge(Node):
    def __init__(self):
        super().__init__('gz_direct_camera_bridge')
        
        # Parametreler
        self.declare_parameter('gz_topic', '/camera')
        self.declare_parameter('ros_topic', '/camera')
        
        self.gz_topic = self.get_parameter('gz_topic').value
        self.ros_topic = self.get_parameter('ros_topic').value
        
        # Sensor Verisi için "Best Effort" QoS (Görüntü aktarımı için en iyisi)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(Image, self.ros_topic, sensor_qos)
        self.get_logger().info(f'Köprü Başlatılıyor: Gazebo [{self.gz_topic}] -> ROS 2 [{self.ros_topic}]')
        
        self.count = 0
        self.gz_node = None # Node'u burada saklamalıyız ki silinmesin
        
        # Gazebo dinleyicisini ayrı bir thread'de başlat
        self.gz_thread = threading.Thread(target=self._start_gz_listener)
        self.gz_thread.daemon = True
        self.gz_thread.start()

    def _start_gz_listener(self):
        """Gazebo topic'ini dinleyen thread"""
        self.gz_node = GzNode()
        
        # Abone ol
        if self.gz_node.subscribe(GzImage, self.gz_topic, self._gz_callback):
            self.get_logger().info(f'BAŞARILI: Gazebo topic dinleniyor: {self.gz_topic}')
            # Gazebo node'u C++ tarafında thread yönetir, bizim burada döngüye girmemize gerek yok.
            # Ancak Python objesinin yaşaması için thread'in bitmemesini sağlayabiliriz.
            self.gz_node.wait_for_shutdown() 
        else:
            self.get_logger().error(f'HATA: {self.gz_topic} dinlenemedi! İsim doğru mu?')

    def _gz_callback(self, gz_msg):
        """Gazebo'dan mesaj geldiğinde tetiklenir"""
        try:
            self._publish_to_ros(gz_msg)
        except Exception as e:
            self.get_logger().error(f'Yayın hatası: {e}')

    def _publish_to_ros(self, gz_msg):
        """Gazebo mesajını ROS 2 formatına çevirip basar"""
        ros_msg = Image()
        
        # 1. Header Bilgileri
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = 'camera_link' # Gerekirse parametre yapılabilir
        
        # 2. Boyutlar
        ros_msg.height = gz_msg.height
        ros_msg.width = gz_msg.width
        
        # 3. Format Dönüşümü (Pixel Format Mapping)
        # Gazebo formatları: https://github.com/gazebosim/gz-msgs/blob/ign-msgs8/proto/gz/msgs/image.proto
        fmt = gz_msg.pixel_format_type
        
        if fmt == 1: # R8G8B8
            ros_msg.encoding = 'rgb8'
            ros_msg.step = gz_msg.width * 3
        elif fmt == 3: # B8G8R8
            ros_msg.encoding = 'bgr8'
            ros_msg.step = gz_msg.width * 3
        elif fmt == 5: # L_INT8 (Mono)
            ros_msg.encoding = 'mono8'
            ros_msg.step = gz_msg.width
        else:
            # Bilinmeyen format gelirse RGB8 varsayalım
            ros_msg.encoding = 'rgb8'
            ros_msg.step = gz_msg.width * 3
            
        ros_msg.is_bigendian = 0
        
        # 4. Veri Transferi
        # Veriyi byte array olarak kopyala
        ros_msg.data = gz_msg.data
        
        # 5. Yayınla
        self.publisher.publish(ros_msg)
        
        # Debug Log (Her 100 karede bir)
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'Yayınlanan kare sayısı: {self.count} (Format: {ros_msg.encoding})')

def main(args=None):
    rclpy.init(args=args)
    node = GzDirectCameraBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()