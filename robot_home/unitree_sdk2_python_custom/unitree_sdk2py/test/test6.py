import rclpy
from rclpy.node import Node
from unitree_go.msg import Go2FrontVideoData  # Подставьте свой тип сообщения
import cv2
import numpy as np

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        
        # Важно! Для видеопотока используем QoS BEST_EFFORT, чтобы не накапливать устаревшие кадры [citation:2]
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.subscription = self.create_subscription(
            Go2FrontVideoData,  # Тип сообщения из пакета unitree_go
            '/frontvideostream', # Имя топика, которое вы нашли
            self.listener_callback,
            qos_profile)
        self.subscription  # предотвращаем удаление подписки сборщиком мусора

    def listener_callback(self, msg):
        # Выбираем нужное разрешение. Например, 720p.
        if len(msg.video720p) > 0:
            # Преобразуем массив байт в массив numpy
            np_arr = np.frombuffer(msg.video720p, dtype=np.uint8)
            
            # Декодируем сжатое изображение (скорее всего H.264)
            # Параметр cv2.IMREAD_COLOR гарантирует цветное изображение BGR
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # Вот он, наш кадр! Теперь его можно передать в детектор.
                self.process_frame_for_detection(frame)
            else:
                self.get_logger().warn("Не удалось декодировать кадр")
        else:
            self.get_logger().warn("Получен пустой кадр")

    def process_frame_for_detection(self, frame):
        # Здесь будет ваш код детектора
        # Например, вызов YOLO или другой модели
        pass

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
