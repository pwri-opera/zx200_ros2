import rclpy
from rclpy.node import Node
import numpy as np
from PIL import Image

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.size_px = 5000
        self.resolution = 10.0  # メートル単位のピクセルあたりの解像度
        self.obstacle_size = 50  # 障害物のサイズ（ピクセル）
        self.map_data = self.generate_map()
        self.save_map()

    def generate_map(self):
        map_data = np.ones((self.size_px, self.size_px), dtype=np.uint8) * 255  # 全体を白で初期化
        # 縁取りの障害物を追加
        map_data[:self.obstacle_size, :] = 0  # 上辺
        map_data[-self.obstacle_size:, :] = 0  # 下辺
        map_data[:, :self.obstacle_size] = 0  # 左辺
        map_data[:, -self.obstacle_size:] = 0  # 右辺
        return map_data

    def save_map(self):
        map_image = Image.fromarray(self.map_data, mode='L')
        map_image.save('map.pgm')
        self.get_logger().info('Map saved as map.pgm')

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    rclpy.spin(map_saver)
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
