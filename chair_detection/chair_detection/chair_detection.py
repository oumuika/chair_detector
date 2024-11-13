import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial import distance_matrix

class ChairDetectionNode(Node):
    def __init__(self):
        super().__init__('chair_detection')
        
        # サブスクライバー設定
        self.subscription = self.create_subscription(
            MarkerArray,
            '/cluster_centers',
            self.cluster_centers_callback,
            10
        )
        
        # パブリッシャー設定
        self.publisher = self.create_publisher(Marker, '/chair_position', 10)

        # 距離閾値の設定
        self.distance_min = 0.2  # 最小距離
        self.distance_max = 0.6  # 最大距離

    def cluster_centers_callback(self, msg):
        # メッセージ内のクラスタの重心座標を抽出
        centroids = [(marker.pose.position.x, marker.pose.position.y) for marker in msg.markers]

        # クラスタ数が3つ以上なければ処理しない
        if len(centroids) < 3:
            return

        # 距離行列の計算
        dist_matrix = distance_matrix(centroids, centroids)
        
        # 椅子の足候補を探す
        chair_leg_candidates = []
        for i, distances in enumerate(dist_matrix):
            # 特定の距離範囲内にあるクラスタをカウント
            neighbors = np.where((distances > self.distance_min) & (distances < self.distance_max))[0]
            if 2 <= len(neighbors) <= 3:  # 自身以外に2～3つのクラスタがあれば候補にする
                chair_leg_candidates.append(i)

        # 椅子の足候補が3～4つであれば中央位置を計算
        if len(chair_leg_candidates) >= 3:
            # 重心の平均位置を計算して椅子の位置とする
            center_x = np.mean([centroids[i][0] for i in chair_leg_candidates])
            center_y = np.mean([centroids[i][1] for i in chair_leg_candidates])

            # 椅子の位置をパブリッシュ
            chair_center_marker = Marker()
            chair_center_marker.header.frame_id = "laser"  # フレームIDを適切に設定
            chair_center_marker.header.stamp = self.get_clock().now().to_msg()
            chair_center_marker.ns = "chair_center"
            chair_center_marker.id = 0
            chair_center_marker.type = Marker.SPHERE
            chair_center_marker.action = Marker.ADD
            chair_center_marker.pose.position.x = center_x
            chair_center_marker.pose.position.y = center_y
            chair_center_marker.pose.position.z = 0.0
            chair_center_marker.pose.orientation.w = 1.0
            chair_center_marker.scale.x = 0.1
            chair_center_marker.scale.y = 0.1
            chair_center_marker.scale.z = 0.1
            chair_center_marker.color.r = 1.0
            chair_center_marker.color.g = 1.0
            chair_center_marker.color.b = 0.0
            chair_center_marker.color.a = 1.0
            chair_center_marker.lifetime = rclpy.duration.Duration(seconds=5).to_msg()

            self.publisher.publish(chair_center_marker)
            self.get_logger().info(f'Published chair center at ({center_x:.2f}, {center_y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = ChairDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
