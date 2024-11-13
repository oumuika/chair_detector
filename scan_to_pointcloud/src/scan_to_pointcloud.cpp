#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

class ScanToPointCloudNode : public rclcpp::Node
{
public:
    ScanToPointCloudNode() : Node("scan_to_pointcloud_node")
    {
        // LaserScanトピックを購読
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_filtered", 10, std::bind(&ScanToPointCloudNode::scanCallback, this, std::placeholders::_1));

        // PointCloud2トピックをパブリッシュ
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    }

private:
    // LaserScanメッセージを受け取り、PointCloud2に変換するコールバック関数
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_.projectLaser(*scan_msg, cloud_msg);  // LaserScanをPointCloud2に変換

        // PointCloud2メッセージをパブリッシュ
        cloud_publisher_->publish(cloud_msg);
    }

    // LaserScanからPointCloud2への変換を行うオブジェクト
    laser_geometry::LaserProjection projector_;

    // トピック購読とパブリッシュの設定
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

int main(int argc, char **argv)
{
    // ROS 2の初期化
    rclcpp::init(argc, argv);

    // ノードの実行
    rclcpp::spin(std::make_shared<ScanToPointCloudNode>());

    // ROS 2の終了
    rclcpp::shutdown();
    return 0;
}
