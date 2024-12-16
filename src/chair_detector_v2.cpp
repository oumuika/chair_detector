#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

class ChairRecognitionNode : public rclcpp::Node {
public:
    ChairRecognitionNode() : Node("chair_recognition") {
        marker_subscriber_ = this->create_subscription<MarkerArray>(
            "chair_legs", 10, std::bind(&ChairRecognitionNode::legsCallback, this, std::placeholders::_1)
        );
        chair_publisher_ = this->create_publisher<MarkerArray>("recognized_chairs", 10);
    }

private:
    void legsCallback(const MarkerArray::SharedPtr msg) {
        // Extract leg positions from the incoming MarkerArray
        std::vector<std::pair<double, double>> leg_positions;
        for (const auto &marker : msg->markers) {
            leg_positions.emplace_back(marker.pose.position.x, marker.pose.position.y);
        }

        // Find clusters of legs with 3 or more legs
        auto chair_clusters = findChairClusters(leg_positions);

        // Create MarkerArray for recognized chairs
        MarkerArray chair_markers;
        for (size_t i = 0; i < chair_clusters.size(); ++i) {
            auto [center_x, center_y] = calculateCenter(chair_clusters[i]);
            chair_markers.markers.push_back(createMarker(center_x, center_y, i));
        }

        // Publish the recognized chairs
        chair_publisher_->publish(chair_markers);
    }

    std::vector<std::vector<std::pair<double, double>>> findChairClusters(const std::vector<std::pair<double, double>> &leg_positions) {
        std::vector<std::vector<std::pair<double, double>>> clusters;

        // Group legs that are within 40cm of each other
        for (size_t i = 0; i < leg_positions.size(); ++i) {
            for (size_t j = i + 1; j < leg_positions.size(); ++j) {
                for (size_t k = j + 1; k < leg_positions.size(); ++k) {
                    for (size_t l = k + 1; l < leg_positions.size(); ++l) {
                        std::vector<std::pair<double, double>> quad = {leg_positions[i], leg_positions[j], leg_positions[k], leg_positions[l]};
                        if (isValidSquare(quad)) {
                            clusters.push_back(quad);
                        }
                    }
                }
            }
        }

        return clusters;
    }

    bool isValidSquare(const std::vector<std::pair<double, double>> &legs) {
        std::vector<double> distances;
        for (size_t i = 0; i < legs.size(); ++i) {
            for (size_t j = i + 1; j < legs.size(); ++j) {
                distances.push_back(std::hypot(legs[i].first - legs[j].first, legs[i].second - legs[j].second));
            }
        }

        // Sort distances and check square properties
        std::sort(distances.begin(), distances.end());
        double side = 0.45; // 40cm in meters
        return (
            std::abs(distances[0] - side) <= 0.05 &&
            std::abs(distances[1] - side) <= 0.05 &&
            std::abs(distances[2] - side) <= 0.05 &&
            std::abs(distances[3] - side) <= 0.05 &&
            std::abs(distances[4] - side * std::sqrt(2)) <= 0.05
        );
    }

    std::pair<double, double> calculateCenter(const std::vector<std::pair<double, double>> &cluster) {
        double x_sum = 0.0, y_sum = 0.0;
        for (const auto &pos : cluster) {
            x_sum += pos.first;
            y_sum += pos.second;
        }
        return {x_sum / cluster.size(), y_sum / cluster.size()};
    }

    Marker createMarker(double x, double y, int marker_id) {
        Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "chairs";
        marker.id = marker_id;
        marker.type = Marker::SPHERE;
        marker.action = Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(0.5));
        return marker;
    }

    rclcpp::Subscription<MarkerArray>::SharedPtr marker_subscriber_;
    rclcpp::Publisher<MarkerArray>::SharedPtr chair_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChairRecognitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
