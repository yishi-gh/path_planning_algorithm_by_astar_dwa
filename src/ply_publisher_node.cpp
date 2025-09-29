#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

class PLYPublisherNode : public rclcpp::Node {
public:
    PLYPublisherNode() : Node("ply_publisher_node") {
        this->declare_parameter<std::string>("ply_file_path", "");
        this->get_parameter("ply_file_path", ply_file_path_);

        if (ply_file_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未提供 .ply 文件路径！请使用 'ply_file_path' 参数指定文件。");
            return;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/planning/njupt_ply_node", 10);

        // 使用定时器周期性地发布点云，这模拟了传感器的实时数据流
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PLYPublisherNode::publishPointCloud, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string ply_file_path_;

    void publishPointCloud() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_file_path_, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法加载文件 %s", ply_file_path_.c_str());
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "lidar_link";
        cloud_msg.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "正在发布包含 %zu 个点的点云...", cloud->points.size());
        publisher_->publish(cloud_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PLYPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

