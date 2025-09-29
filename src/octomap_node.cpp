#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_ros/conversions.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <string>

// 定义一个 ROS 2 节点类，用于管理八叉树地图
class OctoMapNode : public rclcpp::Node {
public:
    // 构造函数
    OctoMapNode() : Node("octomap_node") {
        // 声明并获取参数
        this->declare_parameter<double>("map_resolution", 0.1); // 地图分辨率
        this->get_parameter("map_resolution", map_resolution_);
        RCLCPP_INFO(this->get_logger(), "地图分辨率设置为: %.2f 米", map_resolution_);

        this->declare_parameter<std::string>("map_frame_id", "map"); // 地图坐标系ID
        this->get_parameter("map_frame_id", map_frame_id_);
        
        this->declare_parameter<std::string>("robot_frame_id", "base_link"); // 机器人本体坐标系ID
        this->get_parameter("robot_frame_id", robot_frame_id_);

        // 初始化八叉树，设置分辨率
        octree_map_ = std::make_shared<octomap::OcTree>(map_resolution_);
        
        // 创建点云数据订阅者
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/planning/njupt_ply_node",
            10,
            std::bind(&OctoMapNode::pointcloudCallback, this, std::placeholders::_1)
        );
        
        // 创建 OctoMap 消息发布者
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            "/njupt_ot",
            10
        );

        // 初始化 TF 监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "OctoMap 节点已成功启动，正在等待点云数据...");
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // ========== 核心修改：添加日志输出 ==========
        RCLCPP_INFO(this->get_logger(), "成功接收到点云数据，正在处理...");
        // ============================================

        // 查找 TF 变换
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                map_frame_id_,
                msg->header.frame_id,
                tf2::TimePointZero
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF 转换失败: %s", ex.what());
            return;
        }

        // 将 ROS 点云消息转换为 OctoMap 点云
        octomap::Pointcloud octo_cloud;
        octomap::pointCloud2ToOctomap(*msg, octo_cloud);

        // 获取传感器在地图坐标系下的原点
        geometry_msgs::msg::PoseStamped sensor_pose_in_map;
        geometry_msgs::msg::PoseStamped sensor_pose_in_sensor;
        sensor_pose_in_sensor.header.frame_id = msg->header.frame_id;
        sensor_pose_in_sensor.header.stamp = msg->header.stamp;
        sensor_pose_in_sensor.pose.orientation.w = 1.0; // 四元数 w=1，代表无旋转
        
        // 使用 TF2 转换传感器原点
        tf2::doTransform(sensor_pose_in_sensor, sensor_pose_in_map, transform);
        octomap::point3d sensor_origin(sensor_pose_in_map.pose.position.x,
                                       sensor_pose_in_map.pose.position.y,
                                       sensor_pose_in_map.pose.position.z);
        
        // 将点云插入八叉树，更新占据信息
        octree_map_->insertPointCloud(octo_cloud, sensor_origin);

        // 清空所有未观察的节点，保持地图精简
        octree_map_->prune(); 
        
        // 创建并发布 OctoMap 消息
        auto octomap_msg = std::make_unique<octomap_msgs::msg::Octomap>();
        octomap_msg->header.frame_id = map_frame_id_;
        octomap_msg->header.stamp = this->now();
        if (octomap_msgs::binaryMapToMsg(*octree_map_, *octomap_msg)) {
            octomap_pub_->publish(std::move(octomap_msg));
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法转换并发布八叉树地图！");
        }
    }
    
    // ROS 2 订阅者、发布者和 TF 监听器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    std::shared_ptr<octomap::OcTree> octree_map_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 参数
    double map_resolution_;
    std::string map_frame_id_;
    std::string robot_frame_id_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctoMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
