#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <random>

// DWA 算法的参数和权重
struct DWAParams {
    double max_vel;
    double min_vel;
    double max_accel;
    double min_accel;
    double dt;
    double predict_time;
    double weight_target;
    double weight_obstacles;
    double weight_speed;
    double safe_distance;
};

// 轨迹结构体
struct Trajectory {
    std::vector<geometry_msgs::msg::Pose> poses;
    double score;
    double speed;
    double dist_to_goal;
    double dist_to_obstacle;
};

class DWANode : public rclcpp::Node {
public:
    DWANode() : Node("dwa_node") {
        // 声明并获取参数
        this->declare_parameter<double>("max_vel", 3.0);
        this->declare_parameter<double>("min_vel", -0.5);
        this->declare_parameter<double>("max_accel", 1.5);
        this->declare_parameter<double>("dt", 0.1);
        this->declare_parameter<double>("predict_time", 2.0);
        this->declare_parameter<double>("weight_target", 0.5);
        this->declare_parameter<double>("weight_obstacles", 0.5);
        this->declare_parameter<double>("weight_speed", 0.1);
        this->declare_parameter<double>("safe_distance", 1.0);

        params_.max_vel = this->get_parameter("max_vel").as_double();
        params_.min_vel = this->get_parameter("min_vel").as_double();
        params_.max_accel = this->get_parameter("max_accel").as_double();
        params_.dt = this->get_parameter("dt").as_double();
        params_.predict_time = this->get_parameter("predict_time").as_double();
        params_.weight_target = this->get_parameter("weight_target").as_double();
        params_.weight_obstacles = this->get_parameter("weight_obstacles").as_double();
        params_.weight_speed = this->get_parameter("weight_speed").as_double();
        params_.safe_distance = this->get_parameter("safe_distance").as_double();

        // 订阅全局路径
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10,
            std::bind(&DWANode::globalPathCallback, this, std::placeholders::_1));

        // 订阅无人机当前位姿和速度
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", 10,
            std::bind(&DWANode::odometryCallback, this, std::placeholders::_1));

        // 订阅八叉树地图
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_full", 10,
            std::bind(&DWANode::octomapCallback, this, std::placeholders::_1));

        // 发布速度指令
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel", 10);
        
        // 创建定时器，周期性执行 DWA 算法
        dwa_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10Hz
            std::bind(&DWANode::dwaLoop, this));
    }

private:
    // 成员变量
    DWAParams params_;
    nav_msgs::msg::Path::SharedPtr global_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::shared_ptr<octomap::OcTree> current_octree_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr dwa_timer_;
    
    // 随机数生成器
    std::mt19937 gen_;
    
    /**
     * @brief 全局路径回调函数，保存最新路径
     */
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        global_path_ = msg;
    }

    /**
     * @brief 无人机位姿回调函数，保存最新位姿
     */
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = msg;
    }

    /**
     * @brief 八叉树地图回调函数，保存最新地图
     */
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        if (msg->data.empty()) return;
        current_octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg)));
    }

    /**
     * @brief 预测轨迹
     */
    Trajectory predictTrajectory(const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Twist& vel) {
        Trajectory traj;
        traj.poses.push_back(start_pose);
        geometry_msgs::msg::Pose current_pose = start_pose;
        
        double steps = params_.predict_time / params_.dt;
        for (int i = 0; i < steps; ++i) {
            current_pose.position.x += vel.linear.x * params_.dt;
            current_pose.position.y += vel.linear.y * params_.dt;
            current_pose.position.z += vel.linear.z * params_.dt;
            traj.poses.push_back(current_pose);
        }
        return traj;
    }

    /**
     * @brief 评估轨迹
     */
    void evaluateTrajectory(Trajectory& traj) {
        // 目标评分
        double dist_to_goal = std::numeric_limits<double>::infinity();
        if (global_path_ && !global_path_->poses.empty()) {
            geometry_msgs::msg::Pose goal_pose = global_path_->poses.back().pose;
            dist_to_goal = std::sqrt(std::pow(traj.poses.back().position.x - goal_pose.position.x, 2) +
                                     std::pow(traj.poses.back().position.y - goal_pose.position.y, 2) +
                                     std::pow(traj.poses.back().position.z - goal_pose.position.z, 2));
        }
        
        // 障碍物评分
        double dist_to_obstacle = std::numeric_limits<double>::infinity();
        if (current_octree_) {
            for (const auto& pose : traj.poses) {
                octomap::point3d point(pose.position.x, pose.position.y, pose.position.z);
                octomap::OcTreeNode* node = current_octree_->search(point);
                if (node && current_octree_->isNodeOccupied(node)) {
                    dist_to_obstacle = 0.0; // 撞到障碍物，距离为0
                    break;
                }
            }
        }
        
        // 速度评分
        double avg_speed = 0;
        if (traj.poses.size() > 1) {
            double total_dist = 0;
            for (size_t i = 0; i < traj.poses.size() - 1; ++i) {
                total_dist += std::sqrt(std::pow(traj.poses[i+1].position.x - traj.poses[i].position.x, 2) +
                                        std::pow(traj.poses[i+1].position.y - traj.poses[i].position.y, 2) +
                                        std::pow(traj.poses[i+1].position.z - traj.poses[i].position.z, 2));
            }
            avg_speed = total_dist / params_.predict_time;
        }

        // 归一化评分
        double normalized_target_score = 1.0 / (1.0 + dist_to_goal);
        double normalized_obstacle_score = dist_to_obstacle / params_.safe_distance;
        if (normalized_obstacle_score > 1.0) normalized_obstacle_score = 1.0;
        double normalized_speed_score = avg_speed / params_.max_vel;

        // 加权求和
        traj.score = params_.weight_target * normalized_target_score +
                     params_.weight_obstacles * normalized_obstacle_score +
                     params_.weight_speed * normalized_speed_score;

        traj.dist_to_goal = dist_to_goal;
        traj.dist_to_obstacle = dist_to_obstacle;
        traj.speed = avg_speed;
    }
    
    /**
     * @brief DWA 主循环
     */
    void dwaLoop() {
        if (!current_odom_ || !global_path_ || !current_octree_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "DWA 未收到所有必要数据 (位姿/路径/地图)。");
            return;
        }

        double current_vx = current_odom_->twist.twist.linear.x;
        double current_vy = current_odom_->twist.twist.linear.y;
        double current_vz = current_odom_->twist.twist.linear.z;

        // 计算动态窗口
        double min_vx = std::max(params_.min_vel, current_vx - params_.max_accel * params_.dt);
        double max_vx = std::min(params_.max_vel, current_vx + params_.max_accel * params_.dt);
        double min_vy = std::max(params_.min_vel, current_vy - params_.max_accel * params_.dt);
        double max_vy = std::min(params_.max_vel, current_vy + params_.max_accel * params_.dt);
        double min_vz = std::max(params_.min_vel, current_vz - params_.max_accel * params_.dt);
        double max_vz = std::min(params_.max_vel, current_vz + params_.max_accel * params_.dt);

        std::uniform_real_distribution<double> dist_vx(min_vx, max_vx);
        std::uniform_real_distribution<double> dist_vy(min_vy, max_vy);
        std::uniform_real_distribution<double> dist_vz(min_vz, max_vz);

        Trajectory best_trajectory;
        best_trajectory.score = -std::numeric_limits<double>::infinity();
        geometry_msgs::msg::Twist best_vel;

        // 采样并评估轨迹
        for (int i = 0; i < 100; ++i) { // 采样100次
            geometry_msgs::msg::Twist sampled_vel;
            sampled_vel.linear.x = dist_vx(gen_);
            sampled_vel.linear.y = dist_vy(gen_);
            sampled_vel.linear.z = dist_vz(gen_);

            Trajectory current_traj = predictTrajectory(current_odom_->pose.pose, sampled_vel);
            evaluateTrajectory(current_traj);
            
            if (current_traj.score > best_trajectory.score) {
                best_trajectory = current_traj;
                best_vel = sampled_vel;
            }
        }
        
        if (best_trajectory.score > -std::numeric_limits<double>::infinity()) {
            velocity_pub_->publish(best_vel);
        } else {
            RCLCPP_WARN(this->get_logger(), "未找到可行轨迹，将发布零速度指令。");
            geometry_msgs::msg::Twist stop_vel;
            velocity_pub_->publish(stop_vel);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWANode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}