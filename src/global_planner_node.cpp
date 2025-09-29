#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unordered_map>
#include <queue>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string> 

// 定义一个用于 A* 算法的节点结构体
struct AStarNode {
    octomap::point3d position;
    double g_cost;
    double h_cost;
    double f_cost;
    octomap::point3d parent;

    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

// 定义一个哈希函数，以便在 std::unordered_map 中使用 point3d
struct Point3dHasher {
    std::size_t operator()(const octomap::point3d& p) const {
        // 使用浮点数哈希，结合位移操作
        return std::hash<double>()(p.x()) ^ (std::hash<double>()(p.y()) << 1) ^ (std::hash<double>()(p.z()) >> 1);
    }
};


class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode() : Node("global_planner_node") {
        // 1. 声明并获取静态地图文件路径参数
        this->declare_parameter<std::string>("map_file_path", "");
        std::string map_file_path;
        this->get_parameter("map_file_path", map_file_path);

        if (map_file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未提供静态地图文件路径！请使用 'map_file_path' 参数指定地图文件 (.ot)");
            throw std::runtime_error("缺少地图文件路径参数。"); 
        }

        // 2. 从本地文件加载八叉树地图
        octree_map_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(map_file_path)));

        if (!octree_map_) {
            RCLCPP_ERROR(this->get_logger(), "无法加载本地地图文件: %s，全局规划器将无法运行！", map_file_path.c_str());
            // 如果地图加载失败，抛出异常或退出
            throw std::runtime_error("静态地图文件加载失败。");
        }

        RCLCPP_INFO(this->get_logger(), "A* 全局规划器成功加载静态地图文件: %s, 分辨率: %.3f", map_file_path.c_str(), octree_map_->getResolution());

        // 订阅路径规划的起终点
        start_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/start_goal", 10, // 订阅主题start_goal，并设置缓存长度10
            std::bind(&GlobalPlannerNode::startGoalCallback, this, std::placeholders::_1));

        // 发布规划好的路径（保持不变）
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/global_path", 10);
    }

private:
    // 成员变量
    std::shared_ptr<octomap::OcTree> octree_map_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    octomap::point3d start_point_, goal_point_;

    // 检查节点是否被占据，true为占据
    bool isOccupied(const octomap::point3d& p) {
        if (!octree_map_) return true; // 如果地图未加载，则认为被占据不可通行

        // 缩小到最近的整数分辨率格点
        octomap::point3d query_point = octree_map_->keyToCoord(octree_map_->coordToKey(p));
        
        octomap::OcTreeNode* node = octree_map_->search(query_point);
        
        // 如果节点不存在，通常认为是未知（或自由，取决于OctoMap配置），这里假设为自由
        if (node) {
            // 检查节点是否被占据
            return octree_map_->isNodeOccupied(node);
        }
        
        // 未知空间（没有节点）默认认为是自由的，可以通行
        return false;
    }

    // 启发式函数采用欧几里得距离 
    double heuristic(const octomap::point3d& p1, const octomap::point3d& p2) {
        double dx = p1.x() - p2.x();
        double dy = p1.y() - p2.y();
        double dz = p1.z() - p2.z();
        return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));
    }

    // 路径重构函数，返回值是一个储存了3d坐标的动态数组，即路径
    std::vector<octomap::point3d> reconstructPath(const std::unordered_map<octomap::point3d, octomap::point3d, Point3dHasher>& came_from, // 用键值对储存回溯表，
                                                 const octomap::point3d& current) { //目标点的3d坐标，由此进行回溯
        std::vector<octomap::point3d> total_path;
        octomap::point3d current_point = current;
        while (came_from.count(current_point)) { // 若当前点在came_from里有父节点，就回溯
            total_path.push_back(current_point); // 把当前点压进路径
            current_point = came_from.at(current_point); //把当前点更新成父节点
        }
        total_path.push_back(start_point_); // 添加起点
        std::reverse(total_path.begin(), total_path.end()); //反转路径
        return total_path;
    }

    // 起终点回调函数，当话题/start_goal变化，就自动调用回调函数
    void startGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg->header.frame_id == "start") {
            start_point_ = octomap::point3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            RCLCPP_INFO(this->get_logger(), "已接收起点: (%.2f, %.2f, %.2f)", start_point_.x(), start_point_.y(), start_point_.z());
        } else if (msg->header.frame_id == "goal") {
            goal_point_ = octomap::point3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            RCLCPP_INFO(this->get_logger(), "已接收终点: (%.2f, %.2f, %.2f)", goal_point_.x(), goal_point_.y(), goal_point_.z());
        }

        if (start_point_.x() != 0.0 || start_point_.y() != 0.0 || start_point_.z() != 0.0) {
            if (goal_point_.x() != 0.0 || goal_point_.y() != 0.0 || goal_point_.z() != 0.0) { // 若起终点存在任一分量不为零，认为有效，调用规划发布
                planPathAndPublish();
            }
        }
    }

    // 规划路径并发布 
    void planPathAndPublish() {
        if (!octree_map_) {
            RCLCPP_WARN(this->get_logger(), "地图未加载，无法执行 A* 规划！");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "开始 A* 路径规划...");

        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
        std::unordered_map<octomap::point3d, double, Point3dHasher> g_score;
        std::unordered_map<octomap::point3d, octomap::point3d, Point3dHasher> came_from;

        // 初始化起点节点，g_cost为0，h_cost为起点到终点的启发式距离
        AStarNode start_node = {start_point_, 0.0, heuristic(start_point_, goal_point_), heuristic(start_point_, goal_point_), start_point_};
        open_set.push(start_node);
        g_score[start_point_] = 0.0;
        
        // 定义 26 个可能的邻居方向
        std::vector<octomap::point3d> neighbors;
        double resolution = octree_map_->getResolution();
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue; // 跳过自身
                    neighbors.emplace_back(dx * resolution, dy * resolution, dz * resolution);
                }
            }
        }

        while (!open_set.empty()) {
            AStarNode current_node = open_set.top();
            open_set.pop();

            // 到达终点
            if ((current_node.position - goal_point_).norm() < resolution) {
                publishPath(reconstructPath(came_from, current_node.position));
                RCLCPP_INFO(this->get_logger(), "A* 算法成功找到路径！");
                return;
            }

            for (const auto& neighbor_offset : neighbors) {
                octomap::point3d neighbor_pos = current_node.position + neighbor_offset;

                // 检查邻居是否被占据
                if (isOccupied(neighbor_pos)) continue;

                // 计算到邻居的距离代价
                double tentative_g_score = current_node.g_cost + neighbor_offset.norm();

                if (g_score.find(neighbor_pos) == g_score.end() || tentative_g_score < g_score[neighbor_pos]) {
                    came_from[neighbor_pos] = current_node.position;
                    g_score[neighbor_pos] = tentative_g_score;
                    AStarNode neighbor_node = {neighbor_pos, tentative_g_score, heuristic(neighbor_pos, goal_point_), tentative_g_score + heuristic(neighbor_pos, goal_point_), current_node.position};
                    open_set.push(neighbor_node);
                }
            }
        }
        RCLCPP_WARN(this->get_logger(), "A* 算法未能找到可行路径！");
    }

    // 发布路径
    void publishPath(const std::vector<octomap::point3d>& path) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map"; // 坐标系 ID
        path_msg.header.stamp = this->now();

        for (const auto& point : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            pose.pose.orientation.w = 1.0; // 无旋转
            path_msg.poses.push_back(pose);
        }
        
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<GlobalPlannerNode>());
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "全局规划器启动失败: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}