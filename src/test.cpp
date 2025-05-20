#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "yjkj_location_msgs/Location.h"
#include "yjkj_control_msgs/VehicleCMD.h"

// ====================== 轨迹点结构体 ======================
struct TrajPoint {
    double x, y, yaw, v;
};

// ====================== 全局变量 =========================
std::vector<TrajPoint> current_trajectory;  // 当前跟踪的轨迹
double curr_x = 0, curr_y = 0, curr_yaw = 0, curr_v = 0;
bool state_ready = false;       // 定位状态
nav_msgs::OccupancyGrid::ConstPtr latest_gridmap;
bool map_ready = false;         // 地图状态

// ===================== 参数 ==============================
const double WHEELBASE = 0.55, LOOKAHEAD_MIN = 2.0, MAX_STEER_RAD = 0.296;
const double BOX_LENGTH = 2.0, BOX_WIDTH = 1.2, BOX_STEP = 0.1;
const int OCC_THRESHOLD = 50;

// =========== Pure Pursuit目标点查找 ============
size_t searchTargetIdx(double x, double y, double v, const std::vector<TrajPoint> &traj) {
    // 查找最近点
    size_t nearest_idx = 0;
    double min_dist = 1e8;
    for (size_t i = 0; i < traj.size(); ++i) {
        double dist = hypot(traj[i].x - x, traj[i].y - y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    // 计算前视距离
    double Ld = std::max(LOOKAHEAD_MIN, 0.5 * v);
    double accum_dist = 0.0;
    size_t target_idx = nearest_idx;

    // 从最近点开始累计距离
    for (size_t i = nearest_idx; i < traj.size() - 1; ++i) {
        accum_dist += hypot(traj[i+1].x - traj[i].x, traj[i+1].y - traj[i].y);
        if (accum_dist >= Ld) {
            target_idx = i + 1;
            break;
        }
    }
    return target_idx;
}

// =========== 定位回调 ============
void statusCallback(const yjkj_location_msgs::Location::ConstPtr &msg) {
    curr_x = msg->pos.x;
    curr_y = msg->pos.y;
    curr_yaw = msg->orientation.z;
    curr_v = hypot(msg->velocity.x, msg->velocity.y);
    state_ready = true;
}

// =========== 地图回调 ============
void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    latest_gridmap = msg;
    map_ready = true;
}

// =========== 轨迹回调（直接接收混合A*的输出轨迹）============
void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg) {
    current_trajectory.clear();
    current_trajectory.reserve(msg->poses.size());
    
    for (const auto& pose : msg->poses) {
        TrajPoint point;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        
        // 转换四元数到偏航角
        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        point.yaw = yaw;
        point.v = 0.8;  // 默认速度
        
        current_trajectory.push_back(point);
    }
    ROS_INFO("Received new trajectory with %lu points", current_trajectory.size());
}

// =========== 虚拟车辆包络框障碍检测 ===========
bool checkCollision(double rel_x, double rel_y, double rel_yaw) {
    if (!map_ready) return false;
    
    const double ox = latest_gridmap->info.origin.position.x;
    const double oy = latest_gridmap->info.origin.position.y;
    const double res = latest_gridmap->info.resolution;
    const int width = latest_gridmap->info.width;
    const int height = latest_gridmap->info.height;

    for (double dx = -BOX_LENGTH/2; dx <= BOX_LENGTH/2; dx += BOX_STEP) {
        for (double dy = -BOX_WIDTH/2; dy <= BOX_WIDTH/2; dy += BOX_STEP) {
            // 计算检测点在baselink系下的坐标
            double x = rel_x + dx * cos(rel_yaw) - dy * sin(rel_yaw);
            double y = rel_y + dx * sin(rel_yaw) + dy * cos(rel_yaw);
            
            // 转换为地图坐标
            int u = static_cast<int>((x - ox) / res + 0.5);
            int v = static_cast<int>((y - oy) / res + 0.5);
            
            // 检查边界和占用
            if (u < 0 || u >= width || v < 0 || v >= height) 
                return true;
            if (latest_gridmap->data[v * width + u] > OCC_THRESHOLD) 
                return true;
        }
    }
    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit_tracker");
    ros::NodeHandle nh;

    // 订阅器
    ros::Subscriber traj_sub = nh.subscribe("/hybrid_astar/path", 1, trajectoryCallback);
    ros::Subscriber loc_sub = nh.subscribe("/yjkj/location_odometry", 10, statusCallback);
    ros::Subscriber gridmap_sub = nh.subscribe("/yjkj/remove_ground/costmap", 1, gridmapCallback);

    // 发布器
    ros::Publisher cmd_pub = nh.advertise<yjkj_control_msgs::VehicleCMD>("/yjkj/VehicleCMD", 5);

    ros::Rate rate(20);

    while (ros::ok()) {
        ros::spinOnce();

        // 等待必要数据就绪
        if (!state_ready || !map_ready || current_trajectory.empty()) {
            rate.sleep();
            continue;
        }

        // ==================
        // Pure Pursuit跟踪逻辑
        // ==================
        size_t tgt_idx = searchTargetIdx(curr_x, curr_y, curr_v, current_trajectory);
        const TrajPoint& goal = current_trajectory[tgt_idx];

        // 计算转向角
        double angle_to_goal = atan2(goal.y - curr_y, goal.x - curr_x);
        double alpha = angle_to_goal - curr_yaw;
        double Ld = hypot(goal.x - curr_x, goal.y - curr_y);
        double delta = atan2(2.0 * WHEELBASE * sin(alpha), Ld);
        
        // 限制转向角度
        delta = std::clamp(delta, -MAX_STEER_RAD, MAX_STEER_RAD);

        // 检查前方碰撞
        bool collision = false;
        for (size_t i = tgt_idx; i < std::min(tgt_idx + 10, current_trajectory.size()); ++i) {
            double rel_x = (current_trajectory[i].x - curr_x) * cos(curr_yaw) + 
                          (current_trajectory[i].y - curr_y) * sin(curr_yaw);
            double rel_y = -(current_trajectory[i].x - curr_x) * sin(curr_yaw) + 
                          (current_trajectory[i].y - curr_y) * cos(curr_yaw);
            double rel_yaw = current_trajectory[i].yaw - curr_yaw;
            
            if (checkCollision(rel_x, rel_y, rel_yaw)) {
                collision = true;
                break;
            }
        }

        // 发布控制指令
        yjkj_control_msgs::VehicleCMD cmd;
        cmd.header.stamp = ros::Time::now();
        
        if (collision) {
            // 遇到碰撞则停车
            cmd.vehicle_speed = 0.0;
            cmd.steer_angle = 0.0;
            ROS_WARN("Collision detected! Stopping...");
        } else {
            // 正常跟踪
            cmd.vehicle_speed = 0.8;
            cmd.steer_angle = delta * 16.43;  // 转换为角度
        }
        cmd.gear = (cmd.vehicle_speed > 0) ? 4 : 1;  // 4=前进, 1=停车
        cmd_pub.publish(cmd);

        // 检查是否到达终点
        if (tgt_idx >= current_trajectory.size() - 3) {
            cmd.vehicle_speed = 0.0;
            cmd.steer_angle = 0.0;
            cmd.gear = 1;
            cmd_pub.publish(cmd);
            ROS_INFO("Reached trajectory end");
            break;
        }

        rate.sleep();
    }
    return 0;
}