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
    double t, x, y, yaw, v;
};

// ====================== 全局变量 =========================
std::vector<TrajPoint> reference_traj;  // 跟踪轨迹
double curr_x = 0, curr_y = 0, curr_yaw = 0, curr_v = 0;
bool state_ready = false;       //定位状态
bool path_ready = false;        //接收路径话题标志位
nav_msgs::OccupancyGrid::ConstPtr latest_gridmap;
bool map_ready = false;         //接收栅格地图话题标志位

// 局部路径规划相关
nav_msgs::Path latest_local_path;
bool local_path_ready = false;
bool waiting_for_local_path = false;
size_t fusion_start_idx = 0, fusion_end_idx = 0;

// ===================== 参数 ==============================
const double WHEELBASE = 0.55, LOOKAHEAD_MIN = 2.0, MAX_STEER_RAD = 0.296;
const double BOX_LENGTH = 2.0, BOX_WIDTH = 1.2, BOX_STEP = 0.1;
const double FRONT_DIST_LIM = 4.0;  // 前方检测距离
const int OCC_THRESHOLD = 50;

// =========== Pure Pursuit目标点查找 ============
size_t searchTargetIdx(double x, double y, double v, const std::vector<TrajPoint> &traj) {
    double min_dist = 1E8;
    size_t idx = 0;
    for (size_t i = 0; i < traj.size(); ++i) {
        double dx = traj[i].x - x, dy = traj[i].y - y;
        double dist = hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            idx = i;
        }
    }
    double Ld = std::max(LOOKAHEAD_MIN, 0.0 * v), accum = 0.0;
    size_t tgt_idx = idx;
    for (size_t i = idx; i < traj.size() - 1; ++i) {
        accum += hypot(traj[i + 1].x - traj[i].x, traj[i + 1].y - traj[i].y);
        if (accum >= Ld) {
            tgt_idx = i + 1;
            break;
        }
    }
    return tgt_idx;
}

// =========== 找最近轨迹点 ===========
size_t searchNearestIdx(double x, double y, const std::vector<TrajPoint> &traj) {
    double min_dist = 1E8;
    size_t idx = 0;
    for (size_t i = 0; i < traj.size(); ++i) {
        double dx = traj[i].x - x, dy = traj[i].y - y;
        double dist = hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            idx = i;
        }
    }
    return idx;
}

// =========== 定位回调 ============
void statusCallback(const yjkj_location_msgs::Location::ConstPtr &msg) {
    curr_x = msg->pos.x;
    curr_y = msg->pos.y;
    curr_yaw = msg->orientation.z;
    curr_v = std::hypot(msg->velocity.x, msg->velocity.y);
    state_ready = true;
}

// =========== 地图回调 ============
void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    latest_gridmap = msg;
    map_ready = true;
}

// =========== 路径回调 ============
void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
    reference_traj.clear();
    reference_traj.reserve(msg->poses.size());
    for (const auto &ps : msg->poses) {
        TrajPoint p;
        p.t = ps.header.stamp.toSec();
        p.x = ps.pose.position.x;
        p.y = ps.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(ps.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        p.yaw = yaw;
        p.v = 0.8;
        reference_traj.push_back(p);
    }
    path_ready = !reference_traj.empty();
    ROS_INFO("Received path with %lu points.", reference_traj.size());
}

// =========== 接收混合A*路径 ============
void localPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    latest_local_path = *msg;
    local_path_ready = true;
    ROS_WARN("Received local(Hybrid A*) path %lu points, ready to fuse.", msg->poses.size());
}

// =========== nav_msgs::Path转TrajPoint ===========
std::vector<TrajPoint> pathToTraj(const nav_msgs::Path& path) {
    std::vector<TrajPoint> out;
    for (auto& ps : path.poses) {
        TrajPoint p;
        p.x = ps.pose.position.x;
        p.y = ps.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(ps.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        p.yaw = yaw;
        p.t = ps.header.stamp.toSec();
        p.v = 0.8;
        out.push_back(p);
    }
    return out;
}

// =========== 虚拟车辆包络框障碍检测 ===========
bool collision_box_at_point(double rel_x, double rel_y, double rel_yaw, const nav_msgs::OccupancyGrid &local_map, double &obstacle_dist) {
    double ox = local_map.info.origin.position.x, oy = local_map.info.origin.position.y;
    double res = local_map.info.resolution;
    int width = local_map.info.width, height = local_map.info.height;
    bool collision = false;
    double min_dist = 1e9;
    for (double dx = -BOX_LENGTH / 2; dx <= BOX_LENGTH / 2; dx += BOX_STEP) {
        for (double dy = -BOX_WIDTH / 2; dy <= BOX_WIDTH / 2; dy += BOX_STEP) {
            double x = rel_x + dx * cos(rel_yaw) - dy * sin(rel_yaw);
            double y = rel_y + dx * sin(rel_yaw) + dy * cos(rel_yaw);
            int u = int((x - ox) / res + 0.5), v = int((y - oy) / res + 0.5);
            if (u < 0 || u >= width || v < 0 || v >= height) continue;
            int idx = v * width + u;
            if (local_map.data[idx] > OCC_THRESHOLD) {
                collision = true;
                double d = hypot(x, y);
                if (d < min_dist) min_dist = d;
            }
        }
    }
    if (collision)
        obstacle_dist = min_dist;
    else
        obstacle_dist = -1;
    return collision;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pp_coll_check_trajline_with_box_odom_pathsub");
    ros::NodeHandle nh;

    ros::Subscriber path_sub = nh.subscribe("/csv_path", 1, pathCallback);
    ros::Subscriber loc_sub = nh.subscribe("/yjkj/location_odometry", 10, statusCallback);
    ros::Subscriber gridmap_sub = nh.subscribe("/yjkj/remove_ground/costmap", 1, gridmapCallback);

    ros::Publisher cmd_pub = nh.advertise<yjkj_control_msgs::VehicleCMD>("/yjkj/VehicleCMD", 5);

    ros::Publisher hybrid_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/hybrid_astar/goal", 1);
    ros::Subscriber hybrid_path_sub = nh.subscribe("/hybrid_astar/path", 1, localPathCallback);

    ros::Rate rate(20);

    while (ros::ok()) {
        ros::spinOnce();

        if (!(state_ready && map_ready && path_ready && !reference_traj.empty())) {
            rate.sleep();
            continue;
        }

        if (waiting_for_local_path) {
            if (local_path_ready) {
                std::vector<TrajPoint> local_traj = pathToTraj(latest_local_path);
                if (!local_traj.empty() && fusion_end_idx > fusion_start_idx) {
                    // 替换当前轨迹的局部障碍段
                    reference_traj.erase(reference_traj.begin() + fusion_start_idx, reference_traj.begin() + fusion_end_idx);
                    reference_traj.insert(reference_traj.begin() + fusion_start_idx, local_traj.begin(), local_traj.end());
                }
                waiting_for_local_path = false;
                local_path_ready = false;
                ROS_INFO("Local path fused, replan done.");
            } else {
                // 还在等局部轨迹，保持停车
                yjkj_control_msgs::VehicleCMD cmd;
                cmd.vehicle_speed = 0.0;
                cmd.steer_angle = 0.0;
                cmd.gear = 1;
                cmd.header.stamp = ros::Time::now();
                cmd_pub.publish(cmd);
                rate.sleep();
                continue;
            }
        }

        // ****************************
        // 前方4米障碍检测与绕障请求逻辑
        // ****************************
        size_t idx0 = searchNearestIdx(curr_x, curr_y, reference_traj);

        // 沿轨迹累计4米，只检测这段 [idx0, idx_end]
        double accum = 0.0;
        size_t idx_end = idx0;
        for (size_t i = idx0; i + 1 < reference_traj.size(); ++i) {
            double dx = reference_traj[i + 1].x - reference_traj[i].x;
            double dy = reference_traj[i + 1].y - reference_traj[i].y;
            accum += std::hypot(dx, dy);
            if (accum >= FRONT_DIST_LIM) {
                idx_end = i + 1;
                break;
            }
        }

        // 遍历包络框检测障碍
        bool any_collision = false;
        double min_collision_dist = 1e9;
        size_t collision_idx = idx0;
        for (size_t i = idx0; i <= idx_end; ++i) {
            double px = reference_traj[i].x, py = reference_traj[i].y, pyaw = reference_traj[i].yaw;
            double rel_x = (px - curr_x) * cos(curr_yaw) + (py - curr_y) * sin(curr_yaw);
            double rel_y = (px - curr_x) * sin(-curr_yaw) + (py - curr_y) * cos(curr_yaw);
            double rel_yaw = pyaw - curr_yaw;
            if (rel_x < 0) continue;
            double obs_dist;
            if (collision_box_at_point(rel_x, rel_y, rel_yaw, *latest_gridmap, obs_dist)) {
                any_collision = true;
                if (obs_dist < min_collision_dist) {
                    min_collision_dist = obs_dist;
                    collision_idx = i;
                }
            }
        }

        // 一旦遇障，发起混合A*请求，目标为障碍点后4米
        if (any_collision) {
            ROS_WARN("Obstacle detected! Nearest at: %.2f m. Requesting Hybrid A*...", min_collision_dist);

            // 计算rejoin点
            size_t rejoin_idx = collision_idx;
            double accum2 = 0.0;
            for (size_t i = collision_idx + 1; i < reference_traj.size(); ++i) {
                double dx = reference_traj[i].x - reference_traj[collision_idx].x;
                double dy = reference_traj[i].y - reference_traj[collision_idx].y;
                accum2 = std::hypot(dx, dy);
                if (accum2 > 4.0) {
                    rejoin_idx = i;
                    break;
                }
            }
            if (rejoin_idx >= reference_traj.size()-2)
                rejoin_idx = reference_traj.size()-2;

            // 发布局部目标点
            geometry_msgs::PoseStamped goal_pose;
            goal_pose.header.frame_id = "odom";
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.pose.position.x = reference_traj[rejoin_idx].x;
            goal_pose.pose.position.y = reference_traj[rejoin_idx].y;
            tf2::Quaternion q;
            q.setRPY(0,0,reference_traj[rejoin_idx].yaw);
            goal_pose.pose.orientation = tf2::toMsg(q);

            hybrid_goal_pub.publish(goal_pose);
            ROS_INFO("HybridA* reroute requested to %.2f,%.2f (idx=%zu)", goal_pose.pose.position.x, goal_pose.pose.position.y, rejoin_idx);

            // 设置融合区间
            fusion_start_idx = idx0;
            fusion_end_idx = rejoin_idx;
            waiting_for_local_path = true;
            local_path_ready = false;

            // 停车
            yjkj_control_msgs::VehicleCMD cmd;
            cmd.vehicle_speed = 0.0;
            cmd.steer_angle = 0.0;
            cmd.gear = 1;
            cmd.header.stamp = ros::Time::now();
            cmd_pub.publish(cmd);
            rate.sleep();
            continue;
        }

        // ==================
        // 普通Pure Pursuit跟踪
        // ==================
        size_t tgt_idx = searchTargetIdx(curr_x, curr_y, curr_v, reference_traj);
        const TrajPoint &goal = reference_traj[tgt_idx];

        double angle_to_goal = atan2(goal.y - curr_y, goal.x - curr_x);
        double alpha = angle_to_goal - curr_yaw;
        double Ld = hypot(goal.x - curr_x, goal.y - curr_y);
        double delta = atan2(2.0 * WHEELBASE * sin(alpha), Ld);
        if (delta > MAX_STEER_RAD) delta = MAX_STEER_RAD;
        if (delta < -MAX_STEER_RAD) delta = -MAX_STEER_RAD;

        yjkj_control_msgs::VehicleCMD cmd;
        cmd.vehicle_speed = 0.8;
        cmd.steer_angle = delta * 16.43;
        cmd.gear = 4;
        cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(cmd);

        // 到终点停车
        if (tgt_idx > reference_traj.size() - 3) {
            cmd.vehicle_speed = 0.0;
            cmd.steer_angle = 0.0;
            cmd.gear = 1;
            cmd.header.stamp = ros::Time::now();
            cmd_pub.publish(cmd);
            ROS_INFO("Tracking finished, vehicle stopped.");
            break;
        }

        rate.sleep();
    }
    return 0;
}