#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include "yjkj_location_msgs/Location.h"
#include "yjkj_control_msgs/VehicleCMD.h"

// ====================== 轨迹点结构体 ======================
struct TrajPoint {
    double t, x, y, yaw, v;
};

// ====================== 全局变量 =========================
std::vector<TrajPoint> reference_traj;  // 轨迹点数组
double curr_x = 0, curr_y = 0, curr_yaw = 0, curr_v = 0;
bool state_ready = false;  // 定位是否初始化完成
nav_msgs::OccupancyGrid::ConstPtr latest_gridmap;
bool map_ready = false;  // 地图是否初始化完成

// ===================== 参数 ==============================
const double WHEELBASE = 0.55, LOOKAHEAD_MIN = 2.0, MAX_STEER_RAD = 0.296;
const double BOX_LENGTH = 2.0, BOX_WIDTH = 1.2, BOX_STEP = 0.1;  // 虚拟车辆长宽
const double FRONT_DIST_LIM = 4.0;                               // 只检测前方4米内轨迹
const int OCC_THRESHOLD = 50;                                    // 障碍格栅阈值

// ================== 轨迹CSV文件读取 =====================
std::vector<TrajPoint> readCsvTraj(const std::string &filename) {
    std::vector<TrajPoint> traj;
    std::ifstream file(filename.c_str());
    std::string line;
    getline(file, line);  // 跳过表头
    while (getline(file, line)) {
        std::istringstream sin(line);
        std::vector<std::string> fields;
        std::string field;
        while (getline(sin, field, ',')) fields.push_back(field);
        if (fields.size() < 12) continue;  // 字段数量不足，跳过
        TrajPoint p;
        p.t = atof(fields[0].c_str());
        p.x = atof(fields[3].c_str());
        p.y = atof(fields[4].c_str());
        p.yaw = atof(fields[8].c_str());
        double ve = atof(fields[9].c_str()), vn = atof(fields[10].c_str());
        p.v = hypot(ve, vn);
        traj.push_back(p);
    }
    return traj;
}

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

// =========== 定位回调函数 ============
void statusCallback(const yjkj_location_msgs::Location::ConstPtr &msg) {
    curr_x = msg->pos.x;
    curr_y = msg->pos.y;
    curr_yaw = msg->orientation.z;
    curr_v = std::hypot(msg->velocity.x, msg->velocity.y);
    state_ready = true;
}

// =========== 地图回调函数 ============
void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    latest_gridmap = msg;
    map_ready = true;
}

// =========== 单点虚拟车辆框内检测障碍及最近距离 ===========
bool collision_box_at_point(double rel_x, double rel_y, double rel_yaw, const nav_msgs::OccupancyGrid &local_map,
                            double &obstacle_dist) {
    double ox = local_map.info.origin.position.x, oy = local_map.info.origin.position.y;
    double res = local_map.info.resolution;
    int width = local_map.info.width, height = local_map.info.height;
    bool collision = false;
    double min_dist = 1e9;
    // 遍历框内部采样点
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
    ros::init(argc, argv, "pp_coll_check_trajline_with_box_4m");
    ros::NodeHandle nh;
    std::string trajfile;
    nh.param<std::string>("traj_csv_path", trajfile,
                          "/home/likun/deploy_in_x86_64_noetic/data/NovAtel2025-05-09-09-59-44.csv");
    reference_traj = readCsvTraj(trajfile);
    if (reference_traj.empty()) {
        ROS_ERROR("Trajectory file not found!");
        return 1;
    }
    ros::Subscriber loc_sub = nh.subscribe("/yjkj/sensor/gnss/location", 10, statusCallback);
    ros::Subscriber gridmap_sub = nh.subscribe("/yjkj/remove_ground/costmap", 1, gridmapCallback);
    ros::Publisher cmd_pub = nh.advertise<yjkj_control_msgs::VehicleCMD>("/yjkj/VehicleCMD", 5);

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        if (state_ready && map_ready) {
            bool any_collision = false;
            double min_collision_dist = 1e9;
            // ==== 每一帧遍历前方4米内所有轨迹点 ====
            for (size_t i = 0; i < reference_traj.size(); ++i) {
                double px = reference_traj[i].x, py = reference_traj[i].y, pyaw = reference_traj[i].yaw;
                // 轨迹点全局->base_link系
                double rel_x = (px - curr_x) * cos(-curr_yaw) - (py - curr_y) * sin(-curr_yaw);
                double rel_y = (px - curr_x) * sin(-curr_yaw) + (py - curr_y) * cos(-curr_yaw);
                double rel_yaw = pyaw - curr_yaw;
                double d = hypot(rel_x, rel_y);
                if (rel_x < 0 || d > FRONT_DIST_LIM) continue;  // 只关心前方4米内

                // ---- 检查该点包络框是否碰障 ----
                double obs_dist;
                if (collision_box_at_point(rel_x, rel_y, rel_yaw, *latest_gridmap, obs_dist)) {
                    any_collision = true;
                    if (obs_dist < min_collision_dist) min_collision_dist = obs_dist;
                }
            }

            // ========== 每帧打印最近障碍距离及状态 ==========
            if (any_collision) {
                ROS_WARN("Obstacle ahead in the next %.1f m! Nearest obstacle: %.2f m", FRONT_DIST_LIM,
                         min_collision_dist);
            } else {
                ROS_INFO("No obstacle detected within %.1f meters ahead, safe.", FRONT_DIST_LIM);
            }

            // === 如需遇障停车, 可加此段 ===

            if (any_collision) {
                yjkj_control_msgs::VehicleCMD cmd;
                cmd.vehicle_speed = 0.0;
                cmd.steer_angle = 0.0;
                cmd.gear = 1;
                cmd.header.stamp = ros::Time::now();
                cmd_pub.publish(cmd);
                // 保持停车
                rate.sleep();
                continue;
            }

            // Pure Pursuit跟踪，不受“未遇障”影响
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

            if (tgt_idx > reference_traj.size() - 3) {
                cmd.vehicle_speed = 0.0;
                cmd.steer_angle = 0.0;
                cmd.gear = 2;
                cmd_pub.publish(cmd);
                ROS_INFO("Tracking finished, vehicle stopped.");
                break;
            }
        }
        rate.sleep();
    }
    return 0;
}