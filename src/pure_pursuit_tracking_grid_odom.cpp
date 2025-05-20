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
std::vector<TrajPoint> reference_traj;  // 轨迹点数组
double curr_x = 0, curr_y = 0, curr_yaw = 0, curr_v = 0;
bool state_ready = false;  // 定位是否初始化完成
bool path_ready = false;   // 路径准备好
nav_msgs::OccupancyGrid::ConstPtr latest_gridmap;
bool map_ready = false;  // 地图是否初始化完成

// ===================== 参数 ==============================
const double WHEELBASE = 0.55, LOOKAHEAD_MIN = 2.0, MAX_STEER_RAD = 0.296;  // 车辆轴距 预描距离 车辆前向转角最大值
const double BOX_LENGTH = 2.0, BOX_WIDTH = 1.2,
             BOX_STEP = 0.1;        // 虚拟车辆长宽
const double FRONT_DIST_LIM = 4.0;  // 沿轨迹累计4米
const int OCC_THRESHOLD = 50;       // 障碍格栅阈值

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

// =========== 找最近轨迹点的函数 ===========
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

// =========== 路径（Path）回调函数 ============
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
        reference_traj.push_back(p);
    }
    path_ready = !reference_traj.empty();
    ROS_INFO("Received path with %lu points.", reference_traj.size());
}

// =========== 单点虚拟车辆框内检测障碍及最近距离 ===========
bool collision_box_at_point(double rel_x, double rel_y, double rel_yaw, const nav_msgs::OccupancyGrid &local_map,
                            double &obstacle_dist) {
    double ox = local_map.info.origin.position.x;
    double oy = local_map.info.origin.position.y;
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

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        if (state_ready && map_ready && path_ready && !reference_traj.empty()) {
            bool any_collision = false;
            double min_collision_dist = 1e9;

            // 沿轨迹累计4米，只检测这段 [idx0, idx_end]
            size_t idx0 = searchNearestIdx(curr_x, curr_y, reference_traj);
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
            for (size_t i = idx0; i <= idx_end; ++i) {
                double px = reference_traj[i].x, py = reference_traj[i].y, pyaw = reference_traj[i].yaw;
                // 轨迹点从odom转到baselink下
                double rel_x = (px - curr_x) * cos(curr_yaw) + (py - curr_y) * sin(curr_yaw);
                double rel_y = (px - curr_x) * sin(-curr_yaw) + (py - curr_y) * cos(curr_yaw);
                double rel_yaw = pyaw - curr_yaw;
                if (rel_x < 0) continue;
                double obs_dist;
                if (collision_box_at_point(rel_x, rel_y, rel_yaw, *latest_gridmap, obs_dist)) {
                    any_collision = true;
                    if (obs_dist < min_collision_dist) min_collision_dist = obs_dist;
                }
            }

            if (any_collision) {
                ROS_WARN("Nearest obstacle: %.2f m", min_collision_dist);
                yjkj_control_msgs::VehicleCMD cmd;
                cmd.vehicle_speed = 0.0;
                cmd.steer_angle = 0.0;
                cmd.gear = 1;
                cmd.header.stamp = ros::Time::now();
                cmd_pub.publish(cmd);
                rate.sleep();
                continue;
            } else {
                ROS_INFO("No obstacle detected within %.1f meters ahead along the trajectory, safe.", FRONT_DIST_LIM);
            }

            // Pure Pursuit跟踪
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
                cmd.gear = 1;
                cmd_pub.publish(cmd);
                ROS_INFO("Tracking finished, vehicle stopped.");
                break;
            }
        }
        rate.sleep();
    }
    return 0;
}