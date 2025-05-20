#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

#include "yjkj_location_msgs/Location.h"
#include "yjkj_control_msgs/VehicleCMD.h"

// ========== 定义参数 =============
const double WHEELBASE = 0.55;       // 车辆轴距 [m]
const double LOOKAHEAD_MIN = 2.0;    // 最小前视距离 [m]
const double LOOKAHEAD_K = 0;        // 前视系数
const double MAX_STEER_RAD = 0.296;  // 最大方向盘转角 [rad] 约17°

// ========== 轨迹点结构 ===========
struct TrajPoint {
    double t;
    double x;
    double y;
    double yaw;
    double v;
};

// ========== 全局状态 ============
std::vector<TrajPoint> reference_traj;
double curr_x = 0, curr_y = 0, curr_yaw = 0, curr_v = 0;
bool state_ready = false;

// ========== 轨迹读取 ===========
std::vector<TrajPoint> readCsvTraj(const std::string &filename) {
    std::vector<TrajPoint> traj;
    std::ifstream file(filename.c_str());
    std::string line;
    getline(file, line);  // skip header
    while (getline(file, line)) {
        std::istringstream sin(line);
        std::vector<std::string> fields;
        std::string field;
        while (getline(sin, field, ',')) {
            fields.push_back(field);
        } 
        if (fields.size() < 12) continue;  // 必须有足够字段，按你CSV顺序

        TrajPoint p;
        p.t = atof(fields[0].c_str());
        p.x = atof(fields[3].c_str());
        p.y = atof(fields[4].c_str());
        p.yaw = atof(fields[8].c_str());
        double ve = atof(fields[9].c_str());
        double vn = atof(fields[10].c_str());
        p.v = hypot(ve, vn);
        traj.push_back(p);
    }
    return traj;
}

// ========== Pure Pursuit目标点查找 ===========
size_t searchTargetIdx(double x, double y, double v, const std::vector<TrajPoint> &traj) {
    // 1. 最近点
    double min_dist = 1E8;
    size_t idx = 0;
    for (size_t i = 0; i < traj.size(); ++i) {
        double dx = traj[i].x - x;
        double dy = traj[i].y - y;
        double dist = hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            idx = i;
        }
    }
    // 2. 前视距离
    double Ld = std::max(LOOKAHEAD_MIN, LOOKAHEAD_K * v);
    double accum_dist = 0.0;
    size_t tgt_idx = idx;
    for (size_t i = idx; i < traj.size() - 1; ++i) {
        double dx = traj[i + 1].x - traj[i].x;
        double dy = traj[i + 1].y - traj[i].y;
        accum_dist += hypot(dx, dy);
        if (accum_dist >= Ld) {
            tgt_idx = i + 1;
            break;
        }
    }
    return tgt_idx;
}

// ========== 定位回调 ===========
void statusCallback(const yjkj_location_msgs::Location::ConstPtr &msg) {
    curr_x = msg->pos.x;            // ENU x
    curr_y = msg->pos.y;            // ENU y
    curr_yaw = msg->orientation.z;  // Yaw (rad)
    curr_v = std::hypot(msg->velocity.x, msg->velocity.y);
    state_ready = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit_tracking");
    ros::NodeHandle nh;

    // 读取轨迹文件名参数
    std::string trajfile;
    nh.param<std::string>("traj_csv_path", trajfile,
                          "/home/likun/deploy_in_x86_64_noetic/data/NovAtel2025-05-09-09-59-44.csv");

    // 读入轨迹
    reference_traj = readCsvTraj(trajfile);
    if (reference_traj.empty()) {
        ROS_ERROR("NO FILE");
        return 1;
    }
    ROS_INFO("THE POINT NUMBER: %lu", reference_traj.size());

    // 订阅定位，发布指令
    ros::Subscriber loc_sub = nh.subscribe("/yjkj/sensor/gnss/location", 10, statusCallback);
    ros::Publisher cmd_pub = nh.advertise<yjkj_control_msgs::VehicleCMD>("/yjkj/VehicleCMD", 5);

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        if (state_ready) {
            // Pure Pursuit目标点查找
            size_t tgt_idx = searchTargetIdx(curr_x, curr_y, curr_v, reference_traj);
            const TrajPoint &goal = reference_traj[tgt_idx];

            // 1. 目标点到车头的全局方位角
            double angle_to_goal = atan2(goal.y - curr_y, goal.x - curr_x);
            // 2. 相对偏航角 α（-π..π）
            double alpha = angle_to_goal - curr_yaw;

            // 3. 视距
            double Ld = hypot(goal.x - curr_x, goal.y - curr_y);

            // 4. 计算转向角 δ
            double delta = atan2(2.0 * WHEELBASE * sin(alpha), Ld);

            // 5. 限幅（同之前）
            delta = std::max(-MAX_STEER_RAD, std::min(MAX_STEER_RAD, delta));

            // 限幅
            if (delta > MAX_STEER_RAD) delta = MAX_STEER_RAD;
            if (delta < -MAX_STEER_RAD) delta = -MAX_STEER_RAD;

            // 填充并发布控制指令
            yjkj_control_msgs::VehicleCMD cmd;
            cmd.vehicle_speed = 0.8;                              // 目标点速度
            cmd.steer_angle = delta * 16.43;  // 前轮目标转角[rad]
            cmd.gear = 4;// PRND 1234
            cmd.header.stamp = ros::Time::now();
            cmd_pub.publish(cmd);

            // 末端停车
            if (tgt_idx > reference_traj.size() - 3) {
                cmd.vehicle_speed = 0.0;
                cmd.steer_angle = 0.0;
                cmd.gear = 2;
                cmd_pub.publish(cmd);
                ROS_INFO("OVER");
                break;
            }
        }
        rate.sleep();
    }
    return 0;
}