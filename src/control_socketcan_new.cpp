#include <fcntl.h>
#include <iostream>
#include "ros/ros.h"
#include "yjkj_common_msgs/CANFrame.h"
#include "yjkj_chassis_msgs/Chassis.h"
#include "yjkj_control_msgs/VehicleCMD.h"
#include <yjkj_log/log.h>

using namespace std;

// 全局变量，存放反馈
short back_wheel_speed = 0.0;  // 上报的后轮速度
short turn_angle = 0.0;        // 上报的转向角度
short battery_level = 0.0;     // 上报的电量百分比
short error_flag = 0.0;        // 上报的错误状态

ros::Publisher chassis_pub;  // 发布底盘（车辆）状态
ros::Publisher can_sender;   // 发布CAN下发帧

/**
 * @description: 发送CAN数据到底盘
 * @param {float} lineSpeed
 * @param {float} angle
 * @return {*}
 */
void SendSpeedToAKM(float lineSpeed, float angle) {
    int LineSpeed = 0, Angle = 0;

    // m/s转化为mm/s
    LineSpeed = lineSpeed * 1000;
    Angle = angle * 100 * 180 / M_PI;

    yjkj_common_msgs::CANFrame messages;
    messages.id = 0x00000001;
    messages.dlc = 8;
    messages.is_extended = 0;
    // 要写入的数据
    messages.data[0] = 0x00000001;
    messages.data[1] = 1;
    messages.data[2] = LineSpeed & 0xFF;
    messages.data[3] = (LineSpeed >> 8) & 0xFF;
    messages.data[4] = Angle & 0xFF;
    messages.data[5] = (Angle >> 8) & 0xFF;
    messages.data[6] = 0x00;
    messages.data[7] = 0x00;
    can_sender.publish(messages);
}

/**
 * @description: 控制命令回调（接收/下发话题）
 * @param {VehicleCMD} &cmd 控制命令
 * @return {*}
 */
void vehicle_cmd_Callback(const yjkj_control_msgs::VehicleCMD &cmd) {
    ROS_INFO("vehicle_cmd_Callback!");
    float line_speed = cmd.vehicle_speed;   // VehicleCMD里面就是m/s
    float angle = cmd.steering_tire_angle;  // VehicleCMD里面就是rad
    SendSpeedToAKM(line_speed, angle);
}

/**
 * @description: CAN反馈回调，解析发布到底盘反馈消息
 * @param {CANFrame} &rev_msg
 * @return {*}
 */
void received_velCallback(const yjkj_common_msgs::CANFrame &rev_msg) {
    if (rev_msg.id == 0x01) {
        yjkj_chassis_msgs::Chassis send_msg;
        ROS_INFO("received_velCallback!");
        if ((rev_msg.data[0] == 0x02) && (rev_msg.data[1] == 0x02)) {
            back_wheel_speed = (rev_msg.data[3] << 8) | rev_msg.data[2];  // 上报的后轮速度
            turn_angle = (rev_msg.data[5] << 8) | rev_msg.data[4];        // 上报的转向角度
            battery_level = rev_msg.data[6];                              // 上报的电量百分比
            error_flag = rev_msg.data[7];                                 // 上报的错误状态
            AINFO << "后轮速度(mm/s): " << back_wheel_speed;
            AINFO << "转向角度(deg): " << turn_angle / 100.0;
            AINFO << "错误状态: " << error_flag;
            AINFO << "电量百分比: " << battery_level;

            // Chassis消息字段填充
            send_msg.header.stamp = ros::Time::now();
            send_msg.speed = back_wheel_speed / 1000.0;                // mm/s -> m/s
            send_msg.steer_angle = turn_angle / 100.0 * M_PI / 180.0;  // 1/100° -> 弧度
            send_msg.left_back_wheelspeed = send_msg.speed;
            send_msg.right_back_wheelspeed = send_msg.speed;
            send_msg.control_status = error_flag;
            // 如需battery_level,确定Chassis.msg有battery_level字段再补充
            // send_msg.battery_level = battery_level; （假如有该字段）
        }
        chassis_pub.publish(send_msg);
    }
}

int main(int argc, char **argv) {
    ROS_INFO("control is beginning!");

    ros::init(argc, argv, "robot_control_mt100");
    ros::NodeHandle n;
    // 1. 车辆状态反馈发布话题
    chassis_pub = n.advertise<yjkj_chassis_msgs::Chassis>("/yjkj/VehicleStatus", 10);
    // 2. 控制下发话题（VehicleCMD）
    ros::Subscriber vehicle_cmd_sub = n.subscribe("/yjkj/VehicleCMD", 10, vehicle_cmd_Callback);
    // 3. CAN反馈
    ros::Subscriber received_can_sub = n.subscribe("/can0/received_messages", 10, received_velCallback);
    // 4. CAN下发
    can_sender = n.advertise<yjkj_common_msgs::CANFrame>("/can0/sent_messages", 10);

    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
