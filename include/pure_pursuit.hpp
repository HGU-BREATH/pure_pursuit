#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

/*
Pure Pursuit 알고리즘의 C++ 구현입니다. 동적 전방주시 거리와 LIDAR 데이터를 활용한 장애물 회피 기능을 포함하고 있습니다.
*/

#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit();

private:
    // 웨이포인트 데이터를 저장하기 위한 구조체
    struct csvFileData {
        std::vector<double> X;
        std::vector<double> Y;
        std::vector<double> V;

        int index;
        int velocity_index;

        Eigen::Vector3d lookahead_point_world;  // 월드 좌표계의 전방주시점
        Eigen::Vector3d lookahead_point_car;    // 차량 좌표계의 전방주시점
        Eigen::Vector3d current_point_world;    // 현재 웨이포인트 위치
    };

    // 멤버 변수
    Eigen::Matrix3d rotation_m;

    double x_car_world;
    double y_car_world;

    std::string odom_topic;
    std::string car_refFrame;
    std::string drive_topic;
    std::string global_refFrame;
    std::string rviz_current_waypoint_topic;
    std::string rviz_lookahead_waypoint_topic;
    std::string waypoints_path;
    double K_p;
    double min_lookahead;
    double max_lookahead;
    double lookahead_ratio;
    double steering_limit;
    double velocity_percentage;
    double curr_velocity;
    double wheelbase;
    int waypoint_search_range;
    bool emergency_breaking;
    std::string lane_number;

    // 웨이포인트 파일 객체
    std::fstream csvFile_waypoints;

    // 웨이포인트 데이터
    csvFileData waypoints;
    int num_waypoints;

    // 타이머
    rclcpp::TimerBase::SharedPtr timer_;

    // 구독자
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;

    // 퍼블리셔
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub;

    // TF 변환
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // 최근 수신한 메시지
    nav_msgs::msg::Odometry::ConstSharedPtr last_odom_msg;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg;

    // 개인 멤버 함수
    double to_radians(double degrees);
    double to_degrees(double radians);
    double p2pdist(double &x1, double &x2, double &y1, double &y2);

    void load_waypoints();

    void visualize_lookahead_point(Eigen::Vector3d &point);
    void visualize_current_point(Eigen::Vector3d &point);

    void get_waypoint();

    void quat_to_rot(double q0, double q1, double q2, double q3);

    void transformandinterp_waypoint();

    double p_controller();

    void publish_message(double steering_angle);

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void timer_callback();

    void set_velocity(double steering_angle);

    bool check_obstacle(double x, double y);

    void checkBlock(double x, double y);  // 추가된 함수 선언
};

#endif  // PURE_PURSUIT_HPP
