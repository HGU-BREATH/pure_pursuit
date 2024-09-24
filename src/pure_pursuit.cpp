#include "pure_pursuit.hpp"

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
#include <sstream>
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
    // 파라미터 선언
    this->declare_parameter("waypoints_path", "/sim_ws/src/pure_pursuit/racelines/e7_floor5.csv");
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("car_refFrame", "ego_racecar/base_link");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
    this->declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
    this->declare_parameter("global_refFrame", "map");
    this->declare_parameter("min_lookahead", 0.5);
    this->declare_parameter("max_lookahead", 1.0);
    this->declare_parameter("lookahead_ratio", 8.0);
    this->declare_parameter("K_p", 1.0); // K_p 파라미터 선언 및 초기값 설정
    this->declare_parameter("steering_limit", 25.0);
    this->declare_parameter("velocity_percentage", 0.6);
    this->declare_parameter("waypoint_search_range", 500);
    this->declare_parameter("wheelbase", 0.33); // wheelbase를 파라미터로 선언

    // 파라미터 초기화
    waypoints_path = this->get_parameter("waypoints_path").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    car_refFrame = this->get_parameter("car_refFrame").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    rviz_current_waypoint_topic = this->get_parameter("rviz_current_waypoint_topic").as_string();
    rviz_lookahead_waypoint_topic = this->get_parameter("rviz_lookahead_waypoint_topic").as_string();
    global_refFrame = this->get_parameter("global_refFrame").as_string();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    K_p = this->get_parameter("K_p").as_double(); // K_p 파라미터 초기화
    steering_limit = this->get_parameter("steering_limit").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    waypoint_search_range = this->get_parameter("waypoint_search_range").as_int();
    wheelbase = this->get_parameter("wheelbase").as_double(); // wheelbase 파라미터 초기화

    // waypoints.index 초기화
    waypoints.index = 0;

    // 변수 초기화
    curr_velocity = 0.0;

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
    timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

    subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&PurePursuit::scan_callback, this, std::placeholders::_1));

    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    vis_current_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
    vis_lookahead_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched");

    load_waypoints();
}

double PurePursuit::to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
    return radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(double &x1, double &x2, double &y1, double &y2) {
    double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dist;
}

void PurePursuit::load_waypoints() {
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    if (!csvFile_waypoints.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path.c_str());
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "CSV File Opened");
    }

    std::string line, word;

    while (!csvFile_waypoints.eof()) {
        std::getline(csvFile_waypoints, line, '\n');
        std::stringstream s(line);

        int j = 0;
        while (getline(s, word, ',')) {
            if (!word.empty()) {
                if (j == 0) {
                    waypoints.X.push_back(std::stod(word));
                } else if (j == 1) {
                    waypoints.Y.push_back(std::stod(word));
                }
                // V 값을 읽지 않음
            }
            j++;
        }
    }

    csvFile_waypoints.close();
    num_waypoints = waypoints.X.size();
    RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path.c_str());

    double average_dist_between_waypoints = 0.0;
    for (int i = 0; i < num_waypoints - 1; i++) {
        average_dist_between_waypoints += p2pdist(waypoints.X[i], waypoints.X[i + 1], waypoints.Y[i], waypoints.Y[i + 1]);
    }
    average_dist_between_waypoints /= num_waypoints;
    RCLCPP_INFO(this->get_logger(), "Average distance between waypoints: %f", average_dist_between_waypoints);
}

void PurePursuit::visualize_lookahead_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_lookahead_point_pub->publish(marker);
}

void PurePursuit::visualize_current_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.b = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_current_point_pub->publish(marker);
}

bool PurePursuit::check_obstacle(double x, double y) {
    /// 차량의 orientation (쿼터니언에서 yaw 값 추출)
    double q_x = last_odom_msg->pose.pose.orientation.x;
    double q_y = last_odom_msg->pose.pose.orientation.y;
    double q_z = last_odom_msg->pose.pose.orientation.z;
    double q_w = last_odom_msg->pose.pose.orientation.w;

    // 쿼터니언을 RPY(roll, pitch, yaw)로 변환
    tf2::Quaternion q(q_x, q_y, q_z, q_w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 목표 좌표와 차량 좌표 간 벡터 계산
    double dx = x - x_car_world;
    double dy = y - y_car_world;

    // 목표 지점까지의 각도 (atan2로 구함, 라디안 단위)
    double target_theta = atan2(dy, dx);

    // 차량 정면(yaw)을 기준으로 상대적인 각도 계산
    double relative_theta = target_theta - yaw;

    // 각도 차이를 [-π, π] 범위로 조정
    if (relative_theta > M_PI) {
        relative_theta -= 2 * M_PI;
    } else if (relative_theta < -M_PI) {
        relative_theta += 2 * M_PI;
    }

    // 라디안 값을 도(degrees)로 변환
    double relative_angle_degrees = relative_theta * 180.0 / M_PI;

    // 상대 각도를 라이다 스캔 데이터의 인덱스로 변환
    double lidar_index = (relative_angle_degrees + 135.0) / 270.0 * 1081.0;

    RCLCPP_INFO(this->get_logger(), "차량의 위치는 %f, %f, 목표 위치는 %d 위치에 있습니다. 거리는 %f 입니다.", x_car_world, y_car_world, static_cast<int>(lidar_index), last_scan_msg->ranges[lidar_index]);
    
    if(abs(p2pdist(x, x_car_world, y, y_car_world)-last_scan_msg->ranges[lidar_index])>1) {
        RCLCPP_INFO(this->get_logger(), "라이다데이터는 %f이므로 장애물이 있음", last_scan_msg->ranges[lidar_index]);
        return true;
    }
    return false;
}

void PurePursuit::set_velocity(double steering_angle) {
    // 최소 및 최대 속도 (m/s)
    double minVelocity = 2.0;
    double maxVelocity = 6.0;

    // 최대 조향각 (라디안 단위)
    double maxSteeringAngle = to_radians(steering_limit);

    // reductionFactor의 최소값 설정
    double minReductionFactor = 0.1;

    // 현재 차량 위치와 웨이포인트 간의 실제 거리 계산
    double realDistance = p2pdist(x_car_world, waypoints.X[waypoints.index],
                                  y_car_world, waypoints.Y[waypoints.index]);

    // realDistance를 mld로 제한하여 0에서 1 사이로 정규화
    double mld = max_lookahead;
    double effectiveDistance = std::min(std::max(realDistance, 0.0), mld);
    double normalizedDistance = effectiveDistance / mld;

    // 거리 기반 속도 계산 (비선형 함수 적용)
    double gamma = 0.5;
    double distanceVelocity = minVelocity + (maxVelocity - minVelocity) *
                              pow(normalizedDistance, gamma);

    // 현재 조향각의 절대값 (라디안 단위)
    double absSteeringAngle = std::abs(steering_angle);

    // cosine 함수 사용하여 감소 비율 계산
    double reductionFactor = cos((absSteeringAngle / maxSteeringAngle) * (M_PI / 2));

    // 감소 비율을 최소값과 1.0 사이로 제한
    reductionFactor = std::max(std::min(reductionFactor, 1.0), minReductionFactor);

    // 최종 속도 계산
    double finalVelocity = distanceVelocity * reductionFactor;

    // 속도를 최소 및 최대 속도로 제한
    finalVelocity = std::max(std::min(finalVelocity, maxVelocity), minVelocity);

    // 계산된 속도를 curr_velocity에 저장
    curr_velocity = finalVelocity;

    RCLCPP_INFO(this->get_logger(),
        "Distance Velocity: %.2f m/s, Reduction Factor: %.2f, Final Velocity: %.2f m/s",
        distanceVelocity, reductionFactor, curr_velocity);
}

void PurePursuit::get_waypoint() {
    // 장애물을 고려한 웨이포인트 선택 로직

    // 현재 차량 위치
    double x_car_world = this->x_car_world;
    double y_car_world = this->y_car_world;

    double longest_distance = 0;
    int farthest_waypoint_index = -1;
    int start = waypoints.index;
    int end = (waypoints.index + waypoint_search_range) % num_waypoints;

    // 전방주시 거리를 계산
    double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);

    // 거리 차이 및 최소값 초기화
    double dist = -1;
    double ld_diff;
    double min_ld_diff = std::numeric_limits<double>::max();

    

    // 웨이포인트를 처리하는 함수
    auto process_waypoints = [&](int start_idx, int end_idx) {
        for (int i = start_idx; i != end_idx; i = (i + 1) % num_waypoints) {
            dist = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
            ld_diff = lookahead - dist;

            if (ld_diff >= 0 && ld_diff <= min_ld_diff) {
                min_ld_diff = ld_diff;

                if (!check_obstacle(waypoints.X[i], waypoints.Y[i])) {
                    farthest_waypoint_index = i;
                } else {
                    farthest_waypoint_index = (i > 0) ? i - 1 : num_waypoints - 1;
                    
                    break;
                }
            }
            else{
                break;
            }
        }
    };

    // 웨이포인트 처리
    if (end < start) {
        process_waypoints(start, num_waypoints);
        process_waypoints(0, end);
    } else {
        process_waypoints(start, end);
    }

    if (farthest_waypoint_index == -1) {
        for (int i = 0; i < num_waypoints; i++) {
            dist = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
            ld_diff = lookahead - dist;

            if (ld_diff >= 0 && ld_diff < min_ld_diff) {
                min_ld_diff = ld_diff;
                farthest_waypoint_index = i;
            }
        }
    }

    waypoints.index = farthest_waypoint_index;
    RCLCPP_INFO(this->get_logger(), "Safe waypoint found: %d", waypoints.index);
}

void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3) {
    double r00 = 2.0 * (q0 * q0 + q1 * q1) - 1.0;
    double r01 = 2.0 * (q1 * q2 - q0 * q3);
    double r02 = 2.0 * (q1 * q3 + q0 * q2);

    double r10 = 2.0 * (q1 * q2 + q0 * q3);
    double r11 = 2.0 * (q0 * q0 + q2 * q2) - 1.0;
    double r12 = 2.0 * (q2 * q3 - q0 * q1);

    double r20 = 2.0 * (q1 * q3 - q0 * q2);
    double r21 = 2.0 * (q2 * q3 + q0 * q1);
    double r22 = 2.0 * (q0 * q0 + q3 * q3) - 1.0;

    rotation_m << r00, r01, r02,
                  r10, r11, r12,
                  r20, r21, r22;
}

void PurePursuit::transformandinterp_waypoint() {
    // 벡터 초기화
    waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;

    // 변환 얻기
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform. Error: %s", ex.what());
        return; // 함수 실행 중단
    }

    // 포인트 변환
    Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
}

double PurePursuit::p_controller() {
    double x = waypoints.lookahead_point_car(0);
    double y = waypoints.lookahead_point_car(1);

    // K_p를 포함하여 조향각 계산
    double angle = K_p * atan2(2 * wheelbase * y, x * x + y * y);

    return angle;
}

void PurePursuit::publish_message(double steering_angle) {
    auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();

    if (steering_angle < 0.0) {
        drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit));
    } else {
        drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit));
    }

    drive_msgObj.drive.speed = curr_velocity;

    RCLCPP_INFO(this->get_logger(), "Index: %d ... Distance: %.2f m ... Speed: %.2f m/s ... Steering Angle: %.2f degrees",
                waypoints.index,
                p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world),
                drive_msgObj.drive.speed,
                to_degrees(drive_msgObj.drive.steering_angle));

    publisher_drive->publish(drive_msgObj);
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    x_car_world = odom_submsgObj->pose.pose.position.x;
    y_car_world = odom_submsgObj->pose.pose.position.y;

    // 웨이포인트 업데이트
    get_waypoint();

    // 웨이포인트 변환 및 보간
    transformandinterp_waypoint();

    // 조향각 계산
    double steering_angle = p_controller();

    // 속도 설정 (조향각을 인자로 전달)
    set_velocity(steering_angle);
    last_odom_msg = odom_submsgObj;
    // 메시지 퍼블리시
    publish_message(steering_angle);
}

void PurePursuit::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // LaserScan
    RCLCPP_INFO(this->get_logger(), "스캔 메시지를 받았습니다. %zu개의 범위 값이 있습니다.", scan_msg->ranges.size());
    last_scan_msg = scan_msg;
    // scan_msg->ranges[0]
    checkBlock(1.0, 1.0);
}

void PurePursuit::timer_callback() {
    // 주기적으로 파라미터 업데이트
    K_p = this->get_parameter("K_p").as_double(); // K_p 파라미터 업데이트
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
    wheelbase = this->get_parameter("wheelbase").as_double(); // wheelbase 파라미터 업데이트
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
