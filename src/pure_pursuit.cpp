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

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
    // initialise parameters
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
    this->declare_parameter("K_p", 0.5);
    this->declare_parameter("steering_limit", 25.0);
    this->declare_parameter("velocity_percentage", 0.6);
    //내가 추가
    this->declare_parameter("waypoint_search_range", 500.0);  // 새로운 파라미터 선언

    // Default Values
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
    K_p = this->get_parameter("K_p").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    /////내가 추가
    waypoint_search_range = this->get_parameter("waypoint_search_range").as_int();  // 새로운 파라미터 초기화

    // waypoints.index 초기화
    waypoints.index = 0;  // 초기값을 0으로 설정
    ////여기까지 내가 추가

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
    timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    vis_current_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
    vis_lookahead_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched");

    load_waypoints();
}

double PurePursuit::to_radians(double degrees) {
    double radians;
    return radians = degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
    double degrees;
    return degrees = radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(double &x1, double &x2, double &y1, double &y2) { //포인트 간의 거리계산
    double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dist;
}

void PurePursuit::load_waypoints() {
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    if (!csvFile_waypoints.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path);
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "CSV File Opened");
    }

    // std::vector<std::string> row;
    std::string line, word, temp;

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
                } else if (j == 2) {
                    waypoints.V.push_back(std::stod(word));
                }
            }
            j++;
        }
    }

    csvFile_waypoints.close();
    num_waypoints = waypoints.X.size();
    RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path);

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


void PurePursuit::check_obstacle(){
    
    //세찬
    //웨이포인트와 차량으 현재 위치를 받아와서 그 사이에 장애물 유무 판단
    //장애물 BUBBLE까지 고려.
    //리턴 : true or false
    //장애물 있으면 True / 없으면 False
}
void PurePursuit::set_velocity(){
    //원빈
    //웨이포인트까지의 거리를 계산해서 velocity를 계산하는 함수 작성
}
void PurePursuit::new_get_waypoint(){
    //선환
    //세찬이의 함수를 받아와서
    //현재 위치와 가장 가까운 인덱스 계산
    //이때 MAX_LOOKAHEDAD 보다 작은 인덱스를 불러와야함
     // Get the current vehicle position
    double current_x = x_car_world;
    double current_y = y_car_world;
    double longest_distance = 0;
    // Adjust lookahead distance based on speed
    double lookahead_distance = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);
    
    int farthest_waypoint_index = -1;
    int start = waypoints.index;
    int end = (waypoints.index + waypoint_search_range) % num_waypoints;

// Lookahead needs to be between the min_lookhead and the max_lookahead
    double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);

    if (end < start) {  // If we need to loop around
        for (int i = start; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                farthest_waypoint_index = i;
            }
        }
        for (int i = 0; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                farthest_waypoint_index = i;
            }
        }
    } else {
        for (int i = start; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                farthest_waypoint_index = i;
            }
        }
    }

    if (farthest_waypoint_index == -1) {  // if we haven't found anything, search from the beginning
        farthest_waypoint_index = 0;
        for (int i = 0; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                farthest_waypoint_index = i;
            }
        }
    }
    
    waypoints.index = farthest_waypoint_index;
    RCLCPP_INFO(this->get_logger(), "New farthest waypoint: %d with distance difference: %f", farthest_waypoint_index, longest_distance);
    
    int waypoint_index = waypoints.index;
    // 장애물 판단 함수가 장애물이 없다고 할 때까지 반복
    while (check_obstacle(waypoints.X[waypoint_index], waypoints.Y[waypoint_index])) {
        // 장애물이 있을 경우, 이전 웨이포인트로 이동
        if (waypoint_index > 0) {
            waypoint_index--;  // 인덱스를 이전 웨이포인트로 이동
        } else {
            // 인덱스가 0인 경우, 리스트의 끝으로 이동 (순환형 리스트 처리)
            waypoint_index = num_waypoints - 1;
        }

        // 만약 모든 웨이포인트가 장애물로 막혀있다면 루프 탈출
        if (waypoint_index == farthest_waypoint_index) {
            RCLCPP_WARN(this->get_logger(), "No valid waypoint found. All waypoints blocked by obstacles.");
            return;
        }
    }

    // 최종적으로 장애물이 없는 웨이포인트를 결정
    RCLCPP_INFO(this->get_logger(), "Safe waypoint found: %d", waypoint_index);
    waypoints.index = waypoint_index;
    


}


void PurePursuit::get_waypoint() {
    /*start와 end의 정의:

    start: 현재 경로 탐색이 시작되는 인덱스 (waypoints.index).
    end: start부터 500개의 포인트를 탐색할 때 그 끝 인덱스 ((waypoints.index + 500) % num_waypoints).
    여기서 num_waypoints는 전체 경로 포인트의 개수입니다. % num_waypoints를 사용
해 인덱스가 num_waypoints를 초과하지 않도록 순환 구조를 유지하고 있습니다.

    if end < start가 참인 경우 (배열의 끝을 넘는 경우):

    이 조건은 end가 start보다 작을 때, 즉 경로가 배열의 끝을 넘어서 다시 배열의 처음으로 돌아가는 상황을 나타냅니다. 순환 배열에서는 이런 상황이 발생할 수 있으>며, 이때 경로를 두 번에 나누어 탐색해야 합니다.
    start부터 배열의 끝까지 (start에서 num_waypoints - 1까지)를 탐색.
    배열의 처음부터 end까지 (0에서 end - 1까지)를 탐색.

    if end >= start인 경우 (배열의 끝을 넘지 않는 경우):

    경로가 배열의 끝을 넘지 않으면, 단순히 start부터 end까지 한 번에 탐색하면 됩
니다.
    요약:
    end < start: 경로 탐색 범위가 배열의 끝을 넘어가서 다시 처음부터 탐색해야 하
는 경우.
    end >= start: 경로 탐색 범위가 배열의 중간에서 끝나는 경우로, 순차적으로 탐>색할 수 있습니다.
    */


    // Main logic: Search within the next 500 points
    double longest_distance = 0;
    int final_i = -1;
    int start = waypoints.index; 
    int end = (waypoints.index + waypoint_search_range) % num_waypoints; //default waypoint_search_range = 500

//num_waypoints = 312
//start < end
//start       end    
//0           500 % 312 = 188
//1           501 % 312 = 189
//2           502 % 312 = 190
//123         623 % 312 = 311


//----------------------------
//end < start // If we need to loop around
//start       end  
//124         624 % 312 = 0
//300         800 % 312 = 176
//311         811 % 312 = 187





    // Lookahead needs to be between the min_lookhead and the max_lookahead
    double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);

    if (end < start) {  // If we need to loop around
        for (int i = start; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
        for (int i = 0; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    } else {
        for (int i = start; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    }

    if (final_i == -1) {  // if we haven't found anything, search from the beginning
        final_i = 0;
        for (int i = 0; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    }

    // Find the closest point to the car, and use the velocity index for that
    double shortest_distance = p2pdist(waypoints.X[0], x_car_world, waypoints.Y[0], y_car_world);
    int velocity_i = 0;
    for (int i = 0; i < num_waypoints; i++) {
        if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= shortest_distance) {
            shortest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
            velocity_i = i;
        }
    }

    // If a waypoint is not found within our radius, then waypoints.index = 0
    waypoints.index = final_i;
    waypoints.velocity_index = velocity_i;
}

void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3) { //쿼터니언을 회전 행렬로 변환
    double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);  
    double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
    double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

    double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
    double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
    double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

    double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
    double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
    double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

    rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22;
}

void PurePursuit::transformandinterp_waypoint() {  // pass old waypoint here
    // initialise vectors
    waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
    waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0;

    visualize_lookahead_point(waypoints.lookahead_point_world);
    visualize_current_point(waypoints.current_point_world);

    // look up transformation at that instant from tf_buffer_
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        // Get the transform from the base_link reference to world reference frame
        transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
    }

    // transform points (rotate first and then translate)
    Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
}

double PurePursuit::p_controller() { //차량이 목표 지점을 향해 회전하는 데 필요한 조향 각도를 계산
    double r = waypoints.lookahead_point_car.norm();  // r = sqrt(x^2 + y^2)
    double y = waypoints.lookahead_point_car(1);
    double angle = K_p * 2 * y / pow(r, 2);  // Calculated from https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

    return angle;
}

double PurePursuit::get_velocity(double steering_angle) {
    double velocity = 0;

    if (waypoints.V[waypoints.velocity_index]) {
        velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;
    } else {  // For waypoints loaded without velocity profiles
        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 6.0 * velocity_percentage;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            velocity = 2.5 * velocity_percentage;
        } else {
            velocity = 2.0 * velocity_percentage;
        }
    }

    return velocity;
}

void PurePursuit::publish_message(double steering_angle) {
    auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
    if (steering_angle < 0.0) {
        drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit));  // ensure steering angle is dynamically capable
    } else {
        drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit));  // ensure steering angle is dynamically capable
    }

    curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
    drive_msgObj.drive.speed = curr_velocity;

    RCLCPP_INFO(this->get_logger(), "index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Steering Angle: %.2f ... K_p: %.2f ... velocity_percentage: %.2f", waypoints.index, p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world), drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle), K_p, velocity_percentage);

    publisher_drive->publish(drive_msgObj);
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    x_car_world = odom_submsgObj->pose.pose.position.x;
    y_car_world = odom_submsgObj->pose.pose.position.y;
    // interpolate between different way-points
    get_waypoint();

    // use tf2 transform the goal point
    transformandinterp_waypoint();

    // Calculate curvature/steering angle
    double steering_angle = p_controller();

    // publish object and message: AckermannDriveStamped on drive topic
    publish_message(steering_angle);
}

void PurePursuit::timer_callback() {
    // Periodically check parameters and update
    K_p = this->get_parameter("K_p").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
