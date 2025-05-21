#ifndef SPLINE_FOLLOWER_HPP
#define SPLINE_FOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <Eigen/Dense>
#include <fstream>

using json = nlohmann::json;

class SplineFollower : public rclcpp::Node {
public:
    SplineFollower();
    ~SplineFollower();

    // Declare public member functions for use in the main
    void setSafeStartPose();
    void addGroundPlane();
    void addCanvasPlane();
    bool loadSplines();
    double calculateAverageCanvasHeight();
    const std::vector<geometry_msgs::msg::Pose> computeLinearInterpolationPath(
            const geometry_msgs::msg::Pose& start_pose, 
            const geometry_msgs::msg::Pose& end_pose, 
            int num_waypoints);
    void sendError(bool drawing_incomplete); // Function to publish error message to GUI
    bool generateBorderSpline(double x_offset, double y_offset); // Generates a border spline
    bool generateSignageSpline();
    void exportSplineToCSV(const std::string& filename);
    void waitForContinue(); // Wait function for async debugging
    
    // Declare and define state enum for state machine
    enum class State {
        INIT,
        MOVE_TO_INTERMEDIATE_POS,
        MOVE_TO_CANVAS,
        MOVE_THROUGH_DRAWING_TRAJECTORY,
        MOVE_OFF_CANVAS,
        IDLE,
        STOP,
        SERVICE
    };

    // Declare public member variables for use in main
    geometry_msgs::msg::Pose safe_start_pose_;
    State state_;
    nlohmann::json spline_data_;
    size_t current_spline_index_;
    geometry_msgs::msg::Pose intermediate_pose_;
    geometry_msgs::msg::Pose canvas_pose_;
    geometry_msgs::msg::Pose original_pose_;

    // Declare Publisher & Subscribers for communication with other subsystems
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr toolpath_sub_; // Subscriber to listen for the toolpaths
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_; // Publisher to communicate to the gui
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_; // Publisher to send state to GUI
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr shutdown_sub_; // Subscriber to recieve a shutdown message
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr continue_sub_; // Debug subscriber to allow asynchronous state execution
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr service_sub_; // Subscriber to listen for a service request

    // Variables for shutting down the system
    bool shutdown_;

    // Variables for Debug
    std::mutex continue_mutex_;
    std::condition_variable continue_cv_;
    bool continue_received_ = false; // Debug flag

    // Canvas and lifted z value members
    double lifted_z_;
    double canvas_z_;
    double tooltip_offset_;

    // Planning interface member for canvas collision avoidance
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Variables for servicing
    bool service_started_;

private:
    void toolpath_sub_callback(const std_msgs::msg::String::SharedPtr msg); // Toolpath Callback function
    void process_toolpath_to_json(const std::string& json_str); // Method to process toolpaths to json
    void shutdownCallback(const std_msgs::msg::Empty::SharedPtr msg); // Shutdown sub callback
    void continueCallback(const std_msgs::msg::Empty::SharedPtr msg); // Debug sub callback
    void serviceCallback(const std_msgs::msg::Empty::SharedPtr msg); // Service sub callback
    void addInitObstacles(Eigen::Vector3d canvas_center, double canvas_x, double canvas_y);
    void publishState();

    std::thread state_pub_thread_;
    std::atomic_bool keep_publishing_state_;
    
    void statePublishingLoop();  // Function for the thread
};

#endif  // SPLINE_FOLLOWER_HPP
