#include <memory>
#include <thread>
#include "spline_follower.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <iostream>
#include <string>


int main(int argc, char * argv[])
{
    // Initialize ROS and create the MoveIt Connection Node
    rclcpp::init(argc, argv);
    auto const moveit = std::make_shared<rclcpp::Node>(
        "selfiebot_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("selfiebot_control");

    // Create shared pointer to Spline Follower Class
    auto control = std::make_shared<SplineFollower>();

    // Spin up a SingleThreadedExecutor for the current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(moveit);
    executor.add_node(control);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(moveit, "ur_manipulator");

    // Set the Max Velocity and Acceleration Factors
    move_group_interface.setMaxVelocityScalingFactor(0.1);  // 1.0 = 100% of max velocity
    move_group_interface.setMaxAccelerationScalingFactor(1.0); // 1.0 = 100% of max acceleration

    // Add in a ground plane at z = 0 to ensure the robot does not move through its base during intermediate movements
    control->addGroundPlane();

    // Set the maximum amount of retries for a state to attempt its objective
    unsigned int MAX_RETRIES = 3;
    unsigned int RETRIES = 0;

    bool isIdle = false;

    // Enter state machine for drawing lifecylce
    while (rclcpp::ok()) {
        // Check retries and shutdown if max retries has been reached
        if(RETRIES > MAX_RETRIES){ // If the max retries has been reached report and shutdown the system
            RCLCPP_ERROR(logger, "The maximum amount of retries has been reached. Returning to idle state.");
            // Send error message to the GUI
            bool drawing_incomplete = control->current_spline_index_ != control->spline_data_.size();
            control->sendError(drawing_incomplete);
            control->state_ = SplineFollower::State::IDLE;
            isIdle = false;
            RETRIES = 0;
        }
        // If we recieve the shutdown signal enter the stop state - return home and shutodwn safely. 
        if(control->shutdown_){
            control->state_ = SplineFollower::State::STOP;
        }
        switch (control->state_) {
            // State init: Moves to a safe start pose using the path planner.
            case SplineFollower::State::INIT:
            {
                RCLCPP_INFO(logger, "State: Init");

                isIdle = false;

                control->addCanvasPlane(); // Add in the canvas collision object

                // control->waitForContinue(); // For debug

                // Set the safe start pose: saved to control->safe_start_pose_
                control->setSafeStartPose();

                // Create a path constraint and joint constraints
                moveit_msgs::msg::Constraints constraint; // Create path constraint msg
                std::vector<moveit_msgs::msg::JointConstraint> joint_constraints; // Create joint constraint vector

                 // --- Shoulder Lift Joint Constraint (locked around -45 degrees ±45 deg) ---
                double shoulder_joint_target = -M_PI/4.0; // Set to -45 deg
                moveit_msgs::msg::JointConstraint shoulder_lift_joint_constraint;  // Create specific joint constraint
                shoulder_lift_joint_constraint.set__joint_name("shoulder_lift_joint"); // Set to shoulder_lift_joint
                shoulder_lift_joint_constraint.set__position(shoulder_joint_target); // Set the joint target value
                shoulder_lift_joint_constraint.set__tolerance_above(M_PI/4.0); // Set a reasonable tolerance above
                shoulder_lift_joint_constraint.set__tolerance_below(M_PI/4.0); // Set a reasonable tolerance above
                shoulder_lift_joint_constraint.set__weight(1.0); // Set the weight
                joint_constraints.push_back(shoulder_lift_joint_constraint);

                // --- Wrist 1 Constraint (-140 deg = -2.443 radians) ---
                moveit_msgs::msg::JointConstraint wrist_1_joint_constraint;
                wrist_1_joint_constraint.set__joint_name("wrist_1_joint");
                wrist_1_joint_constraint.set__position(-140.0/180.0*M_PI); // Wrist Out
                // wrist_1_joint_constraint.set__position(50.0/180.0*M_PI); // Wrist In
                wrist_1_joint_constraint.set__tolerance_above(M_PI / 2.0); // ±90 deg
                wrist_1_joint_constraint.set__tolerance_below(M_PI / 2.0);
                wrist_1_joint_constraint.set__weight(1.0);
                joint_constraints.push_back(wrist_1_joint_constraint);

                // --- Wrist 2 Constraint (-90 deg = -M_PI/2) ---
                moveit_msgs::msg::JointConstraint wrist_2_joint_constraint;
                wrist_2_joint_constraint.set__joint_name("wrist_2_joint");
                wrist_2_joint_constraint.set__position(-M_PI / 2.0); // Wrist Out
                // wrist_2_joint_constraint.set__position(-275.0/180.0*M_PI); // Wrist In
                wrist_2_joint_constraint.set__tolerance_above(M_PI / 2.0); // ±90 deg
                wrist_2_joint_constraint.set__tolerance_below(M_PI / 2.0);
                wrist_2_joint_constraint.set__weight(1.0);
                joint_constraints.push_back(wrist_2_joint_constraint);

                constraint.set__joint_constraints(joint_constraints); // Apply the joint constraint to the path constraint

                // Generate a movement plan to the safe start pose
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                move_group_interface.setPathConstraints(constraint); // Set the path constraint
                move_group_interface.setPoseTarget(control->safe_start_pose_); // Set the pose target to the safe start pose
                move_group_interface.setPlanningTime(10.0); // Set the planning time (generous for constrained path)
                move_group_interface.setPlannerId("PTP"); // Set the planner id to point-to-point
                moveit::core::MoveItErrorCode response;
                response = move_group_interface.plan(plan); // Generate the plan - save the response       

                // Check repose and execute plan if safe. If the plan has succeded execute (check execution response) and move to next state.
                if(response == moveit::core::MoveItErrorCode::SUCCESS && move_group_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS){
                    RCLCPP_INFO(logger, "Moved to safe start pose");
                    RETRIES = 0; // Ensure retries is reset to 0
                    control->planning_scene_interface_.removeCollisionObjects({"canvas_plane"}); // Remove canvas collision object
                    rclcpp::sleep_for(std::chrono::milliseconds(10)); // Wait for scene to update
                    // Remove the init collision objects
                    control->planning_scene_interface_.removeCollisionObjects({"obstacle_behind"});
                    rclcpp::sleep_for(std::chrono::milliseconds(10)); // Wait for scene to update
                    control->planning_scene_interface_.removeCollisionObjects({"obstacle_front"});
                    rclcpp::sleep_for(std::chrono::milliseconds(10)); // Wait for scene to update
                    control->planning_scene_interface_.removeCollisionObjects({"obstacle_left"});
                    rclcpp::sleep_for(std::chrono::milliseconds(10)); // Wait for scene to update
                    control->planning_scene_interface_.removeCollisionObjects({"obstacle_right"});
                    // Move to next state
                    control->state_ = SplineFollower::State::IDLE;
                }
                else{ // If the plan has failed remain in this state and retry
                    RCLCPP_ERROR(logger, "The planner/executor has failed, attempting to plan/execute again.");
                    RETRIES++; // Increment retries up
                }

                move_group_interface.clearPathConstraints(); // Clear all constraints

                break;
            }
            // State move to intermediate pos: moves to an intermediate position directly above the next spline
            case SplineFollower::State::MOVE_TO_INTERMEDIATE_POS:
            {
                RCLCPP_INFO(logger, "State: Move to intermediate pos.");

                isIdle = false;
                // control->waitForContinue(); // For debug

                // Ensure we have a next spline to move to
                if (control->current_spline_index_ < control->spline_data_["splines"].size()) {

                    geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose; // Fetch current pose
                    
                    // Get the first waypoint of the next spline
                    json next_spline = control->spline_data_["splines"][control->current_spline_index_];
                    json first_waypoint = next_spline["waypoints"][0];

                    // Calculate the intermediate pose (100mm above the first waypoint)
                    control->intermediate_pose_.position.x = first_waypoint[0].get<double>();
                    control->intermediate_pose_.position.y = first_waypoint[1].get<double>();
                    // control->intermediate_pose_.position.z = first_waypoint[2].get<double>() + 0.05;  // 50mm above
                    control->intermediate_pose_.position.z = current_pose.position.z;
                    
                    // Ensure the orientation is pointing straight down
                    control->intermediate_pose_.orientation.x = 0.0;
                    control->intermediate_pose_.orientation.y = 1.0;
                    control->intermediate_pose_.orientation.z = 0.0;
                    control->intermediate_pose_.orientation.w = 0.0;

                    // Generate a straight line trajectory between the current pose and intermediate pose
                    std::vector<geometry_msgs::msg::Pose> waypoints = // Compute a linear vector of waypoints between current and intermediate pose
                    control->computeLinearInterpolationPath(current_pose, control->intermediate_pose_, 10); 
                    double jump_threshold = 0.0; // Set jump threshold
                    double eef_step = 0.01; // Set eef step
                    moveit_msgs::msg::RobotTrajectory trajectory;
                    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // Calculate trajectory - fration = total fraction of path calculated
                    if (fraction < 0.95) { // Check if we achieved at least 95% of the path
                        RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100); // Report failed trajectory plan
                        RETRIES++; // Increment retries up
                    }
                    else{ // If path is computed above 95% execute the path
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        plan.trajectory_ = trajectory; // Set the calculated trajectory into our plan
                        move_group_interface.execute(plan); // Execute the plan and move to intermediate pose

                        RCLCPP_INFO(logger, "Moved to intermediate pose.");

                        // Move to the next state
                        control->state_ = SplineFollower::State::MOVE_TO_CANVAS;

                        RETRIES = 0; // Ensure retries resets to zero
                    }
                } 
                else {
                    // If no more splines left, go to STOP state
                    control->state_ = SplineFollower::State::IDLE;
                }
                break;
            }
            // State move to canvas: move dirctly downward to touch the pen to the canvas at the first waypoint of the next spline
            case SplineFollower::State::MOVE_TO_CANVAS:
            {
                RCLCPP_INFO(logger, "State: Move to canvas.");

                // control->waitForContinue(); // For debug

                geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose; // Fetch the current pose

                // Set the target canvas pose
                control->canvas_pose_ = current_pose; // Set to intermediate pose directly above the first waypoint of next spline
                control->canvas_pose_.position.z = control->canvas_z_;  // Move to the canvas surface

                // Generate the straight line trajectory to the canvas
                std::vector<geometry_msgs::msg::Pose> waypoints = // Compute a vector of waypoints in a straght line between the current pose and canvas pose
                control->computeLinearInterpolationPath(current_pose, control->canvas_pose_, 10);
                double jump_threshold = 0.0; // Set jump threshold to zero
                double eef_step = 0.01; // Set the eef step to a low value
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // Compute cartesian trajectory
                if (fraction < 0.95) { // Handle incomplete trajectory
                    RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
                    RETRIES++; // Increment retries up
                }
                else{ // If trajectory is complete execute
                    // Execute trajectory
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory; // Set the trajectory to our path plan
                    move_group_interface.execute(plan); // Execute plan and move to canvas

                    RCLCPP_INFO(logger, "Moved to canvas.");

                    // Move to the next state: Move through drawing trajectory
                    control->state_ = SplineFollower::State::MOVE_THROUGH_DRAWING_TRAJECTORY;

                    RETRIES = 0; // Ensure retries resets to zero
                }
                
                break;
            }
            // State move through drawing trajectory: move through the next spline with pen to canvas (drawing state)
            case SplineFollower::State::MOVE_THROUGH_DRAWING_TRAJECTORY:
            {
                RCLCPP_INFO(logger, "State: Move through drawing trajectory.");

                // control->waitForContinue(); // For debug

                geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose; // Fetch the current pose

                // Fetch the way points from our spline data memeber
                std::vector<geometry_msgs::msg::Pose> waypoints;
                for (const auto& waypoint : control->spline_data_["splines"].at(control->current_spline_index_)["waypoints"]) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = waypoint[0].get<double>();
                    pose.position.y = waypoint[1].get<double>();
                    // pose.position.z = waypoint[2].get<double>();  // you can add lift here if needed
                    pose.position.z = current_pose.position.z;
                    pose.orientation.x = 0.0;
                    pose.orientation.y = 1.0;
                    pose.orientation.z = 0.0;
                    pose.orientation.w = 0.0;
                    waypoints.push_back(pose);
                }
            
                // Generate the trajectory through the waypoints
                double jump_threshold = 0.0; // Set jump threshold to zero
                double eef_step = 0.01; // Set low eef step
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // Compute cartesian path through waypoints
                if (fraction < 0.95) { // Handle incomplete path
                    RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
                    RETRIES++; // Increment retries up
                }
                else{ // If trajectory is complete execute
                    // Execute trajectory
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory; // Assign trajectory to our plan
                    move_group_interface.execute(plan); // Execute the plan

                    RCLCPP_INFO(logger, "Completed spline: %ld", control->current_spline_index_+1);

                    // Move to next state: move off canvas
                    control->state_ = SplineFollower::State::MOVE_OFF_CANVAS;

                    RETRIES = 0; // Ensure retries resets to zero
                }

                break;
            }
            // State move off canvas: move in a straight line directly upwards off the canvas
            case SplineFollower::State::MOVE_OFF_CANVAS:
            {
                RCLCPP_INFO(logger, "State: Move off canvas.");

                // control->waitForContinue(); // For debug

                geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose; // Fetch the current pose

                // Compute the safe lifted pose (100mm above the canvas)
                geometry_msgs::msg::Pose lifted_pose = current_pose;
                lifted_pose.position.z = control->lifted_z_;

                // Generate straight line trajectory directly upwards
                std::vector<geometry_msgs::msg::Pose> waypoints = // Compute waypoints of the straight line path between current pose and lifted pose
                control->computeLinearInterpolationPath(current_pose, lifted_pose, 10);
                double jump_threshold = 0.0; // Set the jump threshold to a zero
                double eef_step = 0.01; // Set the eef step to a low value
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // Compute the trajectory
                if (fraction < 0.95) { // Handle incomplete trajectory
                    RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
                    RETRIES++;
                }
                else{ // If trajectory is complete execute
                    // Execute trajectory
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory; // Assign the trajectory to our plan
                    move_group_interface.execute(plan); // Execute the plan

                    // Move to the next state (going to the next spline or stopping)
                    control->current_spline_index_++;
                    if (control->current_spline_index_ < control->spline_data_["splines"].size()) {
                        RCLCPP_INFO(logger, "Moved off canvas.");
                        control->state_ = SplineFollower::State::MOVE_TO_INTERMEDIATE_POS;
                        RETRIES = 0; // Ensure retries is reset to 0
                    } else { // If all splines are completed move to stop state
                        RCLCPP_INFO(logger, "All splines completed.");
                        control->state_ = SplineFollower::State::IDLE;
                        RETRIES = 0; // Ensure retries is reset to 0
                    }
                }

                break;
            }
            
            // State idle
            case SplineFollower::State::IDLE:
            {
                if(!isIdle){
                    RCLCPP_INFO(logger, "State: Idle.");
                    RCLCPP_INFO(logger, "Waiting for a new drawing or service request.");
                    isIdle = true;
                }

                break;
            }

            // State stop: return home and close the program
            case SplineFollower::State::STOP:
            {
                RCLCPP_INFO(logger, "State: Stop.");

                // control->waitForContinue(); // For debug

                RCLCPP_INFO(logger, "Returning home.");

                // Set home joint values to upright position
                sensor_msgs::msg::JointState home_state;
                home_state.name = { // Organise joint names in order
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"
                };
                home_state.position = { // Set the joint values
                    0.0, -M_PI/2.0, 0.0, -M_PI/2.0, 0.0, 0.0
                };

                // Generate the plan to home position
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                move_group_interface.setJointValueTarget(home_state); // Set the target joint values
                move_group_interface.setPlanningTime(10.0); // Apply generous planning time to reduce possibility of error
                move_group_interface.setPlannerId("PTP"); // Set the planner id to point-to-point
                moveit::core::MoveItErrorCode response; // Create a response message
                response = move_group_interface.plan(plan); // Generate the plan - save the response 

                // Check the respose
                if(response == moveit::core::MoveItErrorCode::SUCCESS){ // If successful execute the path
                    // Execute the plan
                    move_group_interface.execute(plan);

                    RCLCPP_INFO(logger, "Moved home.");
                    RCLCPP_INFO(logger, "Shutting down system.");

                    // Shutdown ros and join threads
                    rclcpp::shutdown();
                    spinner.join();
                    return 0;
                }
                else{ // If the planner has failed try again
                    RCLCPP_ERROR(logger, "The planner has failed, attempting to plan again.");
                    RETRIES++; // Increment the retries up
                }
                // Check retries and shutdown if max retries has been reached immediately and report special error message
                if(RETRIES > MAX_RETRIES){ // If the max retries has been reached report and shutdown the system
                    RCLCPP_ERROR(logger, "Drawing is complete but the robot has failed to return home. Please return the robot home manually.");
                    RCLCPP_INFO(logger, "Shutting down system.");
                    // Shutdown ros and join threads
                    rclcpp::shutdown();
                    spinner.join();
                    return 0;
                }

                break;
            }
            // State service: move to allow pen to be serviced then move back. 
            case SplineFollower::State::SERVICE:
            {
                if (control->service_started_)
                    {
                        RCLCPP_INFO(logger, "State: Service. Service in progress.");
                        RCLCPP_INFO(logger, "Returning home.");

                    // Set home joint values to upright position
                    sensor_msgs::msg::JointState home_state;
                    home_state.name = { // Organise joint names in order
                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint"
                    };
                    home_state.position = { // Set the joint values
                        0.0, -M_PI/2.0, 0.0, -M_PI/2.0, 0.0, 0.0
                    };

                    // Generate the plan to home position
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    move_group_interface.setJointValueTarget(home_state); // Set the target joint values
                    move_group_interface.setPlanningTime(10.0); // Apply generous planning time to reduce possibility of error
                    move_group_interface.setPlannerId("PTP"); // Set the planner id to point-to-point
                    moveit::core::MoveItErrorCode response; // Create a response message
                    response = move_group_interface.plan(plan); // Generate the plan - save the response 

                    // Check the respose
                    if(response == moveit::core::MoveItErrorCode::SUCCESS){ // If successful execute the path
                        // Execute the plan
                        move_group_interface.execute(plan);
                        RCLCPP_INFO(logger, "Moved home.");
                        control->service_started_ = false;

                    }
                    else{ // If the planner has failed try again
                        RCLCPP_ERROR(logger, "The planner has failed, attempting to plan again.");
                        RETRIES++; // Increment the retries up
                    }
                }
                break;
            }
        }
    }
    // Shutdown ROS and join threads (in case state machine while loop is broken)
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
