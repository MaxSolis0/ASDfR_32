//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that sends FSM commands and wheel velocities either from
//         a predefined sequence or direct parameter input (YAML / CLI).
//==============================================================================

#ifndef TESTER_HPP_
#define TESTER_HPP_

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// RELbot simulator message types
#include "example_interfaces/msg/float64.hpp"

/**
 * @brief Tester node for FSM + wheel control.
 *
 * Modes:
 * 1. Sequence mode → uses setpoints_array
 * 2. Direct mode → uses FSMcomm, rightSpeed, leftSpeed
 *
 * Parameters:
 * @param FSMcomm [int]        : Default FSM command
 * @param rightSpeed [double]  : Default right wheel speed
 * @param leftSpeed [double]   : Default left wheel speed
 * @param setpoints_array [double[]]:
 *        Format: [duration, FSM, rightSpeed, leftSpeed, ...]
 *
 * QoS:
 * @param reliability [string] : "reliable" | "best_effort"
 * @param durability  [string] : "volatile" | "transient_local"
 * @param history     [string] : "keep_last" | "keep_all"
 * @param depth       [int]    : Queue size
 */

class Tester : public rclcpp::Node
{
public:
    /**
   * @brief Construct a new Tester object
   */
    Tester();

private:

    // ================= ROS INTERFACES =================
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr FSMpub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr leftPub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr rightPub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr FSMsub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ================= PARAMETERS =================
    int FSMcomm_ = 0;
    double rightSpeed_ = 0.0;
    double leftSpeed_ = 0.0;
    std::vector<double> setpointsArray_;

    // ================= STATE =================
    int FSMstate_ = 0;
    size_t seqIndex_ = 0;
    double elapsed_ = 0.0;

    // ================= QoS =================
    int depth_;
    std::string history_;
    std::string reliability_;
    std::string durability_;
    
     /**
   * @brief Run the sequence controller periodically or stop robot if finished
   *
   */
    void timer_callback();
    
    /**
   * @brief Parse the sequence and run publish function appropriately
   *
   */
    void sequence();

    /**
   * @brief Publish velocities to relbot_adapter and FSM command to Xeno bridge
   *
   * @param fsm: FSM command
   * @param right: Left wheel speed
   * @param left: Right wheel speed
   */
    void publish_command(int fsm, double right, double left);

    /**
   * @brief Get FSM state and print
   *
   * @param msg: FSM state
   */
    void loop_callback(const std_msgs::msg::Int32::SharedPtr msg);
    
    /**
   * @brief Get parameters when changed
   *
   * @param params: vector of ROS parameters as previously described
   */
    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter> & params);
};

#endif /* TESTER_HPP_ */
