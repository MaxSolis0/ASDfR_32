#include "tester.hpp"

Tester::Tester()
: Node("tester"),
  seqIndex_(0),
  elapsed_(0.0)
{
    
    // ================= PARAMETERS =================
    this->declare_parameter("FSMcomm", 0);
    this->declare_parameter("rightSpeed", 0.0);
    this->declare_parameter("leftSpeed", 0.0);
    this->declare_parameter("setpoints_array", std::vector<double>{});

    this->declare_parameter("reliability", "reliable");
    this->declare_parameter("durability", "volatile");
    this->declare_parameter("history", "keep_last");
    this->declare_parameter("depth", 1);

    // Get parameters (YAML or CLI override automatically)
    FSMcomm_ = this->get_parameter("FSMcomm").as_int();
    rightSpeed_ = this->get_parameter("rightSpeed").as_double();
    leftSpeed_ = this->get_parameter("leftSpeed").as_double();
    setpointsArray_ = this->get_parameter("setpoints_array").as_double_array();

    reliability_ = this->get_parameter("reliability").as_string();
    durability_ = this->get_parameter("durability").as_string();
    history_ = this->get_parameter("history").as_string();
    depth_ = this->get_parameter("depth").as_int();

    // ================= QoS =================
    rclcpp::QoS qos(depth_);

    (reliability_ == "best_effort") ? qos.best_effort() : qos.reliable();
    (durability_ == "transient_local") ? qos.transient_local() : qos.durability_volatile();

    // FIXED: history logic was reversed
    (history_ == "keep_all") ? qos.keep_all() : qos.keep_last(depth_);

    // ================= PUB/SUB =================
    FSMpub_ = this->create_publisher<std_msgs::msg::Int32>("XenoCmd", qos);

    leftPub_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", qos);

    rightPub_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", qos);

    FSMsub_ = this->create_subscription<std_msgs::msg::Int32>(
        "XenoState", qos,
        std::bind(&Tester::loop_callback, this, std::placeholders::_1));

    // ================= TIMER =================
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),   // clearer than microseconds
        std::bind(&Tester::timer_callback, this));
        
    // ============== PARAMETER UPDATE =========
    paramCallbackHandle_ = this->add_on_set_parameters_callback(
    std::bind(&Tester::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Tester node started");
}

rcl_interfaces::msg::SetParametersResult
Tester::parametersCallback(const std::vector<rclcpp::Parameter> & params)
{
    for (const auto & param : params)
    {
        const auto & name = param.get_name();

        if (name == "FSMcomm") {
            FSMcomm_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "FSMcomm updated: %d", FSMcomm_);
        }
        else if (name == "rightSpeed") {
            rightSpeed_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "rightSpeed updated: %.2f", rightSpeed_);
        }
        else if (name == "leftSpeed") {
            leftSpeed_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "leftSpeed updated: %.2f", leftSpeed_);
        }
        else if (name == "setpoints_array") {
            setpointsArray_ = param.as_double_array();

            // Reset sequence when new one is loaded
            seqIndex_ = 0;
            elapsed_ = 0.0;

            RCLCPP_INFO(this->get_logger(),
                "New sequence loaded (%zu points)",
                setpointsArray_.size() / 4);
        }
        else if (name == "depth") {
            depth_ = param.as_int();
        }
        else if (name == "history") {
            history_ = param.as_string();
        }
        else if (name == "reliability") {
            reliability_ = param.as_string();
        }
        else if (name == "durability") {
            durability_ = param.as_string();
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated successfully";
    return result;
}

// ================= TIMER =================
void Tester::timer_callback()
{
    if (!setpointsArray_.empty() &&
        seqIndex_ < setpointsArray_.size() / 4)
    {
        sequence();
    }
    else
    {
        publish_command(FSMcomm_, rightSpeed_, leftSpeed_);
    }
}

// ================= HELPER =================
void Tester::publish_command(int fsm, double right, double left)
{
    example_interfaces::msg::Float64 lmsg;
    example_interfaces::msg::Float64 rmsg;

    lmsg.data = left;
    rmsg.data = right;

    leftPub_->publish(lmsg);
    rightPub_->publish(rmsg);

    std_msgs::msg::Int32 fmsg;
    fmsg.data = static_cast<int32_t>(fsm);
    FSMpub_->publish(fmsg);
}

// ================= SEQUENCE =================
void Tester::sequence()
{
    double duration   = setpointsArray_[4 * seqIndex_];
    int FSMcomm       = static_cast<int>(setpointsArray_[4 * seqIndex_ + 1]);
    double rightSpeed = setpointsArray_[4 * seqIndex_ + 2];
    double leftSpeed  = setpointsArray_[4 * seqIndex_ + 3];

    publish_command(FSMcomm, rightSpeed, leftSpeed);

    elapsed_ += 0.001; // 1 ms

    if (elapsed_ >= duration)
    {
        seqIndex_++;
        elapsed_ = 0.0;

        RCLCPP_INFO(this->get_logger(),
            "Switching to setpoint %zu", seqIndex_);
    }
}

// ================= SUB =================
void Tester::loop_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    FSMstate_ = msg->data;

    RCLCPP_INFO(this->get_logger(),
        "Current state: %lu",
        static_cast<unsigned long>(FSMstate_));
}

// ================= MAIN =================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tester>());
    rclcpp::shutdown();
    return 0;
}
