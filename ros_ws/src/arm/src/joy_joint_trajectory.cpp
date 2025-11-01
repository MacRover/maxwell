#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#define RAD_STEPS 80

enum Buttons
{
    A_BUTTON = 0,
    B_BUTTON,
    X_BUTTON,
    Y_BUTTON,
    LEFT_BUMPER,
    RIGHT_BUMPER,
    BACK_BUTTON,
    START_BUTTON,
    POWER_BUTTON,
    LEFTSTICK_CLICK,
    RIGHTSTICK_CLICK,
};

enum Axes
{
    DPAD_LEFT_RIGHT = 6,
    DPAD_UP_DOWN = 7
};

class JoyJointTrajectoryController : public rclcpp::Node
{
public:
    JoyJointTrajectoryController();

private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;
    std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> traj_pub_;
    trajectory_msgs::msg::JointTrajectory traj_msg_;
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

JoyJointTrajectoryController::JoyJointTrajectoryController() : Node("joy_joint_trajectory_controller")
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyJointTrajectoryController::joyCallback, this, std::placeholders::_1));
    
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    traj_msg_.joint_names = {"arm_base_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint", "arm_pitch_joint", "gripper_joint"};
}

void JoyJointTrajectoryController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    traj_msg_.points[0].velocities = 
    {
        (msg->buttons[X_BUTTON] - msg->buttons[B_BUTTON]) * RAD_STEPS, // Base rotation
        (msg->buttons[Y_BUTTON] - msg->buttons[A_BUTTON]) * RAD_STEPS, // Shoulder movement
        (msg->axes[DPAD_UP_DOWN]) * RAD_STEPS,                         // Elbow movement
        (msg->axes[DPAD_LEFT_RIGHT]) * RAD_STEPS,                      // Wrist rotation
        (msg->buttons[BACK_BUTTON] - msg->buttons[START_BUTTON]) * RAD_STEPS, // Pitch control
        (msg->buttons[LEFT_BUMPER] - msg->buttons[RIGHT_BUMPER]) * RAD_STEPS, // Gripper open/close
    };
    traj_pub_->publish(traj_msg_);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<JoyJointTrajectoryController>());
    rclcpp::shutdown();
    return 0;
}