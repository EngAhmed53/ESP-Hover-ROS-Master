#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "drone_controller/AttitudeArray.h"

class DroneController
{
public:
    DroneController();

    void changeMotorsArm(bool shouldArm);
    void changeMotorsThrustBy(int16_t value);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    ros::NodeHandle nh_;

    double thrust_scale_, pitch_scale_, roll_scale_, yaw_scale_;
    int arm_thrust;

    ros::Publisher attitude_pub_;
    ros::Subscriber joy_sub_;

    int16_t accum_thrust_{0};
    bool armed_{false};

    int no_control_msg_counter = 5; // Num of msgs with no control inputes to be send after receving the first msg with no control inputes
};

DroneController::DroneController() : thrust_scale_(5),
                                     pitch_scale_(30),
                                     roll_scale_(30),
                                     yaw_scale_(60),
                                     arm_thrust(20)
{

    nh_.param("thrust_scale", thrust_scale_, thrust_scale_);
    nh_.param("pitch_scale", pitch_scale_, pitch_scale_);
    nh_.param("roll_scale", roll_scale_, roll_scale_);
    nh_.param("yaw_scale", yaw_scale_, yaw_scale_);
    nh_.param("arm_thrust", arm_thrust, arm_thrust);

    attitude_pub_ = nh_.advertise<drone_controller::AttitudeArray>("/cmd_attitude", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &DroneController::joyCallback, this);
}

void DroneController::changeMotorsArm(bool shouldArm)
{
    DroneController::armed_ = shouldArm;
    DroneController::accum_thrust_ = shouldArm ? arm_thrust : 0;
}

void DroneController::changeMotorsThrustBy(int16_t value)
{
    int16_t new_thrust = DroneController::accum_thrust_ + value;
    new_thrust = std::min(new_thrust, (int16_t)255);
    new_thrust = std::max(new_thrust, (int16_t)arm_thrust);

    DroneController::accum_thrust_ = new_thrust;
}

void DroneController::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

    drone_controller::AttitudeArray attitude_msg;
    std::vector<int16_t> initial_values{DroneController::armed_ ? 1 : 0, DroneController::accum_thrust_, 0, 0, 0}; // [armed?, thrust, pitch_setpoint, roll_setpoint, yaw_setpoint]
    attitude_msg.data = initial_values;

    // [arm, disarm, thrust, yaw, pitch, roll]
    std::array<double, 8> effectiveButtons{joy->buttons[0], joy->buttons[4], joy->buttons[6],joy->buttons[7], joy->axes[1], joy->axes[0], joy->axes[3], joy->axes[2]};

    if (std::all_of(effectiveButtons.begin(), effectiveButtons.end(), [](double i)
                    { return i == 0; }))
    {
        if (DroneController::no_control_msg_counter > 0)
        {
            DroneController::no_control_msg_counter--;
            attitude_pub_.publish(attitude_msg);
            return;
        } else return;
        
    } else 
    {
        DroneController::no_control_msg_counter = 5;
    }
    
    // for (size_t i = 0; i < effectiveButtons.size(); i++)
    // {
    //     std::cout << effectiveButtons[i] << std::endl;
    // }
    

    // handle armed state
    bool arm = effectiveButtons[0] == 1;
    bool disarm = effectiveButtons[1] == 1;

    if (arm || disarm)
    {
        bool shouldArm{false};

        if (disarm)
            shouldArm = false;
        else if (arm)
            shouldArm = true;

        DroneController::changeMotorsArm(shouldArm);
        attitude_msg.data[0] = shouldArm ? 1 : 0;

        std::string state = shouldArm ? "ON" : "OFF";
        std::cout << state << std::endl;

        attitude_pub_.publish(attitude_msg);

        return;
    }

    // Handle other control inputes if drone is armed
    if (DroneController::armed_)
    {

        // handle thrust
        int16_t added_thrust = (effectiveButtons[3] - effectiveButtons[2]) * thrust_scale_;
        DroneController::changeMotorsThrustBy(added_thrust);
        std::cout << "added_thrust = " << added_thrust << ", updated_thrust = " << DroneController::accum_thrust_ << std::endl;
        attitude_msg.data[1] = DroneController::accum_thrust_;

        // handle angles
        int16_t pitch_angle = effectiveButtons[5] * pitch_scale_;
        int16_t roll_angle = effectiveButtons[4] * roll_scale_;
        int16_t yaw_angle = effectiveButtons[7] * yaw_scale_;

        std::cout << "pitch = " << pitch_angle << ", roll = " << roll_angle << ", yaw = " << yaw_angle << std::endl;
        attitude_msg.data[2] = pitch_angle;
        attitude_msg.data[3] = roll_angle;
        attitude_msg.data[4] = yaw_angle;

        attitude_pub_.publish(attitude_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_controller");
    DroneController teleop_turtle;

    ros::spin();
}