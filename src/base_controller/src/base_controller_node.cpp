#include "base_controller/serial_port_communication.h"

#define PI 3.14159
#define Wheel_Radius 0.19
#define Wheel_Distance 1.4

/*

Example of sending speed command:
Enable A/B circuit motor, A circuit is given a forward speed of 300rpm, B circuit is given a reverse speed of 300rpm,
(Rated speed is set to 3000rpm)
E0 03 00 00 00 00 03 E8 FF FF FC 17

Example of sending query speed command:
Query motor speed (rated speed 3000RPM)
Data sent: ED 02 00 00 00 00 00 00 00 00 00 00
Driver feedback: ED 02 03 E8 FC 17 00 00 00 00 00 00
03E8: A circuit motor forward 300RPM (03E8 is converted to 10 base number 1000, corresponding to 10% of the rated speed)
FC17: Circuit B motor reverse 300RPM (FC17 converted to base 10 number is 64536, corresponding to 10% of the rated speed)

*/

void call_back(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    double linear_velocity = cmd_vel->linear.x;
    double angular_velocity = cmd_vel->angular.z;

    LEFT_ROTATE_SPEED_TARGET = (linear_velocity + angular_velocity * Wheel_Distance / 2.0) / (2 * PI * Wheel_Radius) * 60;
    RIGHT_ROTATE_SPEED_TARGET = (linear_velocity - angular_velocity * Wheel_Distance / 2.0) / (2 * PI * Wheel_Radius) * 60;

    // Send the data
    sendRotateSpeed(LEFT_ROTATE_SPEED_TARGET, RIGHT_ROTATE_SPEED_TARGET);
    // Receieve the rotate speed
    receiveRotateSpeed();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_controller_node"); // Initialize the ROS node
    ros::NodeHandle nh;

    // Initialize the serial port
    serialInit();

    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, call_back);

    ros::spin();

    return 0;
}