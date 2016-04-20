#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "update.cpp"

#define HOR_AXES 0
#define VERT_AXES 1

geometry_msgs::Twist turtle_vel;
bool received_joy = false;
Velocity vel;

/** 
 * Sets the linear and angular velocity based on the input
 * from the left thumbstick
 */
void set_vel(const float LIN_MOD, const float ANG_MOD) {
    turtle_vel.linear.x = LIN_MOD * vel.linear;
    turtle_vel.angular.z = ANG_MOD * vel.angular;
}

/**
 * Callback for the input from the controller
 * Updates the velocity if changed and sets the new
 * linear and angular velocity for the turtle
 */
void controller_cb(const sensor_msgs::Joy::ConstPtr& MSG) {
    received_joy = true;
    update_vel(vel, MSG->buttons);
    set_vel(MSG->axes[VERT_AXES], MSG->axes[HOR_AXES]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "thumbstick_controls_node");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    ros::Subscriber controller_sub = n.subscribe("/joy", 1, controller_cb);
    ros::Rate loop_rate(10);
    vel.linear = 2.0;
    vel.angular = 2.0;

    // Waiting for controller_cb and pose_cb
    while (!received_joy) {
        ros::spinOnce();
    }

    // Moving turtle
    while (ros::ok()) {
        vel_pub.publish(turtle_vel);
        loop_rate.sleep();
        ros::spinOnce();
    }
}
