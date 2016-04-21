/**
 * Node to control turtlesim using a PS3 controller
 * Right and left triggers move the turtle forwards and backwards
 * Left thumbstick turns the turtle
 * D-Pad changes the linear and angular velocity
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "sensor_msgs/Joy.h"
#include "update.cpp"

#define HOR_AXES 0
#define L_TRIGGER 8
#define R_TRIGGER 9

geometry_msgs::Twist turtle_vel;
Velocity vel;
bool received_joy = false;

/**
 * Sets the velocity for the turtle
 * Linear x is positive when right trigger is pressed, negative when
 * left trigger is pressed, and 0 when neither is pressed
 * Angular z is positive when left thumbstick is left, negative when 
 * left thumbstick is right, 0 when in center
 */
void set_vel(const int& R_TRIG, const int& L_TRIG, const float& ANG_MOD) {
    if (R_TRIG == 1) {
        turtle_vel.linear.x = vel.linear;
    } else if (L_TRIG == 1) {
        turtle_vel.linear.x = vel.linear * -1;
    } else {
        turtle_vel.linear.x = 0.0;
    }
    
    turtle_vel.angular.z = vel.angular * ANG_MOD;
}

/**
 * Callback for the input from the controller
 */
void controller_cb(const sensor_msgs::Joy::ConstPtr& MSG) {
    received_joy = true;
    update_vel(vel, MSG->buttons);
    set_vel(MSG->buttons[R_TRIGGER], MSG->buttons[L_TRIGGER], MSG->axes[HOR_AXES]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_controller_node");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    ros::Subscriber controller_sub = n.subscribe("/joy", 1, controller_cb);
    ros::Rate loop_rate(10);
    vel.linear = 2.0;
    vel.angular = 2.0;

    // Waiting for controller_cb
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
