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

#define HOR_AXES 0
#define DPAD_UP 4
#define DPAD_RIGHT 5
#define DPAD_DOWN 6
#define DPAD_LEFT 7
#define L_TRIGGER 8
#define R_TRIGGER 9

bool received_joy = false;
float lin_vel = 2.0;
float ang_vel = 2.0;
sensor_msgs::Joy input;

/**
 * Changes the linear and angular velocity of the turtle
 * Up on the D-Pad increases linear velocity, down decreases linear velocity
 * Right on the D-Pad increases angular velocity, left decreases angular velocity
 */
void update_vel() {
    if (input.buttons[DPAD_UP] == 1) {
        lin_vel *= 2;    
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (input.buttons[DPAD_DOWN] == 1) {
        lin_vel /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (input.buttons[DPAD_RIGHT] == 1) {
        ang_vel *= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (input.buttons[DPAD_LEFT] == 1) {
        ang_vel /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    }
}

/**
 * Callback for the input from the controller
 */
void controller_cb(const sensor_msgs::Joy::ConstPtr& msg) {
    received_joy = true;
    input = *msg;
    update_vel();
}

/**
 * Sets the velocity for the turtle
 * Linear x is positive when right trigger is pressed, negative when
 * left trigger is pressed, and 0 when neither is pressed
 * Angular z is positive when left thumbstick is left, negative when 
 * left thumbstick is right, 0 when in center
 */
void set_vel(geometry_msgs::Twist& turtle_vel) {
    if (input.buttons[R_TRIGGER] == 1) {
        turtle_vel.linear.x = lin_vel;
    } else if (input.buttons[L_TRIGGER] == 1) {
        turtle_vel.linear.x = lin_vel * -1;
    } else {
        turtle_vel.linear.x = 0.0;
    }
    
    turtle_vel.angular.z = ang_vel * input.axes[HOR_AXES];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_controller_node");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    ros::Subscriber controller_sub = n.subscribe("/joy", 1, controller_cb);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist turtle_vel;

    // Waiting for controller_cb
    while (!received_joy) {
        ros::spinOnce();
    }

    while (ros::ok()) {
        set_vel(turtle_vel);
        vel_pub.publish(turtle_vel);
        loop_rate.sleep();
        ros::spinOnce();
    }
}
