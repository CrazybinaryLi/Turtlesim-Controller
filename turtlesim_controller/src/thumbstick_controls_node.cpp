#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define HOR_AXES 0
#define VERT_AXES 1
#define DPAD_UP 4
#define DPAD_RIGHT 5
#define DPAD_DOWN 6
#define DPAD_LEFT 7

geometry_msgs::Twist turtle_vel;
bool received_joy = false;
float lin_vel = 2.0;
float ang_vel = 2.0;

/**
 * Changes the linear and angular velocity of the turtle
 * Up on the D-Pad increases linear velocity, down decreases linear velocity
 * Right on the D-Pad increases angular velocity, left decreases angular velocity
 */
void update_vel(int dpad_up, int dpad_down, int dpad_right, int dpad_left) {
    if (dpad_up == 1) {
        lin_vel *= 2;    
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (dpad_down == 1) {
        lin_vel /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (dpad_right == 1) {
        ang_vel *= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (dpad_left == 1) {
        ang_vel /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    }
}

/** 
 * Sets the linear and angular velocity based on the input
 * from the left thumbstick
 */
void set_vel(const float LIN_MOD, const float ANG_MOD) {
    turtle_vel.linear.x = LIN_MOD * lin_vel;
    turtle_vel.angular.z = ANG_MOD * ang_vel;
}

/**
 * Callback for the input from the controller
 * Updates the velocity if changed and sets the new
 * linear and angular velocity for the turtle
 */
void controller_cb(const sensor_msgs::Joy::ConstPtr& MSG) {
    received_joy = true;
    update_vel(MSG->buttons[DPAD_UP], MSG->buttons[DPAD_DOWN], MSG->buttons[DPAD_RIGHT], MSG->buttons[DPAD_LEFT]);
    set_vel(MSG->axes[VERT_AXES], MSG->axes[HOR_AXES]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "thumbstick_controls_node");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    ros::Subscriber controller_sub = n.subscribe("/joy", 1, controller_cb);
    ros::Rate loop_rate(10);

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
