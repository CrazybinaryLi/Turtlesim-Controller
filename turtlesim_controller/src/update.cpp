#include "update.h"

#define DPAD_UP 4
#define DPAD_RIGHT 5
#define DPAD_DOWN 6
#define DPAD_LEFT 8

/**
 * Changes the linear and angular velocity of the turtle
 * Up on the D-Pad increases linear velocity, down decreases linear velocity
 * Right on the D-Pad increases angular velocity, left decreases angular velocity
 */
void update_vel(float& lin_vel, float& ang_vel, const std::vector<int>& BUTTONS) {
    if (BUTTONS[DPAD_UP] == 1) {
        lin_vel *= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (BUTTONS[DPAD_DOWN] == 1) {
        lin_vel /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (BUTTONS[DPAD_RIGHT] == 1) {
        ang_vel *= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    } else if (BUTTONS[DPAD_LEFT] == 1) {
        ang_vel /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", lin_vel, ang_vel);
    }
}

