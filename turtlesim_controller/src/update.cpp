#include "update.h"

#define DPAD_UP 4
#define DPAD_RIGHT 5
#define DPAD_DOWN 6
#define DPAD_LEFT 7

/**
 * Struct for the turtle's base velocities
 */
struct Velocity {
    float linear;
    float angular;
};

/**
 * Changes the linear and angular velocity of the turtle
 * Up on the D-Pad increases linear velocity, down decreases linear velocity
 * Right on the D-Pad increases angular velocity, left decreases angular velocity
 */
void update_vel(Velocity& vel, const std::vector<int>& BUTTONS) {
    if (BUTTONS[DPAD_UP] == 1) {
        vel.linear *= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", vel.linear, vel.angular);
    } else if (BUTTONS[DPAD_DOWN] == 1) {
        vel.linear /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", vel.linear, vel.angular);
    } else if (BUTTONS[DPAD_RIGHT] == 1) {
        vel.angular *= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", vel.linear, vel.angular);
    } else if (BUTTONS[DPAD_LEFT] == 1) {
        vel.angular /= 2;
        ROS_INFO("Lin vel: %.4f, Ang vel: %.4f", vel.linear, vel.angular);
    }
}

