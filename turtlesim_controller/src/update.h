#ifndef GUARD_update_h
#define GUARD_update_h

#include "std_msgs/Int32.h"
#include <vector>

struct Velocity;

void update_vel(Velocity&, const std::vector<int>&);

#endif
