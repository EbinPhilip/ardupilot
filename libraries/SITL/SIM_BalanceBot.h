/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  BalanceBot simulator class
*/

#pragma once

#include "SIM_Rover.h"

namespace SITL {

class BalanceBot : public SimRover {
public:
    BalanceBot(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new BalanceBot(home_str, frame_str);
    }
private:
    // these three variables are specific to the inverted pendulum and its orientation
    // theta is the angle between the vertical and the rod(counter-clockwise positive)
    float theta = radians(0);
    float ang_vel = 0;
    float angular_accel = 0;
};

} // namespace SITL
