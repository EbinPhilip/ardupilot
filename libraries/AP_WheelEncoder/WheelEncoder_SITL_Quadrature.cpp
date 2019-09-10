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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "WheelEncoder_SITL_Quadrature.h"

extern const AP_HAL::HAL& hal;

AP_WheelEncoder_SITL_Qaudrature::AP_WheelEncoder_SITL_Qaudrature(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state) :
    AP_WheelEncoder_Backend(frontend, instance, state),
    _sitl(AP::sitl()),
    last_location(_sitl->state.home)
{
}

void AP_WheelEncoder_SITL_Qaudrature::update(void)
{
    // Get current location from SITL state
    // scale latitude, longitude and altitude from double to int32_t
    int32_t lat = _sitl->state.latitude * 1.0e7; // latitude
    int32_t lnt = _sitl->state.longitude * 1.0e7; // longitude
    int32_t alt = _sitl->state.altitude * 1.0e2; // altitude
    Location current_location(lat, lnt, alt, _sitl->state.home.get_alt_frame()); // current location object

    // distance travelled after last update in vector form
    Vector2f location_diff = last_location.get_distance_NE(current_location);
    // distance travelled after last update
    float distance_diff = norm(location_diff.x, location_diff.y);
    // change in heading after last update
    double heading_diff = _sitl->state.heading - last_heading;

    // transform location_diff vector to last_heading frame
    // helps to calculate direction of movement
    Vector2f location_diff_rotated;
    location_diff_rotated.x = location_diff.x*cos(radians(last_heading)) + location_diff.y*sin(radians(last_heading));
    location_diff_rotated.y = -location_diff.x*sin(radians(last_heading)) + location_diff.y*cos(radians(last_heading));

    /* direction of movement is decided by sign of x coordinate of location_diff vector 
    in last_heading frame and angle turned after last update*/
    if ( (location_diff_rotated.x < 0) && ( fabsf(radians(heading_diff))<M_PI ) ) {
        distance_count *= -1;
    }

    // calculate dt
    double time_now = AP_HAL::millis();
    double dt = (time_now - _state.last_reading_ms)/1000.0f;

    // calculate speed and turn rate from distance_diff and heading_diff
    double speed = distance_diff/dt;
    double turn_rate = radians(heading_diff)/dt;

    // distance from center of wheel axis to each wheel
    double l = ( fabsf(_frontend.get_pos_offset(1).y) + fabsf(_frontend.get_pos_offset(2).y) )/2.0f;

    if (_state.instance == 1) { 
        speed = speed - turn_rate*l; // for left wheel
    } else if(_state.instance ==2) {
        speed = speed + turn_rate*l; // for right wheel
    } else {
        return; // invalid instance
    }

    // calculate angle turned and corresponding encoder ticks from wheel angular rate
    double angle_turned = ( speed/_frontend.get_wheel_radius(_state.instance) ) * dt;
    int32_t ticks =  static_cast<int>( ( angle_turned/(2 * M_PI) ) * _frontend.get_counts_per_revolution(_state.instance) );

    // update distance count
    distance_count += ticks;
    // update total count of encoder ticks
    total_count += abs(ticks);

    // update previous state to current
    copy_state_to_frontend(distance_count, total_count, 0, time_now);
    last_location = current_location;
    last_heading = _sitl->state.heading;
}

#endif