#include<stdio.h>
#include "Rover.h"


void Rover::balance_pitch(float &throttle)
{
    float balance_throttle = 0;
    float throttle_new = 0;
//    float demanded_pitch = radians(-throttle/20);
    balance_throttle = g2.attitude_control.get_throttle_out_from_pitch(0);
    throttle_new = constrain_float(throttle+balance_throttle*100,-100,100);
//    printf("in_throttle:%f balance_throttle:%f total:%f\n",throttle,balance_throttle,throttle_new);
    throttle = throttle_new;

}
