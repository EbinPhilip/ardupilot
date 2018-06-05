#include<stdio.h>
#include "Rover.h"

void Rover::balance_pitch(float &throttle){

    float balance_throttle = 100.0f*g2.attitude_control.get_throttle_out_from_pitch(0);
    float throttle_new = constrain_float((throttle + balance_throttle),-100.0f,100.0f);

    printf("in_throttle:%f balance_throttle:%f total:%f\n",throttle,balance_throttle,throttle_new);
    throttle = throttle_new;

}
