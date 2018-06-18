#include<stdio.h>
#include "Rover.h"


void Rover::balance_pitch(float &throttle,bool armed)
{
    float balance_throttle = 0;
    float throttle_new = 0;
    float demanded_pitch = 0;//radians(-throttle/50.0f);
    balance_throttle = g2.attitude_control.get_throttle_out_from_pitch(demanded_pitch,armed)*100;
    throttle_new = constrain_float(throttle + balance_throttle,-100,100);
//    printf("in_throttle:%f pitch:%f balance_throttle:%f total:%f\n",throttle,degrees(demanded_pitch),balance_throttle,throttle_new);
    throttle = throttle_new;
}

bool Rover::is_BalanceBot(){
    if((enum frame_class)g2.frame_class.get() == FRAME_BALANCEBOT)
        return true;
    else
        return false;
}
