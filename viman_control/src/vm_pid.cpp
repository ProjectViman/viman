#include "vm_pid.h"

VmPID::VmPID(){
  gain_p = 10.0;
  gain_d = 5.0;
  gain_i = 0.01;
  sp_range = 0.05;
}

double VmPID::update(double set_point, double cur_state, double dt){
  //cur_state = f_round(cur_state, 2);
  
  //if (fabs(cur_state) <= fabs(set_point) + fabs(sp_range))cur_state = set_point;
  
  // update proportional, differential and integral errors
  p = set_point - cur_state;	// current error
  i = i + dt * p;				// i -> sum of prev errors
  d = (p - prevErr) / dt;		// d -> rate of error
  
  prevErr = p;
  
  // update control output
  output = gain_p * p + gain_d * d + gain_i * i;
  
  return output;
}

void VmPID::reset(){
  p = i = d = output = prevErr = 0;
}

float VmPID::f_round(float f, int decimals){	
    float value = (int)(f * pow(10,decimals) + .5); 
    return (float)value / pow(10,decimals); 
}

