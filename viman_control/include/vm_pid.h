#ifndef VM_PID_H
#define VM_PID_H

#include <cmath>

/**
 * @brief PID controller class for VIMAN.
 */

class VmPID {
protected:
	float f_round(float f, int decimals);

public:
  VmPID();

  double gain_p;
  double gain_i;
  double gain_d;
  double sp_range; // set point +- range

  double prevErr;
  double output;
  double p, i, d;

  double update(double set_point, double cur_state, double dt);
  void reset();
};

#endif // VM_PID_H
