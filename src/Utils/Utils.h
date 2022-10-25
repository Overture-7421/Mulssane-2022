#include <cmath>

struct Utils {
  static const double DEFAULT_DEADZONE = 0.15;
  static const double DEFAULT_EXP_GAIN = 0.15;

  template <typename T>
  static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  /**
   * Apply a filter to an Axis, so that when the driver is not using it, the
   * robot doesn't move randomly. It applies an Exponential curve for finer
   * control at smaller inputs.
   *
   * axisValue = The raw axis value given by the joystick
   * deadzone = The threshold for when the axis is considered as valid
   * exponentialGain = How much of an Exponential curve we want
   * 
   * To learn more: https://www.desmos.com/calculator/kvubon8yfw?lang=es
   *
   * */
  static double ApplyAxisFilter(double axisValue, double deadzone = DEFAULT_DEADZONE,
                                double exponentialGain = DEFAULT_EXP_GAIN) {
    double axisMag = std::abs(axisValue);
    if(axisMag < deadzone) {
      return 0.0;
    }

    double res =
        exponentialGain * std::pow((axisMag - deadzone) / (1 - deadzone), 3) +
        (1 - exponentialGain) * (axisMag - deadzone) / (1 - deadzone);

    return res * sgn(axisValue);
  }
};
