#ifndef __MOTIONGENERATOR_H__
#define __MOTIONGENERATOR_H__

/**
 * Generates the analytical solution for the trapezoidal motion.
 *
 * <p>
 * Usage:
 * // Includes
 * #include "MotionGenerator.h"
 *
 * Initialization
 *
 * @param int aVelMax maximum velocity (units/s)
 * @param int aAccMax maximum acceleration (units/s^2)
 * @param int aInitPos initial position (units)
 *

 // Define the MotionGenerator object
 MotionGenerator *trapezoidalProfile = new MotionGenerator(100, 400, 0);

 // Retrieve calculated position
 double positionRef = 1000;
 double position = trapezoidalProfile->update(positionRef)

 // Retrieve current velocity
 double velocity = trapezoidalProfile->getVelocity();

 // Retrieve current acceleration
 double acceleration = trapezoidalProfile->getAcceleration();

 // Check if profile is finished
 if (trapezoidalProfile->getFinished()) {};

 // Reset internal state
 trapezoidalProfile->reset();

 *
 * @author      AerDronix <aerdronix@gmail.com>
 * @web		https://aerdronix.wordpress.com/
 * @version     1.0
 * @since       2016-12-22
 */

class MotionGenerator {
public:
  /**
   * Constructor
   *
   * @param int aVelocityMax maximum velocity
   * @param int aAccelerationMax maximum acceleration
   */
  MotionGenerator(double aMaxVel, double aMaxAcc, double aInitPos);

  void init();

  /**
   * Updates the state, generating new setpoints
   *
   * @param aSetpoint The current setpoint.
   */
  void setPositionRef(double aPosRef);
  double getPosition();
  double getVelocity();
  double getAcceleration();

  bool getFinished();
  void setMaxVelocity(double aMaxVel);
  void setMaxAcceleration(double aMaxAcc);
  void setInitPosition(double aInitPos);
  void reset();
  void updateTrapezoidalProfile(double timestep);

private:
  /**
   * Increments the state number.
   *
   * @see
    currentState
   */
  short int sign(double aVal);

  double maxvel_;
  double maxacc_;
  double initpos_;
  double pos_;
  double posref_;
  double vel_;
  double acc_;
  double oldpos_;
  double oldvel_;

  double dbrk_;
  double dacc_;
  double dvel_;
  double ddec_;
  double dtot_;

  double tbrk_;
  double tacc_;
  double tvel_;
  double tdec_;

  double vels_t_;

  short int signm_; // 1 = positive change, -1 = negative change
  bool shape_;      // true = trapezoidal, false = triangular

  bool isfinished_;
};
#endif
