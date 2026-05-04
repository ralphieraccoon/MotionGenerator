#include "MotionGenerator.h"
#include <cmath>

MotionGenerator::MotionGenerator(double aMaxvel_, double aMaxacc_,
                                 double aInitpos_)
    : maxvel_(aMaxvel_), maxacc_(aMaxacc_), initpos_(aInitpos_) {
  init();
}

void MotionGenerator::init() {
  // State variables
  reset();

  // Misc
  signm_ = 1;    // 1 = pos_itive change, -1 = negative change
  shape_ = true; // true = trapezoidal, false = triangular
  isfinished_ = false;
}

void MotionGenerator::setPositionRef(double pos_Ref) {

  if (posref_ != pos_Ref) // reference changed
  {
    isfinished_ = false;
    // Shift state variables
    posref_ = pos_Ref;
    oldpos_ = pos_;
    oldvel_ = vel_;

    // Calculate braking time and distance (in case is neeeded)
    tbrk_ = std::abs(oldvel_) / maxacc_;
    dbrk_ = tbrk_ * std::abs(oldvel_) / 2;

    // Caculate Sign of motion
    signm_ = sign(pos_Ref - (oldpos_ + sign(oldvel_) * dbrk_));

    if (signm_ != sign(oldvel_)) // means brake is needed
    {
      tacc_ = (maxvel_ / maxacc_);
      dacc_ = tacc_ * (maxvel_ / 2);
    } else {
      tbrk_ = 0;
      dbrk_ = 0;
      tacc_ = (maxvel_ - abs(oldvel_)) / maxacc_;
      dacc_ = tacc_ * (maxvel_ + abs(oldvel_)) / 2;
    }

    // Calculate total distance to go after braking
    dtot_ = std::abs(pos_Ref - oldpos_ + signm_ * dbrk_);

    tdec_ = maxvel_ / maxacc_;
    ddec_ = tdec_ * (maxvel_) / 2;
    dvel_ = dtot_ - (dacc_ + ddec_);
    tvel_ = dvel_ / maxvel_;

    if (tvel_ > 0) // trapezoidal shape
      shape_ = true;
    else // triangular shape
    {
      shape_ = false;
      // Recalculate distances and periods
      if (signm_ != (oldvel_ > 0) - (oldvel_ < 0)) // means brake is needed
      {
        vels_t_ = std::sqrt(maxacc_ * (dtot_));
        tacc_ = (vels_t_ / maxacc_);
        dacc_ = tacc_ * (vels_t_ / 2);
      } else {
        tbrk_ = 0;
        dbrk_ = 0;
        dtot_ = std::abs(pos_Ref - oldpos_); // recalculate total distance
        vels_t_ = std::sqrt(0.5 * oldvel_ * oldvel_ + maxacc_ * dtot_);
        tacc_ = (vels_t_ - std::abs(oldvel_)) / maxacc_;
        dacc_ = tacc_ * (vels_t_ + std::abs(oldvel_)) / 2;
      }
      tdec_ = vels_t_ / maxacc_;
      ddec_ = tdec_ * (vels_t_) / 2;
    }
  }
}

void MotionGenerator::updateTrapezoidalProfile(double timestep) {

  double t = static_cast<double>(timestep) /
             1000; // conversion from milliseconds to seconds

  if (shape_) // trapezoidal shape
  {
    if (t <= (tbrk_ + tacc_)) {
      pos_ = oldpos_ + oldvel_ * t + signm_ * 0.5 * maxacc_ * t * t;
      vel_ = oldvel_ + signm_ * maxacc_ * t;
      acc_ = signm_ * maxacc_;
    } else if (t > (tbrk_ + tacc_) && t < (tbrk_ + tacc_ + tvel_)) {
      pos_ =
          oldpos_ + signm_ * (-dbrk_ + dacc_ + maxvel_ * (t - tbrk_ - tacc_));
      vel_ = signm_ * maxvel_;
      acc_ = 0;
    } else if (t >= (tbrk_ + tacc_ + tvel_) &&
               t < (tbrk_ + tacc_ + tvel_ + tdec_)) {
      pos_ = oldpos_ + signm_ * (-dbrk_ + dacc_ + dvel_ +
                                 maxvel_ * (t - tbrk_ - tacc_ - tvel_) -
                                 0.5 * maxacc_ * (t - tbrk_ - tacc_ - tvel_) *
                                     (t - tbrk_ - tacc_ - tvel_));
      vel_ = signm_ * (maxvel_ - maxacc_ * (t - tbrk_ - tacc_ - tvel_));
      acc_ = -signm_ * maxacc_;
    } else {
      pos_ = posref_;
      vel_ = 0;
      acc_ = 0;
      isfinished_ = true;
    }
  } else // triangular shape
  {
    if (t <= (tbrk_ + tacc_)) {
      pos_ = oldpos_ + oldvel_ * t + signm_ * 0.5 * maxacc_ * t * t;
      vel_ = oldvel_ + signm_ * maxacc_ * t;
      acc_ = signm_ * maxacc_;
    } else if (t > (tbrk_ + tacc_) && t < (tbrk_ + tacc_ + tdec_)) {
      pos_ =
          oldpos_ +
          signm_ * (-dbrk_ + dacc_ + vels_t_ * (t - tbrk_ - tacc_) -
                    0.5 * maxacc_ * (t - tbrk_ - tacc_) * (t - tbrk_ - tacc_));
      vel_ = signm_ * (vels_t_ - maxacc_ * (t - tbrk_ - tacc_));
      acc_ = -signm_ * maxacc_;
    } else {
      pos_ = posref_;
      vel_ = 0;
      acc_ = 0;
      isfinished_ = true;
    }
  }
}

// Getters and setters
bool MotionGenerator::getFinished() { return isfinished_; }

double MotionGenerator::getPosition() { return pos_; }

double MotionGenerator::getVelocity() { return vel_; }

double MotionGenerator::getAcceleration() { return acc_; }

void MotionGenerator::setMaxVelocity(double aMaxvel_) { maxvel_ = aMaxvel_; }

void MotionGenerator::setMaxAcceleration(double aMaxacc_) {
  maxacc_ = aMaxacc_;
}

void MotionGenerator::setInitPosition(double aInitpos_) {
  initpos_ = aInitpos_;
  pos_ = aInitpos_;
  oldpos_ = aInitpos_;
}

short int MotionGenerator::sign(double aVal) {
  if (aVal < 0)
    return -1;
  else if (aVal > 0)
    return 1;
  else
    return 0;
}

void MotionGenerator::reset() {
  // Reset all state variables

  pos_ = initpos_;
  oldpos_ = initpos_;
  vel_ = 0;
  acc_ = 0;
  oldvel_ = 0;

  dbrk_ = 0;
  dacc_ = 0;
  dvel_ = 0;
  ddec_ = 0;
  dtot_ = 0;

  tbrk_ = 0;
  tacc_ = 0;
  tvel_ = 0;
  tdec_ = 0;

  vels_t_ = 0;
}
