//
// Created by giuseppe on 22.03.21.
//

#pragma once

namespace moma_controllers {



/**
 * A Trapezoidal VelocityProfile implementation.
 * @ingroup Motion
 */
class VelocityProfile_Trap
{
  // For "running" a motion profile :
  double a1,a2,a3; // coef. from ^0 -> ^2 of first part
  double b1,b2,b3; // of 2nd part
  double c1,c2,c3; // of 3rd part
  double duration;
  double t1,t2;

  // specification of the motion profile :
  double maxvel;
  double maxacc;
  double startpos;
  double endpos;
 public:

  VelocityProfile_Trap(double _maxvel=0,double _maxacc=0);
  // constructs motion profile class with <maxvel> and <maxacc> as parameters of the
  // trajectory.

  void SetProfile(double pos1,double pos2);

  void SetProfileDuration(
      double pos1,double pos2,double newduration
  );

  /** Compute trapezoidal profile at a given fraction of max velocity
          @param pos1 Position to start from
          @param pos2 Position to end at
          @param newvelocity Fraction of max velocity to use during the
          non-ramp, flat-velocity part of the profile.
          @param KDL::epsilon <= newvelocity <= 1.0 (forcibly clamped to
          this range internally)
  */
  void SetProfileVelocity(
      double pos1,double pos2,double newvelocity
  );

  void SetMax(double _maxvel,double _maxacc);
  double Duration() const;
  double Pos(double time) const;
  double Vel(double time) const;
  double Acc(double time) const;

   // returns copy of current VelocityProfile object. (virtual constructor)
  ~VelocityProfile_Trap();
};
}