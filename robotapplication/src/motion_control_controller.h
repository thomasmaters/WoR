/*
 * motion_control_controller.h
 *
 *  Created on: Apr 5, 2017
 *      Author: Thomas Maters
 */

#ifndef SRC_MOTIONCONTROL_H_
#define SRC_MOTIONCONTROL_H_

#include "InverseKinematics.h"
#include "motion_control_interface.h"

// Official values as taken from the documentation.
// static const Matrix<3, 1, double> armLengths = Matrix<3, 1, double>({ 0, 146, 187 });
// Default values in the simulator.
// static const Matrix<3, 1, double> armLengths = Matrix<3, 1, double>({ 0, 180, 200 });

class MotionControl : public MotionControlInterface
{
  public:
    MotionControl();
    virtual ~MotionControl();

    void moveToTarget(const robotapplication::pick_and_place msg);

  private:
    void moveToPos(const float x, const float y, const float z, const float rotation, const bool gripperState = false);

  private:
    Matrix<3, 1, double> phis;
    Matrix<3, 1, double> lastPhis;
    Matrix<3, 1, double> armLengths;
    Matrix<3, 2, double> solutionSpace;
};

#endif /* SRC_MOTIONCONTROL_H_ */
