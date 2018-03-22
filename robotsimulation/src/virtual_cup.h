/*
 * Cup.h
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#ifndef WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_
#define WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_

#include "virtual_cup_interface.h"

#include "robotsimulation/cup_data.h"

#include <visualization_msgs/Marker.h>

#define CUP_HEIGHT .1
#define CUP_RADIUS 0.05
#define CUP_GRAB_RADIUS 0.045
#define GRAVITY 9.8     // m/s^2
#define CUP_WEIGHT 0.1  // kg
#define FPS 30
#define CUP_TO_GRIPPER_COLLISION 0.07

/**
 * Cup logic.
 * @author Thomas Maters
 */
class Cup : public VirtualCupInterface
{
  public:
    enum CupState
    {
        IDLE,
        FALLING,
        GRABBED,
        PUSHED,
        TIPPED
    };  /// State of the cup.

    enum GripperToCupState
    {
        LEFT = -1,
        RIGHT = 1,
        CENTER = 0
    };  /// Representation if the gripper is left or right of the cup.

    Cup(const std::string& a_namespace, float cup_x = 0, float cup_y = 0, float cup_z = 0);

    void loop();

  private:
    /**
     * Sends cup data(acceleration, rotation, velocity, state) to the interface.
     * @author Thomas Maters
     */
    void updateCupData();

    /**
     * Sends marker message to the interface.
     * @author Thomas Maters
     */
    void updateMarkerData();

    /**
     * Checks if all the conditions are met to let the cup fall.
     * @return True if the cup can fall.
     * @author Thomas Maters
     */
    bool canFall() const;

    /**
     * Checks if the cup is rotated past its tipping point.
     * @return True if tipped.
     * @author Thomas Maters
     */
    bool isOverTippingPoint();

    /**
     * Checks both left and right gripper of the robot arm if they are colliding with the cup.
     * @param gripper_left
     * @param gripper_right
     * @return True if a collision is happening.
     * @author Thomas Maters
     */
    bool checkForCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right);

    /**
     * Checks if the gripper is gripping to cup to high that it will tip.
     * @param frame Frame to check against.
     * @return True if the frame is about to tip the cup.
     * @author Thomas Maters
     */
    bool checkForSpilling(const tf::StampedTransform& frame);

    /**
     * Checks if the cup can be grabbed.
     * @param grip_point Frame representing the grippoint of the arm.
     * @param gripper_left Frame representing the frame of the left gripper.
     * @return True if it the cup can be grabbed.
     * @author Thomas Maters
     */
    bool canBeGrabbed(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left);

    /**
     * Determens on which frame is colliding with the cup and which direction it is colliding from.
     * @param gripper_left
     * @param gripper_right
     * @author Thomas Maters
     */
    void applyCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right);

    /**
     * Applies collision calculation to the cup.
     * @param frame To calculate the push direction from.
     * @param push_direction Which direction to push the cup to.
     * @author Thomas Maters
     */
    void applyCollisionToCup(const tf::StampedTransform& frame, const tf::Vector3& push_direction);

    /**
     * Applies spilling physics to the cup.
     * @param frame To calculate the push direction from.
     * @param fall_direction Which direction to let the cup fall to.
     * @author Thomas Maters
     */
    void applySpilling(const tf::StampedTransform& frame, const tf::Vector3& fall_direction);

    /**
     * Grabs the cup into the grippoint, by setting its position.
     * @param grip_point Frame of the grippoint in relation to the world.
     * @param gripper_left Frame of the left_gripper in relation to the world.
     * @author Thomas Maters
     */
    void applyGrabbing(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left);

    /**
     * Grabs the cup by changing the frame of the cup to the grippoint.
     * @param grip_point Frame of the grippoint in relation to the world.
     * @param gripper_left Frame of the left_gripper in relation to the world.
     * @author Thomas Maters
     */
    void applyGrabbing2(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left);

    /**
     * Applies gravity to the cup.
     * @author Thomas Maters
     */
    void applyGravity();

    /**
     * Calculates the velocity of the cup.
     * @param old_pos Old cup position from last tick.
     * @param new_pos New cup position.
     * @return Velocity in m/s.
     * @author Thomas Maters
     */
    static double calculateVelocity(const tf::Vector3& old_pos, const tf::Vector3& new_pos);

    /**
     * Calculates the acceleration of the cup.
     * @param old_speed Old acceleration from last tick.
     * @param new_speed New acceleration.
     * @return Acceleration in m/s^2.
     * @author Thomas Maters
     */
    static double calculateAcceleration(double old_speed, double new_speed);

    /**
     * Converts current state to a string.
     * @return String representation of the current state.
     * @author Thomas Maters
     */
    std::string cupStateToString() const;

    /**
     * Calculates the direction the gripper is in relation to the cup.
     * @param frame Of the gripper.
     * @return
     * @author Thomas Maters
     */
    Cup::GripperToCupState getDirectionToTransform(const tf::StampedTransform& frame);

    /**
     *
     * @param up Vector of gripper.
     * @param forward Vector of gripper.
     * @param from Right or left from.
     * @param to Right or left in relation to.
     * @return Value representing left or right.
     * @author Thomas Maters
     */
    static double getRightOrLeft(const tf::Vector3& up, const tf::Vector3& forward, const tf::Vector3& from,
                                 const tf::Vector3& to);

    /**
     * Rotates the marker_ to a Quaternion based on a factor to move to.
     * @param rotation Quaternion to move to.
     * @param t [0-1]
     * @author Thomas Maters
     */
    void rotateMarkerToQuaternion(tf::Quaternion rotation, float t = 0.0);

    /**
     * Changes the markers color based on the cup state.
     * @author Thomas Maters
     */
    void updateCupColorOnState();

  public:
    virtual ~Cup();

  private:
    tf::Vector3 cup_origin_;
    tf::Vector3 cup_center_;
    tf::Quaternion cup_rotation_;

    CupState cup_state_;
    std::string namespace_;  /// Unique namespace so we can have multiple markers.

    double fall_velocity_;
    double acceleration_;
    double velocity_;

    visualization_msgs::Marker marker_;
    robotsimulation::cup_data cup_data_;

    ros::Time last_frame_time_;  /// Variable to check if we have got an updated since last frame request.
};

#endif /* WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_ */
