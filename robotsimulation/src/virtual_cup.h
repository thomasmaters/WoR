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
#define MIN_CUP_GRAB_RADIUS 0.035
#define GRAVITY 9.8     // m/s^2
#define CUP_WEIGHT 0.1  // kg
#define FPS 30
#define CUP_TO_GRIPPER_COLLISION 0.07

#define GRIPPER_WIDTH 0.02
#define GRIPPER_HEIGHT 0.02
#define GRIPPER_DEPTH 0.03

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
    bool isOverTippingPoint() const;

    /**
     * Checks based on the gripper hits if the cup is being collided with.
     * @param left_gripper_hits
     * @param right_gripper_hits
     * @return True if colliding with cup.
     */
    static bool checkForCollision(unsigned char left_gripper_hits, unsigned char right_gripper_hits);

    /**
     * Checks if the gripper is gripping to cup to high that it will tip.
     * @param frame Frame to check against.
     * @return True if the frame is about to tip the cup.
     * @author Thomas Maters
     */
    bool checkForSpilling(const tf::StampedTransform& frame) const;

    /**
     * Checks based on distance from gripper to grip_point if the cup can be grabbed.
     * @param grip_point
     * @param left_gripper
     * @return True if cup can be grabbed.
     */
    static bool canBeGrabbed(const tf::StampedTransform& grip_point, const tf::StampedTransform& left_gripper);

    /**
     * Checks if the cup can be grabbed based on the collison points of both grippers.
     * @param left_gripper_hits
     * @param right_gripper_hits
     * @return True if cup can be grabbed.
     */
    static bool canBeGrabbed(unsigned char left_gripper_hits, unsigned char right_gripper_hits);

    /**
     * Checks its collision points if they are in contact it with the cup.
     * @param frame Gripper frame.
     * @return How many points hit and the combined push direction vector.
     */
    std::pair<unsigned char, tf::Vector3> applyCollision2(const tf::StampedTransform& frame);

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
     * Applies pushing of the gripper on the cup based on the left and right push direction vectors.
     * @param left
     * @param right
     */
    void applyPushing(const tf::Vector3& left, const tf::Vector3& right);

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
     * Calculates how far a point is from a line segment.
     * @param line_start
     * @param line_end
     * @param point
     * @return Closest distance from line to point.
     */
    static double getDistanceFromPointToLine(const tf::Vector3& line_start, const tf::Vector3& line_end,
                                             const tf::Vector3& point);

    /**
     * Converts current state to a string.
     * @return String representation of the current state.
     * @author Thomas Maters
     */
    std::string cupStateToString() const;

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
    tf::Vector3 cup_gripper_offset;
    tf::Quaternion cup_rotation_;

    CupState cup_state_;
    std::string namespace_;  /// Unique namespace so we can have multiple markers.

    double fall_velocity_;
    double acceleration_;
    double velocity_;

    visualization_msgs::Marker marker_;
    visualization_msgs::Marker detection_marker_;
    robotsimulation::cup_data cup_data_;

    ros::Time last_frame_time_;  /// Variable to check if we have got an updated since last frame request.
};

#endif /* WOR_ROBOTSIMULATION_SRC_VIRTUAL_CUP_H_ */
