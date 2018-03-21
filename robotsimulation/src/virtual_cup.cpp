/*
 * Cup.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#include "virtual_cup.h"
#include <thread>

const tf::Quaternion STRAIGHT_UP = tf::Quaternion(0.0, -0.707, 0.0, 0.707);
const tf::Quaternion STRAIGHT_DOWN = tf::Quaternion(0.0, 0.707, 0.0, 0.707);

Cup::Cup(const std::string& a_namespace, float cup_x, float cup_y, float cup_z)
  : VirtualCupInterface(a_namespace)
  , cup_origin_(cup_x, cup_y, cup_z)
  , cup_center_(cup_x, cup_y, cup_z + CUP_HEIGHT / 2)
  , cup_state_(Cup::CupState::IDLE)
  , namespace_(a_namespace)
  , fall_velocity_(0)
  , acceleration_(0)
  , velocity_(0)
{
    marker_.header.frame_id = "world";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = namespace_;
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::ARROW;

    marker_.pose.position.x = cup_x;
    marker_.pose.position.y = cup_y;
    marker_.pose.position.z = cup_z;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = -0.707;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 0.707;

    marker_.scale.x = CUP_HEIGHT;  // Length
    marker_.scale.y = CUP_RADIUS;  // Width
    marker_.scale.z = CUP_RADIUS;  // Height

    marker_.color.r = 0.95f;
    marker_.color.g = 0.95f;
    marker_.color.b = 0.75f;
    marker_.color.a = 1.0;

    last_frame_time_ = ros::Time(0);
}

void Cup::loop()
{
    int skipped_frames = 0;

    ros::Rate rate(FPS);
    while (ros::ok())
    {
        cup_rotation_ = tf::Quaternion(marker_.pose.orientation.x, marker_.pose.orientation.y,
                                       marker_.pose.orientation.z, marker_.pose.orientation.w);
        cup_origin_ = tf::Vector3(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z);
        cup_center_ = cup_origin_ + tf::quatRotate(cup_rotation_, tf::Vector3(CUP_HEIGHT / 2, 0, 0));

        tf::StampedTransform world_2_gripper_left = getTransform("/world", "/gripper_left");
        tf::StampedTransform world_2_gripper_right = getTransform("/world", "/gripper_right");
        tf::StampedTransform world_2_grip_point = getTransform("/world", "/grip_point");

        if (last_frame_time_ == world_2_grip_point.stamp_)
        {
            rate.sleep();
            skipped_frames++;
            continue;
        }
        else
        {
            ROS_INFO_STREAM("Skipped: " << skipped_frames << " frames.");
            skipped_frames = 0;
        }

        last_frame_time_ = world_2_grip_point.stamp_;

        switch (cup_state_)
        {
            case Cup::CupState::FALLING:
                if (!canFall())
                {
                    fall_velocity_ = 0;
                    cup_state_ = Cup::CupState::IDLE;
                    break;
                }
                applyGravity();
                break;
            case Cup::CupState::GRABBED:
                applyGrabbing(world_2_grip_point, world_2_gripper_left);
                break;
            case Cup::CupState::IDLE:
                // If the cup is pushed a little, rotate it back up.
                rotateMarkerToQuaternion(STRAIGHT_UP, 0.2);
                if (canFall())
                {
                    cup_state_ = Cup::CupState::FALLING;
                }
                if (checkForCollision(world_2_gripper_left, world_2_gripper_right))
                {
                    cup_state_ = Cup::CupState::PUSHED;
                    break;
                }
                if (canBeGrabbed(world_2_grip_point, world_2_gripper_left))
                {
                    cup_state_ = Cup::CupState::GRABBED;
                }
                break;
            case Cup::CupState::PUSHED:
                applyCollision(world_2_gripper_left, world_2_gripper_right);
                if (isOverTippingPoint())
                {
                    cup_state_ = Cup::CupState::TIPPED;
                }
                if (!checkForCollision(world_2_gripper_left, world_2_gripper_right))
                {
                    cup_state_ = Cup::CupState::IDLE;
                }
                break;
            case Cup::CupState::TIPPED:
                rotateMarkerToQuaternion(STRAIGHT_DOWN, 0.2);
                break;
            default:
                break;
        }

        rate.sleep();
        updateMarkerData();
        updateCupData();
    }
}

///////////////////////////////////////////
///////Communication with interface////////
///////////////////////////////////////////

void Cup::updateCupData()
{
    double temp_velocity = calculateVelocity(
        cup_origin_, tf::Vector3(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z));
    acceleration_ = calculateAcceleration(velocity_, temp_velocity);
    velocity_ = temp_velocity;

    cup_data_.acceleration = acceleration_;
    cup_data_.state = cup_state_;
    cup_data_.velocity = velocity_;

    sendCupData(cup_data_);
}

void Cup::updateMarkerData()
{
    updateCupColorOnState();
    marker_.header.stamp = ros::Time::now();
    sendMarkerData(marker_);
}

///////////////////////////////////////////
///////State change checks/////////////////
///////////////////////////////////////////

bool Cup::canFall()
{
    return marker_.pose.position.z > 0;
}

bool Cup::isOverTippingPoint()
{
    return std::abs(STRAIGHT_UP.angleShortestPath(cup_rotation_)) > M_PI * 0.15;
}

bool Cup::checkForCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right)
{
    GripperToCupState leftGripperState = getDirectionToTransform(gripper_left);
    GripperToCupState rightGripperState = getDirectionToTransform(gripper_right);

    return (cup_center_.distance(gripper_left.getOrigin()) < CUP_TO_GRIPPER_COLLISION &&
            leftGripperState == Cup::GripperToCupState::RIGHT) ||
           (cup_center_.distance(gripper_right.getOrigin()) < CUP_TO_GRIPPER_COLLISION &&
            rightGripperState == Cup::GripperToCupState::LEFT);
}

bool Cup::checkForSpilling(const tf::StampedTransform& frame)
{
    return frame.getOrigin().z() > cup_origin_.z() + CUP_HEIGHT * 0.75;
}

bool Cup::canBeGrabbed(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left)
{
    return cup_center_.distance(grip_point.getOrigin()) <= CUP_GRAB_RADIUS &&
           cup_center_.distance(gripper_left.getOrigin()) <= CUP_GRAB_RADIUS;
}

///////////////////////////////////////////
///////State functions/////////////////////
///////////////////////////////////////////

void Cup::applyCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right)
{
    GripperToCupState leftGripperState = getDirectionToTransform(gripper_left);
    GripperToCupState rightGripperState = getDirectionToTransform(gripper_right);

    if (cup_center_.distance(gripper_left.getOrigin()) < CUP_TO_GRIPPER_COLLISION &&
        leftGripperState == Cup::GripperToCupState::RIGHT)
    {
        applyCollisionToCup(gripper_left, tf::Vector3(0, 1, 0));
    }
    else if (cup_center_.distance(gripper_right.getOrigin()) < CUP_TO_GRIPPER_COLLISION &&
             rightGripperState == Cup::GripperToCupState::LEFT)
    {
        applyCollisionToCup(gripper_right, tf::Vector3(0, -1, 0));
    }
}

void Cup::applyCollisionToCup(const tf::StampedTransform& frame, const tf::Vector3& fall_direction)
{
    if (checkForSpilling(frame))
    {
        applySpilling(frame, fall_direction);
    }
    else
    {
        tf::Vector3 moveDirection = tf::quatRotate(frame.getRotation(), fall_direction).normalized();
        marker_.pose.position.x += moveDirection.x() / 100;
        marker_.pose.position.y += moveDirection.y() / 100;
    }
}

void Cup::applySpilling(const tf::StampedTransform& frame, const tf::Vector3& fall_direction)
{
    tf::Vector3 moveDirection = tf::quatRotate(frame.getRotation(), fall_direction).normalized();
    tf::Vector3 w = tf::Vector3(1, 0, 0).cross(moveDirection);
    tf::Quaternion move_to_quaternion = tf::Quaternion(w.x(), w.y(), w.z(), tf::Vector3(1, 0, 0).dot(moveDirection));
    move_to_quaternion.setW(move_to_quaternion.length() + move_to_quaternion.w());
    move_to_quaternion = move_to_quaternion.normalized();
    rotateMarkerToQuaternion(move_to_quaternion, 0.05);
}

void Cup::applyGrabbing2(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left)
{
    marker_.header.frame_id = "grip_point";
    marker_.pose.position.x = 0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0 - CUP_HEIGHT / 2;

    // Is the gripper to wide open?
    if (grip_point.getOrigin().distance(gripper_left.getOrigin()) > CUP_RADIUS)
    {
        marker_.header.frame_id = "world";
        marker_.pose.position.x = grip_point.getOrigin().x();
        marker_.pose.position.y = grip_point.getOrigin().y();
        marker_.pose.position.z = grip_point.getOrigin().z() - CUP_HEIGHT / 2;
        cup_state_ = Cup::CupState::IDLE;
    }
}

void Cup::applyGrabbing(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left)
{
    marker_.pose.position.x = grip_point.getOrigin().x();
    marker_.pose.position.y = grip_point.getOrigin().y();
    marker_.pose.position.z = grip_point.getOrigin().z() - CUP_HEIGHT / 2;

    // Is the gripper to wide open?
    if (grip_point.getOrigin().distance(gripper_left.getOrigin()) > CUP_RADIUS)
    {
        cup_state_ = Cup::CupState::IDLE;
    }
}

void Cup::applyGravity()
{
    if (cup_state_ == Cup::CupState::FALLING)
    {
        fall_velocity_ = fall_velocity_ + (1.0 / FPS) * GRAVITY;
        marker_.pose.position.z -= fall_velocity_ * (1.0 / FPS);
        if (marker_.pose.position.z < 0)
        {
            // Enable this to demo rqt plot of acceleration and velocity.
            //            fall_velocity_ = 0;
            //            velocity_ = 0;
            //            acceleration_ = 0;
            marker_.pose.position.z = 0;
        }
    }
}

///////////////////////////////////////////
///////Pyhisics functions//////////////////
///////////////////////////////////////////

double Cup::calculateVelocity(const tf::Vector3& old_pos, const tf::Vector3& new_pos)
{
    double distance = std::abs(old_pos.distance(new_pos));

    // Velocity in m/s over 1 frame.
    return distance / (1.0 / FPS);
}

double Cup::calculateAcceleration(double old_speed, double new_speed)
{
    // Acceleration in m/s^2
    return (new_speed - old_speed) / (1.0 / FPS);
}

///////////////////////////////////////////
///////Misc functions//////////////////////
///////////////////////////////////////////

std::string Cup::cupStateToString()
{
    switch (cup_state_)
    {
        case Cup::CupState::FALLING:
            return "FALLING";
        case Cup::CupState::GRABBED:
            return "GRABBED";
        case Cup::CupState::IDLE:
            return "IDLE";
        case Cup::CupState::PUSHED:
            return "PUSHED";
        default:
            return "Non state";
    }
}

Cup::GripperToCupState Cup::getDirectionToTransform(const tf::StampedTransform& frame)
{
    // Get relative forward en upwards vector.
    tf::Vector3 forward = tf::quatRotate(frame.getRotation(), tf::Vector3(0, 0, 1));
    tf::Vector3 up = tf::quatRotate(frame.getRotation(), tf::Vector3(1, 0, 0));

    float dir = getRightOrLeft(up, forward, cup_center_, frame.getOrigin());
    return dir <= 0 ? Cup::GripperToCupState::RIGHT : Cup::GripperToCupState::LEFT;
}

double Cup::getRightOrLeft(const tf::Vector3& up, const tf::Vector3& forward, const tf::Vector3& from,
                           const tf::Vector3& to) const
{
    tf::Vector3 delta = from - to;
    tf::Vector3 cross = forward.cross(delta);
    return cross.dot(up);
}

void Cup::rotateMarkerToQuaternion(tf::Quaternion rotation, float t)
{
    rotation = cup_rotation_.slerp(rotation, t);
    marker_.pose.orientation.w = rotation.w();
    marker_.pose.orientation.x = rotation.x();
    marker_.pose.orientation.y = rotation.y();
    marker_.pose.orientation.z = rotation.z();
}

void Cup::updateCupColorOnState()
{
    switch (cup_state_)
    {
        case Cup::CupState::FALLING:
            marker_.color.r = 0;
            marker_.color.g = 0.4;
            marker_.color.b = 1;
            break;
        case Cup::CupState::GRABBED:
            marker_.color.r = 0;
            marker_.color.g = 1;
            marker_.color.b = 0;
            break;
        case Cup::CupState::IDLE:
            marker_.color.r = 0.95;
            marker_.color.g = 0.95;
            marker_.color.b = 0.75;
            break;
        case Cup::CupState::PUSHED:
            marker_.color.r = 0.8;
            marker_.color.g = 0.4;
            marker_.color.b = 0.6;
            break;
        case Cup::CupState::TIPPED:
            marker_.color.r = 1;
            marker_.color.g = 0;
            marker_.color.b = 0;
            break;
        default:
            // Something is terribly wrong????
            break;
    }
}

Cup::~Cup()
{
}
