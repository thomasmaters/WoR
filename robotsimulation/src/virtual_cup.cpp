/*
 * Cup.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#include "virtual_cup.h"
#include <thread>

Cup::Cup(const std::string& a_namespace, float cup_x, float cup_y, float cup_z)
  : VirtualCupInterface(a_namespace)
  , cup_origin_(cup_x, cup_y, cup_z)
  , cup_center_(cup_x, cup_y, cup_z + CUP_HEIGHT / 2)
  , cup_state(Cup::CupState::IDLE)
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

    cup_data_.acceleration = acceleration_;
    cup_data_.state = cup_state;
    cup_data_.velocity = velocity_;

    last_frame_time_ = ros::Time(0);
}

void Cup::loop()
{
    int skipped_frames = 0;

    ros::Rate rate(FPS);
    while (ros::ok())
    {
        cup_origin_ = tf::Vector3(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z);
        cup_center_ = cup_origin_ + tf::Vector3(0, 0, CUP_HEIGHT / 2);

        tf::StampedTransform world_2_gripper_left = getTransform("/world", "/gripper_left");
        tf::StampedTransform world_2_gripper_right = getTransform("/world", "/gripper_right");
        tf::StampedTransform world_2_grip_point = getTransform("/world", "/grip_point");

        if (last_frame_time_ == world_2_grip_point.stamp_)
        {
            // ROS_WARN("Frame time did not update in time.");
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

        //        std::cout << cupStateToString() << std::endl;

        switch (cup_state)
        {
            case Cup::CupState::FALLING:
                if (!canFall())
                {
                    fall_velocity_ = 0;
                    cup_state = Cup::CupState::IDLE;
                    break;
                }
                applyGravity();
                break;
            case Cup::CupState::GRABBED:
                applyGrabbing(world_2_grip_point, world_2_gripper_left);
                break;
            case Cup::CupState::IDLE:
                if (canFall())
                {
                    cup_state = Cup::CupState::FALLING;
                }
                if (checkForCollision(world_2_gripper_left, world_2_gripper_right))
                {
                    cup_state = Cup::CupState::PUSHED;
                    break;
                }
                if (canBeGrabbed(world_2_grip_point, world_2_gripper_left))
                {
                    cup_state = Cup::CupState::GRABBED;
                }
                break;
            case Cup::CupState::PUSHED:
                applyCollision(world_2_gripper_left, world_2_gripper_right);
                if (!checkForCollision(world_2_gripper_left, world_2_gripper_right))
                {
                    cup_state = Cup::CupState::IDLE;
                }
                break;
            case Cup::CupState::TIPPED:
                break;
            default:
                break;
        }
        rate.sleep();

        marker_.header.stamp = ros::Time::now();
        sendMarkerData(marker_);
        updateCupData();
    }
}

void Cup::updateCupData()
{
    double temp_velocity = calculateSpeed(
        cup_origin_, tf::Vector3(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z));
    acceleration_ = calculateAcceleration(velocity_, temp_velocity);
    velocity_ = temp_velocity;

    cup_data_.acceleration = acceleration_;
    cup_data_.state = cup_state;
    cup_data_.velocity = velocity_;

    sendCupData(cup_data_);
}

std::string Cup::cupStateToString()
{
    switch (cup_state)
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

Cup::GripperToCupState Cup::getDirectionToTransform(const tf::StampedTransform& transform)
{
    // Get relative forward en upwards vector.
    tf::Vector3 forward = tf::quatRotate(transform.getRotation(), tf::Vector3(0, 0, 1));
    tf::Vector3 up = tf::quatRotate(transform.getRotation(), tf::Vector3(1, 0, 0));

    float dir = getRightOrLeft(up, forward, cup_center_, transform.getOrigin());
    return dir <= 0 ? Cup::GripperToCupState::RIGHT : Cup::GripperToCupState::LEFT;
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

void Cup::applyCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right)
{
    GripperToCupState leftGripperState = getDirectionToTransform(gripper_left);
    GripperToCupState rightGripperState = getDirectionToTransform(gripper_right);

    tf::Vector3 cup_origin = tf::Vector3(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z);
    tf::Vector3 cup_center = cup_origin + tf::Vector3(0, 0, CUP_HEIGHT / 2);

    if (cup_center.distance(gripper_left.getOrigin()) < CUP_TO_GRIPPER_COLLISION &&
        leftGripperState == Cup::GripperToCupState::RIGHT)
    {
        tf::Vector3 moveDirection = tf::quatRotate(gripper_left.getRotation(), tf::Vector3(0, 1, 0)).normalized() / 100;
        marker_.pose.position.x += moveDirection.x();
        marker_.pose.position.y += moveDirection.y();
    }
    else if (cup_center.distance(gripper_right.getOrigin()) < CUP_TO_GRIPPER_COLLISION &&
             rightGripperState == Cup::GripperToCupState::LEFT)
    {
        tf::Vector3 moveDirection =
            tf::quatRotate(gripper_right.getRotation(), tf::Vector3(0, -1, 0)).normalized() / 100;
        marker_.pose.position.x += moveDirection.x();
        marker_.pose.position.y += moveDirection.y();
    }
}

bool Cup::canBeGrabbed(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left)
{
    return cup_center_.distance(grip_point.getOrigin()) <= 0.045 &&
           cup_center_.distance(gripper_left.getOrigin()) <= 0.045;
}

void Cup::applyGrabbing(const tf::StampedTransform& grip_point, const tf::StampedTransform& gripper_left)
{
    marker_.pose.position.x = grip_point.getOrigin().x();
    marker_.pose.position.y = grip_point.getOrigin().y();
    marker_.pose.position.z = grip_point.getOrigin().z() - CUP_HEIGHT / 2;

    // Is the gripper to wide open?
    if (grip_point.getOrigin().distance(gripper_left.getOrigin()) > 0.045)
    {
        cup_state = Cup::CupState::IDLE;
    }
}

double Cup::getRightOrLeft(const tf::Vector3& up, const tf::Vector3& forward, const tf::Vector3& from,
                           const tf::Vector3& to) const
{
    tf::Vector3 delta = from - to;
    tf::Vector3 cross = forward.cross(delta);
    return cross.dot(up);
}

void Cup::applyGravity()
{
    if (cup_state == Cup::CupState::FALLING)
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

bool Cup::canFall()
{
    return marker_.pose.position.z > 0 /* && cup_state != Cup::CupState::GRABBED*/;
}

double Cup::calculateSpeed(const tf::Vector3& old_pos, const tf::Vector3& new_pos)
{
    double distance = std::abs(old_pos.distance(new_pos));

    //    std::cout << "Distance: " << distance << " pos: " << old_pos.x() << "," << old_pos.y() << "," << old_pos.z()
    //              << " - " << new_pos.x() << "," << new_pos.y() << "," << new_pos.z() << std::endl;

    // Snelheid in m/s over 1 frame.
    //    if (distance == 0)
    //    {
    //        return velocity_;
    //    }
    return distance / (1.0 / FPS);
}

double Cup::calculateAcceleration(double old_speed, double new_speed)
{
    // Versnelling in m/s^2
    return (new_speed - old_speed) / (1.0 / FPS);
}

Cup::~Cup()
{
}
