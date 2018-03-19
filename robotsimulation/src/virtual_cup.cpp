/*
 * Cup.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#include "virtual_cup.h"

const tf::Matrix3x3 left_rotation_matrix =
    tf::Matrix3x3(1, 0, 0, 0, std::cos(-90), -std::sin(-90), 0, std::sin(-90), std::cos(-90));

const tf::Matrix3x3 up_rotation_matrix =
    tf::Matrix3x3(std::cos(90), 0, std::sin(90), 0, 1, 0, -std::sin(90), 0, std::cos(90));

const tf::Matrix3x3 forward_rotation_matrix =
    tf::Matrix3x3(1, 0, 0, 0, std::cos(-90), -std::sin(-90), 0, std::sin(-90), std::cos(-90));

Cup::Cup(const std::string& a_namespace)
  : VirtualCupInterface(a_namespace), cup_state(Cup::CupState::IDLE), namespace_(a_namespace), fall_velocity_(0)
{
    // TODO Maken we de cup relatief aan de robotarm of de wereld?
    marker_.header.frame_id = "world";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = namespace_;
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::ARROW;

    marker_.pose.position.x = 0.35;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0.1;
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

    //    test_marker_.header.frame_id = "world";
    //    test_marker_.header.stamp = ros::Time::now();
    //    test_marker_.ns = "test_marker";
    //    test_marker_.id = 1;
    //    test_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    //
    //    test_marker_.pose.position.x = 0;
    //    test_marker_.pose.position.y = 0;
    //    test_marker_.pose.position.z = 0;
    //    test_marker_.pose.orientation.x = 0.0;
    //    test_marker_.pose.orientation.y = 0.0;
    //    test_marker_.pose.orientation.z = 0.0;
    //    test_marker_.pose.orientation.w = 1.0;
    //
    //    geometry_msgs::Point test;
    //    test.x = 0;
    //    test.y = 0;
    //    test.z = 0;
    //    test_marker_.points.push_back(test);
    //    test_marker_.points.push_back(test);
    //
    //    test_marker_.scale.x = CUP_HEIGHT;  // Length
    //    test_marker_.scale.y = CUP_RADIUS;  // Width
    //    test_marker_.scale.z = CUP_RADIUS;  // Height
    //
    //    test_marker_.color.r = 0.0f;
    //    test_marker_.color.g = 0.95f;
    //    test_marker_.color.b = 0.75f;
    //    test_marker_.color.a = 1.0;
}

void Cup::loop()
{
    ros::Rate rate(FPS);
    while (ros::ok())
    {
        cup_origin_ = tf::Vector3(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z);
        cup_center_ = cup_origin_ + tf::Vector3(0, 0, CUP_HEIGHT / 2);

        tf::StampedTransform world_2_gripper_left = getTransform("/world", "/gripper_left");
        tf::StampedTransform world_2_gripper_right = getTransform("/world", "/gripper_right");
        tf::StampedTransform world_2_grip_point = getTransform("/world", "/grip_point");

        std::cout << cupStateToString() << std::endl;

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
            default:
                break;
        }
        rate.sleep();
        sendMarkerData(marker_);
        //        sendMarkerData(test_marker_);
    }
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

    //    std::cout << "Left gripper: " << (leftGripperState == LEFT ? "LEFT" : "RIGHT") << std::endl;
    //    std::cout << "Right gripper: " << (rightGripperState == LEFT ? "LEFT" : "RIGHT") << std::endl;

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
    //    std::cout << "Distance cup to gripperleft: " << grip_point.getOrigin().distance(gripper_left.getOrigin())
    //              << std::endl;
    if (canBeGrabbed(grip_point, gripper_left))
    {
        marker_.pose.position.x = grip_point.getOrigin().x();
        marker_.pose.position.y = grip_point.getOrigin().y();
        marker_.pose.position.z = grip_point.getOrigin().z() - CUP_HEIGHT / 2;
    }
    else
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
            marker_.pose.position.z = 0;
        }
    }
}

bool Cup::canFall()
{
    return marker_.pose.position.z > 0 /* && cup_state != Cup::CupState::GRABBED*/;
}

void calculateSpeed(const tf::Vector3& old_pos, const tf::Vector3& new_pos)
{
    double distance = old_pos.distance(new_pos);

    // Snelheid in m/s over 1 frame.
    double speed = distance / (1 / FPS);
}

void calculateAcceleration(double old_speed, double new_speed)
{
    // Versnelling in m/s^2
    double acceleration = (new_speed - old_speed) / (1 / FPS);
}

Cup::~Cup()
{
}
