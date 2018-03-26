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
const tf::Vector3 ZERO_VECTOR3 = tf::Vector3(0, 0, 0);
const std::array<std::array<float, 3>, 8> GRIPPER_POINTS = {
    { { -GRIPPER_HEIGHT / 2, GRIPPER_WIDTH / 2, 0 },
      { -GRIPPER_HEIGHT / 2, GRIPPER_WIDTH / 2, GRIPPER_DEPTH },
      { -GRIPPER_HEIGHT / 2, -GRIPPER_WIDTH / 2, 0 },
      { -GRIPPER_HEIGHT / 2, -GRIPPER_WIDTH / 2, GRIPPER_DEPTH },
      { GRIPPER_HEIGHT / 2, GRIPPER_WIDTH / 2, 0 },
      { GRIPPER_HEIGHT / 2, GRIPPER_WIDTH / 2, GRIPPER_DEPTH },
      { GRIPPER_HEIGHT / 2, -GRIPPER_WIDTH / 2, 0 },
      { GRIPPER_HEIGHT / 2, -GRIPPER_WIDTH / 2, GRIPPER_DEPTH } }
};

Cup::Cup(const std::string& a_namespace, float cup_x, float cup_y, float cup_z)
  : VirtualCupInterface(a_namespace)
  , cup_origin_(cup_x, cup_y, cup_z)
  , cup_center_(cup_x, cup_y, cup_z + CUP_HEIGHT / 2)
  , cup_gripper_offset(0, 0, 0)
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

    test_marker_.header.frame_id = "world";
    test_marker_.header.stamp = ros::Time::now();
    test_marker_.ns = "test_marker";
    test_marker_.id = 1;
    test_marker_.type = visualization_msgs::Marker::POINTS;

    test_marker_.pose.position.x = 0;
    test_marker_.pose.position.y = 0;
    test_marker_.pose.position.z = 0;
    test_marker_.pose.orientation.x = 0.0;
    test_marker_.pose.orientation.y = 0.0;
    test_marker_.pose.orientation.z = 0.0;
    test_marker_.pose.orientation.w = 1.0;

    test_marker_.scale.x = 0.01;  // Length
    test_marker_.scale.y = 0.01;  // Width
    test_marker_.scale.z = 0;     // Height

    test_marker_.color.r = 0.0f;
    test_marker_.color.g = 0.95f;
    test_marker_.color.b = 0.75f;
    test_marker_.color.a = 1.0;

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
        test_marker_.points.clear();
        test_marker_.colors.clear();
        std::pair<unsigned char, tf::Vector3> left_gripper_hits = applyCollision2(world_2_gripper_left);
        std::pair<unsigned char, tf::Vector3> right_gripper_hits = applyCollision2(world_2_gripper_right);

        std::cout << cupStateToString() << std::endl;
        std::cout << "Hits left: " << std::to_string(left_gripper_hits.first)
                  << " Hits right: " << std::to_string(right_gripper_hits.first) << std::endl;
        std::cout << "Cup position: " << cup_origin_.x() << "," << cup_origin_.y() << "," << cup_origin_.z()
                  << std::endl;
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
                if (!canBeGrabbed(left_gripper_hits.first, right_gripper_hits.first) &&
                    !canBeGrabbed(world_2_grip_point, world_2_gripper_left))
                {
                    cup_state_ = Cup::CupState::IDLE;
                    break;
                }
                applyGrabbing(world_2_grip_point, world_2_gripper_left);
                break;
            case Cup::CupState::IDLE:
                // If the cup is pushed a little, rotate it back up.
                rotateMarkerToQuaternion(STRAIGHT_UP, 0.2);
                std::cout << "canfall" << std::endl;
                if (canFall())
                {
                    cup_state_ = Cup::CupState::FALLING;
                }
                std::cout << "check collision" << std::endl;
                if (checkForCollision(left_gripper_hits.first, right_gripper_hits.first))
                {
                    cup_state_ = Cup::CupState::PUSHED;
                    break;
                }
                std::cout << "can be grabbed" << std::endl;
                if (canBeGrabbed(left_gripper_hits.first, right_gripper_hits.first))
                {
                    cup_gripper_offset = cup_origin_ - world_2_grip_point.getOrigin();
                    cup_state_ = Cup::CupState::GRABBED;
                }
                break;
            case Cup::CupState::PUSHED:
                if (isOverTippingPoint())
                {
                    cup_state_ = Cup::CupState::TIPPED;
                }

                if (!checkForCollision(left_gripper_hits.first, right_gripper_hits.first))
                {
                    cup_state_ = Cup::CupState::IDLE;
                }
                else if (checkForSpilling(world_2_gripper_right))
                {
                    applySpilling(world_2_gripper_right, tf::Vector3(0, 1, 0));
                    break;
                }
                else if (checkForSpilling(world_2_gripper_left))
                {
                    applySpilling(world_2_gripper_left, tf::Vector3(0, -1, 0));
                    break;
                }

                applyPushing(left_gripper_hits.second, right_gripper_hits.second);
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

void Cup::applyPushing(const tf::Vector3& left, const tf::Vector3& right)
{
    tf::Vector3 pushDirection = ZERO_VECTOR3;
    if (left != ZERO_VECTOR3)
    {
        pushDirection += left.normalized();
    }
    if (right != ZERO_VECTOR3)
    {
        pushDirection += right.normalized();
    }

    marker_.pose.position.x -= pushDirection.x() / 200;
    marker_.pose.position.y -= pushDirection.y() / 200;
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

    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(cup_rotation_);

    mat.getEulerYPR(yaw, pitch, roll);

    cup_data_.acceleration = acceleration_;
    cup_data_.state = cup_state_;
    cup_data_.velocity = velocity_;
    cup_data_.roll = roll;
    cup_data_.pitch = pitch;
    cup_data_.yaw = yaw;

    sendCupData(cup_data_);
}

void Cup::updateMarkerData()
{
    updateCupColorOnState();
    marker_.header.stamp = ros::Time::now();
    sendMarkerData(marker_);
    test_marker_.header.stamp = ros::Time::now();
    sendMarkerData(test_marker_);
}

///////////////////////////////////////////
///////State change checks/////////////////
///////////////////////////////////////////

bool Cup::canFall() const
{
    return marker_.pose.position.z > 0;
}

bool Cup::isOverTippingPoint()
{
    return std::abs(STRAIGHT_UP.angleShortestPath(cup_rotation_)) > M_PI * 0.15;
}

bool Cup::checkForCollision(unsigned char left_gripper_hits, unsigned char right_gripper_hits)
{
    return (left_gripper_hits > 0 && right_gripper_hits == 0) || (left_gripper_hits == 0 && right_gripper_hits > 0);
}

bool Cup::checkForSpilling(const tf::StampedTransform& frame)
{
    return frame.getOrigin().z() > cup_origin_.z() + CUP_HEIGHT * 0.75;
}

bool Cup::canBeGrabbed(unsigned char left_gripper_hits, unsigned char right_gripper_hits)
{
    return left_gripper_hits >= 1 && right_gripper_hits >= 1;
}

bool Cup::canBeGrabbed(const tf::StampedTransform& grip_point, const tf::StampedTransform& left_gripper)
{
    return grip_point.getOrigin().distance(left_gripper.getOrigin()) < CUP_RADIUS * 0.75;
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

std::pair<unsigned char, tf::Vector3> Cup::applyCollision2(const tf::StampedTransform& frame)
{
    tf::Vector3 cup_height = cup_origin_ + tf::quatRotate(cup_rotation_, tf::Vector3(CUP_HEIGHT, 0, 0));
    tf::Vector3 directionVector = ZERO_VECTOR3;
    unsigned char hits = 0;

    geometry_msgs::Point hit_point;
    std_msgs::ColorRGBA hit_point_color;

    for (auto& point : GRIPPER_POINTS)
    {
        tf::Vector3 transformed_point =
            tf::quatRotate(frame.getRotation(), tf::Vector3(point[0], point[1], point[2])) + frame.getOrigin();

        hit_point.x = transformed_point.x();
        hit_point.y = transformed_point.y();
        hit_point.z = transformed_point.z();

        double distanceToCup = getDistanceFromPointToLine(cup_origin_, cup_height, transformed_point);

        if (distanceToCup <= 0.025)
        {
            directionVector += (transformed_point - cup_center_);
            hits++;

            hit_point_color.r = 1;
            hit_point_color.g = 0;
            hit_point_color.b = 0;
            hit_point_color.a = 1;
        }
        else
        {
            hit_point_color.r = 0;
            hit_point_color.g = 1;
            hit_point_color.b = 0;
            hit_point_color.a = 1;
        }

        test_marker_.colors.push_back(hit_point_color);
        test_marker_.points.push_back(hit_point);
    }

    return std::make_pair(hits, directionVector);
}

double Cup::getDistanceFromPointToLine(const tf::Vector3& line_start, const tf::Vector3& line_end,
                                       const tf::Vector3& point)
{
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    tf::Vector3 v = line_end - line_start;
    tf::Vector3 w = point - line_start;
    double c1 = w.dot(v);
    double c2 = v.dot(v);

    if (c1 <= 0)
    {
        return (point - line_start).length();
    }
    else if (c2 <= c1)
    {
        return (point - line_end).length();
    }
    else
    {
        double b = c1 / c2;
        tf::Vector3 Pb = line_start + b * v;
        return (point - Pb).length();
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
    marker_.pose.position.x = grip_point.getOrigin().x() + cup_gripper_offset.x();
    marker_.pose.position.y = grip_point.getOrigin().y() + cup_gripper_offset.y();
    marker_.pose.position.z = grip_point.getOrigin().z() + cup_gripper_offset.z();

    // Is the gripper to wide open?
    //    if ((grip_point.getOrigin() + cup_gripper_offset).distance(gripper_left.getOrigin()) > CUP_RADIUS / 2)
    //    {
    //        cup_state_ = Cup::CupState::IDLE;
    //    }
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

std::string Cup::cupStateToString() const
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
                           const tf::Vector3& to)
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
