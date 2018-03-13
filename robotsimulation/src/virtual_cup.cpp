/*
 * Cup.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#include "virtual_cup.h"

Cup::Cup(const std::string& a_namespace)
  : VirtualCupInterface(a_namespace), cup_state(Cup::CupState::IDLE), namespace_(a_namespace), fall_velocity_(0)
{
    // TODO Maken we de cup relatief aan de robotarm of de wereld?
    marker_.header.frame_id = "world";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = namespace_;
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::ARROW;

    marker_.pose.position.x = 0.38;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0.1;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 1.0;
    marker_.pose.orientation.w = 0.0;

    marker_.scale.x = CUP_HEIGHT;  // Length
    marker_.scale.y = CUP_RADIUS;  // Width
    marker_.scale.z = CUP_RADIUS;  // Height

    marker_.color.r = 0.95f;
    marker_.color.g = 0.95f;
    marker_.color.b = 0.75f;
    marker_.color.a = 1.0;
}

void Cup::loop()
{
    ros::Rate rate(FPS);
    while (ros::ok())
    {
        tf::StampedTransform world_2_gripper_left = getTransform("/world", "/gripper_left");
        tf::StampedTransform world_2_gripper_right = getTransform("/world", "/gripper_right");

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

                break;
            case Cup::CupState::IDLE:
                checkForCollision(world_2_gripper_left, world_2_gripper_right);
                if (canFall())
                {
                    // cup_state = Cup::CupState::FALLING;
                }
                break;
            case Cup::CupState::PUSHED:

                break;
            default:
                break;
        }
        rate.sleep();
        sendMarkerData(marker_);
    }
}

void Cup::checkForCollision(const tf::StampedTransform& gripper_left, const tf::StampedTransform& gripper_right)
{
    tf::Vector3 left_origin = gripper_left.getOrigin();
    tf::Vector3 cup_origin = tf::Vector3(marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z);
    tf::Vector3 delta = cup_origin - left_origin;
    tf::Vector3 forward = gripper_left.getRotation().getAxis() * tf::Vector3(1, 0, 0);
    tf::Vector3 up = gripper_left.getRotation().getAxis() * tf::Vector3(0, 0, 1);
    tf::Vector3 cross = forward.cross(delta);
    float dir = cross.dot(up);
    std::cout << dir << " forward: " << forward.x() << "," << forward.y() << "," << forward.z() << " up: " << up.x()
              << "," << up.y() << "," << up.z() << std::endl;
    //	Vector3 heading = target.position - transform.position;
    //	dirNum = AngleDir(transform.forward, heading, transform.up);
    //
    //
    //
    // float AngleDir(Vector3 fwd, Vector3 targetDir, Vector3 up) {
    //	Vector3 perp = Vector3.Cross(fwd, targetDir);
    //	float dir = Vector3.Dot(perp, up);
    //

    // std::cout << "Angle between left gripper and cup: " << left_origin.<< std::endl;
}

void Cup::applyGravity()
{
    if (cup_state == Cup::CupState::FALLING)
    {
        fall_velocity_ = fall_velocity_ + (1 / FPS) * GRAVITY;
        marker_.pose.position.z -= fall_velocity_ * (1 / FPS);
    }
}

bool Cup::canFall()
{
    return marker_.pose.position.z > CUP_HEIGHT / 2 /* && cup_state != Cup::CupState::GRABBED*/;
}

void calculateSpeed(const geometry_msgs::Point& old_pos, const geometry_msgs::Point& new_pos)
{
    double distance = std::sqrt((new_pos.x - old_pos.x) * (new_pos.x - old_pos.x) +
                                (new_pos.y - old_pos.y) * (new_pos.y - old_pos.y) +
                                (new_pos.z - old_pos.z) * (new_pos.z - old_pos.z));

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
