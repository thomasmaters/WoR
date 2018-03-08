/*
 * Cup.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Thomas Maters
 */

#include "cup.h"

Cup::Cup(const std::string& a_namespace) : cup_state(Cup::CupState::IDLE), namespace_(a_namespace)
{
    marker_.header.frame_id = "base_link";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = namespace_;
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::CYLINDER;

    marker_.pose.position.x = 0.38;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0.5;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    marker_.scale.x = CUP_RADIUS;
    marker_.scale.y = CUP_RADIUS;
    marker_.scale.z = CUP_HEIGHT;

    marker_.color.r = 0.95f;
    marker_.color.g = 0.95f;
    marker_.color.b = 0.75f;
    marker_.color.a = 1.0;
}

Cup::~Cup()
{
}
