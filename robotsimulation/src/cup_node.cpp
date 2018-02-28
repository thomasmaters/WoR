#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>

#include <thread>

#define CUP_HEIGHT .1
#define CUP_RADIUS 0.05
#define FPS 100
#define PICKUP_SENSITIVETY 0.04

bool g_picked_up = false;
visualization_msgs::Marker g_client_marker;

tf::StampedTransform getTransform(const std::string& frame, const std::string& frame_2)
{
	tf::TransformListener listener;
	tf::StampedTransform stamped_transform;

	listener.waitForTransform(frame, frame_2, ros::Time(0), ros::Duration(0.1));
	listener.lookupTransform(frame, frame_2, ros::Time(0), stamped_transform);
	return stamped_transform;
}

void initGlobalMarker()
{
	g_client_marker.header.frame_id = "base_link";
	g_client_marker.header.stamp = ros::Time::now();
	g_client_marker.ns = "cup";
	g_client_marker.id = 0;
	g_client_marker.type = visualization_msgs::Marker::CYLINDER;

	g_client_marker.pose.position.x = 0.38;
	g_client_marker.pose.position.y = 0;
	g_client_marker.pose.position.z = 0.5;
	g_client_marker.pose.orientation.x = 0.0;
	g_client_marker.pose.orientation.y = 0.0;
	g_client_marker.pose.orientation.z = 0.0;
	g_client_marker.pose.orientation.w = 1.0;

	g_client_marker.scale.x = CUP_RADIUS;
	g_client_marker.scale.y = CUP_RADIUS;
	g_client_marker.scale.z = CUP_HEIGHT;

	g_client_marker.color.r = 0.95f;
	g_client_marker.color.g = 0.95f;
	g_client_marker.color.b = 0.75f;
	g_client_marker.color.a = 1.0;
}

//Inspiration from:
//http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
int main(int argc, char** argv)
{
	ros::init(argc, argv, "beker_node");

	ros::start();

	initGlobalMarker();

	ros::NodeHandle node_handle;
	ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "cup" , 0 );
	tf::TransformBroadcaster transform_broadcaster;
	ros::Rate rate(FPS);
	tf::Pose marker_pose;
	tf::Transform transform;

	bool l_picked_up = false;

	while (ros::ok())
	{
		try
		{
			tf::StampedTransform stamped_transform = getTransform("/base_link", "/gripper_left");
			tf::Vector3 distance = stamped_transform.getOrigin();
			std::cout << distance.getX()- g_client_marker.pose.position.x << " " <<
					distance.getY() - g_client_marker.pose.position.y << " " <<
					distance.getZ() - g_client_marker.pose.position.z << std::endl;

			l_picked_up = ((distance.getX() - g_client_marker.pose.position.x )< CUP_HEIGHT / 2 &&
					(distance.getY() - g_client_marker.pose.position.y ) < PICKUP_SENSITIVETY &&
					(distance.getZ() - g_client_marker.pose.position.z + 0.98 /FPS ) < PICKUP_SENSITIVETY);

			if (g_picked_up || l_picked_up)
			{
				std::cout << "picked up" << std::endl;
				tf::StampedTransform tran = getTransform("/base_link", "/grip_point");

				g_client_marker.pose.position.x = tran.getOrigin().getX();
				g_client_marker.pose.position.y = tran.getOrigin().getY();
				g_client_marker.pose.position.z = tran.getOrigin().getZ();
//				g_client_marker.header.frame_id = "grip_point";
//				g_client_marker.pose.position.x = 0;
//				g_client_marker.pose.position.y = 0;
//				g_client_marker.pose.position.z = 0;
			}

			g_picked_up = l_picked_up;
		}
		catch(std::exception& ex)
		{
			std::cout << ex.what() << std::endl;
		}
		if (g_client_marker.pose.position.z > CUP_HEIGHT / 2)
		{
			std::cout << "falling" << std::endl;
			g_client_marker.pose.position.z -= 0.98 / FPS;
		}
		g_client_marker.header.stamp = ros::Time::now();
		vis_pub.publish(g_client_marker);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
