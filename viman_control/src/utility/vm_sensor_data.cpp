#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

#include "imumath_brief.h"

// If set displays quaternion read by IMU
#define SHOW_QUAT 0


Quaternion q;
Cardan angles;
double height;

void HeightCallbck(const geometry_msgs::PointStamped& height_){
	height = height_.point.z;
}

void ImuCallbck(const sensor_msgs::Imu& imu_){
	q.w = imu_.orientation.w;
	q.x = imu_.orientation.x;
	q.y = imu_.orientation.y;
	q.z = imu_.orientation.z;
	
	angles = get_cardan_angles(q);
}

int main(int argc, char **argv){
	ros::init(argc,argv,"vm_sensor_data");
	ros::NodeHandle node;
	
	ros::Subscriber height_subs_ = node.subscribe("/viman/height",500,HeightCallbck);
	ros::Subscriber imu_subs_ = node.subscribe("/viman/imu",500,ImuCallbck);
	
	if( height_subs_.getTopic() != "")
        ROS_INFO("found altimeter height topic");
    else
        ROS_WARN("cannot find altimeter height topic!");
    if( imu_subs_.getTopic() != "")
        ROS_INFO("found imu topic");
    else
        ROS_WARN("cannot find imu topic!");
	
	ros::Rate rate(20);
	
	while(ros::ok()){
		std::cout << "Time (in secs): " << ros::Time::now().toSec() << std::endl
			 << "Height (in m): "  << height << std::endl
			 << "Yaw (deg): "      <<  angles.yaw  << std::endl
			 << "Pitch (deg): "    << angles.pitch << std::endl
			 << "Roll (deg): "     <<  angles.roll << std::endl;
		#if SHOW_QUAT
		std::cout << "qx: "  << q.x << std::endl
				  << "qy: "  << q.y << std::endl
				  << "qz: "  << q.z << std::endl
				  << "qw: "  << q.w << std::endl;
		#endif
		std::cout << "---------" << std::endl << std::endl;
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}
