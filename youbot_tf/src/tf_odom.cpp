#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class youbot_odom_broadcaster {
	private:
		geometry_msgs::TransformStamped odom_trans;
		tf::TransformBroadcaster 		odom_broadcaster;
		ros::Subscriber 				baseOdometrySubscriber;
		ros::NodeHandle 				n;

	public:
		youbot_odom_broadcaster();
		virtual ~youbot_odom_broadcaster() {}

		void baseOdometryCallback(const nav_msgs::Odometry& odomertry);
};

youbot_odom_broadcaster::youbot_odom_broadcaster() {
	baseOdometrySubscriber	 	= n.subscribe("/odom", 1, &youbot_odom_broadcaster::baseOdometryCallback, this);

	odom_trans.header.frame_id 	= "/odom";
    odom_trans.child_frame_id 	= "/base_footprint";
}


void youbot_odom_broadcaster::baseOdometryCallback(const nav_msgs::Odometry& odometry) {
    odom_trans.header.stamp 			= odometry.header.stamp;

    odom_trans.transform.translation.x 	= odometry.pose.pose.position.x;
    odom_trans.transform.translation.y 	= odometry.pose.pose.position.y;
    odom_trans.transform.translation.z 	= 0.0;

    odom_trans.transform.rotation.x 	= odometry.pose.pose.orientation.x;
	odom_trans.transform.rotation.y 	= odometry.pose.pose.orientation.y;
	odom_trans.transform.rotation.z 	= odometry.pose.pose.orientation.z;
	odom_trans.transform.rotation.w 	= odometry.pose.pose.orientation.w;

	odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_tf_broadcaster");

	youbot_odom_broadcaster youbot_odom_tf;

	ros::spin();
}

