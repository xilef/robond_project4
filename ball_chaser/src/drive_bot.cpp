#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBotService
{
public:
	DriveBotService()
	{
		ROS_INFO_STREAM("Setup drive_bot service class.");

		m_pubMotorCommand = m_hNode.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

		m_serv = m_hNode.advertiseService("/ball_chaser/command_robot", &DriveBotService::driveRequestHandler, this);

		ROS_INFO("Ready to send movement commands");
	}

private:
	bool driveRequestHandler(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
	{
		geometry_msgs::Twist twMotorCommand;

		twMotorCommand.linear.x = req.linear_x;
		twMotorCommand.angular.z = req.angular_z;

		m_pubMotorCommand.publish(twMotorCommand);

		res.msg_feedback = "Movement set - linear.x: " + std::to_string(req.linear_x) + " angular.z: " + std::to_string(req.angular_z) + "!";

		//return true;
		ROS_INFO_STREAM(res.msg_feedback);
	}

	ros::NodeHandle		m_hNode;
	ros::Publisher		m_pubMotorCommand;
	ros::ServiceServer	m_serv;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_bot");

	DriveBotService obj;

	ros::spin();

	return 0;
}
