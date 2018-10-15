#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopOmnirob
{
public:
	TeleopOmnirob();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	int linear_x_, linear_y_, angular_;
	double a_scale_, l_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
};

TeleopOmnirob::TeleopOmnirob() :
				linear_x_(3),
				linear_y_(2),
				angular_(0),
				a_scale_(0.8),
				l_scale_(0.4)
{

	ros::NodeHandle nh_private("~");
	nh_private.param("axis_linear_x", linear_x_, linear_x_);
	nh_private.param("axis_linear_y", linear_y_, linear_y_);
	nh_private.param("axis_angular", angular_, angular_);
	nh_private.param("scale_angular", a_scale_, a_scale_);
	nh_private.param("scale_linear", l_scale_, l_scale_);
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("omnirob/cmd_vel", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopOmnirob::joyCallback, this);
}

void TeleopOmnirob::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist cmd;

	if ((fabs(joy->axes[linear_x_]) < 0.15 && fabs(joy->axes[linear_y_]) < 0.15 && fabs(joy->axes[angular_]) < 0.15) || joy->buttons[4] != 1)
	{
		cmd.linear.x = 0.0;
		cmd.linear.y = 0.0;
		cmd.linear.z = 0.0;
		cmd.angular.x = 0.0;
		cmd.angular.y = 0.0;
		cmd.angular.z = 0.0;
	}
	else
	{
		cmd.linear.x = l_scale_ * joy->axes[linear_x_];
		cmd.linear.y = l_scale_ * joy->axes[linear_y_];
		cmd.linear.z = 0.0;
		cmd.angular.x = 0.0;
		cmd.angular.y = 0.0;
		cmd.angular.z = a_scale_ * joy->axes[angular_];
	}

	vel_pub_.publish(cmd);
}

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "omnirob_joy");
	TeleopOmnirob teleop_omnirob;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::waitForShutdown();
}
