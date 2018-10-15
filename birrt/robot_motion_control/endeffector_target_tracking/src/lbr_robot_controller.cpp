/*
 *lbr_robot_controller.cpp
 *
 *  Created on: May 02, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <kuka_motion_control/control_laws.h>



using namespace std;

//Desired end-effector pose
double des_ee_pos_x = 0.0;
double des_ee_pos_y = 0.0;
double des_ee_pos_z = 0.0;

void new_target_pose(const visualization_msgs::Marker::ConstPtr& new_pose)
{
  //cout<<"New ee position: "<<new_pose->pose.position.x<<" "<<new_pose->pose.position.y<<" "<<new_pose->pose.position.z<<endl;
  des_ee_pos_x = new_pose->pose.position.x;
  des_ee_pos_y = new_pose->pose.position.y;
  des_ee_pos_z = new_pose->pose.position.z;

}



int main(int argc, char** argv)
{
	//Init Node
	ros::init(argc, argv, "lbr_robot_controller_node");

    //Node Handle
    ros::NodeHandle nh;

    //Subscribe to target point topic
    ros::Subscriber ee_pose_sub = nh.subscribe("ee_target", 1000, new_target_pose);

    //Create controller for Kuka LBR
    kuka_motion_controller::RobotController lbr_controller("robot_description", "kuka_complete_arm");

    //Set Motion Strategy (0 = all joints weighted equal)
    lbr_controller.set_motion_strategy(0);

    //Set start config
    vector<double> start_conf(7);
    start_conf[0] = 0.05;
    start_conf[1] = 0.05;
    start_conf[2] = 0.05;
    start_conf[3] = 0.05;
    start_conf[4] = 0.05;
    start_conf[5] = 0.05;
    start_conf[6] = 0.0;
    lbr_controller.setStartConf(start_conf);

    //Set constraints for controller
    vector<int> constraint_vec(6);
    constraint_vec[0] = 1;
    constraint_vec[1] = 1;
    constraint_vec[2] = 1;
    constraint_vec[3] = 0;
    constraint_vec[4] = 0;
    constraint_vec[5] = 0;
    //Permitted displacement for ee coordinates w.r.t desired target frame
    vector<pair<double,double> > target_coordinate_dev(6);
    target_coordinate_dev[0].first = 0.0;    //negative X deviation [m]
    target_coordinate_dev[0].second = 0.0;    //positive X deviation
    target_coordinate_dev[1].first = -0.0;    //negative Y deviation
    target_coordinate_dev[1].second = 0.0;    //positive Y deviation
    target_coordinate_dev[2].first = -0.0;    //negative Z deviation
    target_coordinate_dev[2].second = 0.0;    //positive Z deviation
    target_coordinate_dev[3].first = -0.0;     //negative Xrot deviation [rad]
    target_coordinate_dev[3].second = 0.0;     //positive Xrot deviation
    target_coordinate_dev[4].first = -0.0;     //negative Yrot deviation
    target_coordinate_dev[4].second = 0.0;     //positive Yrot deviation
    target_coordinate_dev[5].first = -0.0;     //negative Zrot deviation
    target_coordinate_dev[5].second = 0.0;     //positive Zrot deviation
    lbr_controller.setVariableConstraints(constraint_vec,target_coordinate_dev);

    //Desired ee pose as vector
    vector<double> start_ee_pose(6);
    start_ee_pose[0] = 0.0; //X
    start_ee_pose[1] = 0.0; //Y
    start_ee_pose[2] = 0.0; //Z
    start_ee_pose[3] = 0.0; //rotX
    start_ee_pose[4] = 0.0; //rotY
    start_ee_pose[5] = 0.0; //rotZ
    //Convert orientation to quaternion
    vector<double> start_ee_pose_quat = lbr_controller.convertEulertoQuat(start_ee_pose[3],start_ee_pose[4],start_ee_pose[5]);

    //Init desired endeffector pose
    vector<double> des_ee_pose(7);
    //Keep start orientation
    des_ee_pose[3] = start_ee_pose_quat[0];
    des_ee_pose[4] = start_ee_pose_quat[1];
    des_ee_pose[5] = start_ee_pose_quat[2];
    des_ee_pose[6] = start_ee_pose_quat[3];

    //Run control loop
    while (ros::ok())
    {
      //Get current desired ee pose
      des_ee_pose[0] = des_ee_pos_x;
      des_ee_pose[1] = des_ee_pos_y;
      des_ee_pose[2] = des_ee_pos_z;

      //Set the desired ee pose in the controller
      lbr_controller.set_EE_goal_pose(des_ee_pose);

      //Generate motion towards des_ee_pose
      lbr_controller.run_J_vdls_Target_Tracking(true);

      //Spin
      ros::spinOnce();
    }

	return 0;
}
