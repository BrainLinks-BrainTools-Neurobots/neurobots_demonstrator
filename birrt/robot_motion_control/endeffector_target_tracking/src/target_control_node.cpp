/*
 *target_control_node.cpp
 *
 *  Created on: May 02, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include<stdio.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO


using namespace std;


int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char** argv)
{
	//Init Node
	ros::init(argc, argv, "target_control_node");


	//Node Handle
	ros::NodeHandle nh;


    //Target for Endeffector
    visualization_msgs::Marker ee_target;

    // ---------- Publisher for End-effector Target Point ----------

    //Set up properties for add nodes marker
    ee_target.header.frame_id = "lbr_0_link";
    ee_target.ns = "target";
    ee_target.action = visualization_msgs::Marker::ADD;
    //Set Marker Type
    ee_target.type = visualization_msgs::Marker::SPHERE;
    //Nodes markers
    ee_target.scale.x = 0.1;
    ee_target.scale.y = 0.1;
    ee_target.scale.z = 0.1;
    ee_target.pose.orientation.w = 1.0;
    //Set colour for ee target point
    ee_target.color.r = 1.0;
    ee_target.color.g = 1.0;
    ee_target.color.b = 0.0;
    ee_target.color.a = 1.0;

    //Initial End-effector target position
    ee_target.pose.position.x = 0.0;
    ee_target.pose.position.y = 0.0;
    ee_target.pose.position.z = 0.0;

    //Tree Nodes Visualization Topic for START Tree
    ros::Publisher ee_target_pub = nh.advertise<visualization_msgs::Marker>("ee_target", 100);

    //Publish initial target
    ee_target_pub.publish(ee_target);


    // ---------- Subscriber for Keyboard Input ----------
    //ros::Subscriber keyboard_sub = nh.subscribe<>;


    while (ros::ok())
    {
      int c = getch();   // call your non-blocking input function
      if (c == 'w')
      {
          //Move point up
          ee_target.pose.position.z += 0.01;
          //Publish new ee target position
          ee_target_pub.publish(ee_target);
      }
      else if (c == 'y')
      {
          //Move point down
          ee_target.pose.position.z -= 0.01;
          //Publish new ee target position
          ee_target_pub.publish(ee_target);
      }
      else if (c == 'a')
      {
          //Move point left
          ee_target.pose.position.y += 0.01;
          //Publish new ee target position
          ee_target_pub.publish(ee_target);
      }
      else if (c == 's')
      {
          //Move point right
          ee_target.pose.position.y -= 0.01;
          //Publish new ee target position
          ee_target_pub.publish(ee_target);
      }
      else if (c == 'd')
      {
          //Move point forwards
          ee_target.pose.position.x += 0.01;
          //Publish new ee target position
          ee_target_pub.publish(ee_target);
      }
      else if (c == 'f')
      {
          //Move point backwards
          ee_target.pose.position.x -= 0.01;
          //Publish new ee target position
          ee_target_pub.publish(ee_target);
      }
      else
      {
          cout<<"No valid command!"<<endl;
      }

      ros::spinOnce();
    }

	
	
    //ros::spin();

	return 0;
}
