/*
 *camera_calibration.cpp
 *
 *  Created on: August 08, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <iostream>
#include <fstream>



using namespace std;

void write_extrinsic_params(char *param_file, tf::Transform tf_map_to_camera)
{   
    //+++++++++ Write extrinsic camera parameters to file ++++++++

    //Remove the planner statistics file
    if( remove( param_file ) != 0 )
    {
        cout<< "Error deleting file (extrinsic camera params file)" <<endl;
    }

    //File to store extrinsic camera parameters
    ofstream ext_cam_params_file;
    //Open file
    ext_cam_params_file.open(param_file);

    // file couldn't be opened
    if( !ext_cam_params_file)
    {
        cerr << "Error: extrinsic camera params file could not be opened" << endl;
        exit(1);
    }


    //------------------ Write data
    //Get rotation matrix and translation
    tf::Matrix3x3 rot_matrix = tf_map_to_camera.getBasis();
    tf::Vector3 translation = tf_map_to_camera.getOrigin();

    //Write Transform as 4x4 matrix
    ext_cam_params_file <<rot_matrix.getRow(0).x()<<" "<<rot_matrix.getRow(0).y()<<" "<<rot_matrix.getRow(0).z()<<" "<<translation.x()<<endl \
                        <<rot_matrix.getRow(1).x()<<" "<<rot_matrix.getRow(1).y()<<" "<<rot_matrix.getRow(1).z()<<" "<<translation.y()<<endl \
                        <<rot_matrix.getRow(2).x()<<" "<<rot_matrix.getRow(2).y()<<" "<<rot_matrix.getRow(2).z()<<" "<<translation.z()<<endl \
                        <<0.0<<" "<<0.0<<" "<<0.0<<" "<<1.0<<endl;


//    ext_cam_params_file << "x" <<" "<< "y" <<" "<< "z" <<" "<< "qx" <<" "<< "qy" <<" "<< "qz" <<" "<< "qw" << endl \
//                        << tf_map_to_camera.getOrigin().x() \
//                        <<" "<< tf_map_to_camera.getOrigin().y() \
//                        <<" "<< tf_map_to_camera.getOrigin().z() \
//                        <<" "<< tf_map_to_camera.getRotation().getX() \
//                        <<" "<< tf_map_to_camera.getRotation().getY() \
//                        <<" "<< tf_map_to_camera.getRotation().getZ() \
//                        <<" "<< tf_map_to_camera.getRotation().getW() << endl;

    //Close the file
    ext_cam_params_file.close();
    
    ROS_INFO("Extrinsic camera parameters written to file");

}

bool getMapToMarkerTransform(tf::StampedTransform& tf_map_to_marker)
{
    //TODO's;
    // -> Get transform map to base_link
     // -> Get transform base_link to marker (needs to be set manually)
    // -> Compute transform map to marker and return it

    //Get current pose of robot in the map frame
    tf::TransformListener listener;
    tf::StampedTransform transform_map_to_base;
    try {
        listener.waitForTransform("/map", "omnirob_lbr/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map", "omnirob_lbr/base_link", ros::Time(0), transform_map_to_base);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        //No transform available
        ROS_INFO("Transform /map to /base_link not available!");
        return false;
    }
    
    //Manually define tranformation between robot base_link and marker frame
    double base_to_maker_x = -0.542;
    double base_to_maker_y = 0.0;
    double base_to_maker_z = 0.25;
    double base_to_maker_roll = 1.57079;
    double base_to_maker_pitch = 0.0;
    double base_to_maker_yaw = -1.57079;

    tf::StampedTransform transform_base_to_marker;
    transform_base_to_marker.setOrigin(tf::Vector3(base_to_maker_x,base_to_maker_y, base_to_maker_z));
    transform_base_to_marker.setRotation(tf::createQuaternionFromRPY(base_to_maker_roll,base_to_maker_pitch,base_to_maker_yaw));
    //transform_base_to_marker.setRotation(tf::createQuaternionFromYaw(goal_theta_map));


    //Transform map to marker frame
    tf_map_to_marker.mult(transform_map_to_base,transform_base_to_marker);

    //map to base_link transform available
    return true;
}

bool getCameraToMarkerTransform(tf::StampedTransform& tf_camera_to_marker, string camera_name)
{
    //TODO's;
    // -> Get transform optical frame to marker frame and return it

    cout<<"/" + camera_name + "_camera_link"<< " to "<<camera_name + "_camera/simtrack/ros_hydro"<<endl;

    //Get current pose of robot in the map frame
    tf::TransformListener listener;
    try {
        listener.waitForTransform("/" + camera_name + "_camera_link", camera_name + "_camera/simtrack/ros_hydro", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/" + camera_name + "_camera_link", camera_name + "_camera/simtrack/ros_hydro", ros::Time(0), tf_camera_to_marker);
        return true;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        //No transform available
        ROS_INFO("Transform /camera_link to /ros_hydro not available!");
        return false;
    }
}

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "camera_calibration_node");

    //Node Handle
    ros::NodeHandle nh;
    
    //Get File name and package path
    string param_file_name;
    if(argc > 1){
    	stringstream s(argv[1]);
    	s >> param_file_name;
    }
    
    std::string package_path = ros::package::getPath("neurobots_calibration");
    //Set path to extrinsic camera params file
    string file_path = package_path + "/calibration_files/map_to_"+ param_file_name + "_camera.mat";

    //Set path to the file that will store the start and goal config for each scenario
    char* file_path_camera_params;
    file_path_camera_params = new char[file_path.size() + 1];
    copy(file_path.begin(), file_path.end(), file_path_camera_params);
    file_path_camera_params[file_path.size()] = '\0'; // don't forget the terminating 0
    cout<<file_path_camera_params<<endl;
    

    static tf::TransformBroadcaster br;

    //TF Transforms
    tf::StampedTransform map_to_marker;
    tf::StampedTransform camera_to_marker;
    
    //Final Transform
    tf::StampedTransform map_to_camera;
    tf::Transform transform_pub;

    //Transform available flags
    bool tf_map_to_marker_ok = false;
    bool tf_camera_to_marker_ok = false;

    while(ros::ok()){
        //Get map to marker transform
        tf_map_to_marker_ok= getMapToMarkerTransform(map_to_marker);

        //Get camera to marker transform
        tf_camera_to_marker_ok = getCameraToMarkerTransform(camera_to_marker,param_file_name);

        //If both transforms are available
        if(tf_map_to_marker_ok && tf_camera_to_marker_ok)
        {
            //Compute map to camera transform
            map_to_camera.mult(map_to_marker,camera_to_marker.inverse());


            //cout<<"Marker in map frame"<<endl;
            //cout<<map_to_marker.getOrigin().x()<<endl;
            //cout<<map_to_marker.getOrigin().y()<<endl;
            //cout<<map_to_marker.getOrigin().z()<<endl;
            //cout<<map_to_marker.getRotation().getAxis().z()<<endl;

            //cout<<"Marker in camera frame"<<endl;
            //cout<<camera_to_marker.getOrigin().x()<<endl;
            //cout<<camera_to_marker.getOrigin().y()<<endl;
            //cout<<camera_to_marker.getOrigin().z()<<endl;

            //cout<<"Camera in map frame"<<endl;
            //cout<<map_to_camera.getOrigin().x()<<endl;
            //cout<<map_to_camera.getOrigin().y()<<endl;
            //cout<<map_to_camera.getOrigin().z()<<endl;

            transform_pub.setOrigin(map_to_camera.getOrigin());
            tf::Quaternion q;
            q.setX(map_to_camera.getRotation().x());
            q.setY(map_to_camera.getRotation().y());
            q.setZ(map_to_camera.getRotation().z());
            q.setW(map_to_camera.getRotation().w());
            transform_pub.setRotation(q);
            //Send map to camera transform
            br.sendTransform(tf::StampedTransform(transform_pub, ros::Time::now(), "map", param_file_name + "_camera_link"));
            
            //Write extrinsic parameters to file
    		write_extrinsic_params(file_path_camera_params,transform_pub);

        }
	
        ros::spinOnce();
    }
   
    ros::shutdown();

    return 0;
}




