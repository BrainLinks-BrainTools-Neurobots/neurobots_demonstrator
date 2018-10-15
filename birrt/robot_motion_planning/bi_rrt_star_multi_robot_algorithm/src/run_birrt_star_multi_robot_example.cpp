/*
 *run_birrt_star_multi_robot_example.cpp
 *
 *  Created on: May 19, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <bi_rrt_star_multi_robot_algorithm/birrt_star_multi_robot.h>
#include <planning_world_builder/planning_world_builder.h>

using namespace std;

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "run_birrt_star_multi_robot_node");

    //Node Handle
    ros::NodeHandle nh;


    //TODO
    // - Read planning group from terminal input
    string planning_group_rob_1 = "omnirob_lbr_sdh";
    string planning_group_rob_2 = "robotino_robot";

    //Get names of robot_description parameters
    string robot_description_first_robot;
    string robot_description_second_robot;
    nh.param("robot_description_first_robot", robot_description_first_robot, std::string("omnirob_robot_description"));
    nh.param("robot_description_second_robot", robot_description_second_robot, std::string("robotino_robot_description"));

    //Get namespaces of robots
    string ns_first_robot;
    string ns_second_robot;
    nh.param("ns_first_robot", ns_first_robot, std::string("omnirob_group/"));
    nh.param("ns_second_robot", ns_second_robot, std::string("robotino_group/"));

    // -------------------- Planning World Setup ----------------------------
    //Load Planning World
    planning_world::PlanningWorldBuilder world_builder_rob_1(robot_description_first_robot, planning_group_rob_1, ns_first_robot);
    planning_world::PlanningWorldBuilder world_builder_rob_2(robot_description_second_robot, planning_group_rob_2, ns_second_robot);
    //Enter Environment Borders
    vector<double> env_size_x(2);
    env_size_x[0] = -6.0;
    env_size_x[1] = 6.0;
    vector<double> env_size_y(2);
    env_size_y[0] = -6.0;
    env_size_y[1] = 6.0;
    double env_size_z = 0.6;
    world_builder_rob_1.insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);
    world_builder_rob_2.insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);


    //Insert first block
    vector<double> pos(3);
    pos[0] = -1.0;
    pos[1] = 3.0;
    pos[2] = 0.0;
    vector<double> dim(3);
    dim[0] = 1.0;
    dim[1] = 1.0;
    dim[2] = 1.0;
    world_builder_rob_1.insertWall(pos,dim);
    world_builder_rob_2.insertWall(pos,dim);

    //Insert second block
    pos[1] = -3.0;
    world_builder_rob_1.insertWall(pos,dim);
    world_builder_rob_2.insertWall(pos,dim);

    //Insert beam
    pos[1] = 0.0;
    pos[2] = 0.8;
    dim[1] = 5.0;
    dim[2] = 0.5;
    world_builder_rob_1.insertWall(pos,dim);
    world_builder_rob_2.insertWall(pos,dim);



    // -------------------- Planner Setup ----------------------------

    //birrt_star_motion_planning::BiRRTstarPlanner planner("kuka_complete_arm");
    birrt_star_multi_robot::BiRRTstarMultiRobotPlanner planner(planning_group_rob_1,planning_group_rob_2);

    //Set planning scene
    planner.setPlanningSceneInfo(world_builder_rob_1);

 
    //Set default values
    double FLAG_ITERATIONS_OR_TIME = 0;
    double MAX_ITERATIONS_TIME = 200;
    bool RVIZ_SHOW_TREE = 1;

	//Set flag indicating whether maximum iterations or time for planner is used
    if(argc > 1) {
      stringstream s(argv[1]);
      s >> FLAG_ITERATIONS_OR_TIME;
    }
    //Set maximum iterations or time for planner
    if(argc > 2) {
      stringstream s(argv[2]);
      s >> MAX_ITERATIONS_TIME;
    }
    //Activate/Deactivate tree visualization in RVIZ
    if(argc > 3) {
      stringstream s(argv[3]);
      s >> RVIZ_SHOW_TREE;
    }
    
   


//    //Set Start configuration and endeffector goal pose
//    vector<double> start_conf(7);
//    start_conf[0] = 0.74;
//    start_conf[1] =-0.61;
//    start_conf[2] =-0.74;
//    start_conf[3] =-0.80;
//    start_conf[4] =-1.45;
//    start_conf[5] =0.77;
//    start_conf[6] =0.3;


//    vector<double> ee_goal_pose(7);
//    ee_goal_pose[0] = -0.108012;
//    ee_goal_pose[1] = -0.59835;  //0.59835;
//    ee_goal_pose[2] = 0.215184;
//    ee_goal_pose[3] = 1000.0;
//    ee_goal_pose[4] = 1000.0;
//    ee_goal_pose[5] = 1000.0;
//    ee_goal_pose[6] = 1000.0;


//    //Set EE Start endeffector pose and constraint variables
//    vector<double> start_ee_pose(6);
//    start_ee_pose[0] = -0.6; //X
//    start_ee_pose[1] = 0.4;  //Y
//    start_ee_pose[2] = 0.3;  //Z
//    start_ee_pose[3] = 0.0;    //RotX
//    start_ee_pose[4] = -1.57;    //RotY
//    start_ee_pose[5] = 0.0;    //RotZ
//    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
//    constraint_vec_start_pose[0] = 1; //X
//    constraint_vec_start_pose[1] = 1; //Y
//    constraint_vec_start_pose[2] = 1; //Z
//    constraint_vec_start_pose[3] = 1; //RotX
//    constraint_vec_start_pose[4] = 1; //RotY
//    constraint_vec_start_pose[5] = 1; //RotZ


//    //Set EE Goal endeffector pose and constraint variables
//    vector<double> ee_goal_pose(6);
//    ee_goal_pose[0] = -0.1; //X
//    ee_goal_pose[1] = 0.4;  //Y
//    ee_goal_pose[2] = 0.3;  //Z
//    ee_goal_pose[3] = 0.0;    //RotX
//    ee_goal_pose[4] = -1.57;    //RotY
//    ee_goal_pose[5] = 0.0;    //RotZ
//    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
//    constraint_vec_goal_pose[0] = 1; //X
//    constraint_vec_goal_pose[1] = 1; //Y
//    constraint_vec_goal_pose[2] = 1; //Z
//    constraint_vec_goal_pose[3] = 1; //RotX
//    constraint_vec_goal_pose[4] = 1; //RotY
//    constraint_vec_goal_pose[5] = 1; //RotZ



    //Global Frame (top view)
    //       ^ Y
    //       |
    //       | f
    //       |Z
    //   f   X-----> X
    //
    //         f

    //Set EE Start endeffector pose and constraint variables
    vector<double> start_ee_pose(6);
    start_ee_pose[0] = 3.6;    //X
    start_ee_pose[1] = 0.5;    //Y
    start_ee_pose[2] = 0.9;    //Z
    start_ee_pose[3] = 1.57;   //RotX
    start_ee_pose[4] = 0.0;    //RotY
    start_ee_pose[5] = 1.57;   //RotZ
    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
    constraint_vec_start_pose[0] = 1; //X
    constraint_vec_start_pose[1] = 1; //Y
    constraint_vec_start_pose[2] = 1; //Z
    constraint_vec_start_pose[3] = 1; //RotX
    constraint_vec_start_pose[4] = 1; //RotY
    constraint_vec_start_pose[5] = 1; //RotZ


    //Set EE Goal endeffector pose and constraint variables
    vector<double> ee_goal_pose(6);
    ee_goal_pose[0] = -4.3;       //X
    ee_goal_pose[1] = 0.5;        //Y
    ee_goal_pose[2] = 0.6;        //Z
    ee_goal_pose[3] = 1.57;       //RotX
    ee_goal_pose[4] = 0.0;        //RotY
    ee_goal_pose[5] = 1.57;       //RotZ
    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
    constraint_vec_goal_pose[0] = 1; //X
    constraint_vec_goal_pose[1] = 1; //Y
    constraint_vec_goal_pose[2] = 1; //Z
    constraint_vec_goal_pose[3] = 1; //RotX
    constraint_vec_goal_pose[4] = 1; //RotY
    constraint_vec_goal_pose[5] = 1; //RotZ

    //Permitted displacement for ee coordinates w.r.t desired target frame
    vector<pair<double,double> > target_coordinate_dev(6);
    target_coordinate_dev[0].first = -0.005;    //negative X deviation [m]
    target_coordinate_dev[0].second = 0.005;    //positive X deviation
    target_coordinate_dev[1].first = -0.005;    //negative Y deviation
    target_coordinate_dev[1].second = 0.005;    //positive Y deviation
    target_coordinate_dev[2].first = -0.005;    //negative Z deviation
    target_coordinate_dev[2].second = 0.005;    //positive Z deviation
    target_coordinate_dev[3].first = -0.05;     //negative Xrot deviation [rad]
    target_coordinate_dev[3].second = 0.05;     //positive Xrot deviation
    target_coordinate_dev[4].first = -0.05;     //negative Yrot deviation
    target_coordinate_dev[4].second = 0.05;     //positive Yrot deviation
    target_coordinate_dev[5].first = -0.05;     //negative Zrot deviation
    target_coordinate_dev[5].second = 0.05;     //positive Zrot deviation


     cout<<"Starting planner init ..."<<endl;

    //Initialize planner (with start and ee goal pose)
    planner.init_planner(start_ee_pose, constraint_vec_start_pose, ee_goal_pose, constraint_vec_goal_pose, target_coordinate_dev, 1);


    cout<<"Init completed successfully"<<endl;


    //Set constraint parameters / permitted axes for displacement (x,y,z,roll,pitch,yaw) relative to start ee pose during planning
    //  1 -> constraint
    //  0 -> unconstraint
    vector<int> constraint_vector(6);
    constraint_vector[0] = 1.0; //X translation
    constraint_vector[1] = 0.0; //Y translation
    constraint_vector[2] = 0.0; //Z translation
    constraint_vector[3] = 0.0; //X rotation
    constraint_vector[4] = 1.0; //Y rotation
    constraint_vector[5] = 1.0; //Z rotation
    //Permitted displacement for ee coordinates w.r.t task frame
    vector<pair<double,double> > permitted_coordinate_dev(6);
    permitted_coordinate_dev[0].first = 0.0;    //negative X deviation [m]
    permitted_coordinate_dev[0].second = 0.0;   //positive X deviation
    permitted_coordinate_dev[1].first = 0.0;    //negative Y deviation
    permitted_coordinate_dev[1].second = 0.0;   //positive Y deviation
    permitted_coordinate_dev[2].first = 0.0;    //negative Z deviation
    permitted_coordinate_dev[2].second = 0.0;   //positive Z deviation
    permitted_coordinate_dev[3].first = 0.0;    //negative Xrot deviation [rad]
    permitted_coordinate_dev[3].second = 0.0;   //positive Xrot deviation
    permitted_coordinate_dev[4].first = -0.52;    //negative Yrot deviation
    permitted_coordinate_dev[4].second = 0.0;   //positive Yrot deviation
    permitted_coordinate_dev[5].first = 0.0;    //negative Zrot deviation
    permitted_coordinate_dev[5].second = 0.0;   //positive Zrot deviation
    //Activate the constraint
    // -> Syntax: planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, bool task_pos_global, bool task_orient_global);
    // bool task_pos_global -> indicates whether task frame position is expressed w.r.t near node ee pos or always w.r.t start frame ee pos
    // bool task_orient_global -> indicates whether task frame orientation is expressed w.r.t near node ee orientation or always w.r.t start frame ee orientation
    //planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, true, true);


    //Set edge cost variable weights (to apply motion preferences)
    // -> For first robot (robot 1)
    vector<double> edge_cost_weights_rob_1(10);
    edge_cost_weights_rob_1[0] = 1.0; //base_x
    edge_cost_weights_rob_1[1] = 1.0; //base_y
    edge_cost_weights_rob_1[2] = 1.0; //base_theta
    edge_cost_weights_rob_1[3] = 1.0; //manipulator joint 1
    edge_cost_weights_rob_1[4] = 1.0; //manipulator joint 2
    edge_cost_weights_rob_1[5] = 1.0; //manipulator joint 3
    edge_cost_weights_rob_1[6] = 1.0; //manipulator joint 4
    edge_cost_weights_rob_1[7] = 1.0; //manipulator joint 5
    edge_cost_weights_rob_1[8] = 1.0; //manipulator joint 6
    edge_cost_weights_rob_1[9] = 1.0; //manipulator joint 7
    planner.setEdgeCostWeights(edge_cost_weights_rob_1, 1);
    // -> For second robot (robot 2)
    vector<double> edge_cost_weights_rob_2(8);
    edge_cost_weights_rob_2[0] = 1.0; //base_x
    edge_cost_weights_rob_2[1] = 1.0; //base_y
    edge_cost_weights_rob_2[2] = 1.0; //base_theta
    edge_cost_weights_rob_2[3] = 1.0; //manipulator joint 1
    edge_cost_weights_rob_2[4] = 1.0; //manipulator joint 2
    edge_cost_weights_rob_2[5] = 1.0; //manipulator joint 3
    edge_cost_weights_rob_2[6] = 1.0; //manipulator joint 4
    edge_cost_weights_rob_2[7] = 1.0; //manipulator joint 5
    planner.setEdgeCostWeights(edge_cost_weights_rob_2, 2);



    // -------------------- Motion Planning Execution ----------------------------

    //Planner run index
    int planner_run_number = 0;

    //Run planner
    //planner.run_planner(start_conf, ee_goal_pose, SEARCH_SPACE, MAX_ITERATIONS, RVIZ_SHOW_TREE, ITERATION_SLEEP_TIME);
    planner.run_planner(1, FLAG_ITERATIONS_OR_TIME, MAX_ITERATIONS_TIME, RVIZ_SHOW_TREE, 0.0, planner_run_number);

    //End of planning phase
    cout<<"..... Planner finished"<<endl;

    ros::shutdown();

    return 0;
}


