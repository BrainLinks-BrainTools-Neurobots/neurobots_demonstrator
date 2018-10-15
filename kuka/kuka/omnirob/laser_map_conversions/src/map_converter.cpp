


#include <laser_map_conversions/map_converter.h>

namespace map_converter
{


LaserMapConverter::LaserMapConverter(int8_t obs_threshold, bool unknown_obstacles, double extrusion, int padding, std::string planning_scene_prefix) : m_unknown_obstacles(unknown_obstacles), m_obs_threshold(obs_threshold), m_desired_extrusion(extrusion), m_map_padding(padding)
{
	//Subscriber to /map topic (published by "map_server" node)
    m_map_sub = m_nh.subscribe("/map", 1, &LaserMapConverter::map_callback, this);

    //Publish generated octomap
    m_octomap_pub = m_nh.advertise<octomap_msgs::Octomap>("/octomap_demo", 1);

    //Get namespaces of robot
    //m_nh.param("ns_prefix_planning_scene", m_ns_prefix_planning_scene, std::string("/omnirob_lbr"));

    //Publisher for collision objects
    m_ps_octo_pub = m_nh.advertise<moveit_msgs::PlanningScene>(planning_scene_prefix + "/planning_scene", 1);

    
    //Init flag indicating whether map has been converted to octomap
    m_map_received = false;
    
	//Nothing else to do here
}

LaserMapConverter::~LaserMapConverter()
{
	//Nothing to do here
}


//------------------ FUNCTIONS ------------------------

moveit_msgs::OrientedBoundingBox LaserMapConverter::get_bbx(const moveit_msgs::CollisionObject& collision_map)
{
    moveit_msgs::OrientedBoundingBox bbx;

    if (collision_map.primitives.empty()) {
        bbx.extents = geometry_msgs::CreatePoint32(0.0, 0.0, 0.0);
        bbx.pose = geometry_msgs::IdentityPose();
        return bbx;
    }

    assert(!collision_map.primitive_poses.empty());

    const geometry_msgs::Pose& first_pose =
            collision_map.primitive_poses.front();

    double min_x = first_pose.position.x;
    double min_y = first_pose.position.y;
    double min_z = first_pose.position.z;

    double max_x = first_pose.position.x;
    double max_y = first_pose.position.y;
    double max_z = first_pose.position.z;

    for (const geometry_msgs::Pose& pose : collision_map.primitive_poses) {
        if (pose.position.x < min_x) {
            min_x = pose.position.x;
        }
        if (pose.position.y < min_y) {
            min_y = pose.position.y;
        }
        if (pose.position.z < min_z) {
            min_z = pose.position.z;
        }

        if (pose.position.x > max_x) {
            max_x = pose.position.x;
        }
        if (pose.position.y > max_y) {
            max_y = pose.position.y;
        }
        if (pose.position.z > max_z) {
            max_z = pose.position.z;
        }
    }

    bbx.extents.x = max_x - min_x;
    bbx.extents.y = max_y - min_y;
    bbx.extents.z = max_z - min_z;
    bbx.pose.position.x = 0.5 * (max_x + min_x);
    bbx.pose.position.y = 0.5 * (max_y + min_y);
    bbx.pose.position.z = 0.5 * (max_z + min_z);
    bbx.pose.orientation = geometry_msgs::IdentityQuaternion();

    return bbx;
}


moveit_msgs::CollisionObject LaserMapConverter::extrude_to_collision_map(const nav_msgs::OccupancyGrid& grid, double extrusion) 
{
    ROS_INFO("Extruding Occupancy Grid");

    moveit_msgs::CollisionObject cmap;

    cmap.header.seq = 0;
    cmap.header.stamp = ros::Time(0);
    cmap.header.frame_id = grid.header.frame_id;

    const uint32_t width = grid.info.width;
    const uint32_t height = grid.info.height;
    const float res = grid.info.resolution;


    int num_occupied_cells = 0;
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            // only add occuped cells to the collision map
            bool obstacle = (grid.data[y * width + x] >= m_obs_threshold) || (m_unknown_obstacles && grid.data[y * width + x] < 0);

            if (!obstacle) {
                continue;
            }

            ++num_occupied_cells;

            //Position for cell box
            double map_x = grid.info.origin.position.x + x * res + 0.5 * res;
            double map_y = grid.info.origin.position.y + y * res + 0.5 * res;

            for (double z = 0.0; z <= extrusion; z += res)
            {
                const double& map_z = z + 0.5 * res;

                // construct
                geometry_msgs::Point32 box_extents(geometry_msgs::CreatePoint32(res, res, res));
                shape_msgs::SolidPrimitive sp;
                sp.type = shape_msgs::SolidPrimitive::BOX;
                sp.dimensions.resize(3);
                sp.dimensions[shape_msgs::SolidPrimitive::BOX_X] = box_extents.x;
                sp.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = box_extents.y;
                sp.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = box_extents.z;
                cmap.primitives.push_back(sp);

                geometry_msgs::Point box_pos(geometry_msgs::CreatePoint(map_x, map_y, map_z));
                geometry_msgs::Quaternion box_rot(geometry_msgs::IdentityQuaternion());
                geometry_msgs::Pose box_pose(geometry_msgs::CreatePose(box_pos, box_rot));
                cmap.primitive_poses.push_back(box_pose);
            }
        }
    }

    ROS_INFO("Extruded %d occupied cells", num_occupied_cells);

    moveit_msgs::OrientedBoundingBox bbx = get_bbx(cmap);
    //ROS_INFO("  Collision Map Bounding Box: %s", to_string(bbx).c_str());

    return cmap;
}


moveit_msgs::CollisionObject LaserMapConverter::extrude_to_collision_map_padded(const nav_msgs::OccupancyGrid& grid, double extrusion)
{
    ROS_INFO("Extruding Occupancy Grid");

    moveit_msgs::CollisionObject cmap;

    cmap.header.seq = 0;
    cmap.header.stamp = ros::Time(0);
    cmap.header.frame_id = grid.header.frame_id;

    const uint32_t width = grid.info.width;
    const uint32_t height = grid.info.height;
    const float res = grid.info.resolution;


    int num_occupied_cells = 0;
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            // only add occuped cells to the collision map
            bool obstacle = (grid.data[y * width + x] >= m_obs_threshold) || (m_unknown_obstacles && grid.data[y * width + x] < 0);

            if (!obstacle) {
                continue;
            }

            ++num_occupied_cells;

            //Position for cell box
            double map_x = grid.info.origin.position.x + x * res + 0.5 * res;
            double map_y = grid.info.origin.position.y + y * res + 0.5 * res;

            for (double z = 0.0; z <= extrusion; z += res)
            {
                const double& map_z = z + 0.5 * res;

                // construct
                geometry_msgs::Point32 box_extents(geometry_msgs::CreatePoint32(res, res, res));
                shape_msgs::SolidPrimitive sp;
                sp.type = shape_msgs::SolidPrimitive::BOX;
                sp.dimensions.resize(3);
                sp.dimensions[shape_msgs::SolidPrimitive::BOX_X] = box_extents.x;
                sp.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = box_extents.y;
                sp.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = box_extents.z;
                cmap.primitives.push_back(sp);

                geometry_msgs::Point box_pos(geometry_msgs::CreatePoint(map_x, map_y, map_z));
                geometry_msgs::Quaternion box_rot(geometry_msgs::IdentityQuaternion());
                geometry_msgs::Pose box_pose(geometry_msgs::CreatePose(box_pos, box_rot));
                cmap.primitive_poses.push_back(box_pose);
            }

            if(m_map_padding != 0)
            {
                //Add padding around obstacle cell box
                for (int i = 0 ; i < m_map_padding+2 ; i++)
                {
                    for (int j = 0 ; j < m_map_padding+2 ; j++)
                    {
                        int curr_pad_x = x - m_map_padding + i;
                        int curr_pad_y = y + m_map_padding - j;

                        //Check whether cell is already occupied
                        obstacle = (grid.data[curr_pad_y * width + curr_pad_x] >= m_obs_threshold) || (m_unknown_obstacles && grid.data[curr_pad_y * width + curr_pad_x] < 0);

                        //Cell is currently unoccupied
                        if (!obstacle)
                        {
                            ++num_occupied_cells;

                            //Position for cell box
                            map_x = grid.info.origin.position.x + curr_pad_x * res + 0.5 * res;
                            map_y = grid.info.origin.position.y + curr_pad_y * res + 0.5 * res;

                            for (double z = 0.0; z <= extrusion; z += res)
                            {
                                const double& map_z = z + 0.5 * res;

                                // construct
                                geometry_msgs::Point32 box_extents(geometry_msgs::CreatePoint32(res, res, res));
                                shape_msgs::SolidPrimitive sp;
                                sp.type = shape_msgs::SolidPrimitive::BOX;
                                sp.dimensions.resize(3);
                                sp.dimensions[shape_msgs::SolidPrimitive::BOX_X] = box_extents.x;
                                sp.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = box_extents.y;
                                sp.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = box_extents.z;
                                cmap.primitives.push_back(sp);

                                geometry_msgs::Point box_pos(geometry_msgs::CreatePoint(map_x, map_y, map_z));
                                geometry_msgs::Quaternion box_rot(geometry_msgs::IdentityQuaternion());
                                geometry_msgs::Pose box_pose(geometry_msgs::CreatePose(box_pos, box_rot));
                                cmap.primitive_poses.push_back(box_pose);
                            }
                        }
                    }
                }
            }

        }
    }

    ROS_INFO("Extruded %d occupied cells", num_occupied_cells);

    moveit_msgs::OrientedBoundingBox bbx = get_bbx(cmap);
    //ROS_INFO("  Collision Map Bounding Box: %s", to_string(bbx).c_str());

    return cmap;
}

void LaserMapConverter::convert(const moveit_msgs::CollisionObject& cmap, octomap::Pointcloud& octomap_cloud)
{
    octomap_cloud.clear();
    octomap_cloud.reserve(cmap.primitives.size());
    assert(cmap.primitives.size() == cmap.primitive_poses.size());
    for (size_t i = 0; i < cmap.primitives.size(); ++i)
    {
        const geometry_msgs::Pose& pose = cmap.primitive_poses[i];
        octomap_cloud.push_back(
                pose.position.x, pose.position.y, pose.position.z);
    }
}



void LaserMapConverter::log_octomap(const octomap::OcTree& octree)
{
    size_t num_nodes = octree.calcNumNodes();
    ROS_INFO("  Num Nodes: %zd", num_nodes);
    ROS_INFO("  Memory Usage: %zd bytes", octree.memoryUsage());
    ROS_INFO("  Num Leaf Nodes: %zd", octree.getNumLeafNodes());

    // TODO: where has this function gone?
//    unsigned num_thresholded, num_other;
//    octree.calcNumThresholdedNodes(num_thresholded, num_other);
//    ROS_INFO("  Num Thresholded Nodes: %u", num_thresholded);
//    ROS_INFO("  Num Other Nodes: %u", num_other);

    const octomap::point3d octomap_min = octree.getBBXMin();
    const octomap::point3d octomap_max = octree.getBBXMax();
    const octomap::point3d octomap_center = octree.getBBXCenter();
    double clamping_thresh_min = octree.getClampingThresMin();
    double clamping_thresh_max = octree.getClampingThresMax();

    ROS_INFO("  Bounding Box Set: %s", octree.bbxSet() ? "TRUE" : "FALSE");
    ROS_INFO("  Bounding Box Min: (%0.3f, %0.3f, %0.3f)", octomap_min.x(), octomap_min.y(), octomap_min.z());
    ROS_INFO("  Bounding Box Max: (%0.3f, %0.3f, %0.3f)", octomap_max.x(), octomap_max.y(), octomap_max.z());
    ROS_INFO("  Bounding Box Center: (%0.3f, %0.3f, %0.3f)", octomap_center.x(), octomap_center.y(), octomap_center.z());
    ROS_INFO("  Clamping Threshold Min: %0.3f", clamping_thresh_min);
    ROS_INFO("  Clamping Threshold Max: %0.3f", clamping_thresh_max);

    double metric_min_x, metric_min_y, metric_min_z;
    double metric_max_x, metric_max_y, metric_max_z;
    double metric_size_x, metric_size_y, metric_size_z;
    octree.getMetricMin(metric_min_x, metric_min_y, metric_min_z);
    octree.getMetricMax(metric_max_x, metric_max_y, metric_max_z);

    ROS_INFO("  Metric Min: (%0.3f, %0.3f, %0.3f)", metric_min_x, metric_min_y, metric_min_z);
    ROS_INFO("  Metric Max: (%0.3f, %0.3f, %0.3f)", metric_max_x, metric_max_y, metric_max_z);

    octree.getMetricSize(metric_size_x, metric_size_y, metric_size_z);
    ROS_INFO("  Metric Size: (%0.3f, %0.3f, %0.3f)", metric_size_x, metric_size_y, metric_size_z);

    ROS_INFO("  Node Size (max depth): %0.6f", octree.getNodeSize(octree.getTreeDepth()));
    ROS_INFO("  Occupancy Threshold: %0.3f", octree.getOccupancyThres());
    ROS_INFO("  Probability Hit: %0.3f", octree.getProbHit());
    ROS_INFO("  Probability Miss: %0.3f", octree.getProbMiss());
    ROS_INFO("  Resolution: %0.3f", octree.getResolution());
    ROS_INFO("  Depth: %u", octree.getTreeDepth());
    ROS_INFO("  Tree Type: %s", octree.getTreeType().c_str());
}



bool LaserMapConverter::extrude(const nav_msgs::OccupancyGrid& grid, double extrusion, octomap_msgs::Octomap& map)
{
    //moveit_msgs::CollisionObject collision_map = extrude_to_collision_map(grid, extrusion);
    moveit_msgs::CollisionObject collision_map = extrude_to_collision_map_padded(grid, extrusion);


    // convert the collision map to a point cloud
    ROS_INFO("Creating Octomap from Collision Object");
    ROS_INFO("  %zd solid primitives (assumed boxes)", collision_map.primitives.size());

    const shape_msgs::SolidPrimitive& first_box =
            collision_map.primitives.front();
    assert(!first_box.dimensions.empty());
    const double res = first_box.dimensions[0];
    ROS_INFO("  Resolution: %0.3f", res);

    octomap::Pointcloud octomap_cloud;
    convert(collision_map, octomap_cloud);

    octomap::OcTree octree(res);
    octree.insertPointCloud(octomap_cloud, octomap::point3d(0.0, 0.0, 0.0)); //old insertScan, if there are problems in ROS jade

    log_octomap(octree);

    octomap_msgs::Octomap octomap;

    if (!octomap_msgs::fullMapToMsg(octree, octomap)) {
        ROS_ERROR("Failed to convert OcTree to octomap_msgs/Octomap");
        return false;
    }

    octomap.header = collision_map.header;
    map = octomap;

    return true;
}


//------------------ Map topic "/map" callback ------------------------


void LaserMapConverter::map_callback(const nav_msgs::OccupancyGridConstPtr& map_grid)
{
    ROS_INFO("Occupancy Grid received in topic callback!!!");

    if(!m_map_received){

        //Syntax: extrude(const nav_msgs::OccupancyGrid& grid, double extrusion, octomap_msgs::Octomap& map)
        bool success = extrude(*map_grid,m_desired_extrusion,m_map_octo);

        if(success)
            ROS_INFO("Occupancy Grid successfully converted to octomap!!!");
        else
            ROS_INFO("Occupancy Grid conversion to octomap failed!!!");

        //Set Flag to true(map has been converted)
        m_map_received = true;

    }
    else{

    }

}


//To publish the generated octomap
bool LaserMapConverter::publish_extruded_map(std::string octomap_ref_frame){

    if(m_map_received){

        //Publish octomap_msgs::Octomap message on topic "/octomap_demo"
        m_octomap_pub.publish(m_map_octo);

        //Publish Octomap on "planning_scene" topic
        moveit_msgs::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.octomap.header.frame_id = octomap_ref_frame;

        //Define the pose of the octree with respect to the header frame
        geometry_msgs::Pose oct_map_pose_wrt_header_frame;
        oct_map_pose_wrt_header_frame.position.x = 0.0;
        oct_map_pose_wrt_header_frame.position.y = 0.0;
        oct_map_pose_wrt_header_frame.position.z = 0.0;
        oct_map_pose_wrt_header_frame.orientation.x = 0.0;
        oct_map_pose_wrt_header_frame.orientation.y = 0.0;
        oct_map_pose_wrt_header_frame.orientation.z = 0.0;
        oct_map_pose_wrt_header_frame.orientation.w = 1.0;

        //Set the pose of the octree with respect to the header frame
        planning_scene_msg.world.octomap.origin = oct_map_pose_wrt_header_frame;

        //The octomap_msgs::Octomap message
        planning_scene_msg.world.octomap.octomap = m_map_octo;

        ROS_INFO("Adding the octomap into the world.");
        m_ps_octo_pub.publish(planning_scene_msg);

        return true;
    }
    else
        return false;
}

}//end of namespace
