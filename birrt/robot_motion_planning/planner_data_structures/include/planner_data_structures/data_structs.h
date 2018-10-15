#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <ros/ros.h>

// --Namespaces --
using namespace std;


// --Structs for a Planning Tree --
struct Edge
{
    int edge_id; //ID of edge (used to remove edges from the rrt* tree visualization)
    int root_node_id; //Index of child node
    int child_node_id; //Index of child node
    vector<vector<double> > joint_trajectory;
    vector<vector<double> > ee_trajectory;
    double cost_g; //Cost of moving from edge_start to edge_end node
};

struct CostsReach
{
    double revolute; //Cost of reaching the node from the root node (considering only prismatic joints)
    double prismatic; //Cost of reaching the node from the root node (considering only prismatic joints)
    double total; //Total cost of reaching the node from the root node

};

struct Node
{
    int node_id; //Index of node
    int parent_id; //Index of parent node
    vector<double> config; //Configuration of manipulator
    vector<double> ee_pose; //Pose of the endeffector
    CostsReach cost_h; //Cost from the node to the goal node (estimated by some heuristic)
    //double cost_reach; //Cost of travelling to this node from root node
    CostsReach cost_reach; //Cost of travelling to this node from root node
    vector<Edge> outgoing_edges; //Outgoing edges from the node

    //Overloading "<" operator for sorting nodes according to their total cost to reach (ascending)
    bool operator < (const Node& str) const
    {
        return (cost_reach.total < str.cost_reach.total);
    }
};

struct Rrt_star_tree
{
    string name;
    vector<Node> nodes; //Nodes of the tree (containing outgoing edges)
    int num_nodes; //Total number of nodes in the tree
    int num_edges; //Total number of edges in the tree (only edges present in the tree)
    int num_rewire_operations; //Total number add/remove edge operations in the tree construction process
};


#endif // DATA_STRUCTS_H
