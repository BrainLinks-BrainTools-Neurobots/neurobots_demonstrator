/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: ros_database.h
 */

#ifndef H50653E13_2734_4433_988C_246D4112AF5C
#define H50653E13_2734_4433_988C_246D4112AF5C
#include <actionlib/client/simple_action_client.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/variant/variant.hpp>
#include <robot_interface_msgs/drinkSubjectAction.h>
#include <robot_interface_msgs/dropObjectAction.h>
#include <robot_interface_msgs/graspObjectAction.h>
#include <robot_interface_msgs/moveToAction.h>
#include <robot_interface_msgs/pourLiquidAction.h>
#include <robot_interface_msgs/arrangeFlowerAction.h>
#include <robot_interface_msgs/pickFlowerAction.h>
#include <robot_interface_msgs/openBottleAction.h>
#include <robot_interface_msgs/giveObjectAction.h>
#include <string>
#include <unordered_map>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/problem.h>

namespace goal_planner_gui
{

class NeurobotsWorld;
class PlanNode;
class Selector;

class ROSDatabase
{
public:
	typedef std::unordered_map<std::string, boost::variant<std::string, bool, std::vector<std::string>>>StrippedMap;

	ROSDatabase();
	virtual ~ROSDatabase();

	bool getProblemFromDatabase(std::shared_ptr<pddl::Domain> domain,
			std::shared_ptr<pddl::Problem>& problem,
			std::vector<StrippedMap>& neurobotsObjects);

	void executeOnRobot(const std::shared_ptr<PlanNode>& planNode);

	void setNeurobotsWorld(const std::vector<StrippedMap>& dict);

	void executionDoneMoveTo(const actionlib::SimpleClientGoalState& state,
			const robot_interface_msgs::moveToResultConstPtr& result);

	void feedbackMoveTo(const robot_interface_msgs::moveToFeedbackConstPtr& feedback);

	const std::shared_ptr<Selector>& getSelector() const;
	void setSelector(const std::shared_ptr<Selector>& selector);

private:
	void determineSupertypes(const std::string& type,
			std::shared_ptr<pddl::Domain> domain,
			std::unordered_set<std::string>& supertypes);

	void addChange(const std::string& object, const std::string& argument, const boost::variant<std::string, bool>& value);

	void feedbackReceived(const std::string& message);
	void executionDone(bool result);

private:
	const std::unordered_map<int, std::string> m_signalDict;
	std::shared_ptr<Selector> m_selector;
	bool m_callbackDone;

	boost::shared_ptr<NeurobotsWorld> m_world;

	actionlib::SimpleActionClient<robot_interface_msgs::drinkSubjectAction>* m_actionClientDrink;
	actionlib::SimpleActionClient<robot_interface_msgs::dropObjectAction>* m_actionClientDrop;
	actionlib::SimpleActionClient<robot_interface_msgs::graspObjectAction>* m_actionClientGrasp;
	actionlib::SimpleActionClient<robot_interface_msgs::moveToAction>* m_actionClientMoveTo;
	actionlib::SimpleActionClient<robot_interface_msgs::pourLiquidAction>* m_actionClientPour;
	actionlib::SimpleActionClient<robot_interface_msgs::arrangeFlowerAction>* m_actionClientArrangeFlower;
	actionlib::SimpleActionClient<robot_interface_msgs::pickFlowerAction>* m_actionClientPickFlower;
	actionlib::SimpleActionClient<robot_interface_msgs::openBottleAction>* m_actionClientOpenBottle;
	actionlib::SimpleActionClient<robot_interface_msgs::giveObjectAction>* m_actionClientGiveObject;
	ros::ServiceClient m_getWorldClient;

	std::string m_currentAction;
	std::unordered_set<std::tuple<std::string, std::string, boost::variant<std::string, bool>>> m_outstandingChanges;
};

}
/* namespace goal_planner_gui */

#endif /* H50653E13_2734_4433_988C_246D4112AF5C */
