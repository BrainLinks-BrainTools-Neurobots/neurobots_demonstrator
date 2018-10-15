/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: ros_database.cpp
 */

#include <goal_planner_gui/ros_database.h>
#include <goal_planner_gui/neurobots_world.h>

#include <database_msgs/get_world.h>

#include <database_conversions/database_conversions.h>
#include <goal_planner_gui/gui/selector.h>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/planning/plan_node.h>
#include <ros/node_handle.h>
#include <QString>

#define ROS_ACTION_TIMEOUT 10

namespace goal_planner_gui
{

ROSDatabase::ROSDatabase() :
				m_signalDict( { { 1, "up" }, { 2, "down" }, { 3, "select" }, { 4, "abort" }, { 5, "abort/down-contextsensitive" } }),
//				m_widget(NULL),
				m_actionClientDrink(NULL),
				m_actionClientDrop(NULL),
				m_actionClientGrasp(NULL),
				m_actionClientMoveTo(NULL),
				m_actionClientPour(NULL),
				m_actionClientArrangeFlower(NULL),
				m_actionClientPickFlower(NULL),
				m_actionClientOpenBottle(NULL),
				m_actionClientGiveObject(NULL),
				m_callbackDone(false)
{
	ros::NodeHandle n;
	ros::Duration d(ROS_ACTION_TIMEOUT);

	m_getWorldClient = n.serviceClient<database_msgs::get_world>("neurobots_database/get_world");
	if (!m_getWorldClient.waitForExistence(d))
		throw pddl::Exception("Database not available!");

	m_actionClientDrink = new actionlib::SimpleActionClient<robot_interface_msgs::drinkSubjectAction>("neurobots_demo/drink_subject", true);
	m_actionClientDrop = new actionlib::SimpleActionClient<robot_interface_msgs::dropObjectAction>("neurobots_demo/drop_object", true);
	m_actionClientGrasp = new actionlib::SimpleActionClient<robot_interface_msgs::graspObjectAction>("neurobots_demo/grasp_object", true);
	m_actionClientMoveTo = new actionlib::SimpleActionClient<robot_interface_msgs::moveToAction>("neurobots_demo/move_to", true);
	m_actionClientPour = new actionlib::SimpleActionClient<robot_interface_msgs::pourLiquidAction>("neurobots_demo/pour_liquid", true);
	m_actionClientArrangeFlower = new actionlib::SimpleActionClient<robot_interface_msgs::arrangeFlowerAction>("neurobots_demo/arrange_flower", true);
	m_actionClientPickFlower = new actionlib::SimpleActionClient<robot_interface_msgs::pickFlowerAction>("neurobots_demo/pick_flower", true);
	m_actionClientOpenBottle = new actionlib::SimpleActionClient<robot_interface_msgs::openBottleAction>("neurobots_demo/open_bottle", true);
	m_actionClientGiveObject = new actionlib::SimpleActionClient<robot_interface_msgs::giveObjectAction>("neurobots_demo/give_object", true);
}

ROSDatabase::~ROSDatabase()
{
	delete m_actionClientDrink;
	delete m_actionClientDrop;
	delete m_actionClientGrasp;
	delete m_actionClientMoveTo;
	delete m_actionClientPour;
	delete m_actionClientArrangeFlower;
	delete m_actionClientPickFlower;
	delete m_actionClientOpenBottle;
	delete m_actionClientGiveObject;
}

bool ROSDatabase::getProblemFromDatabase(std::shared_ptr<pddl::Domain> domain,
		std::shared_ptr<pddl::Problem>& problem,
		std::vector<StrippedMap>& neurobotsObjects)
{
	bool hasRooms = false;
	const std::string& domainName = domain->m_name;

	if (domainName == "neurobots-demo"
			|| domainName == "neurobots-iros2"
			|| domainName == "neurobots-simgen")
	{
		hasRooms = true;
		LOG_DEBUG("Domain " + domainName + " has rooms");
	}
	else
	{
		LOG_DEBUG("Domain " + domainName + " has no rooms");
	}

	LOG_DEBUG("Trying to fetch problem file from Neurobots Database");

	database_msgs::get_world srv;
	if (!m_getWorldClient.call(srv))
	{
		LOG_ERROR("Service call to get the world from the database failed");
		return false;
	}

	auto& world = srv.response.world;

	LOG_INFO("fetched world + " + world.domain + " with " + std::to_string(world.objects.size()) + " objects");

	std::unordered_set<std::shared_ptr<pddl::TypedObject>, pddl::TypedObjectHasher> problemObjects;
	std::unordered_set<std::string> initStrings;

	auto checkIfASuperTypeIsInList = [](const std::unordered_set<std::string>& supertypes, const std::unordered_set<std::string>& toCheck)
	{
		for (auto& s: supertypes)
		{
			if (toCheck.find(s) != toCheck.end())
			{
				return true;
			}
		}
		return false;
	};

	for (auto& wobj : world.objects)
	{
		Object object;
		database_conversions::convertMsgToObject(wobj, object);
		StrippedMap strippedMap;

		if (CONTAINS_NOT("name", object))
			throw pddl::Exception("name not available");
		std::string name = CONVERT_TO_STRING(object["name"]);
		strippedMap["name"] = name;

		if (CONTAINS_NOT("type", object))
			throw pddl::Exception("type not available");
		std::string type = CONVERT_TO_STRING(object["type"]);
		strippedMap["type"] = type;

//		LOG_DEBUG("Detected object " + name + " of type " + type);

		std::unordered_set<std::string> supertypes;
		determineSupertypes(type, domain, supertypes);

		if (type != "poses")
		{
//			LOG_INFO("processing " << name)
			problemObjects.insert(std::shared_ptr<pddl::TypedObject>(new pddl::TypedObject(name, domain->m_types[type])));

			if (checkIfASuperTypeIsInList(supertypes, { "transportable", "vessel" }))
			{
				if (CONTAINS_NOT("position", object))
					throw pddl::Exception("position not available in object: " + name + " (type " + type + ")");
				std::string position = CONVERT_TO_STRING(object["position"]);
				strippedMap["position"] = position;
				initStrings.insert("(= (position " + name + ") " + position + ")");
				if (CONTAINS("vessel", supertypes))
				{
					std::string contains = "empty";
					if (CONTAINS("contains", object))
					{
						contains = CONVERT_TO_STRING(object["contains"]);
					}
					else
					{
						LOG_DEBUG("Uh, object named " << name << " is a vessel with no specified content");
					}
					initStrings.insert("(= (contains " + name + ") " + contains + ")");
					strippedMap["contains"] = contains;

					if (CONTAINS("is-open", object) && CONVERT_TO_BOOL(object["is-open"]))
					{
						initStrings.insert("(is-open " + name + ")");
						strippedMap["is-open"] = true;
					}

					if (type == "glass")
					{
						if (CONTAINS_NOT("shaped", object))
							throw pddl::Exception("shape not available for object " + name);
						const std::string shaped = CONVERT_TO_STRING(object["shaped"]);
						strippedMap["shaped"] = shaped;
						initStrings.insert("(= (shaped " + name + ") " + shaped + ")");
					}
				}

				if (checkIfASuperTypeIsInList(supertypes, { "flower", "vase", "cup" }))
				{
					std::string colored = "uncolored";
					if (CONTAINS("colored", object))
					{
						colored = CONVERT_TO_STRING(object["colored"]);
					}
					else
					{
						LOG_DEBUG("Uh, object named " << name << " is a flower, vase or cup with no specified color");
					}
					initStrings.insert("(= (colored " + name + ") " + colored + ")");
					strippedMap["colored"] = colored;
				}
			}
			else if (checkIfASuperTypeIsInList(supertypes, { "robot", "furniture", "human" }))
			{
				if (hasRooms)
				{
					if (CONTAINS("in", object))
					{
						std::string loc = CONVERT_TO_STRING(object["in"]);
						initStrings.insert("(= (in " + name + ") " + loc + ")");
						strippedMap["in"] = loc;
					}
					else
					{
						if (object.size() == 4)
							throw pddl::Exception(
									"Base " + name
											+ " is not located anywhere (no 'in' attribute found) and does not appear to be a type definition");
					}

					if (CONTAINS("aligned", object))
					{
						std::string aligned = CONVERT_TO_STRING(object["aligned"]);
						initStrings.insert("(= (aligned " + name + ") " + aligned + ")");
						strippedMap["aligned"] = aligned;
					}

					if (type == "robot")
					{
						if (CONTAINS("mobile", object) && CONVERT_TO_BOOL(object["mobile"]))
						{
							initStrings.insert("(mobile " + name + ")");
							strippedMap["mobile"] = true;
						}
						else
						{
							strippedMap["mobile"] = false;
						}

						if (CONTAINS("arm-empty", object) && CONVERT_TO_BOOL(object["arm-empty"]))
						{
							initStrings.insert("(arm-empty " + name + ")");
							strippedMap["arm-empty"] = true;
						}
						else
						{
							strippedMap["arm-empty"] = false;
						}

						std::string at = "nowhere";
						if (CONTAINS("at", object))
							at = CONVERT_TO_STRING(object["at"]);

						initStrings.insert("(= (at " + name + ") " + at + ")");
						strippedMap["at"] = at;
					}
				}
			}
			else if (type == "room")
			{
//				LOG_DEBUG("Connected predicates for room " + name);
				if (CONTAINS_NOT("connected", object))
					throw pddl::Exception("connected not available");

				std::vector<std::string> rooms;
				for (auto& room : object["connected"])
					rooms.push_back(boost::get<std::string>(room));

				strippedMap["connected"] = rooms;

				for (auto& room : rooms)
				{
//					LOG_DEBUG("Adding connection (connected + " + name + ", " + room + ")");
					initStrings.insert("(connected " + name + " " + room + ")");
				}
			}
		}

		neurobotsObjects.push_back(strippedMap);
	}

	problem.reset(new pddl::Problem("neurobots-test-problem", problemObjects, domain));

	for (auto& it : initStrings)
	{
//		LOG_INFO("Trying to process " + it);

//HACK! sometimes objects are detected by the camera falsely, so an object
//could be in the database without position. The attribute position is
//then present, but the position is False "(= (position object) 0)"
//instead. In this case we ignore the initial fact
		if (it.find("= (") != std::string::npos && it.find(") 0") != std::string::npos)
		{
			LOG_ERROR("Warning: init_string ignored because of HACK: " + it);
			continue;
		}

		pddl::Parser elemParser( { it }, "neurobots-database", { });
		std::shared_ptr<pddl::InitLiteral> initElem = pddl::InitLiteral::parse(elemParser.getRoot(), problem);
		problem->m_init.insert(initElem);
	}

	return true;
}

void ROSDatabase::determineSupertypes(const std::string& type,
		std::shared_ptr<pddl::Domain> domain,
		std::unordered_set<std::string>& supertypes)
{
	supertypes.insert(type);
	std::shared_ptr<pddl::Type> tobj;
	try
	{
		tobj = domain->m_types.at(type);
	}
	catch (...)
	{
	}

	if (tobj)
	{
		std::unordered_set<std::shared_ptr<pddl::Type>, pddl::TypeHasher> super;
		tobj->getSupertypes(super);
//		LOG_INFO("Supertypes of " + tobj->getName() + ": ")
//		std::string str = "---> ";
//		for (auto& it : super)
//			str += it->getName() + " ";
//		LOG_INFO(str);
		for (auto& it : super)
			supertypes.insert(it->getName());
	}
}

void ROSDatabase::executeOnRobot(const std::shared_ptr<PlanNode>& planNode)
{
	static const ros::Duration d(ROS_ACTION_TIMEOUT);

	m_currentAction = planNode->getAction()->m_name;
	std::vector<std::string> args;
	std::string argString;
	for (auto& it : planNode->getArgs())
	{
		args.push_back(it->getName());
		argString += it->getName() + ", ";
	}
	REMOVE_LAST_COMMA(argString);

	LOG_INFO("Execute action " << m_currentAction << " with args " << argString);

	if (m_currentAction == "approach")
	{
		if (!m_actionClientMoveTo->waitForServer(d))
			throw pddl::Exception("Action Server exceeded timeout: move");

		addChange(args[0], "at", args[2]);
		robot_interface_msgs::moveToGoal goal;
		goal.action_name = "approach";
		goal.robot = args[0];
		goal.target_object = args[2];
		m_actionClientMoveTo->sendGoal(goal,
				boost::bind(&ROSDatabase::executionDoneMoveTo, this, _1, _2),
				actionlib::SimpleActionClient<robot_interface_msgs::moveToAction>::SimpleActiveCallback(),
				boost::bind(&ROSDatabase::feedbackMoveTo, this, _1));
	}
	else if (m_currentAction == "go")
	{
		if (!m_actionClientMoveTo->waitForServer(d))
			throw pddl::Exception("Action Server exceeded timeout: move");

		addChange(args[0], "in", args[2]);
		addChange(args[0], "at", std::string("nowhere"));

		robot_interface_msgs::moveToGoal goal;
		goal.action_name = "move_to";
		goal.robot = args[0];
		goal.target_object = args[2];
		m_actionClientMoveTo->sendGoal(goal,
				boost::bind(&ROSDatabase::executionDoneMoveTo, this, _1, _2),
				actionlib::SimpleActionClient<robot_interface_msgs::moveToAction>::SimpleActiveCallback(),
				boost::bind(&ROSDatabase::feedbackMoveTo, this, _1));
	}
	else if (m_currentAction == "move")
	{
		if (!m_actionClientMoveTo->waitForServer(d))
			throw pddl::Exception("Action Server exceeded timeout: move");

		addChange(args[0], "in", args[2]);
		addChange(args[0], "at", args[1]);

		robot_interface_msgs::moveToGoal goal;
		goal.action_name = "move_to";
		goal.robot = args[0];
		goal.target_object = args[2];
		m_actionClientMoveTo->sendGoal(goal,
				boost::bind(&ROSDatabase::executionDoneMoveTo, this, _1, _2),
				actionlib::SimpleActionClient<robot_interface_msgs::moveToAction>::SimpleActiveCallback(),
				boost::bind(&ROSDatabase::feedbackMoveTo, this, _1));
	}
	else if (m_currentAction == "grasp")
	{
		if (!m_actionClientGrasp->waitForServer(d))
			throw pddl::Exception("Action Server exceeded timeout: grasp");

		addChange(args[1], "position", args[0]);
		addChange(args[0], "arm-empty", false);

//		robot_interface_msgs::moveToGoal goal;
//		goal.action_name = "move_to";
//		goal.robot = args[0];
//		goal.target_object = args[2];
//		m_actionClientMoveTo->sendGoal(goal);
	}
	else if (m_currentAction == "pick")
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (m_currentAction == "open")
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (m_currentAction == "drop")
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (m_currentAction == "arrange")
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (m_currentAction == "give")
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (m_currentAction == "drink")
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
	else if (m_currentAction == "pour")
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}
}

void ROSDatabase::setNeurobotsWorld(const std::vector<StrippedMap>& dict)
{
	m_world.reset(new NeurobotsWorld(dict));
}

void ROSDatabase::executionDoneMoveTo(const actionlib::SimpleClientGoalState& state,
		const robot_interface_msgs::moveToResultConstPtr& result)
{
	executionDone(result->arrived);
}

void ROSDatabase::feedbackMoveTo(const robot_interface_msgs::moveToFeedbackConstPtr& feedback)
{
	feedbackReceived(feedback->state);
}

void ROSDatabase::addChange(const std::string& object,
		const std::string& argument,
		const boost::variant<std::string, bool>& value)
{
	if (!m_world->get(object)->isEqual(argument, value))
	{
		m_outstandingChanges.insert(std::make_tuple(object, argument, value));
		LOG_INFO("add change of " << argument << " to " << value);
	}
}

void ROSDatabase::feedbackReceived(const std::string& message)
{
	m_selector->displayStatusText(QString("Action Status: ") + QString::fromStdString(message));
	LOG_INFO("Received feedback: " << message);
}

void ROSDatabase::executionDone(bool result)
{
	if (!result)
	{
		m_outstandingChanges.clear();
	}

	m_selector->setCurrentActionSucceeded(result);
	m_callbackDone = true;

	if (m_outstandingChanges.empty())
	{
		m_selector->actionExecuted();
		m_currentAction = "";
	}
	else
	{
		LOG_ERROR("Outstanding changes still available:");
		for (auto& it : m_outstandingChanges)
		{
			LOG_ERROR("- " << std::get<0>(it) << " " << std::get<1>(it) << " " << std::get<2>(it));
		}
	}

	LOG_INFO("execution done");
}

const std::shared_ptr<Selector>& ROSDatabase::getSelector() const
{
	return m_selector;
}

void ROSDatabase::setSelector(const std::shared_ptr<Selector>& selector)
{
	m_selector = selector;
}

} /* namespace goal_planner_gui */

