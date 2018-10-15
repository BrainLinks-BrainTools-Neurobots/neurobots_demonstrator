/*
 * drinking_problem_definition.h
 *
 *  Created on: Sep 29, 2016
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_PROBLEM_DEFINITIONS_DRINKING_PROBLEM_DEFINITION_H_
#define PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_PROBLEM_DEFINITIONS_DRINKING_PROBLEM_DEFINITION_H_

#ifdef FOUND_PRM_PLANNER

#include <prm_planner/problem_definitions/approaching_problem_definition.h>
#include <Eigen/Geometry>

FORWARD_DECLARE_N(prm_planner, Robot);
FORWARD_DECLARE_N(prm_planner, Constraint);
FORWARD_DECLARE_N(prm_planner, PRMPlanner);
FORWARD_DECLARE_N(prm_planner, Path);
FORWARD_DECLARE_N(prm_planner, PathPlanner);
FORWARD_DECLARE_N(prm_planner, PRM);
FORWARD_DECLARE_N(prm_planner, InteractiveMarker);
FORWARD_DECLARE_N(prm_planner, GraspableObject);
FORWARD_DECLARE_N(prm_planner, PRMAStar);

namespace neurobots_prm_planner_problems
{

class DrinkingProblemDefinition: public prm_planner::ApproachingProblemDefinition
{
public:
	enum Mode
	{
		InitToMouth,
		MouthToInit,
	};

	DrinkingProblemDefinition();
	virtual ~DrinkingProblemDefinition();

	virtual void init(const boost::shared_ptr<prm_planner::Robot> robot,
			const boost::shared_ptr<prm_planner::Constraint> constraint,
			const boost::shared_ptr<prm_planner::PRMPlanner> planner,
			const prm_planner::parameters::ProblemConfig& config);

//	virtual Eigen::Affine3d samplePose();
	virtual std::vector<boost::shared_ptr<prm_planner::InteractiveMarker>> getInteractiveMarkers();

	virtual bool updateToolFrames();

	virtual prm_planner::PRM* createDrinkingPRM();

	//planning
	virtual bool plan(const Eigen::Affine3d& goal,
			boost::shared_ptr<prm_planner::Path>& path,
			const prm_planner::PlanningParameters& parameters);

	virtual void update(const boost::shared_ptr<prm_planner::PlanningScene>& planningScene);

	virtual void publish();

	Mode getMode() const;
	void setMode(Mode mode);

protected:
	bool drink(boost::shared_ptr<prm_planner::Path>& path);
	bool toInit(boost::shared_ptr<prm_planner::Path>& path);

	virtual void initPlanner();

private:
	boost::shared_ptr<prm_planner::PRMAStar> m_prmPlanner;
	std::string m_robotName;
	Eigen::Affine3d m_tcp;
	std::vector<boost::shared_ptr<prm_planner::InteractiveMarker>> m_interactiveMarker;
	boost::shared_ptr<prm_planner::GraspableObject> m_cup;
	boost::shared_ptr<prm_planner::GraspableObject> m_head;
	boost::shared_ptr<prm_planner::Path> m_drinkPath;
	Eigen::Affine3d m_startPose;
	prm_planner::PRM* m_prm;
	boost::atomic_bool m_prmInitialized;

	Mode m_mode;
};

} /* namespace neurobots_prm_planner_problems */

#endif

#endif /* PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_PROBLEM_DEFINITIONS_DRINKING_PROBLEM_DEFINITION_H_ */
