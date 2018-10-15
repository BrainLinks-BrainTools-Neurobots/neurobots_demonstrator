/*
 * pouring_problem_definition.h
 *
 *  Created on: Sep 8, 2016
 *      Author: kuhnerd
 */

#ifndef H0AC6F4ED_6DE5_4DEB_866C_816EE9354B5D
#define H0AC6F4ED_6DE5_4DEB_866C_816EE9354B5D

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
FORWARD_DECLARE_N(prm_planner, PouringInteractiveMarker);
FORWARD_DECLARE_N(prm_planner, GraspableObject);
FORWARD_DECLARE_N(prm_planner, PRMAStar);
FORWARD_DECLARE_N(prm_planner, PRM);

namespace neurobots_prm_planner_problems
{

class PouringProblemDefinition: public prm_planner::ApproachingProblemDefinition
{
public:
	enum Mode
	{
		InitToPour,
		PourToInit,
	};

	PouringProblemDefinition();
	virtual ~PouringProblemDefinition();

	virtual void init(const boost::shared_ptr<prm_planner::Robot> robot,
			const boost::shared_ptr<prm_planner::Constraint> constraint,
			const boost::shared_ptr<prm_planner::PRMPlanner> planner,
			const prm_planner::parameters::ProblemConfig& config);

	virtual std::vector<boost::shared_ptr<prm_planner::InteractiveMarker>> getInteractiveMarkers();

	virtual bool updateToolFrames();

	//planning
	virtual bool plan(const Eigen::Affine3d& goal,
			boost::shared_ptr<prm_planner::Path>& path,
			const prm_planner::PlanningParameters& parameters);

	virtual void publish();

	virtual void update(const boost::shared_ptr<prm_planner::PlanningScene>& planningScene);

	Mode getMode() const;
	void setMode(Mode mode);

protected:
	virtual void initPlanner();

private:
	bool pour(boost::shared_ptr<prm_planner::Path>& path);
	bool toInit(boost::shared_ptr<prm_planner::Path>& path);

	prm_planner::PRM* createPouringPRM();

	/**
	 * Returns the id of the last waypoint passed
	 * on the the way to the goal. It just compares
	 * the current task pose with the task poses of
	 * the path to determine the correponding id.
	 */
	int getCurrentWaypointId();

protected:
	boost::shared_ptr<prm_planner::PRMAStar> m_prmPlanner;
	std::string m_robotName;
	Eigen::Affine3d m_transformationPlanningToArm;
	Eigen::Affine3d m_tcp;
	std::vector<boost::shared_ptr<prm_planner::InteractiveMarker>> m_interactiveMarker;
	boost::shared_ptr<prm_planner::GraspableObject> m_bottle;
	boost::shared_ptr<prm_planner::GraspableObject> m_cup;
	Mode m_mode;
	boost::shared_ptr<prm_planner::Path> m_pourPath;
	Eigen::Affine3d m_startPose;
	prm_planner::PRM* m_prm;
	boost::atomic_bool m_prmInitialized;
	bool m_slowPouring;
};

} /* namespace neurobots_prm_planner_problems */

#endif

#endif /* H0AC6F4ED_6DE5_4DEB_866C_816EE9354B5D */

