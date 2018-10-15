/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 17, 2017
 *      Author: kuhnerd
 * 	  Filename: selector.h
 */

#ifndef HF1A9157F_8B06_4C95_AA5A_3CF5DD3EA431
#define HF1A9157F_8B06_4C95_AA5A_3CF5DD3EA431

#include <goal_planner_gui/planning/plan.h>
#include <QtQml/qqmlcontext.h>
#include <memory>
#include <QObject>
#include <QList>
#include <QWidget>
#include <QQuickView>
#include <stack>

namespace goal_planner_gui
{

class GoalMenuItem;
class PlanItem;
class CurrentGoalItem;
struct GoalContextResult;
class GoalSpec;
class Planner;
class ROSDatabase;

class Selector: public QWidget
{
Q_OBJECT

public:
	enum Mode
	{
		ModeGoal,
		ModePlan
	};

	enum MenuType
	{
		MenuTypeActionMenu,
		MenuTypeGoalMenu,
		MenuTypeUnknown
	};

	Selector(std::shared_ptr<GoalContextResult> context,
			const std::string& configPath);
	virtual ~Selector();

	void setupUI();

	void actionExecuted();

public slots:
	void displayStatus(int index);
	void displayStatusText(const QString& text);
	void quit();
	void quit(QObject* close);
	void back();
	void goalMenu();

	/**
	 * Is called when a goal is selected in the selection
	 * process when defining a goal
	 */
	void selectGoal(int index);

	/**
	 * Is called when an action is to be executed
	 */
	void selectAction(int index);
	const std::shared_ptr<ROSDatabase>& getRosHandler() const;
	void setRosHandler(const std::shared_ptr<ROSDatabase>& rosHandler);
	int getCurrentActionSucceeded() const;
	void setCurrentActionSucceeded(int currentActionSucceeded);

protected:
	bool event(QEvent *event) override;

private:
	void setupRootMenu();
	void initMenu();
	void showGoalUi();
	void showPlanUi();
	void displayGoal(const std::shared_ptr<GoalSpec>& goal);
	void plan(const std::shared_ptr<GoalSpec>& goal,
			bool updateGui = true);
	void refreshContext(const pddl::ProblemPtr& problem);

	bool filterGoal(const std::shared_ptr<GoalSpec>& goal);

	inline QString getAbsPath(const QString& path)
	{
		return m_path + path;
	}

	void setMenu(const QList<QObject*>& elems);
	void setPlan(const QList<QObject*>& elems);
	void setCurrent(QObject* goal);

	void buildHeader(const std::shared_ptr<GoalSpec>& goal = std::shared_ptr<GoalSpec>());

	void onPlanUpdate();

private:
	struct MenuStackEntry
	{
		MenuStackEntry() :
						type(MenuTypeUnknown)
		{
		}
		std::shared_ptr<GoalSpec> goal;
		MenuType type;
	};

	QString m_path;

	QQuickView* m_view;
	QWidget* m_container;
	QQmlContext* m_qmlContext;
	QList<QObject*> m_rootMenu;
	QList<QObject*> m_elems;
	QList<QObject*> m_plans;
	QObject* m_goal;

	Mode m_mode;
	std::shared_ptr<GoalContextResult> m_context;

	bool m_showPlanFlag;
	bool m_ignoreMoveEventes;

	int m_currentIndex;
	int m_currentActionIndex;
	int m_currentActionSucceeded;

	std::shared_ptr<MenuStackEntry> m_currentGoal;

	std::shared_ptr<Planner> m_planner;
	std::shared_ptr<Plan> m_currentPlan;

	std::vector<std::shared_ptr<GoalSpec>> m_nextGoals;
	std::stack<std::shared_ptr<MenuStackEntry>> m_menuStack;

	std::shared_ptr<ROSDatabase> m_rosHandler;
};

} /* namespace goal_planner_gui */

#endif /* HF1A9157F_8B06_4C95_AA5A_3CF5DD3EA431 */
