/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 17, 2017
 *      Author: kuhnerd
 * 	  Filename: selector.cpp
 */

#include <goal_planner_gui/goals/action_goal.h>
#include <goal_planner_gui/goals/goal_context.h>
#include <goal_planner_gui/gui/back_menu_item.h>
#include <goal_planner_gui/gui/current_goal_item.h>
#include <goal_planner_gui/gui/goal_menu_item.h>
#include <goal_planner_gui/gui/misc_menu_item.h>
#include <goal_planner_gui/gui/plan_item.h>
#include <goal_planner_gui/gui/selector.h>
#include <goal_planner_gui/pddl/utils.h>
#include <goal_planner_gui/planning/planner.h>
#include <goal_planner_gui/ros_database.h>
#include <qapplication.h>
#include <QVBoxLayout>
#include <qurl.h>
#include <QQuickItem>
#include <qmap.h>

namespace goal_planner_gui
{

const QMap<QString, QVariant> uiConstants {
		{ "EXECUTABLE", QVariant(QString(ACTION_STATUS_EXECUTABLE)) },
		{ "IN_PROGRESS", QVariant(QString(ACTION_STATUS_IN_PROGRESS)) },
		{ "EXECUTED", QVariant(QString(ACTION_STATUS_EXECUTED)) },
		{ "UNSUCCESSFUL", QVariant(QString(ACTION_STATUS_UNSUCCESSFUL)) },
		{ "FAILED", QVariant(QString(ACTION_STATUS_FAILED)) }
};

#define AUTOEXECUTE false

Selector::Selector(std::shared_ptr<GoalContextResult> context,
		const std::string& configPath) :
				m_view(NULL),
				m_path(QString::fromStdString(configPath)),
				m_mode(Mode::ModeGoal),
				m_context(context),
				m_goal(NULL),
				m_showPlanFlag(false),
				m_planner(new Planner(context->goalContext->m_init, m_path.toStdString())),
				m_currentGoal(new MenuStackEntry()),
				m_ignoreMoveEventes(false),
				m_currentActionIndex(0),
				m_currentActionSucceeded(true),
				m_currentIndex(0),
				m_container(NULL),
				m_qmlContext(NULL)
{
}

Selector::~Selector()
{
	delete m_view;
	delete m_container;
}

void Selector::setupUI()
{
	m_view = new QQuickView();
	m_view->setResizeMode(QQuickView::SizeRootObjectToView);
	m_container = QWidget::createWindowContainer(m_view);
	m_container->setFocusPolicy(Qt::StrongFocus);
//	m_container->setAttribute(Qt::WA_DeleteOnClose);
//	QObject::connect(m_container, SIGNAL(destroyed(QObject*)), this, SLOT(quit(QObject*)));

	QVBoxLayout* layout = new QVBoxLayout();
	layout->addWidget(m_container);
	setLayout(layout);

	m_qmlContext = m_view->rootContext();
	m_qmlContext->setContextProperty("selector", this);
	m_qmlContext->setContextProperty("Constants", QVariant(uiConstants));

	setMenu( { });
	setPlan( { });
	setCurrent(NULL);

	m_view->setSource(QUrl(getAbsPath("src/gui/ui.qml")));

	resize(800, 800);
	move(300, 300);
	setWindowTitle("Goal Planner GUI");
	setWindowIcon(QIcon(getAbsPath("images/icon.png")));

	showMaximized();

//	initMenu();
	goalMenu();
	showGoalUi();
}

void Selector::setupRootMenu()
{
	//TODO: delete vars
//	for (auto i : m_rootMenu)
//	{
//		delete i;
//	}

	m_rootMenu.clear();

	m_rootMenu.append(new MiscMenuItem("Plan for Goal", getAbsPath("images/plan.png"), "goals", true));
//	m_rootMenu.append(new MiscMenuItem("Execute Action", getAbsPath("images/step.png"), "actions", true));
	m_rootMenu.append(new MiscMenuItem("Quit", getAbsPath("images/quit.png"), "quit", false));
}

void Selector::initMenu()
{
	setupRootMenu();
//	goalMenu();
	m_mode = ModeGoal;
	m_qmlContext->setContextProperty("selectionsModel", QVariant::fromValue(m_rootMenu));
	setMenu(m_rootMenu);
}

void Selector::showGoalUi()
{
	m_mode = ModeGoal;
	QQuickItem* gui = m_view->rootObject()->findChild<QQuickItem*>("goal_ui");
	QQuickItem* pui = m_view->rootObject()->findChild<QQuickItem*>("planner_ui");
	pui->setVisible(false);
	gui->setVisible(true);
	gui->setFocus(true);
}

void Selector::showPlanUi()
{
	m_mode = ModePlan;
	QQuickItem* gui = m_view->rootObject()->findChild<QQuickItem*>("goal_ui");
	QQuickItem* pui = m_view->rootObject()->findChild<QQuickItem*>("planner_ui");
	gui->setVisible(false);
	pui->setVisible(true);
	pui->setFocus(true);
	displayStatus(0);
}

void Selector::setMenu(const QList<QObject*>& elems)
{
	m_qmlContext->setContextProperty("selectionsModel", QVariant::fromValue(elems));

	//TODO: delete vars
//	for (auto e : m_elems)
//		delete e;

	m_elems = elems;
}

void Selector::setPlan(const QList<QObject*>& elems)
{
	m_qmlContext->setContextProperty("planModel", QVariant::fromValue(elems));
	m_plans = elems;
}

void Selector::displayStatus(int index)
{
	QString text = QString("Unknown element with index %1").arg(index);

	if (m_mode == ModeGoal)
	{
		if (m_currentGoal)
		{
			if (index < m_nextGoals.size())
			{
				auto& nextGoal = m_nextGoals[index];
				text = QString::fromStdString(nextGoal->str());
			}
			else
			{
				text = "Back";
			}
		}
		else
		{
			text = "Root Menu";
		}
	}
	else if (m_currentPlan)
	{
		if (index < m_currentPlan->size())
		{
			text = QString::fromStdString((*m_currentPlan)[index]->str());
		}
		else
		{
			text = QString("Cannot display status of step %1 in a plan of length %2").arg(index, m_currentPlan->size());
		}
	}

	displayStatusText(text);
}

void Selector::quit()
{
	ros::shutdown();
	QApplication::exit(0);
	exit(0);
}

void Selector::quit(QObject* close)
{
	quit();
}

void Selector::back()
{
	if (m_ignoreMoveEventes && m_currentPlan && (*m_currentPlan)[m_currentActionIndex]->getStatus() == ACTION_STATUS_IN_PROGRESS)
	{
		throw pddl::NotImplementedException(FILE_AND_LINE);
	}

	if (m_menuStack.empty())
	{
		initMenu();
		showGoalUi();
	}
	else
	{
		auto& g = m_menuStack.top();
		m_menuStack.pop();
//		m_currentGoal.reset(new MenuStackEntry());
//		LOG_INFO("Pop " << g->goal << " and type " << g->type << " from stack");

		ASSERT(g);

		LOG_INFO(g->type);
		if (g->type == MenuTypeGoalMenu)
		{
			goalMenu();
		}
		else if (g->type == MenuTypeActionMenu)
		{
			throw pddl::NotImplementedException(FILE_AND_LINE);
		}
		else if (g->goal)
		{
			displayGoal(g->goal);
		}
		else
		{
			LOG_WARNING("Something went wrong!");
			initMenu();
			showGoalUi();
		}
	}
}

void Selector::goalMenu()
{
	m_mode = ModeGoal;

	//clean up first
	m_nextGoals.clear();

	ActionGoal::initialGoals(m_context->goalContext, m_nextGoals);

	QList<QObject*> items;
	for (auto& g : m_nextGoals)
	{
		items.append(new GoalMenuItem(g, -1));
	}
	items.append(new BackMenuItem(m_path));
	m_currentGoal->type = MenuTypeGoalMenu;
	buildHeader();
	setMenu(items);
}

void Selector::setCurrent(QObject* goal)
{
	m_qmlContext->setContextProperty("currentGoal", goal);
	if (m_goal)
		delete m_goal;
	m_goal = goal;
}

void Selector::selectGoal(int index)
{
	if (!m_nextGoals.empty())
	{
		displayGoal(m_nextGoals[index]);
	}
}

void Selector::selectAction(int index)
{
	if (m_ignoreMoveEventes)
	{
		LOG_WARNING("aborting action execution because an action is already in progress");
		return;
	}

	if (!(m_currentActionIndex == index || m_currentActionIndex + 1 == index))
	{
		LOG_WARNING("Execute previous actions first!!!");
		return;
	}

	if (m_currentPlan && index < m_currentPlan->size())
	{
		std::shared_ptr<PlanNode> planNode = (*m_currentPlan)[index];
		LOG_INFO("Executing " << index <<"th action " << planNode->getAction()->m_name << " " << planNode->getArgs()[1]);
		if (planNode->getStatus() == ACTION_STATUS_EXECUTED)
		{
			LOG_WARNING("The action was already executed");
			return;
		}

		planNode->setStatus(ACTION_STATUS_IN_PROGRESS);
		displayStatusText("Action is in progress");
		m_ignoreMoveEventes = true;
		m_rosHandler->executeOnRobot(planNode);

		QQuickItem* planView = m_view->rootObject()->findChild<QQuickItem*>("planView");
		planView->setProperty("currentIndex", QVariant(index));
		m_currentActionIndex = index;
		m_currentActionSucceeded = false;
	}
}

void Selector::actionExecuted()
{
	QQuickItem* planView = m_view->rootObject()->findChild<QQuickItem*>("planView");
	bool succes = m_currentActionSucceeded;
	int index = m_currentActionIndex;

	if (succes)
	{
		ASSERT(index < m_currentPlan->size());
		auto& planNode = (*m_currentPlan)[index];
		if (!m_planner->executeAction(planNode))
		{
			planNode->setStatus(ACTION_STATUS_FAILED);
			LOG_ERROR("Action could not be executed by the planner");
		}

		planNode->setStatus(ACTION_STATUS_EXECUTED);
	}
	else
	{
		(*m_currentPlan)[index]->setStatus(ACTION_STATUS_UNSUCCESSFUL);
	}

	m_ignoreMoveEventes = false;

	if (succes && index + 1 < m_currentPlan->size())
	{
		planView->setProperty("currentIndex", index + 1);
		if (AUTOEXECUTE)
		{
			LOG_INFO("Auto-executing next");
			QQuickItem* pui = m_view->rootObject()->findChild<QQuickItem*>("planner_ui");
			QMetaObject::invokeMethod(pui, "teleop_select");
		}
		else
		{
			refreshContext(m_planner->getProblem());
		}
	}
	else
	{
		planView->setProperty("currentIndex", index);
	}
}

void Selector::displayGoal(const std::shared_ptr<GoalSpec>& goal)
{
	if (m_currentGoal)
	{
//		LOG_INFO("Add " << m_currentGoal->goal << " and type " << m_currentGoal->type << " to stack");
		m_menuStack.push(m_currentGoal);
	}

	m_currentGoal.reset(new MenuStackEntry());
	m_currentGoal->goal = goal;
	m_currentGoal->type = MenuTypeGoalMenu;
	m_showPlanFlag = false;

	LOG_INFO("DISPLAY GOAL: " << goal->str());
	LOG_INFO("next:")
	LOG_INFO_CONTAINER_STR(goal->next())

	if (goal->isUnique())
	{
		LOG_INFO("GOAL IS UNIQUE");
		plan(goal, true);
		m_showPlanFlag = true;

		return;
	}

	m_currentIndex = goal->argIndex(goal->getCurrentArg());
	m_nextGoals.clear();
	for (auto& g : m_currentGoal->goal->nextFlattened())
	{
		if (!g->isEmpty() && filterGoal(g))
		{
			m_nextGoals.push_back(g);
		}
	}

	if (m_showPlanFlag)
	{
		onPlanUpdate();
	}

	QList<QObject*> items;
	for (auto& g : m_nextGoals)
	{
		items.append(new GoalMenuItem(g, m_currentIndex));
	}

	if (!m_menuStack.empty())
	{
		items.append(new BackMenuItem(m_path));
	}

	buildHeader(m_currentGoal->goal);
	setMenu(items);
}

void Selector::plan(const std::shared_ptr<GoalSpec>& goal,
		bool updateGui)
{
	//stack has no clear!
	while (!m_menuStack.empty())
	{
		m_menuStack.pop();
	}

	m_currentGoal.reset(new MenuStackEntry());

	m_planner->plan(goal);
	m_currentPlan = m_planner->getPlan();

	if (updateGui)
	{
		onPlanUpdate();
	}
}

bool Selector::filterGoal(const std::shared_ptr<GoalSpec>& goal)
{
	return !goal->isUniversial();
}

void Selector::buildHeader(const std::shared_ptr<GoalSpec>& goal)
{
	if (!goal)
	{
		setCurrent(NULL);
		return;
	}

	setCurrent(new CurrentGoalItem(goal));
}

void Selector::displayStatusText(const QString& text)
{
	QObject* statusbar = m_view->rootObject()->findChild<QObject*>("statusbar");
	QMetaObject::invokeMethod(statusbar, "display", Q_ARG(QVariant, QVariant::fromValue(text)));
}

const std::shared_ptr<ROSDatabase>& Selector::getRosHandler() const
{
	return m_rosHandler;
}

void Selector::setRosHandler(const std::shared_ptr<ROSDatabase>& rosHandler)
{
	m_rosHandler = rosHandler;
}

int Selector::getCurrentActionSucceeded() const
{
	return m_currentActionSucceeded;
}

void Selector::setCurrentActionSucceeded(int currentActionSucceeded)
{
	m_currentActionSucceeded = currentActionSucceeded;
}

void Selector::refreshContext(const pddl::ProblemPtr& problem)
{
	m_context->goalContext.reset(new GoalContext(problem, m_context->goalContext->m_refs));
}

bool Selector::event(QEvent* event)
{
	if (event->type() == QEvent::Close)
	{
		quit();
	}
	return QWidget::event(event);
}

void Selector::onPlanUpdate()
{
	QList<QObject*> items;
	for (auto& it : *m_currentPlan)
	{
		items.append(new PlanItem(it, m_context));
	}
	setPlan(items);
	showPlanUi();
}

} /* namespace goal_planner_gui */

