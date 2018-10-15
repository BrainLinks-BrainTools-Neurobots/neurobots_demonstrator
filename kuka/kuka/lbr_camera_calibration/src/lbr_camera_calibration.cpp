/*
 * lbr_camera_calibration.h
 *
 * Copyright (C) 2015 Daniel Kuhner (kuhnerd@informatik.uni-freiburg.de)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <lbr_camera_calibration/camera_calibration.h>
#include <qcommandlineparser.h>
#include <qcoreapplication.h>
#include <ros/package.h>

using namespace aruco;

int main(int argc,
		char** argv)
{
	QCoreApplication app(argc, argv);
	QCoreApplication::setApplicationName("lbr_camera_calibration");
	QCoreApplication::setApplicationVersion("1.0");

	//parser
	QCommandLineParser parser;
	parser.setApplicationDescription("Camera calibration tool to estimate the "
			"transformation between an external camera and the robot. There are "
			"4 different ways to use the calibration tool:\n"
			"  1. -t <trajectory>: Use this method if you want to specify the "
			"calibration trajectory by yourself. Just place an aruco marker "
			"at the end-effector and define a trajectory (file that contains a "
			"named joint pose in each line, e.g., lbr_2_joint=1.2, lbr_3_joint=2.0, "
			"...). \033[31mIMPORTANT\033[0m: Don't specify the trajectory on a plane and be careful "
			"to rotate the marker as much as possible to get a good calibration.\n"
			"  2. -l <seconds>: The calibration tool listens to the tf and camera "
			"topics for a specified amount of seconds.\n"
			"  3. -r <file>: Reads the marker and end-effector poses from a file, "
			"which was stored to disk from the calibration tool using the -w flag.\n"
			"  4. --test: Runs a simple test case.\n"
			"  5. -a: Automatic trajectory estimation. You need to provide the "
			"rough position of the camera in this mode.\n"
			"If you are happy with the calibration, use the -s <cal_save_path> flag to store the "
			"calibration matrix to disk.");
	parser.addHelpOption();
	parser.addVersionOption();

	
	//which robot is used: IIWA or Omnirob
	const QCommandLineOption cameraOption(QStringList() << "camera" << "camera", "Which camera is used? Kinectv1 or Kinectv2?", "camera");
	parser.addOption(cameraOption);

	//which robot is used: IIWA or Omnirob
	const QCommandLineOption robotOption(QStringList() << "robot" << "robot", "Which robot is used? IIWA or Omnirob?", "robot");
	parser.addOption(robotOption);

	//direction of the calibration: camera to robot (1) or robot to camera (2)
	const QCommandLineOption dOption(QStringList() << "d" << "direction", "In which direction is calibrated?"
			"1: camera to robot or 2: robot to camera", "direction_number", QString::number(1));
	parser.addOption(dOption);

	//trajectory
	const QCommandLineOption tOption(QStringList() << "t" << "trajectory", "A file containing a list of waypoints.", "trajectory");
	parser.addOption(tOption);

	//listen to the topics for x seconds
	const QCommandLineOption listenOption(QStringList() << "l" << "listen", "Listen to the topics.", "seconds", QString::number(15));
	parser.addOption(listenOption);

	//read from poses
	const QCommandLineOption readPosesOption(QStringList() << "r" << "read-poses", "Read poses from file.", "file", QString("poses.txt"));
	parser.addOption(readPosesOption);

	//run test case
	const QCommandLineOption testCaseOption(QStringList() << "test", "Run test case?");
	parser.addOption(testCaseOption);

	//marker size
	const QCommandLineOption markerSizeOption(QStringList() << "m" << "marker-size", "The size of the marker in m.", "marker-size",
			QString::number(0.075));
	parser.addOption(markerSizeOption);

	//board file
	const QCommandLineOption boardConfigOption(QStringList() << "b" << "aruco-board-config", "The aruco board config.", "aruco-board-config");
	parser.addOption(boardConfigOption);

	//write poses
	const QCommandLineOption writePosesOption(QStringList() << "w" << "write-poses", "Write the poses into the given file.", "file",
			QString("poses.txt"));
	parser.addOption(writePosesOption);

	//iterations
	const QCommandLineOption iterationsOption(QStringList() << "i" << "iterations", "Max number of iterations in the optimization.", "number",
			QString::number(100));
	parser.addOption(iterationsOption);

	//threshold
	const QCommandLineOption thresholdOption(QStringList() << "th" << "threshold", "Threshold (||delta||) to stop optimization.", "threshold",
			QString::number(1e-15));
	parser.addOption(thresholdOption);

	//automatic trajectory calculation
	const QCommandLineOption automaticOption(QStringList() << "a" << "automatic", "Automatic calculation of the robot trajectory."
			" Please provide the camera position in x,y,z coordinate, relative to the robot. Use a comma separated list and don't use spaces.",
			"camera-position");
	parser.addOption(automaticOption);

	//init
	QCommandLineOption initOption(QStringList() << "init", "Initialization values: (theta1, theta2, theta3, x, y, z, "
			"mee_x, mee_y, mee_z). theta_i are given in angle-axis representation! theta_i, x, y, and z determine the transformation "
			"from the robot base to the camera. mee is the translation between end-effector and marker. Provide a comma separated "
			"list and don't use spaces!",
			"t1,t2,t3,x,y,z,mx,my,mz");
	parser.addOption(initOption);

	//visualize
	const QCommandLineOption visualizeOption(QStringList() << "visualize", "Visualization: Sends marker array and start tf broadcaster "
			"to visualize the result in rviz.");
	parser.addOption(visualizeOption);

	//save calibration
	const QCommandLineOption saveCalibrationOption(QStringList() << "s" << "save-calibration", "Saves calibration to specified path",
			"Please provide path, where to save calibration", "cal_save_path");
	parser.addOption(saveCalibrationOption);

	const QCommandLineOption saveCalibrationTempOption(QStringList() << "save-temp-calibration", "Saves calibration to disk (calibration.mat "
			"in /tmp).");
	parser.addOption(saveCalibrationTempOption);

	parser.process(app);

	if (parser.isSet(markerSizeOption) && parser.isSet(boardConfigOption) && parser.isSet(robotOption))
	{
		double markerSize = parser.value(markerSizeOption).toDouble();

		lbr_camera_calibration::CameraCalibration calibration(argc, argv, markerSize);
		calibration.setBoardConfigFile(parser.value(boardConfigOption).toStdString());
		calibration.setRobot(parser.value(robotOption).toStdString());

		if (parser.isSet(readPosesOption))
		{
			calibration.setReadPosesFromFile(true);
			calibration.setPosesFileNameRead(parser.value(readPosesOption).toStdString());
		}
		else if (parser.isSet(testCaseOption))
		{
			calibration.setTestCase(true);
		}
		else if (parser.isSet(tOption))
		{
			calibration.setTrajectory(parser.value(tOption));
		}
		else if (parser.isSet(dOption))
		{
			calibration.setDirection(parser.value(dOption).toInt());
		}
		else if (parser.isSet(listenOption))
		{
			calibration.setListenToTopicsSeconds(parser.value(listenOption).toInt());
		}
		else if (parser.isSet(automaticOption))
		{
			QString value = parser.value(automaticOption);

			QStringList values = value.split(',');

			if (values.size() != 3)
			{
				ROS_ERROR("You need to specify 3 values for the camera position!");
				parser.showHelp(-1);
			}

			Eigen::VectorXd param(3);
			for (size_t i = 0; i < 3; ++i)
			{
				param(i) = values[i].toDouble();
			}

			calibration.setCameraPosition(param);
		}
		else
		{
			ROS_ERROR("Please set either -r, --test, -t, or -l!");
			parser.showHelp(-1);
		}

		if (parser.isSet(writePosesOption))
		{
			calibration.setWritePosesToFile(true);
			calibration.setPosesFileNameWrite(parser.value(writePosesOption).toStdString());
		}

		if (parser.isSet(iterationsOption))
		{
			calibration.setIterations(parser.value(iterationsOption).toInt());
		}

		if (parser.isSet(thresholdOption))
		{
			calibration.setThreshold(parser.value(thresholdOption).toDouble());
		}

		if (parser.isSet(cameraOption))
		{
			calibration.setCameraName(parser.value(cameraOption).toStdString());
		}

		if (parser.isSet(initOption))
		{
			QString value = parser.value(initOption);

			QStringList values = value.split(',');

			if (values.size() != 9)
			{
				ROS_ERROR("You need to specify 9 values for initialization.");
				parser.showHelp(-1);
			}

			Eigen::VectorXd param(9);
			for (size_t i = 0; i < 9; ++i)
			{
				param(i) = values[i].toDouble();
			}

			calibration.setParameters(param);
		}

		calibration.setPublishMarkerArray(parser.isSet(visualizeOption));

		if (parser.isSet(saveCalibrationOption))
		{
			calibration.setSaveCalibration(parser.value(saveCalibrationOption).toStdString());
			//calibration.setCalibrationResultPath(ros::package::getPath("lbr_camera_calibration"));
		}

		if (parser.isSet(saveCalibrationTempOption))
		{
			calibration.setSaveCalibration("/tmp/calibration.mat");
			//calibration.setCalibrationResultPath("/tmp/calibration.mat");
		}

		calibration.run();
	}
	else
	{
		ROS_ERROR("You need to specify the marker size, the board config file and the robot.");
		parser.showHelp(0);
	}

	return 0;
}

