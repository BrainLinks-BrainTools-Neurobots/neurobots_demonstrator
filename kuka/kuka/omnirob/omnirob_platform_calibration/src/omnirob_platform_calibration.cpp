/*
 * omnirob_platform_calibration.h
 *
 * Copyright (C) 2015 Daniel Kuhner (kuhnerd@informatik.uni-freiburg.de)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

#include <omnirob_platform_calibration/camera_calibration.h>
#include <qcommandlineparser.h>
#include <qcoreapplication.h>
#include <ros/package.h>

using namespace aruco;

int main(int argc,
		char** argv)
{
	QCoreApplication app(argc, argv);
	QCoreApplication::setApplicationName("omnirob_platform_calibration");
	QCoreApplication::setApplicationVersion("1.0");

	//parser
	QCommandLineParser parser;
	parser.setApplicationDescription("Camera calibration tool to estimate the "
			"transformation between an external camera and the robot. There are "
			"2 different ways to use the calibration tool:\n"
			"  1. -r <file>: Reads the marker and end-effector poses from a file, "
			"which was stored to disk from the calibration tool using the -w flag.\n"
			"  2. --test: Runs a simple test case.\n");
	parser.addHelpOption();
	parser.addVersionOption();

	//read from poses
	const QCommandLineOption readPosesOption(QStringList() << "r" << "read-poses", "Read poses from file.", "file", QString("poses.txt"));
	parser.addOption(readPosesOption);

	//direction of the calibration: map to camera (1) or camera to map (2)
	const QCommandLineOption dOption(QStringList() << "d" << "direction", "In which direction is calibrated?"
			"1: map to camera or 2: camera to map", "direction_number", QString::number(1));
	parser.addOption(dOption);

	//read from poses
	const QCommandLineOption cameraPrefixOption(QStringList() << "c" << "camera_prefix", "Camera prefix name.", "camera_prefix", QString(""));
	parser.addOption(cameraPrefixOption);

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

	const QCommandLineOption calibrationPatternIdOption(QStringList() << "calibration-pattern",
			"Calibration pattern id (0 - default, 1 - camera upside down)");
	parser.addOption(calibrationPatternIdOption);

	parser.process(app);

	if (parser.isSet(markerSizeOption) && parser.isSet(boardConfigOption))
	{
		double markerSize = parser.value(markerSizeOption).toDouble();

		omnirob_platform_calibration::CameraCalibration calibration(argc, argv, markerSize);
		calibration.setBoardConfigFile(parser.value(boardConfigOption).toStdString());

		if (parser.isSet(readPosesOption))
		{
			calibration.setReadPosesFromFile(true);
			calibration.setPosesFileNameRead(parser.value(readPosesOption).toStdString());
		}
		else if (parser.isSet(testCaseOption))
		{
			calibration.setTestCase(true);
		}
		else
		{
//			ROS_ERROR("Please set either -r, --test!");
//			parser.showHelp(-1);
		}

		if (parser.isSet(writePosesOption))
		{
			calibration.setWritePosesToFile(true);
			calibration.setPosesFileNameWrite(parser.value(writePosesOption).toStdString());
		}

		if (parser.isSet(cameraPrefixOption))
		{
			calibration.setCameraPrefix(parser.value(cameraPrefixOption).toStdString());
		}

		if (parser.isSet(iterationsOption))
		{
			calibration.setIterations(parser.value(iterationsOption).toInt());
		}

		if (parser.isSet(dOption))
		{
			calibration.setDirection(parser.value(dOption).toInt());
		}

		if (parser.isSet(thresholdOption))
		{
			calibration.setThreshold(parser.value(thresholdOption).toDouble());
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
			//calibration.setCalibrationResultPath(ros::package::getPath("omnirob_platform_calibration"));
		}

		if (parser.isSet(saveCalibrationTempOption))
		{
			calibration.setSaveCalibration("/tmp/calibration.mat");
			//calibration.setCalibrationResultPath("/tmp/calibration.mat");
		}

		if (parser.isSet(calibrationPatternIdOption))
		{
			calibration.setCalibrationPatternId(parser.value(calibrationPatternIdOption).toInt());
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

