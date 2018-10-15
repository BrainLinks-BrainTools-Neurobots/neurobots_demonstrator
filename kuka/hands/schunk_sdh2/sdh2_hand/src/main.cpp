#include <ros/ros.h>
#include <sdh2_hand/SDHAction.h>
#include <schunk_sdh_wrapper/ais_sdh.h>
#include <schunk_sdh/dsa.h>
#include <sensor_msgs/JointState.h>
#include <ros/callback_queue.h>
#include <vector>
#include <sdh2_hand/TactileSensor.h>
#include <schunk_sdh/tcpserial.h>

AisSdh sdh;
SDH::cDSA* dsa; //(0,"192.168.42.3");

//bool use_dsa = false;

sensor_msgs::JointState jointMsg;
sdh2_hand::TactileSensor tactileMsg;
int counter = 0;
ros::Publisher jointPublisher, tactilePublisher;
double lastDist;
int frameCounter;

bool moveHandTo(sdh2_hand::SDHAction::Request &req,
		sdh2_hand::SDHAction::Response &res);
void publishAngles();

bool runAction(sdh2_hand::SDHAction::Request &req,
		sdh2_hand::SDHAction::Response &res)
{
	bool result = false;
	result = moveHandTo(req, res);

	res.result = result;
	return true;
}

int positionAchieved(std::vector<double>* target,
		std::vector<bool>* stoppedFinger,
		int moveType)
{
	static const int dsa_reorder[6] = { 2, 3, 4, 5, 0, 1 };
	static const int dsaTosdh_reorder[6] = { 3, 4, 5, 6, 1, 2 };
	double dist = 0, dummy = 0;
	ros::Time times = ros::Time::now();
	ros::Time timee = ros::Time::now();
	std::vector<double> actualPose = sdh.GetAxisActualAngle(sdh.all_axes);
	for (size_t i = 0; i < target->size(); i++)
	{
		dummy = actualPose[i] - (*target)[i];
		dist += dummy * dummy;
	}

	// Get one frame from DSA controller
	try
	{
		dsa->SetFramerate(1, true, true);
		dsa->UpdateFrame();
		dsa->SetFramerate(0, true, false);
	}
	catch (SDH::cSDHLibraryException* e)
	{
		ROS_ERROR("An exception was caught: %s", e->what());
		delete e;
	}

	// Check if any object is blocking
	float intensity = 0;
	bool isBlocked = false;
	for (size_t i = 0; i < dsa->GetSensorInfo().nb_matrices; i++)
	{
		if (!stoppedFinger->at(dsaTosdh_reorder[i]))
		{
			for (size_t j = 0; j < dsa->GetMatrixInfo(dsa_reorder[i]).cells_x; j++)
			{
				for (size_t k = 0; k < dsa->GetMatrixInfo(dsa_reorder[i]).cells_y; k++)
				{
					intensity += dsa->GetTexel(dsa_reorder[i], j, k);
				}
			}

			if (intensity > 0.1)
			{
				switch (moveType)
				{
					case sdh2_hand::SDHAction::Request::NOSTOP:
						std::cout << "Object detected! But not stopping..." << std::endl;
						break;
					case sdh2_hand::SDHAction::Request::STOPHAND:
						std::cout << "Object detected! Hand stopped" << std::endl;
						sdh.Stop();
						return 1;
						break;
					case sdh2_hand::SDHAction::Request::STOPFINGER:
						{
						std::cout << "Object detected at finger " << dsa_reorder[i] << " ! Finger stopped at "
								<< sdh.GetAxisActualAngle(dsaTosdh_reorder[i])
								<< std::endl;
						stoppedFinger->at(dsaTosdh_reorder[i]) = true;
						isBlocked = true;
						target->at(dsaTosdh_reorder[i]) = sdh.GetAxisActualAngle(dsaTosdh_reorder[i]);
						break;
					}
				}
			}

			intensity = 0;
		}
	}

	if (isBlocked && sdh2_hand::SDHAction::Request::STOPFINGER == moveType)
	{
		sdh.SetAxisTargetAngle(sdh.all_axes, *target);
		sdh.MoveHand(false);
	}

	// Publish angles
	publishAngles();

	// Check if object has reached goal or didn't move for last 10 frames.
	if (dist < 0.0001)
	{
		return 1;
	}
	else if (frameCounter == 10)
	{
		sdh.SetAxisTargetAngle(sdh.all_axes, sdh.GetAxisActualAngle(sdh.all_axes));
		sdh.MoveHand(false);
		sdh.Stop();
		return 1;
	}
	else
	{
		if ((lastDist - dist) < 0.01)
		{
			frameCounter++;
		}
		lastDist = dist;
		return 0;
	}
}

bool moveHandTo(sdh2_hand::SDHAction::Request &req,
		sdh2_hand::SDHAction::Response &res)
{
	bool result = false;
	std::vector<double> gripPos, gripVelo;

	// Settings
//	sdh.UseRadians();
	sdh.SetController(sdh.eCT_POSE);
	sdh.SetVelocityProfile(sdh.eVP_RAMP);

	// Check if ratio is <0 oder >1 and if velocity is correct
	if (req.ratio > 1)
		req.ratio = 1;
	else if (req.ratio < 0)
		req.ratio = 0;

	// Init values
	lastDist = std::numeric_limits<double>::infinity();
	frameCounter = 0;

	switch (req.type)
	{
		case sdh2_hand::SDHAction::Request::CENTRICAL:
			try
			{
				// Fill proper values into vector
				/* Closed, ratio = 1
				 [ INFO] [1371201123.324176803]: angle number 0 of joint 1.047300
				 [ INFO] [1371201123.324202245]: angle number 1 of joint 0.122412
				 [ INFO] [1371201123.324224316]: angle number 2 of joint -0.122186
				 [ INFO] [1371201123.324247710]: angle number 3 of joint 0.122412
				 [ INFO] [1371201123.324269000]: angle number 4 of joint -0.122202
				 [ INFO] [1371201123.324291671]: angle number 5 of joint 0.122412
				 [ INFO] [1371201123.324314507]: angle number 6 of joint -0.122202
				 [ INFO] [1371201123.324336636]: angle number 7 of joint 0.000000*/

				/* Open, ratio = 0
				 [ INFO] [1371201615.728320438]: angle number 0 of joint 1.047300
				 [ INFO] [1371201615.728391089]: angle number 1 of joint -1.309181
				 [ INFO] [1371201615.728521142]: angle number 2 of joint 1.309035
				 [ INFO] [1371201615.728716069]: angle number 3 of joint -1.309181
				 [ INFO] [1371201615.728762582]: angle number 4 of joint 1.309115
				 [ INFO] [1371201615.728802307]: angle number 5 of joint -1.309181
				 [ INFO] [1371201615.728843233]: angle number 6 of joint 1.309083
				 [ INFO] [1371201615.728884465]: angle number 7 of joint 0.000000*/

				// Fill positions
				float dist = 1.047300 - 1.047300;
				gripPos.push_back(1.047300 + req.ratio * dist);

				dist = 0.122412 + 1.309181;
				gripPos.push_back(-1.309181 + req.ratio * dist);

				gripPos.push_back(0);
				//dist = -0.122186 - 1.309035;
				//gripPos.push_back(1.309035 + req.ratio * dist);

				dist = 0.122412 + 1.309181;
				gripPos.push_back(-1.309181 + req.ratio * dist);

				gripPos.push_back(0);
				//dist = -0.122202 - 1.309115;
				//gripPos.push_back(1.309115 + req.ratio * dist);

				dist = 0.122412 + 1.309181;
				gripPos.push_back(-1.309181 + req.ratio * dist);

				gripPos.push_back(0);
				//dist = -0.122202 - 1.309083;
				//gripPos.push_back(1.309083 + req.ratio * dist);

				dist = 0.000000 - 0.000000;
				gripPos.push_back(0.000000 + req.ratio * dist);

				// Fill velocities
				gripVelo = std::vector<double>(8, (double) req.velocity);
				sdh.SetAxisTargetVelocity(sdh.all_axes, gripVelo);

				// Execute move, bottom part first
				std::vector<bool> stoppedFinger(8, false);
				int positionValue = 0;
				ros::Rate r(100);
				// Execute only if gripType is not ClOSINGGRIP
				if (sdh2_hand::SDHAction::Request::CLOSINGGRIP != req.gripType)
				{
					sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
					sdh.MoveHand(false);

					// Check if move is finished
					ROS_INFO("Moving first part of finger...");

					// Values to check if position reached and if one finger is blocked
					while (ros::ok() && positionValue == 0)
					{
						positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
						r.sleep();
					}
					stoppedFinger = std::vector<bool>(8, false);
					lastDist = std::numeric_limits<double>::infinity();
					frameCounter = 0;
				}
				// Reset position value
				positionValue = 0;

				// Set other fingers to target position
				dist = -0.122186 - 1.309035;
				gripPos[2] = 1.309035 + req.ratio * dist;

				dist = -0.122202 - 1.309115;
				gripPos[4] = 1.309115 + req.ratio * dist;

				dist = -0.122202 - 1.309083;
				gripPos[6] = 1.309083 + req.ratio * dist;

				// Execute move, top part
				sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
				sdh.MoveHand(false);

				ROS_INFO("Moving second part of finger...");

				while (ros::ok() && positionValue == 0)
				{
					positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
					r.sleep();
				}

				result = true;
			}
			catch (std::exception& e)
			{
				std::cerr << "Exception: " << e.what() << std::endl;
				result = false;
			}
			break;
		case sdh2_hand::SDHAction::Request::CYLINDRICAL:
			try
			{
				/* Closed, ratio = 1
				 [ INFO] [1371202916.653637859]: angle number 0 of joint 0.000043
				 [ INFO] [1371202916.653674127]: angle number 1 of joint 0.000102
				 [ INFO] [1371202916.653710172]: angle number 2 of joint 1.309002
				 [ INFO] [1371202916.653828484]: angle number 3 of joint 0.000061
				 [ INFO] [1371202916.653870838]: angle number 4 of joint 1.309083
				 [ INFO] [1371202916.653909640]: angle number 5 of joint -0.000020
				 [ INFO] [1371202916.653949431]: angle number 6 of joint 1.308954
				 [ INFO] [1371202916.653987717]: angle number 7 of joint 0.000000*/

				/* Open, ratio = 0
				 [ INFO] [1371202790.037561296]: angle number 0 of joint 0.000051
				 [ INFO] [1371202790.037602424]: angle number 1 of joint -0.523742
				 [ INFO] [1371202790.037639029]: angle number 2 of joint 0.523701
				 [ INFO] [1371202790.037691157]: angle number 3 of joint -0.523742
				 [ INFO] [1371202790.037790601]: angle number 4 of joint 0.523540
				 [ INFO] [1371202790.037891686]: angle number 5 of joint -0.523783
				 [ INFO] [1371202790.037980927]: angle number 6 of joint 0.523523
				 [ INFO] [1371202790.038072750]: angle number 7 of joint 0.000000*/

				// Fill positions
				float dist = 0.000043 - 0.000051;
				gripPos.push_back(0.000051 + req.ratio * dist);

				dist = 0.000102 + 0.523742;
				gripPos.push_back(-0.523742 + req.ratio * dist);

				//dist = 1.309002 - 0.523701;
				gripPos.push_back(0);
				//gripPos.push_back(0.523701 + req.ratio * dist);

				dist = 0.000061 + 0.523742;
				gripPos.push_back(-0.523742 + req.ratio * dist);

				//dist = 1.309083 - 0.523540;
				gripPos.push_back(0);
				//gripPos.push_back(0.523540 + req.ratio * dist);

				dist = -0.000020 + 0.523783;
				gripPos.push_back(-0.523783 + req.ratio * dist);

				//dist = 1.308954 - 0.523523;
				gripPos.push_back(0);
				//gripPos.push_back(0.523523 + req.ratio * dist);

				dist = 0.000000 - 0.000000;
				gripPos.push_back(0.000000 + req.ratio * dist);

				// Fill velocities
				gripVelo = std::vector<double>(8, (double) req.velocity);
				sdh.SetAxisTargetVelocity(sdh.all_axes, gripVelo);

				// Execute move, bottom part first
				std::vector<bool> stoppedFinger(8, true);
				int positionValue = 0;
				ros::Rate r(100);
				// Execute only if gripType is not ClOSINGGRIP
				if (sdh2_hand::SDHAction::Request::CLOSINGGRIP != req.gripType)
				{
					sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
					sdh.MoveHand(false);

					// Check if move is finished
					ROS_INFO("Moving first part of finger...");

					// Values to check if position reached and if one finger is blocked
					while (ros::ok() && positionValue == 0)
					{
						positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
						r.sleep();
					}
					stoppedFinger = std::vector<bool>(8, false);
					lastDist = std::numeric_limits<double>::infinity();
					frameCounter = 0;
				}
				// Reset position value
				positionValue = 0;

				// Set other fingers to target position
				dist = 1.309002 - 0.523701;
				gripPos[2] = 0.523701 + req.ratio * dist;

				dist = 1.309083 - 0.523540;
				gripPos[4] = 0.523540 + req.ratio * dist;

				dist = 1.308954 - 0.523523;
				gripPos[6] = 0.523523 + req.ratio * dist;

				// Execute move, top part
				sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
				sdh.MoveHand(false);

				ROS_INFO("Moving second part of finger...");
				while (ros::ok() && positionValue == 0)
				{
					positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
					r.sleep();
				}

				result = true;
			}
			catch (std::exception& e)
			{
				std::cerr << "Exception: " << e.what() << std::endl;
				result = false;
			}
			break;
		case sdh2_hand::SDHAction::Request::PARALLEL:
			try
			{
				/* Closed, ratio = 1
				 [ INFO] [1371202698.132573058]: angle number 0 of joint 0.000086
				 [ INFO] [1371202698.132613864]: angle number 1 of joint 0.122330
				 [ INFO] [1371202698.132654017]: angle number 2 of joint -0.122234
				 [ INFO] [1371202698.132689688]: angle number 3 of joint 0.122309
				 [ INFO] [1371202698.132762090]: angle number 4 of joint -0.122234
				 [ INFO] [1371202698.132889502]: angle number 5 of joint 0.122309
				 [ INFO] [1371202698.132992024]: angle number 6 of joint -0.122218
				 [ INFO] [1371202698.133110206]: angle number 7 of joint 0.000000*/

				/* Open, ratio = 0
				 [ INFO] [1371202595.273564835]: angle number 0 of joint 0.000137
				 [ INFO] [1371202595.273595300]: angle number 1 of joint -1.308772
				 [ INFO] [1371202595.273651315]: angle number 2 of joint 1.309035
				 [ INFO] [1371202595.273683475]: angle number 3 of joint -1.308649
				 [ INFO] [1371202595.273721924]: angle number 4 of joint 1.309099
				 [ INFO] [1371202595.273750765]: angle number 5 of joint -1.309181
				 [ INFO] [1371202595.273775542]: angle number 6 of joint 1.308841
				 [ INFO] [1371202595.273800784]: angle number 7 of joint 0.000000 */

				// Fill positions
				float dist = 0.000086 - 0.000137;
				gripPos.push_back(0.000137 + req.ratio * dist);

				dist = 0.122330 + 1.308772;
				gripPos.push_back(-1.308772 + req.ratio * dist);

				//dist = -0.122234 - 1.309035;
				gripPos.push_back(0);
				//gripPos.push_back(1.309035 + req.ratio * dist);

				dist = 0.122309 + 1.308649;
				gripPos.push_back(-1.308649 + req.ratio * dist);

				//dist = -0.122234 - 1.309099;
				gripPos.push_back(0);
				//gripPos.push_back(1.309099 + req.ratio * dist);

				dist = 0.122309 + 1.309181;
				gripPos.push_back(-1.309181 + req.ratio * dist);

				//dist = -0.122218 - 1.308841;
				gripPos.push_back(0);
				//gripPos.push_back(1.308841 + req.ratio * dist);

				dist = 0.000000 - 0.000000;
				gripPos.push_back(0.000000 + req.ratio * dist);

				// Fill velocities
				gripVelo = std::vector<double>(8, (double) req.velocity);
				sdh.SetAxisTargetVelocity(sdh.all_axes, gripVelo);

				// Execute move, bottom part first
				std::vector<bool> stoppedFinger(8, false);
				int positionValue = 0;
				ros::Rate r(100);
				// Execute only if gripType is not ClOSINGGRIP
				if (sdh2_hand::SDHAction::Request::CLOSINGGRIP != req.gripType)
				{
					sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
					sdh.MoveHand(false);

					// Check if move is finished
					ROS_INFO("Moving first part of finger...");

					// Values to check if position reached and if one finger is blocked
					while (ros::ok() && positionValue == 0)
					{
						positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
						r.sleep();
					}
					stoppedFinger = std::vector<bool>(8, false);
					lastDist = std::numeric_limits<double>::infinity();
					frameCounter = 0;
				}
				// Reset position value
				positionValue = 0;

				// Set other fingers to target position
				dist = -0.122234 - 1.309035;
				gripPos[2] = 1.309035 + req.ratio * dist;

				dist = -0.122234 - 1.309099;
				gripPos[4] = 1.309099 + req.ratio * dist;

				dist = -0.122218 - 1.308841;
				gripPos[6] = 1.308841 + req.ratio * dist;

				// Execute move, top part
				sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
				sdh.MoveHand(false);

				ROS_INFO("Moving second part of finger...");
				while (ros::ok() && positionValue == 0)
				{
					positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
					r.sleep();
				}

				result = true;
			}
			catch (std::exception& e)
			{
				std::cerr << "Exception: " << e.what() << std::endl;
				result = false;
			}
			break;
		case sdh2_hand::SDHAction::Request::SPHERICAL:
			try
			{
				/* Closed, ratio = 1
				 [ INFO] [1371203054.537790507]: angle number 0 of joint 1.047137
				 [ INFO] [1371203054.537816955]: angle number 1 of joint -0.261636
				 [ INFO] [1371203054.537841615]: angle number 2 of joint 0.960030
				 [ INFO] [1371203054.537870269]: angle number 3 of joint -0.261615
				 [ INFO] [1371203054.537897571]: angle number 4 of joint 0.960094
				 [ INFO] [1371203054.537924501]: angle number 5 of joint -0.261615
				 [ INFO] [1371203054.537948235]: angle number 6 of joint 0.959981
				 [ INFO] [1371203054.537974816]: angle number 7 of joint 0.000000*/

				/* Open, ratio = 0
				 [ INFO] [1371203008.462596450]: angle number 0 of joint 1.047043
				 [ INFO] [1371203008.462641535]: angle number 1 of joint -0.698370
				 [ INFO] [1371203008.462680676]: angle number 2 of joint 0.698252
				 [ INFO] [1371203008.462769210]: angle number 3 of joint -0.698370
				 [ INFO] [1371203008.462834103]: angle number 4 of joint 0.698220
				 [ INFO] [1371203008.462898567]: angle number 5 of joint -0.698350
				 [ INFO] [1371203008.462962707]: angle number 6 of joint 0.698155
				 [ INFO] [1371203008.463028950]: angle number 7 of joint 0.000000*/

				// Fill positions
				float dist = 1.047137 - 1.047043;
				gripPos.push_back(1.047043 + req.ratio * dist);

				dist = -0.261636 + 0.698370;
				gripPos.push_back(-0.698370 + req.ratio * dist);

				//dist = 0.960030 - 0.698252;
				gripPos.push_back(0);
				//gripPos.push_back(0.698252 + req.ratio * dist);

				dist = -0.261615 + 0.698370;
				gripPos.push_back(-0.698370 + req.ratio * dist);

				//dist = 0.960094 - 0.698220;
				gripPos.push_back(0);
				//gripPos.push_back(0.698220 + req.ratio * dist);

				dist = -0.261615 + 0.698350;
				gripPos.push_back(-0.698350 + req.ratio * dist);

				//dist = 0.959981 - 0.698155;
				gripPos.push_back(0);
				//gripPos.push_back(0.698155 + req.ratio * dist);

				dist = 0.000000 - 0.000000;
				gripPos.push_back(0.000000 + req.ratio * dist);

				// Fill velocities
				gripVelo = std::vector<double>(8, (double) req.velocity);
				sdh.SetAxisTargetVelocity(sdh.all_axes, gripVelo);

				// Execute move, bottom part first
				std::vector<bool> stoppedFinger(8, true);
				int positionValue = 0;
				ros::Rate r(100);
				// Execute only if gripType is not ClOSINGGRIP
				if (sdh2_hand::SDHAction::Request::CLOSINGGRIP != req.gripType)
				{
					sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
					sdh.MoveHand(false);

					// Check if move is finished
					ROS_INFO("Moving first part of finger...");

					// Values to check if position reached and if one finger is blocked
					while (ros::ok() && positionValue == 0)
					{
						positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
						r.sleep();
					}
					stoppedFinger = std::vector<bool>(8, false);
					lastDist = std::numeric_limits<double>::infinity();
					frameCounter = 0;
				}
				// Reset position value
				positionValue = 0;

				// Set other fingers to target position
				dist = 0.960030 - 0.698252;
				gripPos[2] = 0.698252 + req.ratio * dist;

				dist = 0.960094 - 0.698220;
				gripPos[4] = 0.698220 + req.ratio * dist;

				dist = 0.959981 - 0.698155;
				gripPos[6] = 0.698155 + req.ratio * dist;

				// Execute move, top part
				sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
				sdh.MoveHand(false);

				ROS_INFO("Moving second part of finger...");
				while (ros::ok() && positionValue == 0)
				{
					positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
					r.sleep();
				}

				result = true;
			}
			catch (std::exception& e)
			{
				std::cerr << "Exception: " << e.what() << std::endl;
				result = false;
			}
			break;
		case sdh2_hand::SDHAction::Request::HOME:
			try
			{
				sdh.SetController(sdh.eCT_POSE);

				gripPos = std::vector<double>(8, 0);
				gripVelo = std::vector<double>(8, (double) req.velocity);

				// Fill velocities
//			for (size_t i = 0; i < 8; i++)

				// Execute move, bottom part first
				sdh.SetAxisTargetAngle(sdh.all_axes, gripPos);
				sdh.SetAxisTargetVelocity(sdh.all_axes, gripVelo);
				sdh.MoveHand(false);

				// Check if move is finished
				ros::Rate r(100);
				ROS_INFO("Moving into home position...");

				// Values to check if position reached and if one finger is blocked
				int positionValue = 0;
				std::vector<bool> stoppedFinger(8, false);
				while (ros::ok() && positionValue == 0)
				{
					positionValue = positionAchieved(&gripPos, &stoppedFinger, req.gripType);
					r.sleep();
				}

				result = true;
			}
			catch (std::exception& e)
			{
				std::cerr << "Exception: " << e.what() << std::endl;
				result = false;
			}
			break;
	}

	// In case of type: close until block
	if (req.gripType == sdh2_hand::SDHAction::Request::CLOSINGGRIP)
	{
		req.gripType = sdh2_hand::SDHAction::Request::STOPFINGER;
		req.ratio = 1;

		result = moveHandTo(req, res);
	}

	// Target achieved
	ROS_INFO("SDH2 has achieved target");
	return result;
}

// Angles in Radians, velocity in radians/sec?
void runActionWithoutBlock(sensor_msgs::JointState jointMsg)
{
	sdh.SetAxisTargetAngle(sdh.all_axes, jointMsg.position);
	sdh.SetAxisTargetVelocity(sdh.all_axes, jointMsg.velocity);
	sdh.MoveHand(false);
	ROS_INFO("SDH2 moving...");
	ros::Rate r(100);
	// Values to check if position reached and if one finger is blocked
	int positionValue = 0;
	std::vector<bool> stoppedFinger(8, false);
	while (ros::ok() && positionValue == 0)
	{
		positionValue = positionAchieved(&jointMsg.position, &stoppedFinger, 0);
		r.sleep();
	}

	ROS_INFO("SDH2 achieved target!");
}

void publishAngles()
{
	// Recieving angles
	//ROS_INFO("Get angles...");
	jointMsg.position = sdh.GetAxisActualAngle(sdh.all_axes);

	// Get tactile matrices and get actual frame
	int m, x, y;
	static const int dsa_reorder[6] = { 2, 3, 4, 5, 0, 1 };
	for (int i = 0; i < dsa->GetSensorInfo().nb_matrices; i++)
	{
		m = dsa_reorder[i];
		sdh2_hand::TactileMatrix &tm = tactileMsg.tactile_matrix[i];
		tm.matrix_id = i;
		tm.cells_x = dsa->GetMatrixInfo(m).cells_x;
		tm.cells_y = dsa->GetMatrixInfo(m).cells_y;
		tm.tactile_array.resize(tm.cells_x * tm.cells_y);

		for (y = 0; y < tm.cells_y; y++)
		{
			for (x = 0; x < tm.cells_x; x++)
			{
				tm.tactile_array[tm.cells_x * y + x] = dsa->GetTexel(m, x, y);
			}
		}

		// Debug
		//int counter = 0;
		/*if(m == 0)
		 {
		 std::stringstream backup;
		 for ( y = 0; y < tm.cells_y; y++ )
		 {
		 for ( x = 0; x < tm.cells_x; x++ )
		 {
		 //std::cout << " " << (dsa->GetTexel( m, x, y ) != 0 ? 1 : 0);
		 if(dsa->GetTexel(m, x, y) != 0)
		 counter++;
		 }
		 //std::cout << std::endl;
		 //backup << " \e[A";
		 }
		 //std::cout << counter << backup.str() << "\r";
		 //std::cout <<"counter: "<< counter << std::endl;
		 }*/
	}

	//ROS_INFO("Angles of joints:");
//	for (size_t i = 0; i < jointMsg.position.size(); i++)
//	{
//		jointMsg.position[i] = jointMsg.position[i] / 180.0 * M_PI;
////			ROS_INFO("angle number %d of joint %f", (int)i, jointMsg.position[i]);
//
//	}

	// Debug
	/*jointMsg.position.push_back(0.0);
	 jointMsg.position.push_back(0.0);
	 jointMsg.position.push_back(0.0);
	 jointMsg.position.push_back(0.0);
	 jointMsg.position.push_back(0.0);
	 jointMsg.position.push_back(0.0);
	 jointMsg.position.push_back(0.0);*/

	// Send joints
	jointMsg.header.stamp = ros::Time::now();
	jointMsg.header.frame_id = "1";
	jointMsg.header.seq = counter;

	// Complete tactile msg
	tactileMsg.header.stamp = ros::Time::now();
	tactileMsg.header.seq = counter;
	// Set missing joint
	jointMsg.position[7] = jointMsg.position[0];

	// Publish
	jointPublisher.publish(jointMsg);
	tactilePublisher.publish(tactileMsg);
}

int main(int argc,
		char **argv)
{
	ros::init(argc, argv, "schunk_sdh_wrapper");

	ROS_INFO("load parameters...");

	std::string ip;
	int port;
	ros::param::param<std::string>("/sdh2_hand/ip", ip, "192.168.42.3");
	ros::param::param<int>("/sdh2_hand/port", port, 6666);

	ROS_INFO("open tcp connection to sdh: ip: %s, port: %d", ip.c_str(), port);

	try
	{
		sdh.OpenTCP(ip.c_str(), port, -1);
		sdh.UseRadians();
		dsa = new SDH::cDSA(0, ip.c_str());
		dsa->SetFramerate(1, true, true);
	}
	catch (SDH::cTCPSerialException &e)
	{
		std::cout << e.what() << std::endl;
	}

	ROS_INFO("connected!");
	for (size_t i = 0; i < dsa->GetSensorInfo().nb_matrices; i++)
		std::cout << "Used touch sensivity of tactile " << i << ": " << dsa->GetMatrixSensitivity(i).cur_sens << std::endl;
	ros::NodeHandle n;

	// Publisher for joints and tactiles
	ros::ServiceServer actionService = n.advertiseService("sdh_action", runAction);
	jointPublisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000); //old topic: finger_states
	tactilePublisher = n.advertise<sdh2_hand::TactileSensor>("tactile_matrices", 1000);

	// Receive messages
	ros::Subscriber subGetAction = n.subscribe("sdh_action_without_block", 1000, runActionWithoutBlock);

	// Initialize msgs
	tactileMsg.tactile_matrix.resize(dsa->GetSensorInfo().nb_matrices);

	// Names of joints
	// Finger 1 rotate
	jointMsg.name.push_back("sdh2_finger_21_joint");

	// Finger 1 whole
	jointMsg.name.push_back("sdh2_finger_22_joint");

	// Finger 1 Top
	jointMsg.name.push_back("sdh2_finger_23_joint");

	// Finger 2 whole
	jointMsg.name.push_back("sdh2_thumb_2_joint");

	// Finger 2 top
	jointMsg.name.push_back("sdh2_thumb_3_joint");

	// Finger 3 whole
	jointMsg.name.push_back("sdh2_finger_12_joint");

	// Finger 3 top
	jointMsg.name.push_back("sdh2_finger_13_joint");

	// Finger 3 rotate = Finger 1 rotate
	jointMsg.name.push_back("sdh2_knuckle_joint");

	// Debug
	/*jointMsg.name.push_back("lwr_link1");
	 jointMsg.name.push_back("lwr_link2");
	 jointMsg.name.push_back("lwr_link3");
	 jointMsg.name.push_back("lwr_link4");
	 jointMsg.name.push_back("lwr_link5");
	 jointMsg.name.push_back("lwr_link6");
	 jointMsg.name.push_back("lwr_link7");*/

	ros::Rate r(100);
//	ros::AsyncSpinner spinner(2);
//	spinner.start();

	while (ros::ok())
	{
		// Angles of joints
		try
		{
			publishAngles();
			dsa->SetFramerate(1, true, true);
			dsa->UpdateFrame();
			dsa->SetFramerate(0, true, false);
		}
		catch (SDH::cSDHLibraryException* e)
		{
			ROS_ERROR("An exception was caught: %s", e->what());
			delete e;
		}
		ros::spinOnce();
		r.sleep();
		counter++;
	}
	return 0;
}

