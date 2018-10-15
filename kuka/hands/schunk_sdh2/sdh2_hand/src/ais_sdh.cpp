#include <schunk_sdh_wrapper/ais_sdh.h>

using namespace SDH;

AisSdh::AisSdh(bool useRadians,
		bool useFahrenheit,
		int debugLevel) :
				cSDH(useRadians, useFahrenheit, debugLevel)
{
}

AisSdh::~AisSdh()
{
}

bool AisSdh::GraspCentrical(double velocity,
		double closeRatio)
{
	eGraspId graspID = eGID_CENTRICAL;
	return RunGrasp(graspID, velocity, closeRatio);
}

bool AisSdh::GraspCylindrical(double velocity,
		double closeRatio)
{
	eGraspId graspID = eGID_CYLINDRICAL;
	return RunGrasp(graspID, velocity, closeRatio);
}

bool AisSdh::GraspParallel(double velocity,
		double closeRatio)
{
	eGraspId graspID = eGID_PARALLEL;
	return RunGrasp(graspID, velocity, closeRatio);
}

bool AisSdh::GraspSpherical(double velocity,
		double closeRatio)
{
	eGraspId graspID = eGID_SPHERICAL;
	return RunGrasp(graspID, velocity, closeRatio);
}

bool AisSdh::Home(double velocity)
{
	try
	{
		SetController(eCT_POSE);
		SetAxisTargetVelocity(All, velocity);
		std::vector<int> axes = all_axes;
		std::vector<double> angles = zeros_v;
		SetAxisTargetAngle(axes, angles);
		MoveAxis(axes);
		return true;
	} catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << std::endl;
		return false;
	}

	return false;
}

bool AisSdh::RunGrasp(eGraspId graspId,
		double velocity,
		double closeRatio)
{
	try
	{
		SetController(eCT_POSE);
		GripHand(graspId, closeRatio, velocity, true);
		return true;
	} catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << std::endl;
		return false;
	}

	return false;
}

