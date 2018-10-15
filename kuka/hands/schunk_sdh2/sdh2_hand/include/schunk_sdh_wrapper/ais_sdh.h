#ifndef AIS_SDH_H_
#define AIS_SDH_H_

#include <schunk_sdh/sdh.h>

class AisSdh: public SDH::cSDH
{
public:
	AisSdh(bool useRadians = false,
			bool useFahrenheit = false,
			int debugLevel = 0);
	virtual ~AisSdh();

	virtual bool GraspCentrical(double velocity,
			double closeRatio);

	virtual bool GraspCylindrical(double velocity,
			double closeRatio);

	virtual bool GraspParallel(double velocity,
			double closeRatio);

	virtual bool GraspSpherical(double velocity,
			double closeRatio);

	virtual bool Home(double velocity = 40.0);

private:
	virtual bool RunGrasp(eGraspId graspId,
			double velocity,
			double closeRatio);
};

#endif /* AIS_SDH_H_ */
