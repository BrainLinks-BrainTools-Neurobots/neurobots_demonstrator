/*
 * definitions.h
 *
 *  Created on: Dec 6, 2013
 *      Author: sprunkc
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <stdint.h>

struct __attribute__ ((__packed__)) VelocityPacket {
	int counter;
	short mode;
	double v_x;
	double v_y;
	double v_theta;
	uint64_t timestamp_msecs;

	VelocityPacket() :
			mode(0 ) {
	}
};

struct __attribute__ ((__packed__)) JointPacket {
	int counter;
	short mode;
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;
	double j6;
	double j7;
	uint64_t timestamp_msecs;

	JointPacket() :
			mode(0) {
	}
};

struct __attribute__ ((__packed__)) CartesianPacket {
	int counter;
	short mode;
	double x;
	double y;
	double z;
	double alpha;
	double beta;
	double gamma;
	uint64_t timestamp_msecs;

	CartesianPacket() :
			mode(1) {
	}
};

struct __attribute__ ((__packed__)) JointStatePacket {
	int counter;
	short mode;
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;
	double j6;
	double j7;
	double t1;
	double t2;
	double t3;
	double t4;
	double t5;
	double t6;
	double t7;
	double forceFlangeX;
	double forceFlangeY;
	double forceFlangeZ;
	double torqueFlangeX;
	double torqueFlangeY;
	double torqueFlangeZ;
	uint64_t timestamp_msecs;

	JointStatePacket() :
			mode(3) {
	}
};

struct __attribute__ ((__packed__)) ActionFinishedPacket {
	short isFinished;
};

struct __attribute__ ((__packed__)) VelAccPacket {
	int counter;
	short mode;
	double velX;
	double velY;
	double velZ;
	double velTheta;
	double accX;
	double accY;
	double accZ;
	double accTheta;

	VelAccPacket() :
			counter(0), mode(3), velX(0), velY(0), velZ(0), velTheta(0), accX(0), accY(0), accZ(0), accTheta(0) {
	}
};

struct __attribute__ ((__packed__)) TrajectoryPacket {
	int counter;
	short mode;
	int pathLength;
	double joints[1050][7];
	//double joints[256][7];
	//double timeToStart[256][7];

	TrajectoryPacket() : mode(4){
	}
};

struct __attribute__ ((__packed__)) StopPacket {
	int counter;
	short mode;

	StopPacket() : mode(5){
	}
};

#endif /* DEFINITIONS_H_ */
