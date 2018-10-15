/*
 * iiwa.h
 *
 *  Created on: Nov 04, 2015
 *      Author: killmani
 */

#ifndef KUKA_IIWA_H_
#define KUKA_IIWA_H_

struct IIWAJoints
{
	static constexpr const char* joint1 = "iiwa_1_joint";
	static constexpr const char* joint2 = "iiwa_2_joint";
	static constexpr const char* joint3 = "iiwa_3_joint";
	static constexpr const char* joint4 = "iiwa_4_joint";
	static constexpr const char* joint5 = "iiwa_5_joint";
	static constexpr const char* joint6 = "iiwa_6_joint";
	static constexpr const char* joint7 = "iiwa_7_joint";

	static constexpr const char* joints[] = { joint1, joint2, joint3, joint4, joint5, joint6, joint7 };
};

#endif /* KUKA_IIWA_H_ */
