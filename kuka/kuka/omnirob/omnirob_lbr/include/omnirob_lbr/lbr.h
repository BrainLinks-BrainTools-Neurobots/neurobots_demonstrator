/*
 * lbr.h
 *
 *  Created on: Jun 12, 2015
 *      Author: kuhnerd
 */

#ifndef KUKA_OMNIROB_LBR_LBR_H_
#define KUKA_OMNIROB_LBR_LBR_H_

struct LBRJoints
{
	static constexpr const char* base_x_joint = "base_x_joint";
	static constexpr const char* base_y_joint = "base_y_joint";
	static constexpr const char* base_theta_joint = "base_theta_joint";
	static constexpr const char* joint1 = "lbr_1_joint";
	static constexpr const char* joint2 = "lbr_2_joint";
	static constexpr const char* joint3 = "lbr_3_joint";
	static constexpr const char* joint4 = "lbr_4_joint";
	static constexpr const char* joint5 = "lbr_5_joint";
	static constexpr const char* joint6 = "lbr_6_joint";
	static constexpr const char* joint7 = "lbr_7_joint";

	static constexpr const char* joints[] = { joint1, joint2, joint3, joint4, joint5, joint6, joint7 };
};

#endif /* KUKA_OMNIROB_LBR_LBR_H_ */
