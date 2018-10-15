/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: constants.h
 */

#ifndef H225EC5A2_C693_433C_A652_40C55AEA080E
#define H225EC5A2_C693_433C_A652_40C55AEA080E

namespace goal_planner_gui
{

class Constants
{
public:
	Constants();
	virtual ~Constants();

	const static bool ALLOW_OTHER = true;
	const static int COMBINATION_OF_NO_BEST_PARTITIONS = 5;
	const static int MAX_SIZE_NEXT = 7;
};

} /* namespace goal_planner_gui */

#endif /* H225EC5A2_C693_433C_A652_40C55AEA080E */
