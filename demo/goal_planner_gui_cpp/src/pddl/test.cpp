/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Nov 29, 2017
 *      Author: kuhnerd
 * 	  Filename: test.cpp
 */

#include <gtest/gtest.h>
#include <goal_planner_gui/pddl/utils.h>
#include <algorithm>
#include <vector>
#include <iostream>

// Declare a test
TEST(PYDDL_TEST, product)
{
	std::vector<std::vector<int>> vals =
			{
					{ 1, 2, 6 },
					{ 3, 4 },
					{ 7, 8 }
			};

	std::vector<std::vector<int>> wanted =
			{
					{ 1, 3, 7 },
					{ 1, 3, 8 },
					{ 1, 4, 7 },
					{ 1, 4, 8 },
					{ 2, 3, 7 },
					{ 2, 3, 8 },
					{ 2, 4, 7 },
					{ 2, 4, 8 },
					{ 6, 3, 7 },
					{ 6, 3, 8 },
					{ 6, 4, 7 },
					{ 6, 4, 8 }
			};


	std::vector<std::vector<int>> res;
	helpers::ProductVector<int>::compute(vals, res);

	for (auto& c: res) {
		std::cout << c[0] << " " << c[1] << std::endl;
	}

	EXPECT_TRUE(res.size() == wanted.size());
	for (size_t i = 0; i < wanted.size(); ++i)
	{
		EXPECT_TRUE(std::find(res.begin(), res.end(), wanted[i]) != res.end());
	}
}

// Run all the tests that were declared with TEST()
int main(int argc,
		char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
