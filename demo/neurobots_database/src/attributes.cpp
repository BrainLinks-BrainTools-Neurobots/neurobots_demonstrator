/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 6, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: attributes.cpp
 */
#include <neurobots_database/attributes.h>

namespace neurobots_database
{
std::unordered_map<std::string, std::unordered_map<std::string, AttributeValues>> Attributes::defaultAttributes {
		{ "locatable",
				{
						{ "pose", { Eigen::Affine3d::Identity() } },
				}
		},
		{ "perceptible",
				{
						{ "seen", { bool(false) } }
				}
		}
};
}

