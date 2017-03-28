/*********************************************************************************
 * Copyright (C) 2016 by Ignacio Perez                                           *
 *               https://github.com/Ignacio-Perez/                               *
 *                                                                               *
 * This file is part of LightPOMCP                                               *
 *                                                                               *
 *   LightPOMCP is free software: you can redistribute it and/or modify it       *
 *   under the terms of the GNU Lesser General Public License as published       *
 *   by the Free Software Foundation, either version 3 of the License, or        *
 *   (at your option) any later version.                                         *
 *                                                                               *
 *   LightPOMCP is distributed in the hope that it will be useful,               *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                *
 *   GNU Lesser General Public License for more details.                         *
 *                                                                               *
 *   You should have received a copy of the GNU Lesser General Public            *
 *   License along with LightPOMCP. If not, see <http://www.gnu.org/licenses/>.  *
 *********************************************************************************/

#include <iostream>

#include "ZmdpSimulator.hpp"
#include "Pomcp.hpp"

#define TIGER_FILE "../models/tiger.pomdp"
#define TIGER_PLANNING_TIME 0.9
#define TIGER_RESAMPLING_TIME 0.1
#define TIGER_THRESHOLD 0.01
#define TIGER_EXPLORATION_CTE 100
#define TIGER_PARTICLES 1000


using namespace pomcp;

bool tigerStopCondition(int s0, int a0, int z1, int s1)
{
	return a0!=0;
}

int main()
{
	ZmdpSimulator simulator(TIGER_FILE, tigerStopCondition);
	PomcpPlanner<int,int,int,pomcp::MultisetBelief<int>> planner(simulator,TIGER_PLANNING_TIME,TIGER_RESAMPLING_TIME,TIGER_THRESHOLD,TIGER_EXPLORATION_CTE,TIGER_PARTICLES);
	int s0,s1,z;
	unsigned a;
	double reward;
	simulator.sampleInitialState(s0);
	int counter=0;
	while (counter<100) {
		a = planner.getAction();
		
		simulator.simulate(s0,a,s1,z,reward,0);
		if (a==0) {
			std::cout<<"I'm going to hear..."<<std::endl;
			if (z==0) {
				std::cout<<"... I hear some grunts behind the left door..."<<std::endl;
			} else {
				std::cout<<"... I hear some scratchs at the right door..."<<std::endl;
			}
		}
		else {
			if (a==1) {
				std::cout<<"I'm going to open the left door..."<<std::endl;
			} else {
				std::cout<<"I'm going to open the right door..."<<std::endl;
			}
			if (reward==-100) {
				std::cout<<"AAAAH!!!! I'm tiger food!!!! :( :( :("<<std::endl;
			} else {
				std::cout<<"No tiger here!! I'm safe!! :)"<<std::endl;
			}
			std::cout<<"-------"<<std::endl;
			counter++;
		}

		planner.moveTo(a,z);
		s0 = s1;
	}
	return 0;
}
