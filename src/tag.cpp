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

#define TAG_ACTION_NORTH 0
#define TAG_ACTION_SOUTH 1
#define TAG_ACTION_EAST  2
#define TAG_ACTION_WEST  3
#define TAG_ACTION_CATCH 4

#define TAG_FILE "../models/tag.pomdp"
#define TAG_PLANNING_TIME 0.9
#define TAG_RESAMPLING_TIME 0.1
#define TAG_THRESHOLD 0.01
#define TAG_EXPLORATION_CTE 100
#define TAG_PARTICLES 1000

using namespace pomcp;

inline
bool tagStopCondition(int s0, int a0, int z1, int s1)
{
	return a0==TAG_ACTION_CATCH && (s1+1)%30 == 0;
}

int getRobotVerticalObservation(int z)
{
	if (z<10) {
		return 4;
	}
	if (z<20) {
		return 3;
	}
	if (z<23) {
		return 2;
	}
	if (z<26) {
		return 1;
	}
	return 0;
}

int getRobotHorizontalObservation(int z)
{
	if (z<20) {
		return z%10;
	}
	if (z<23) {
		return z-15;
	}
	if (z<26) {
		return z-18;
	}
	return z-21;
}

int getTargetVerticalState(int s)
{
	return getRobotVerticalObservation(s%30);
}

int getTargetHorizontalState(int s)
{
	return getRobotHorizontalObservation(s%30);

}

int getRobotVerticalState(int s)
{
	return getRobotVerticalObservation(s/30);
}

int getRobotHorizontalState(int s)
{
	return getRobotHorizontalObservation(s/30);
}

void printTag(int s0,int a0, int z1, double reward)
{
	int th = getTargetHorizontalState(s0);
	int tv = getTargetVerticalState(s0);
	int rh = getRobotHorizontalState(s0);
	int rv = getRobotVerticalState(s0);
	char robot='R';
	if (th==rh && tv==rv) {
		robot='&';
	}
	for (int v = 0; v< 5; v++) {
		for (int h = 0;h<10; h++) {
			if (rh==h && rv==v) {
				std::cout<<robot;
			} else if (th==h && tv==v) {
				std::cout<<"T";
			} else	if (v>2 || (h>4 && h<8)) {
				std::cout<<"·";
			} else {

				std::cout<<" ";
			}
		}
		std::cout<<std::endl;
	}
	std::cout<<"Action: ";
	switch(a0) {
		case 0:
			std::cout<<"NORTH"<<std::endl;
		break;

		case 1:
			std::cout<<"SOUTH"<<std::endl;
		break;

		case 2:
			std::cout<<"EAST"<<std::endl;
		break;

		case 3:
			std::cout<<"WEST"<<std::endl;
		break;

		case 4:
			std::cout<<"CATCH"<<std::endl;
		break;

	}
	std::cout<<"Reward: "<<reward<<std::endl;
}

int main()
{
	ZmdpSimulator simulator(TAG_FILE, tagStopCondition);
	PomcpPlanner<int,int,int,pomcp::MultisetBelief<int>> planner(simulator,TAG_PLANNING_TIME,TAG_RESAMPLING_TIME,TAG_THRESHOLD,TAG_EXPLORATION_CTE,TAG_PARTICLES);

	int s0,a0,s1,z1;
	double r,reward,discount;
	double sum=0;
	unsigned depth,size;
	for(int i=1;i<=100;i++) {
		std::cout<<"Simulation "<<i<<std::endl;
		simulator.sampleInitialState(s0);
		a0 = planner.getAction();
		simulator.simulate(s0,a0,s1,z1,r);
		discount = 1.0;
		reward = r;
		while(!tagStopCondition(s0,a0,z1,s1)) {
			printTag(s0,a0,z1,reward);
			planner.computeInfo(size,depth);
			std::cout<<"Size: "<<size<<std::endl;
			std::cout<<"Depth: "<<depth<<std::endl;
			planner.moveTo(a0,z1);
			s0 = s1;
			a0 = planner.getAction();
			simulator.simulate(s0,a0,s1,z1,r);
			discount *= simulator.getDiscount();
			reward += discount*r;
		}
		sum+=reward;
		printTag(s0,a0,z1,reward);	
		planner.computeInfo(size,depth);
		std::cout<<"Size: "<<size<<std::endl;
		std::cout<<"Depth: "<<depth<<std::endl;
		std::cout<<"Average reward: "<<sum/(double)i<<std::endl;		
		planner.reset();
	}
	return 0;
}
