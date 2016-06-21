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

#ifndef _ZMDP_SIMULATOR_HPP_
#define _ZMDP_SIMULATOR_HPP_

#include <assert.h>
#include <vector>

#include <boost/functional/hash.hpp>
#include "MonteCarloSimulator.hpp"

#include "Random.hpp"
#include "Pomdp.h"

namespace pomcp
{
/**
 * class ZmdpProbs
 *
 * A class representing a probability distribution by using sla::cvector
 *
 * @author Ignacio Perez
 */
class ZmdpProbs 
{

public:
	ZmdpProbs(const sla::cvector& v);
	ZmdpProbs(const sla::cmatrix& m, unsigned row);
	~ZmdpProbs() {}
	int size() const {return probs.size();}
	double getProbability(int index) const {return probs(index);}
	int sample() const;
private:
	static bool comparator (const cvector_entry& a, const cvector_entry& b) { return a.value > b.value; }
	sla::cvector probs;
	sla::cvector sortedProbs;
	
};
/**
 * class ZmdpSimulator
 *
 * An implementation of Simulator<int,int,int> by using the Zmdp library for the POMDP definition
 * See https://github.com/trey0/zmdp for more information about the Zmdp library
 * See MontecarloSimulator.hpp for more information about the interface of the simulator
 *
 * @author Ignacio Perez
 */
class ZmdpSimulator : public Simulator<int,int,int>
{
public:
	/**
       	* Create a new simulator
	* @param file: The file defining the POMDP model to be used.
	* @param stopCondition: The stop condition. 
	* This function should return true if the problem is finished after executing action a0 in state s0, 
	* obtaining observation z1 and going to state s1.
     	*/
	ZmdpSimulator(const std::string& file, bool (*stopCondition)(int s0, int a0, int z1, int s1));
	virtual ~ZmdpSimulator();
	virtual double getDiscount() const {return model->discount;}
	virtual int& sampleInitialState(int& state) const;
	virtual bool simulate(const int& state, unsigned actionIndex, int& nextState, int& observation, double& reward) const;
	virtual bool simulate(const int& state, unsigned actionIndex, int& nextState, double& reward) const;
	virtual unsigned getNumActions() const {return actions.size();}
	virtual const int& getAction(unsigned actionIndex) const {return actions[actionIndex];}
	virtual bool allActionsAreValid(const int& state) const {return true;}
	virtual bool isValidAction(const int& state, unsigned actionIndex) const {return true;}
private:
	bool (*stopCondition)(int s0, int a0, int z1, int s1);
	std::vector<int> actions;
	zmdp::Pomdp* model;
	ZmdpProbs *initialBelief;
	std::vector<std::vector<ZmdpProbs*> > transitionProbs;
	std::vector<std::vector<ZmdpProbs*> > observationProbs;
};

inline
ZmdpProbs::ZmdpProbs(const sla::cmatrix& m, unsigned row)
{
	for (unsigned col=0; col< m.size2(); col++) {
		for (unsigned i=m.col_starts[col]; i<m.col_starts[col+1]; i++) {
			if (m.data[i].index>=row) {			
				if (m.data[i].index==row) {
					probs.data.emplace_back(col,m.data[i].value);
					sortedProbs.data.emplace_back(col,m.data[i].value);
				}
				break; 
			}
		}
	}
	std::sort(sortedProbs.data.begin(),sortedProbs.data.end(),comparator);
}

inline
ZmdpProbs::ZmdpProbs(const sla::cvector& v)
: probs(v),
  sortedProbs(v)
{
	std::sort(sortedProbs.data.begin(),sortedProbs.data.end(),comparator);
}

inline
int ZmdpProbs::sample() const
{
	double x = utils::RANDOM();
	double sum=0;
	for (auto it = sortedProbs.data.begin(); it!= sortedProbs.data.end(); ++it) {
		sum+=it->value;
		if (x<sum) {
			return it->index;
		}
	}
	return probs.data.back().index;	 	
}

inline
ZmdpSimulator::ZmdpSimulator(const std::string& file, bool (*stopCondition)(int s0, int a0, int z1, int s1))
: stopCondition(stopCondition)
{
	zmdp::ZMDPConfig config;

	config.setString("binaryName", file);
	config.setBool("useFastModelParser",false);
	config.setInt("maxHorizon",-1);
	config.setBool("useMaxPlanesMasking",false);
	config.setBool("useMaxPlanesSupportList",false);
	config.setBool("useMaxPlanesCache",true);
	config.setBool("useMaxPlanesExtraPruning",true);
	config.setBool("useSawtoothSupportList",false);
	config.setBool("useSawtoothCache",true);
	config.setBool("useSawtoothExtraPruning",true);
	model = new zmdp::Pomdp(file, &config);
	actions.resize(model->getNumActions());
	for (int i=0;i<model->getNumActions();i++) {
		actions[i]=i;
	}
	initialBelief = new ZmdpProbs(model->getInitialBelief());
	transitionProbs.resize(model->getNumActions());
	observationProbs.resize(model->getNumActions());
	for (int a = 0; a< model->getNumActions(); a++) {
		transitionProbs[a].resize(model->getBeliefSize());
		observationProbs[a].resize(model->getBeliefSize());
		for (int s = 0; s < model->getBeliefSize(); s++) {
			transitionProbs[a][s] = new ZmdpProbs(model->T[a], s);
			observationProbs[a][s] = new ZmdpProbs(model->O[a], s);	
		}
	} 
}

inline
ZmdpSimulator::~ZmdpSimulator()
{
	delete model;
	delete initialBelief;
	for (unsigned i=0; i< transitionProbs.size(); i++) {
		for (unsigned j=0; j< transitionProbs[i].size(); j++) {
			delete transitionProbs[i][j];
		}
	}
	for (unsigned i=0; i< observationProbs.size(); i++) {
		for (unsigned j=0; j< observationProbs[i].size(); j++) {
			delete observationProbs[i][j];
		}
	}
}

inline
int& ZmdpSimulator::sampleInitialState(int& state) const
{
	state = initialBelief->sample();
	return state;	
}

inline
bool ZmdpSimulator::simulate(const int& state, unsigned actionIndex, int& nextState, int& observation, double& reward) const
{
	const int& a = getAction(actionIndex);
	reward = model->R(state,a);
	nextState = transitionProbs[a][state]->sample();
	observation = observationProbs[a][nextState]->sample();
	return stopCondition(state,getAction(actionIndex),observation,nextState);
}

inline
bool ZmdpSimulator::simulate(const int& state, unsigned actionIndex, int& nextState, double& reward) const
{
	int observation;
	bool stop = simulate(state,actionIndex,nextState,observation,reward);
	return stop;
}

}
#endif
