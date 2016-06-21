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

#ifndef _POMCP_HPP_
#define _POMCP_HPP_

#include <vector>
#include <unordered_map>
#include <cmath>
#include <boost/functional/hash.hpp>

#ifdef _POMCP_DEBUG_
#include <assert.h>
#endif

#include "MonteCarloSimulator.hpp"
#include "Timer.hpp"
#include "Random.hpp"
#include "Multiset.hpp"

namespace pomcp
{
/**
 * struct Edge<Z>
 *
 * A structure for the edges of the search tree.
 * Each edge contains the pair <actionIndex,observation> connecting beliefs
 *
 * Z is the type for observations
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<Z>
 *	   bool Z::operator ==(const Z& other) const
 *
 * @author Ignacio Perez
 */
template<typename Z>
struct Edge
{
	Edge(unsigned actionIndex, const Z& observation) : actionIndex(actionIndex), observation(observation) {}
	unsigned actionIndex;
	Z observation;
	bool operator ==(const Edge<Z>& other) const {return actionIndex==other.actionIndex && observation==other.observation;}
};
/**
 * class EdgeHasher<Z>
 *
 * A Hasher implementing the operator() in order to use the structure Edge<Z> as key for std::unordered_map
 *
 * @author Ignacio Perez
 */
template<typename Z>
struct EdgeHasher
{
	std::size_t operator()(const Edge<Z>& edge) const
	{
		using boost::hash_value;
		using boost::hash_combine;
		static std::hash<Z> observationHasher;		
		std::size_t seed = 0;
		hash_combine(seed,hash_value(edge.actionIndex));
		hash_combine(seed,observationHasher(edge.observation));
		return seed;
	}
};

/**
 * class ActionData
 *
 * A structure containing information related to the actions that can be executed from a node of the search tree
 *
 * @author Ignacio Perez
 */
struct ActionData
{
	ActionData() : counter(0), value(0) {}
	virtual ~ActionData() {}
	unsigned counter;
	double value;
};

/**
 * class Node<S,Z>
 *
 * A structure representing the nodes of the search Tree
 *
 * S is the type for states
 * Z is the type for observations
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<S>
 * 	   std::hash<Z>
 *	   bool S::operator ==(const S& other) const
 *	   bool Z::operator ==(const Z& other) const
 *
 * @author Ignacio Perez
 */
template <typename S, typename Z>
struct Node 
{
	Node() : counter(0) {}
	virtual ~Node() {}
	unsigned counter;
	utils::Multiset<S> particles;
	std::vector<ActionData> actionData;	
	std::unordered_map<Edge<Z>,Node*,EdgeHasher<Z> > childs;
};

/**
 * class PomcpPlanner<S,Z,A>
 *
 * Implementation of the POMCP algorithm
 *
 * S is the type for states
 * Z is the type for observations
 * A is the type for actions
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<S>
 * 	   std::hash<Z>
 *	   bool S::operator ==(const S& other) const
 *	   bool Z::operator ==(const Z& other) const
 *
 * @author Ignacio Perez
 */

template <typename S, typename Z, typename A>
class PomcpPlanner 
{
public:
        /**
     	 * Create a new POMCP planner
	 * @param Simulator<S,Z,A>& simulator: A reference to the Simulator<S,Z,A> to be used, see MonteCarlo.hpp
	 * @param timeout: Planning time in seconds
	 * @param threshold: Discount factor threshold to be used, the algorithm does not expand the tree at a
         *                   given depth if discount_factor^depth is less than the threshold.
	 *	             See the POMCP paper for more information
	 * @param explorationConstant: Exploration constant, see the POMCP paper for more information.
     	 */
	PomcpPlanner(Simulator<S,Z,A>& simulator, double timeout, double threshold, double explorationConstant);
	virtual ~PomcpPlanner()
	{
		simulator.cleanup();
		eraseTree(root);
	}
	/**
     	 * Get the index of the best action to be executed at the current belief
	 * Note: The planning algorithm will be called if this is the first time the method is called or 
	 *       the current belief has been updated by moveTo(action,observation)
	 * @return the index of the best action to be executed
     	 */
	virtual unsigned getAction();
	/**
     	 * Update the current belief
	 * @param actionIndex: The index of the last executed action
	 * @param observation: The obtained observation
	 */
	virtual void moveTo(unsigned actionIndex, const Z& observation);
	/**
     	 * Reset the planner to the initial belief
	 */
	virtual void reset();
	/**
     	 * Get the size (number of nodes) of the tree
	 * @return the size of the tree
     	 */
	virtual std::size_t size() const {return size_;}

private:
	void search();
	double simulate(const S& state, Node<S,Z>* node, double depth);
	double rollout(const S& state, double depth);
	unsigned getRolloutAction(const S& state);	
	Node<S,Z>* getNode(Node<S,Z>* parent, unsigned actionIndex, const Z& observation);
	void eraseTree(Node<S,Z>* node);
	void getRootParticles();
	
	Simulator<S,Z,A>& simulator;

	double timeout;
	double threshold;
	double explorationConstant;
	unsigned currentAction;
	std::size_t size_;
	Node<S,Z> *root;
	std::vector<const S*> rootParticles;
	std::vector<unsigned> actionIndexes;
};

template <typename S, typename Z, typename A>	
inline
PomcpPlanner<S,Z,A>::PomcpPlanner(Simulator<S,Z,A>& simulator, double timeout, double threshold, double explorationConstant)
: simulator(simulator),
  timeout(timeout),
  threshold(threshold),
  explorationConstant(explorationConstant),
  currentAction(simulator.getNumActions()),
  size_(1),
  root(new Node<S,Z>())
{
	actionIndexes.resize(simulator.getNumActions());
}

template <typename S, typename Z, typename A>
inline
void PomcpPlanner<S,Z,A>::reset()
{
	currentAction = simulator.getNumActions();
	simulator.cleanup();
        rootParticles.clear();
	eraseTree(root);
	#ifdef _POMCP_DEBUG_
	assert(size_==0);
	#endif
        size_=1;
	root = new Node<S,Z>();
}

template <typename S, typename Z, typename A>
inline
Node<S,Z>* PomcpPlanner<S,Z,A>::getNode(Node<S,Z>* parent, unsigned actionIndex, const Z& observation)
{
	Edge<Z> edge(actionIndex,observation);
	auto it = parent->childs.find(edge);
	if(it == parent->childs.end()) {
		size_++;
		Node<S,Z>* node = new Node<S,Z>();
		parent->childs[edge] = node;
		return node;
	}
	return it->second;
}

template<typename S, typename Z, typename A>
inline
unsigned PomcpPlanner<S,Z,A>::getRolloutAction(const S& state)
{
	if (simulator.allActionsAreValid(state)) {
		return utils::RANDOM(simulator.getNumActions());
	}
	unsigned n=0;
	for (unsigned a=0;a<simulator.getNumActions();a++) {
		if (simulator.isValidAction(state,a)) {
			actionIndexes[n++]=a;
		}
	}
	return actionIndexes[utils::RANDOM(n)];
}

template <typename S, typename Z, typename A>
inline
double PomcpPlanner<S,Z,A>::rollout(const S& state, double depth)
{
	if (depth < threshold) {
		return 0;
	}
	unsigned action = getRolloutAction(state);
	double reward;
	S nextState;
	bool stop = simulator.simulate(state, action, nextState, reward);
	if (!stop) {
		reward += simulator.getDiscount() * rollout(nextState, depth*simulator.getDiscount()); 
	}
	return reward; 
}

template <typename S, typename Z, typename A>
inline
double PomcpPlanner<S,Z,A>::simulate(const S& state, Node<S,Z>* node, double depth)
{
	if (depth < threshold) {
		return 0;
	}
	if (node->actionData.empty()) {
		node->actionData.resize(simulator.getNumActions());
		return rollout(state, depth);
	}
	unsigned actionIndex = simulator.getNumActions();
	if (node->counter == 0) {
		actionIndex = getRolloutAction(state);
	} else {
		double aux, max = 0;
		for (unsigned a = 0; a<simulator.getNumActions(); a++) {
			if (!simulator.isValidAction(state,a)) {
				continue;			
			}
			if (node->actionData[a].counter == 0 && explorationConstant > 0.0) {
				actionIndex = a;
				break;
			}
			aux = node->actionData[a].value;
			if (explorationConstant > 0.0) {
				aux += explorationConstant * 
					std::sqrt(std::log((double)(node->counter))/(double)(node->actionData[a].counter));
			}
			if (actionIndex == simulator.getNumActions() || aux > max) {
				actionIndex = a;
				max = aux;
			}		
		}
	}
	#ifdef _POMCP_DEBUG_
	assert(actionIndex != simulator.getNumActions());
	#endif
	Z observation;
	double reward;
	S nextState;
	bool stop  = simulator.simulate(state, actionIndex, nextState, observation, reward);
	
	if (!stop) {
		Node<S,Z>* nextNode = getNode(node,actionIndex,observation);
		nextNode->particles.add(nextState);
		reward += simulator.getDiscount() * simulate(nextState, nextNode , depth*simulator.getDiscount());
	} 
	node->counter++;
	node->actionData[actionIndex].counter++;
	node->actionData[actionIndex].value += 
			(reward - node->actionData[actionIndex].value)/(double)(node->actionData[actionIndex].counter);	
	return reward;
}

template<typename S, typename Z, typename A>
inline
void PomcpPlanner<S,Z,A>::search()
{
	if (rootParticles.empty()) {
		S s0;
		utils::Timer timer;
		do {
			simulate(simulator.sampleInitialState(s0),root,1.0);
		} while (timer.elapsed()<timeout);
	} else {
		utils::Timer timer;
		do {
			simulate(*rootParticles[utils::RANDOM(rootParticles.size())],root,1.0);
		} while (timer.elapsed()<timeout);
	}
	double aux=0;
	for (unsigned a = 0; a<simulator.getNumActions(); a++) {
		if (root->actionData[a].counter==0) {
			continue;
		}
		if (currentAction == simulator.getNumActions() || root->actionData[a].value > aux) {
			currentAction = a;
			aux = root->actionData[a].value;

		}

	}
	#ifdef _POMCP_DEBUG_
	assert(currentAction != simulator.getNumActions());
	#endif
}

template<typename S, typename Z, typename A>
inline
unsigned PomcpPlanner<S,Z,A>::getAction()
{
	if (currentAction==simulator.getNumActions()) {
		search();
	}
	return currentAction;
}

template<typename S, typename Z, typename A>
inline
void PomcpPlanner<S,Z,A>::eraseTree(Node<S,Z>* node)
{
	for (auto it = node->childs.begin(); it!=node->childs.end(); ++it) {
		eraseTree(it->second);
	}
	size_--;
	delete node;
}

template<typename S, typename Z, typename A>
inline
void PomcpPlanner<S,Z,A>::getRootParticles()
{
	rootParticles.resize(root->particles.size());
	unsigned counter=0;
	for (auto it = root->particles.data().begin(); it != root->particles.data().end(); ++it) {
		for (unsigned i = 0; i< it->second; i++) {
			rootParticles[counter++] = &(it->first);
		}
	}
}

template<typename S, typename Z, typename A>
inline
void PomcpPlanner<S,Z,A>::moveTo(unsigned actionIndex, const Z& observation)
{
	Edge<Z> edge(actionIndex,observation);
	auto it = root->childs.find(edge);
	if (it == root->childs.end() ||
		it->second->particles.size()==0) {
		reset();
	} else {
		Node<S,Z>* nextRoot = it->second;
		for (auto it1 = root->childs.begin(); it1!= root->childs.end(); ++it1) {
			if (!(it1->first == edge)) {
				eraseTree(it1->second);
			} 
		}
		size_--;
		delete root;
		root = nextRoot;
		getRootParticles();
		currentAction = simulator.getNumActions();
		simulator.cleanup();
	}
}

}
#endif
