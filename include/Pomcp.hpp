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
#include <boost/thread.hpp>

#define _POMCP_DEBUG_
#ifdef _POMCP_DEBUG_
#include <assert.h>
#endif

#include "MonteCarloSimulator.hpp"
#include "Timer.hpp"
#include "Random.hpp"
#include "Multiset.hpp"
#include "Belief.hpp"

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
 * class Node<S,Z,B>
 *
 * A structure representing the nodes of the search Tree
 *
 * S is the type for states
 * Z is the type for observations
 * B is the type for the belief (inherit from pomcp::Belief<S>)
 *   tip: Try pomcp::VectorBelief<S> if you have a continous state space (default)
 *        Try pomcp::MultisetBelief<S> if you have a discrete state space 
 *        Compare (profile) different implementations
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<S> (if you use a belief based on hash function)
 * 	   std::hash<Z>
 *	   bool S::operator ==(const S& other) const (if you use a belief based on hash function)
 *	   bool Z::operator ==(const Z& other) const
 *
 * @author Ignacio Perez
 */
template <typename S, typename Z, typename B>
struct Node 
{
	Node() : counter(0) {}
	virtual ~Node() {}
	unsigned counter;
	B belief;
	std::vector<ActionData> actionData;	
	std::unordered_map<Edge<Z>,Node*,EdgeHasher<Z> > childs;
};

/**
 * class PomcpPlanner<S,Z,A,B>
 *
 * Implementation of the POMCP algorithm
 *
 * S is the type for states
 * Z is the type for observations
 * A is the type for actions
 * B is the type for the belief (inherit from pomcp::Belief<S>)
 *   tip: Try pomcp::VectorBelief<S> if you have a continous state space (default)
 *        Try pomcp::MultisetBelief<S> if you have a discrete state space 
 *        Compare (profile) different implementations
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<S> (if you use a belief based on hash function)
 * 	   std::hash<Z>
 *	   bool S::operator ==(const S& other) const (if you use a belief based on hash function)
 *	   bool Z::operator ==(const Z& other) const
 *
 * @author Ignacio Perez
 */

template <typename S, typename Z, typename A, typename B=VectorBelief<S>>
class PomcpPlanner 
{
public:
        /**
     	 * Create a new POMCP planner
	 * @param Simulator<S,Z,A>& simulator: A reference to the Simulator<S,Z,A> to be used, see MonteCarlo.hpp
	 * @param plannigTimeout: Planning time in seconds
	 * @param resamplingTimeout: Time in seconds to resampling particles and increase the beliefe size after moving in the tree
	 * @param threshold: Discount factor threshold to be used, the algorithm does not expand the tree at a
         *                   given depth if discount_factor^depth is less than the threshold.
	 *	             See the POMCP paper for more information
	 * @param explorationConstant: Exploration constant, see the POMCP paper for more information.
	 * @param particlesInitialBelief: number of particles for the initial belief in the tree. The planner will sample until this
	 *			number is reached, or resamplingTimeout.
     	 */
	PomcpPlanner(Simulator<S,Z,A>& simulator, double planningTimeout, double resamplingTimeout, double threshold, double explorationConstant, unsigned particlesInitialBelief);
	virtual ~PomcpPlanner()
	{
		simulator.cleanup();
		eraseTree(root);
	}
	/**
     	 * Get the index of the best action to be executed at the current belief
	 * Note: The planning algorithm will be called if the shouldSearch argument is true
	 * @return the index of the best action to be executed
     	 */
	virtual unsigned getAction(bool shouldSearch=true);
	/**
     	 * Update the current belief
	 * @param actionIndex: The index of the last executed action
	 * @param observation: The obtained observation
	 * @return true if the planner has been reseted, false otherwise
	 */
	virtual bool moveTo(unsigned actionIndex, const Z& observation);
	/**
     	 * Reset the planner to the initial belief
	 */
	virtual void reset();
	/**
	 * Get the current belief 
	 * @return the current belief
	 */
	virtual const B& getCurrentBelief() const {return root->belief;}

	/**
	 * Compute the current size (number of nodes) and tree depth
	 */
	virtual void computeInfo(unsigned& size, unsigned& depth) const
	{
		size=0;
		depth = computeInfo(root,size);
	}

	virtual const Node<S,Z,B>* getRoot() const {return root;}
	virtual void search();
private:
	void generateInitialBelief();	
	
	double simulate(const S& state, Node<S,Z,B>* node, double depth);
	double rollout(const S& state, double depth);
	unsigned getRolloutAction(const S& state);	
	Node<S,Z,B>* getNode(Node<S,Z,B>* parent, unsigned actionIndex, const Z& observation);
	static void eraseTree(Node<S,Z,B>* node);
	unsigned computeInfo(Node<S,Z,B>* node, unsigned& size) const;
	static void eraseNodeAndChilds(const Edge<Z>& doNotErasethisEdge, Node<S,Z,B>* node);
	
	Simulator<S,Z,A>& simulator;

	double planningTimeout;
	double resamplingTimeout;
	double threshold;
	double explorationConstant;
	unsigned currentAction;
	unsigned numParticlesInitialBelief;
		
	Node<S,Z,B> *root;
	std::vector<unsigned> actionIndexes;
};

template <typename S, typename Z, typename A, typename B>	
inline
PomcpPlanner<S,Z,A,B>::PomcpPlanner(Simulator<S,Z,A>& simulator, double timeout, double resamplingTimeout, double threshold, double explorationConstant, unsigned particlesInitialBelief)
: simulator(simulator),
  planningTimeout(timeout),
  resamplingTimeout(resamplingTimeout),
  threshold(threshold),
  explorationConstant(explorationConstant),
  numParticlesInitialBelief(particlesInitialBelief),
  currentAction(simulator.getNumActions()),
  root(new Node<S,Z,B>())
{
	actionIndexes.resize(simulator.getNumActions());
}

template <typename S, typename Z, typename A, typename B>
inline
void PomcpPlanner<S,Z,A,B>::reset()
{
	currentAction = simulator.getNumActions();
	simulator.cleanup();
        boost::thread freeMemThread(eraseTree,root);
	root = new Node<S,Z,B>();
}

template <typename S, typename Z, typename A, typename B>
inline
Node<S,Z,B>* PomcpPlanner<S,Z,A,B>::getNode(Node<S,Z,B>* parent, unsigned actionIndex, const Z& observation)
{
	Edge<Z> edge(actionIndex,observation);
	auto it = parent->childs.find(edge);
	if(it == parent->childs.end()) {
		Node<S,Z,B>* node = new Node<S,Z,B>();
		parent->childs[edge] = node;
		return node;
	}
	return it->second;
}

template<typename S, typename Z, typename A, typename B>
inline
unsigned PomcpPlanner<S,Z,A,B>::getRolloutAction(const S& state)
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

template <typename S, typename Z, typename A, typename B>
inline
double PomcpPlanner<S,Z,A,B>::rollout(const S& state, double depth)
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


template <typename S, typename Z, typename A, typename B>
inline
double PomcpPlanner<S,Z,A,B>::simulate(const S& state, Node<S,Z,B>* node, double depth)
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
		Node<S,Z,B>* nextNode = getNode(node,actionIndex,observation);
		nextNode->belief.add(nextState);
		reward += simulator.getDiscount() * simulate(nextState, nextNode , depth*simulator.getDiscount());
	} 
	node->counter++;
	node->actionData[actionIndex].counter++;
	node->actionData[actionIndex].value += 
			(reward - node->actionData[actionIndex].value)/(double)(node->actionData[actionIndex].counter);	
	return reward;
}

template <typename S, typename Z, typename A, typename B>
inline
void PomcpPlanner<S,Z,A,B>::generateInitialBelief()
{
	utils::Timer timer;
	S s0;
	do {
		simulator.sampleInitialState(s0);
		root->belief.add(s0);
	} while(timer.elapsed() < resamplingTimeout && root->belief.size()<numParticlesInitialBelief);	
}

template<typename S, typename Z, typename A, typename B>
inline
void PomcpPlanner<S,Z,A,B>::search()
{
	if (root->belief.empty()) {
		S s0;
		utils::Timer timer;
		do {
			simulate(simulator.sampleInitialState(s0),root,1.0);
			root->belief.add(s0);
		} while (timer.elapsed()< (planningTimeout)); 
	} else {
		utils::Timer timer;
		do {
			simulate(root->belief.sample(),root,1.0);
		} while (timer.elapsed()<planningTimeout);
	}
	
}

template<typename S, typename Z, typename A, typename B>
inline
unsigned PomcpPlanner<S,Z,A,B>::getAction(bool shouldSearch)
{
	if (shouldSearch && (currentAction == simulator.getNumActions())) {
		search();
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
	//assert(currentAction != simulator.getNumActions());
	if(currentAction == simulator.getNumActions())
		std::cout << "POMCP: getAction() gives no action" << std::endl;
	#endif

	return currentAction;
}

template<typename S, typename Z, typename A, typename B>
inline
void PomcpPlanner<S,Z,A,B>::eraseTree(Node<S,Z,B>* node)
{
	for (auto it = node->childs.begin(); it!=node->childs.end(); ++it) {
		eraseTree(it->second);
	}
	delete node;
	
}

template<typename S,typename Z, typename A, typename B>
inline
unsigned PomcpPlanner<S,Z,A,B>::computeInfo(Node<S,Z,B>* node, unsigned& size) const
{
	size++;
	if (node->childs.empty()) {
		return 0;
	}
	unsigned max=0;
	for (auto it = node->childs.begin(); it!=node->childs.end(); ++it) {
		unsigned aux = computeInfo(it->second,size);
		if (aux > max) {
			max=aux;
		}
	}
	return max+1;
	
}


template<typename S, typename Z, typename A, typename B>
inline
void PomcpPlanner<S,Z,A,B>::eraseNodeAndChilds(const Edge<Z>& doNotErasethisEdge, Node<S,Z,B>* node)
{
	for (auto it = node->childs.begin(); it!= node->childs.end(); ++it) {
		if (!(it->first == doNotErasethisEdge)) {
			eraseTree(it->second);
		} 
	}
	delete node;
}

template<typename S, typename Z, typename A, typename B>
inline
bool PomcpPlanner<S,Z,A,B>::moveTo(unsigned actionIndex, const Z& observation)
{
	/*bool reseted;
	Edge<Z> edge(actionIndex,observation);
	auto it = root->childs.find(edge);
	if (it == root->childs.end() ||
		it->second->belief.size()==0) {
		reset();
		reseted=true;
	} else {
		Node<S,Z,B>* nextRoot = it->second;
		boost::thread freeMemThread(eraseNodeAndChilds,edge,root);
		root = nextRoot;
		currentAction = simulator.getNumActions();
		simulator.cleanup();
		reseted=false;
	}
	return reseted;*/

	bool reseted;
	Edge<Z> edge(actionIndex,observation);
	auto it = root->childs.find(edge);
	if (it == root->childs.end() ||
		it->second->belief.size()==0) {
		reset();
		reseted=true;
		generateInitialBelief();
		#ifdef _POMCP_DEBUG_
		if(it == root->childs.end()) {
			std::cout << "No child " << std::endl;
		}
		#endif
	} else {
		#ifdef _POMCP_DEBUG_
		std::cout << "POMCP. Particles root node:" << root->belief.size() << " Next node: " << it->second->belief.size() <<  std::endl;
		#endif		
		utils::Timer timer;
		do {
			Z sObservation;
			double reward;
			S nextState;
			simulator.simulate(root->belief.sample(), actionIndex, nextState, sObservation, reward);
			if (observation == sObservation) {
				it->second->belief.add(nextState);
			}

		} while (timer.elapsed()<resamplingTimeout);
		Node<S,Z,B>* nextRoot = it->second;
		boost::thread freeMemThread(eraseNodeAndChilds,edge,root);
		root = nextRoot;
		currentAction = simulator.getNumActions();
		simulator.cleanup();
		reseted=false;
	}
	return reseted;
}

}
#endif
