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

#ifndef _MONTECARLOSIMULATOR_HPP_
#define _MONTECARLOSIMULATOR_HPP_

namespace pomcp
{
/**
 * class Simulator<S,Z,A>
 *
 * This interface should be implemented in order to be used by the Pomcp Planner
 * Note: In general, try to implement it as efficient as possible
 *
 * S is the type for states
 * Z is the type for observations
 * A is the type for actions
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<S> (if you use a belief based on hash function in pomcp::PomcpPlanner<S,Z,A,B>)
 * 	   std::hash<Z>
 *	   bool S::operator ==(const S& other) const (if you use a belief based on hash function in pomcp::PomcpPlanner<S,Z,A,B>)
 *	   bool Z::operator ==(const Z& other) const
 *
 * @author Ignacio Perez
 */
template <typename S, typename Z, typename A>
class Simulator 
{
public:
	Simulator() {}
	virtual ~Simulator() {}
	/**
   	 * Get the discount factor of the POMDP model
   	 * @return the discount factor of the POMDP model
   	 */
	virtual double getDiscount() const = 0;
	/**
	 * Get a random initial state
         * @param state: [IN/OUT] variable of type S to be filled with one random initial state
	 * @return a reference to the in/out variable given as parameter
	 */
	virtual S& sampleInitialState(S& state) const = 0;
	/**
	 * Simulate one transition (State,Action) -> (NextState,Observation,Reward) 
	 * following the rules of the model
	 * @param state: [IN] the current state
	 * @param actionIndex: [IN] index of the action to be executed over the current state
	 * @param nextState: [IN/OUT] variable of type S to be filled with the next state
	 * @param observation: [IN/OUT] variable of type Z to be filled with the obtained observation
	 * @param reward: [IN/OUT] variable of type double to be filled with the inmediate reward R(state,action)  
	 * @return true if the next state is a terminal state, false otherwise
	 */
        virtual bool simulate(const S& state, unsigned actionIndex, S& nextState, Z& observation, double& reward) const = 0;
	/**
	 * Simulate one transition (State,Action) -> (NextState,Observation,Reward) 
	 * following the rules of the model
	 * @param state: [IN] the current state
	 * @param actionIndex: [IN] index of the action to be executed over the current state
	 * @param nextState: [IN/OUT] variable of type S to be filled with the next state
	 * @param reward: [IN/OUT] variable of type double to be filled with the inmediate reward R(state,action)  
	 * @return true if the next state is a terminal state, false otherwise
	 */
        virtual bool simulate(const S& state, unsigned actionIndex, S& nextState, double& reward) const = 0;
       	/**
   	 * Get the number of actions
       	 * @return the number of actions
   	 */
	virtual unsigned getNumActions() const = 0;
	/**
   	 * Get one action
	 * @param actionIndex: [IN] the index of the action
   	 * @return the action corresponding to the index 
	 * Tip: The actions could be stored in a vector generated in the constructor
   	 */
	virtual const A& getAction(unsigned actionIndex) const = 0;
	/**
   	 * Check if all actions are valid for a given state
	 * Note: Override this method only if you can provide an efficiency better than O(n) over the size of actions
	 * @param state: [IN] one state
	 * @return true if all actions are valid for the given state, false otherwise
	 */
	virtual bool allActionsAreValid(const S& state) const {return false;}
	/**
   	 * Check if one action is valid for a given state
	 * @param state: [IN] one state
	 * @param actionIndex: [IN] the index of the action to be checked
	 * @return true if the corresponding action is valid for the given state, false otherwise
	 */
	virtual bool isValidAction(const S& state, unsigned actionIndex) const = 0;
	/**
	 * Cleanup method.
	 * This function will be called by the POMCP planner periodically:
	 *   - after each moveTo(action,observation)
	 *   - in the reset function
	 *   - in the destructor
	 * Implement only if needed
	 */
	virtual void cleanup() {}
	/**
	 * Get all valid actions for a given state
	 * Note: Override this method only if you can provide an efficiency better than O(n) over the size of actions
	 * @param state: [IN] one state
	 * @param validActions: [IN/OUT] array to store the indexes of the valid actions
	 * @return validActions
	 */
	virtual std::vector<unsigned>& getValidActions(const S& state, std::vector<unsigned>& validActions) const
	{
		validActions.clear();
		for (unsigned a=0;a<getNumActions();a++) {
			if (isValidAction(state,a)) {
				validActions.push_back(a);
			}
		}
		return validActions;
	}
};

}
#endif
