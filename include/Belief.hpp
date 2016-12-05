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

#ifndef _BELIEF_HPP_
#define _BELIEF_HPP_

#include <vector>
#include "Multiset.hpp"

namespace pomcp
{

/**
 * class Belief<S>
 *
 * A virtual class for storing a belief as a set of particles
 * Tip: Profile different implementations
 *
 * S is the type for states
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<S> (if you use a belief based on hash function)
 *	   bool S::operator ==(const S& other) const (if you use a belief based on hash function)
 *
 * @author Ignacio Perez
 */

template<typename S>
class Belief
{
public:
	Belief() {}
	virtual ~Belief() {}
	virtual void add(const S& particle) = 0;
	virtual std::size_t size() const = 0;
	virtual bool empty() const = 0;
	virtual const S& sample() const = 0;
};

/**
 * class VectorBelief<S>
 *
 * An implementation of Belief<S> by using std::vector<S>
 *
 * S is the type for states
 *
 * @author Ignacio Perez
 */

template<typename S>
class VectorBelief : public Belief<S>
{
public:
	VectorBelief() {}
	virtual ~VectorBelief() {}
	virtual void add(const S& particle)
	{
		particles.push_back(particle);
	}
	virtual std::size_t size() const
	{
		return particles.size();
	} 
	virtual bool empty() const
	{
		return particles.empty();
	}
	virtual const S& sample() const
	{
		return particles[utils::RANDOM(particles.size())];
	}
	const std::vector<S>& getParticles() const {return particles;}

private:
	std::vector<S> particles;
};

/**
 * class MultisetBelief<S>
 *
 * An implementation of Belief<S> by using utils::Multiset<S>
 *
 * S is the type for states
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<S>
 *	   bool S::operator ==(const S& other) const
 *
 * @author Ignacio Perez
 */

template<typename S>
class MultisetBelief : public Belief<S>
{
public:
	MultisetBelief() {}
	virtual ~MultisetBelief() {}
	virtual void add(const S& particle)
	{
		particles.add(particle);
	}
	virtual std::size_t size() const
	{
		return particles.size();
	} 
	virtual bool empty() const
	{
		return particles.data().empty();
	}
	virtual const S& sample() const
	{
		unsigned index = utils::RANDOM(particles.size());
		unsigned counter=0;
		auto it = particles.data().begin();
		while(it!=particles.data().end()) {
			counter += it->second;
			if (index<counter) {
				return it->first;
			}
			++it;
		}
		return it->first;
	}

	const utils::Multiset<S>& getParticles() const {return particles;}
	
private:
	utils::Multiset<S> particles;
};

}

#endif
