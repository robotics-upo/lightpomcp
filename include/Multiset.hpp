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

#ifndef _MULTISET_HPP_
#define _MULTISET_HPP_

#include <unordered_map>

namespace utils
{
/**
 * class Multiset<T>
 *
 * A Multiset implementation over std::unordered_map
 *
 * T is the type for the elements to be stored
 *
 * Note: the next structs and operators should be defined:
 * 	   std::hash<T>
  *	   bool T::operator ==(const T& other) const
 *
 * @author Ignacio Perez
 */

template <typename T>
class Multiset
{
public:
	Multiset() : size_(0) {}
	virtual ~Multiset() {}
	/**
   	 * Get the data of the multiset as a read-only std::unordered_map<T,unsigned> 
	 * containing the set of pairs <element,multiplicity>
   	 * @return the data of the multiset
   	 */
	const std::unordered_map<T,unsigned>& data() const {return data_;}
	/**
   	 * Get the size of the multiset
   	 * @return the size of the multiset
   	 */
	std::size_t size() const {return size_;} 
	/**
   	 * Add N elements
	 * @param element: The element to be added
	 * @multiplicity: The multiplicity of the element
   	 */
	virtual void add(const T& element, unsigned multiplicity);
	/**
   	 * Add one element
	 * @param element: The element to be added 
   	 */
	virtual void add(const T& element);
	/**
   	 * Clear the multiset
   	 */
	void clear();
private:
	std::unordered_map<T,unsigned> data_;
	std::size_t size_;
};

template <typename T>
inline
void Multiset<T>::add(const T& element, unsigned multiplicity)
{
	if (multiplicity==0) {
		return;
	}
	data_[element]+=multiplicity;
	size_+=multiplicity;
}

template <typename T>
inline
void Multiset<T>::add(const T& element)
{
	add(element,1);
}

template <typename T>
inline
void Multiset<T>::clear()
{
	data_.clear();
	size_ = 0;
}

}
#endif
