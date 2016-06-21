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

#ifndef _TIMER_HPP_
#define _TIMER_HPP_
#include <chrono>

namespace utils
{
/**
 * class Timer
 *
 * A simple timer implementation by using std::chrono
 *
 * @author Ignacio Perez
 */
class Timer
{
public:
    /**
     * Create a new timer and start it
     */
    Timer() : beg_(clock_::now()) {}
    /**
     * Reset the timer to 0
     */
    void reset() { beg_ = clock_::now(); }
    /**
     * Get the elapsed time in seconds since the creation
     * of the object or the last time reset() method was called
     * @return elapsed: the elapsed time in seconds
     */
    double elapsed() const { 
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

}
#endif
