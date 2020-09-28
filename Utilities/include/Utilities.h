/*
 * Copyright (C) 2013 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Hang Yin
 * email:   hang.yin@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/*==========================================================================
 *
 *  Utility functions  
 *
 * ========================================================================*/

#ifndef Utilities_H_
#define Utilities_H_

#include "MathLib/MathLib.h"

void GetRotationMatrixFromQuaternion(const MathLib::Vector& quat, MathLib::Matrix3& mat);

#endif 
