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

#include "Utilities.h"

using namespace MathLib;

void GetRotationMatrixFromQuaternion(const Vector& quat, Matrix3 mat)
{
    //vector should be a normalized quaternion
    if(quat.Size() != 4)
    {
        return;
    }
    else
    {
        //see wikipedia - quaternion and spatial rotation
        double a = quat(0), b = quat(1), c = quat(2), d = quat(3);
        double rot11 = a * a + b * b - c * c - d * d;
        double rot12 = 2 * b * c - 2 * a * d;
        double rot13 = 2 * b * d + 2 * a * c;
        double rot21 = 2 * b * c + 2 * a * d;
        double rot22 = a * a - b * b + c * c - d * d;
        double rot23 = 2 * c * d - 2 * a * b;
        double rot31 = 2 * b * d - 2 * a * c;
        double rot32 = 2 * c * d + 2 * a * b;
        double rot33 = a * a - b * b - c * c + d * d;

        mat.Set(rot11, rot12, rot13,
                rot21, rot22, rot23,
                rot31, rot32, rot33);
    }

    return;
}
