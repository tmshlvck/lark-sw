/*  Lark TE vario computation
 *
 *  Copyright (C) 2014  The openvario project
 *  Copyright (C) 2018  Tomas Hlavacek
 *
 *  This program is free software; you can redistribute it and/or 
 *  modify it under the terms of the GNU General Public License 
 *  as published by the Free Software Foundation; either version 3
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>



const float ComputeVario(const float p, const float d_p)
{
  static const float FACTOR = -2260.389548275485;
  static const float EXP = -0.8097374740609689;
  return FACTOR*pow(p, EXP) * d_p;
}
