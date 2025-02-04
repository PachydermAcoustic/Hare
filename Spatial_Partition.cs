//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)
//'
//'Copyright (c) 2008 - 2025, Arthur van der Harten			
//'This program is free software; you can redistribute it and/or modify
//'it under the terms of the GNU General Public License as published 
//'by the Free Software Foundation; either version 3 of the License, or
//'(at your option) any later version.
//'This program is distributed in the hope that it will be useful,
//'but WITHOUT ANY WARRANTY; without even the implied warranty of
//'MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//'GNU General Public License for more details.
//'
//'You should have received a copy of the GNU General Public 
//'License along with this program; if not, write to the Free Software
//'Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

using System;
using System.Collections.Generic;

namespace Hare
{
    namespace Geometry
    {
        /// <summary>
        /// The parent class of all spatial partitions...
        /// </summary>
        public abstract class Spatial_Partition
        {
            public Topology[] Model;

            public double Char_Step;
            public abstract bool Shoot(Ray R, int top_index, out X_Event Ret_event);
            public abstract bool Shoot(Ray R, int top_index, out X_Event Ret_event, int poly_origin1, int poly_origin2 = -1);
            //public abstract void Box_Intersect(AABB box, out System.Collections.Generic.List<int> isect);
        }
    }
}