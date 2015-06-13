﻿//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL) by Arthur van der Harten
//'
//'Copyright (c) 2008, 2009, Arthur van der Harten			
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
            public int[,][] Poly_Ray_ID;
            public double Char_Step;
            public abstract bool Shoot(Ray R, int top_index, out X_Event Ret_event);
            public abstract void Box_Intersect(AABB box, out System.Collections.Generic.List<int> isect);
        }
    }
}