//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)  
//'
//'Copyright (c) 2008, 2025, Open Research in Acoustical Science and Education, Inc. - a 501(c)3 nonprofit			
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

namespace Hare.Geometry
{
    /// <summary>Immutable spatial index; rebuild after changing topology geometry.</summary>
    public class BVH : Spatial_Partition
    {
        private readonly PartitionTree tree;
        public BVH(Topology[] Model_In, int maxDepth, int maxPolygonsPerNode)
        {
            Model = Model_In;
            tree = new PartitionTree(Model_In, maxDepth, maxPolygonsPerNode, PartitionTreeKind.BVH);
        }
        public override bool Shoot(Ray ray, int top_index, out X_Event Ret_Event)
            => tree.Shoot(ray, top_index, out Ret_Event, -1, -1);
        public override bool Shoot(Ray ray, int top_index, out X_Event Ret_Event, int poly_origin1, int poly_origin2 = -1)
            => tree.Shoot(ray, top_index, out Ret_Event, poly_origin1, poly_origin2);
    }
}
