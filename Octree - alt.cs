//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL) by Arthur van der Harten
//'
//'Copyright (c) 2008, 2025, Arthur van der Harten			
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

namespace Hare.Geometry
{
    public class Octree : Spatial_Partition
    {
        private class OctreeNode : AABB
        {
            public List<int> Polygons;
            public OctreeNode[] Children;

            public OctreeNode(Point Min_in, Point Max_in)
                : base(Min_in, Max_in)
            {
                Polygons = new List<int>();
                Children = new OctreeNode[8];
            }

            public bool IsLeaf => Children[0] == null;
        }

        private OctreeNode root;
        private int maxDepth;
        private int maxPolygonsPerNode;
        private int no_of_boxes = 2000;
        private int[,][] Poly_Ray_ID;

        public Octree(Topology[] Model_In, int maxDepth, int maxPolygonsPerNode)
        {
            Model = Model_In;
            this.maxDepth = maxDepth;
            this.maxPolygonsPerNode = maxPolygonsPerNode;

            // Initialize the mailbox
            Poly_Ray_ID = new int[Model_In.Length, no_of_boxes][];

            for (int i = 0; i < Model.Length; i++)
            {
                for (int p = 0; p < no_of_boxes; p++)
                {
                    Poly_Ray_ID[i, p] = new int[Model[i].Polygon_Count];
                }
            }

            // Calculate the bounding box for the entire model
            foreach (Topology topo in Model_In)
            {
                Point min = new Point(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
                Point max = new Point(double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity);

                for (int i = 0; i < topo.Vertex_Count; i++)
                {
                    if (topo[i].x < min.x) min.x = topo[i].x;
                    if (topo[i].y < min.y) min.y = topo[i].y;
                    if (topo[i].z < min.z) min.z = topo[i].z;
                    if (topo[i].x > max.x) max.x = topo[i].x;
                    if (topo[i].y > max.y) max.y = topo[i].y;
                    if (topo[i].z > max.z) max.z = topo[i].z;
                }

                double maxdim = Math.Max(max.x - min.x, Math.Max(max.y - min.y, max.z - min.z));
                Point center = max + min / 2;

                min.x = center.x - maxdim - 1e-1; min.y = center.y - maxdim - 1e-1; min.z = center.z - maxdim - 1e-1;
                max.x = center.x + maxdim + 1e-1; max.y = center.y + maxdim + 1e-1; max.z = center.z + maxdim + 1e-1;

                root = new OctreeNode(min, max);
                for (int i = 0; i < topo.Polygon_Count; i++) root.Polygons.Add(i);

                BuildOctree(root, 0);
            }
        }

        private void BuildOctree(OctreeNode node, int depth)
        {
            if (depth >= maxDepth || node.Polygons.Count <= maxPolygonsPerNode)
                return;

            Point center = node.Center;
            Point halfSize = new Point(node.Width.dx * .5, node.Width.dy * .5, node.Width.dz * .5);

            for (int i = 0; i < 8; i++)
            {
                Point min = new Point(
                    ((i & 4) == 0 ? node.Min.x : center.x) - 0.1,
                    ((i & 2) == 0 ? node.Min.y : center.y) - 0.1,
                    ((i & 1) == 0 ? node.Min.z : center.z) - 0.1
                );

                Point max = new Point(
                    ((i & 4) == 0 ? center.x : node.Max.x) + 0.1,
                    ((i & 2) == 0 ? center.y : node.Max.y) + 0.1,
                    ((i & 1) == 0 ? center.z : node.Max.z) + 0.1
                );

                node.Children[i] = new OctreeNode(min, max);
            }

            List<int> lostpolys = new List<int>();

            foreach (int polyId in node.Polygons)
            {
                bool stored = false;
                foreach (OctreeNode child in node.Children)
                {
                    if (child.PolyBoxOverlap(Model[0].Polygon_Vertices(polyId)))
                    {
                        child.Polygons.Add(polyId);
                        stored = true;
                    }
                }
                if (!stored) lostpolys.Add(polyId);
            }

            node.Polygons.Clear();

            foreach (OctreeNode child in node.Children)
            {
                BuildOctree(child, depth + 1);
            }
        }

        uint rayno = 0;
        object ctlock = new object();

        protected uint assign_id()
        {
            lock (ctlock)
            {
                rayno++;
                if (rayno == no_of_boxes) { rayno = 0; } 

                return rayno;
            }
        }

        public override bool Shoot(Ray ray, int top_index, out X_Event Ret_Event)
        {
            return Shoot(ray, top_index, out Ret_Event, -1, -1);
        }

        public override bool Shoot(Ray ray, int top_index, out X_Event Ret_Event, int poly_origin1 = -1, int poly_origin2 = -1)
        {
            Ret_Event = null;
            int ray_id = (int)assign_id();

            // Precompute inverse direction components
            double invDx = Math.Abs(ray.dx) > 1e-16 ? 1.0 / ray.dx : 1e16;
            double invDy = Math.Abs(ray.dy) > 1e-16 ? 1.0 / ray.dy : 1e16;
            double invDz = Math.Abs(ray.dz) > 1e-16 ? 1.0 / ray.dz : 1e16;

            // Compute initial t-values for the root node
            double tx0 = (root.Min.x - ray.x) * invDx;
            double tx1 = (root.Max.x - ray.x) * invDx;
            double ty0 = (root.Min.y - ray.y) * invDy;
            double ty1 = (root.Max.y - ray.y) * invDy;
            double tz0 = (root.Min.z - ray.z) * invDz;
            double tz1 = (root.Max.z - ray.z) * invDz;

            // Correct tmin and tmax for negative directions
            if (invDx < 0) { double temp = tx0; tx0 = tx1; tx1 = temp; }
            if (invDy < 0) { double temp = ty0; ty0 = ty1; ty1 = temp; }
            if (invDz < 0) { double temp = tz0; tz0 = tz1; tz1 = temp; }

            double tmin = Math.Max(Math.Max(tx0, ty0), tz0);
            double tmax = Math.Min(Math.Min(tx1, ty1), tz1);

            if (tmax < tmin || tmax < 0)
            {
                // Ray misses the node
                Ret_Event = new X_Event();
                return false;
            }

            // Determine the traversal order for the children
            int[] childOrder = ComputeTraversalOrder(ray);

            // Initialize the stack for iterative traversal
            Stack<(OctreeNode node, double tmin, double tmax)> stack = new Stack<(OctreeNode, double, double)>();
            stack.Push((root, tmin, tmax));

            bool hit = false;
            double closestT = double.MaxValue;
            X_Event closestEvent = null;

            while (stack.Count > 0)
            {
                var (node, nodeTmin, nodeTmax) = stack.Pop();

                if (nodeTmax < nodeTmin || nodeTmax < 0)
                    continue; // Skip if there's no intersection

                if (hit && closestT <= nodeTmin)
                    continue; // Skip if we've already found a closer hit

                if (node.IsLeaf)
                {
                    // Test the polygons in this leaf node
                    foreach (int polyId in node.Polygons)
                    {
                        if (polyId == poly_origin1 || polyId == poly_origin2) continue;

                        // Check the mailbox
                        //if (Poly_Ray_ID[top_index, ray_id][polyId] == ray.Ray_ID) continue;
                        //Poly_Ray_ID[top_index, ray_id][polyId] = ray.Ray_ID;

                        if (Model[top_index].intersect(polyId, ray, out Point intersection, out double u, out double v, out double t) && t > 0.0000000001)
                        {
                            if (t < closestT)
                            {
                                closestT = t;
                                closestEvent = new X_Event(intersection, u, v, t, polyId);
                                hit = true;

                                // Early termination if an intersection closer than the current node's tmin is found
                                if (closestT <= nodeTmin)
                                {
                                    Ret_Event = closestEvent;
                                    return true;
                                }
                            }
                        }
                    }
                }
                else
                {
                    // For internal nodes, push child nodes onto the stack in the determined order
                    foreach (int childIndex in childOrder)
                    {
                        OctreeNode child = node.Children[childIndex];

                        if (child == null)
                            continue;

                        // Compute child tmin and tmax
                        double cTx0 = (child.Min.x - ray.x) * invDx;
                        double cTx1 = (child.Max.x - ray.x) * invDx;
                        double cTy0 = (child.Min.y - ray.y) * invDy;
                        double cTy1 = (child.Max.y - ray.y) * invDy;
                        double cTz0 = (child.Min.z - ray.z) * invDz;
                        double cTz1 = (child.Max.z - ray.z) * invDz;

                        // Correct tmin and tmax for negative directions
                        if (invDx < 0) { double temp = cTx0; cTx0 = cTx1; cTx1 = temp; }
                        if (invDy < 0) { double temp = cTy0; cTy0 = cTy1; cTy1 = temp; }
                        if (invDz < 0) { double temp = cTz0; cTz0 = cTz1; cTz1 = temp; }

                        double childTmin = Math.Max(Math.Max(cTx0, cTy0), cTz0);
                        double childTmax = Math.Min(Math.Min(cTx1, cTy1), cTz1);

                        if (childTmax < childTmin || childTmax < 0 || childTmin > nodeTmax || childTmax < nodeTmin)
                            continue; // No intersection with this child

                        stack.Push((child, Math.Max(childTmin, nodeTmin), Math.Min(childTmax, nodeTmax)));
                    }
                }
            }

            if (hit)
            {
                Ret_Event = closestEvent;
            }

            if (Ret_Event == null) 
                Ret_Event = new X_Event();
            return hit;
        }

        private int[] ComputeTraversalOrder(Ray ray)
        {
            int xDir = ray.dx >= 0 ? 0 : 1;
            int yDir = ray.dy >= 0 ? 0 : 1;
            int zDir = ray.dz >= 0 ? 0 : 1;

            int[] order = new int[8];
            int i = 0;
            for (int ix = xDir; ix <= 1 && ix >= 0; ix += (ray.dx >= 0 ? 1 : -1))
            {
                for (int iy = yDir; iy <= 1 && iy >= 0; iy += (ray.dy >= 0 ? 1 : -1))
                {
                    for (int iz = zDir; iz <= 1 && iz >= 0; iz += (ray.dz >= 0 ? 1 : -1))
                    {
                        int childIndex = (ix << 2) | (iy << 1) | iz;
                        order[i++] = childIndex;
                    }
                }
            }
            return order;
        }
    }
}
