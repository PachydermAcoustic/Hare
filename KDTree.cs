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
using System.Linq;

namespace Hare.Geometry
{
    public class KDTree : Spatial_Partition
    {
        private class KDTreeNode:AABB
        {
            public List<int> Polygons;
            public KDTreeNode Left;
            public KDTreeNode Right;
            public double SplitValue;
            public int SplitAxis;

            public KDTreeNode(Point Min, Point Max)
            :base(Min, Max)
            {

                Polygons = new List<int>();
            }

            public bool IsLeaf => Left == null && Right == null;
        }

        private KDTreeNode root;
        private int maxDepth;
        private int maxPolygonsPerNode;
        private int[,][] Poly_Ray_ID;
        private uint rayno = 0;
        private object ctlock = new object();
        private const uint no_of_boxes = 500;

        public KDTree(Topology[] Model_In, int maxDepth, int maxPolygonsPerNode)
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
            Point min = new Point(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
            Point max = new Point(double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity);

            foreach (Topology topo in Model_In)
            {
                for (int i = 0; i < topo.Vertex_Count; i++)
                {
                    if (topo[i].x < min.x) min.x = topo[i].x;
                    if (topo[i].y < min.y) min.y = topo[i].y;
                    if (topo[i].z < min.z) min.z = topo[i].z;
                    if (topo[i].x > max.x) max.x = topo[i].x;
                    if (topo[i].y > max.y) max.y = topo[i].y;
                    if (topo[i].z > max.z) max.z = topo[i].z;
                }

                root = new KDTreeNode(min, max);
                for (int i = 0; i < topo.Polygon_Count; i++) root.Polygons.Add(i);

                BuildKDTree(root, 0, min, max);
            }
        }

        private void BuildKDTree(KDTreeNode node, int depth, Point min, Point max)
        {
            if (depth >= maxDepth || node.Polygons.Count <= maxPolygonsPerNode)
                return;

            int axis = depth % 3;

            // Sort polygons based on their centroids along the split axis
            var sortedPolygons = node.Polygons
                .Select(polyId => new { PolyId = polyId, Centroid = Model[0].Polygon_Centroid(polyId) })
                .OrderBy(p => p.Centroid.byint(axis))
                .ToList();

            // Choose the median polygon's centroid as the split value
            int medianIndex = sortedPolygons.Count / 2;
            double splitValue = sortedPolygons[medianIndex].Centroid.byint(axis);

            node.SplitAxis = axis;
            node.SplitValue = splitValue;

            Point leftMax = new Point(max);
            if (axis == 0) leftMax.x = splitValue;
            else if (axis == 1) leftMax.y = splitValue;
            else leftMax.z = splitValue;

            Point rightMin = new Point(min);
            if (axis == 0) rightMin.x = splitValue;
            else if (axis == 1) rightMin.y = splitValue;
            else rightMin.z = splitValue;

            node.Left = new KDTreeNode(min, leftMax);
            node.Right = new KDTreeNode(rightMin, max);

            foreach (var poly in sortedPolygons)
            {
                if (Model[0].Polygon_Vertices(poly.PolyId).Any(p => p.byint(axis) <= splitValue))
                {
                    node.Left.Polygons.Add(poly.PolyId);
                }
                if (Model[0].Polygon_Vertices(poly.PolyId).Any(p => p.byint(axis) > splitValue))
                {
                    node.Right.Polygons.Add(poly.PolyId);
                }
            }

            node.Polygons.Clear();

            BuildKDTree(node.Left, depth + 1, min, leftMax);
            BuildKDTree(node.Right, depth + 1, rightMin, max);
        }

        //private void BuildKDTree(KDTreeNode node, int depth, Point min, Point max)
        //{
        //    if (depth >= maxDepth || node.Polygons.Count <= maxPolygonsPerNode)
        //        return;

        //    int axis = depth % 3;
        //    double splitValue = (min.byint(axis) + max.byint(axis)) * 0.5;

        //    node.SplitAxis = axis;
        //    node.SplitValue = splitValue;

        //    Point leftMax = new Point(max);
        //    if (axis == 0) leftMax.x = splitValue;
        //    else if (axis == 1) leftMax.y = splitValue;
        //    else leftMax.z = splitValue;

        //    Point rightMin = new Point(min);
        //    if (axis == 0) rightMin.x = splitValue;
        //    else if (axis == 1) rightMin.y = splitValue;
        //    else rightMin.z = splitValue;

        //    node.Left = new KDTreeNode(min, leftMax);
        //    node.Right = new KDTreeNode(rightMin, max);

        //    foreach (int polyId in node.Polygons)
        //    {
        //        if (Model[0].Polygon_Vertices(polyId).Any(p => p.byint(axis) <= splitValue))
        //        {
        //            node.Left.Polygons.Add(polyId);
        //        }
        //        if (Model[0].Polygon_Vertices(polyId).Any(p => p.byint(axis) > splitValue))
        //        {
        //            node.Right.Polygons.Add(polyId);
        //        }
        //    }

        //    node.Polygons.Clear();

        //    BuildKDTree(node.Left, depth + 1, min, leftMax);
        //    BuildKDTree(node.Right, depth + 1, rightMin, max);
        //}

        private uint assign_id()
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

        public override bool Shoot(Ray ray, int top_index, out X_Event Ret_Event, int poly_origin1, int poly_origin2 = -1)
        {
            uint rayid = assign_id();
            return Shoot(root, ray, top_index, rayid, out Ret_Event, poly_origin1, poly_origin2);
        }

        private bool Shoot(KDTreeNode node, Ray ray, int top_index, uint rayid, out X_Event Ret_Event, int poly_origin1, int poly_origin2)
        {
            Ret_Event = new X_Event();
            bool hit = false;
            double closestT = double.MaxValue;

            Stack<KDTreeNode> stack = new Stack<KDTreeNode>();
            stack.Push(node);

            while (stack.Count > 0)
            {
                KDTreeNode currentNode = stack.Pop();

                if (currentNode.IsLeaf)
                {
                    foreach (int polyId in currentNode.Polygons)
                    {
                        if (polyId == poly_origin1 || polyId == poly_origin2) continue;

                        // Check the mailbox to avoid redundant intersection tests
                        if (Poly_Ray_ID[top_index, rayid][polyId] == ray.Ray_ID)
                        {
                            continue;
                        }

                        Poly_Ray_ID[top_index, rayid][polyId] = ray.Ray_ID;

                        Point intersection;
                        double u, v, t;
                        if (Model[top_index].intersect(polyId, ray, out intersection, out u, out v, out t) && t > 0.0000000001)
                        {
                            if (t < closestT)
                            {
                                closestT = t;
                                Ret_Event = new X_Event(intersection, u, v, t, polyId);
                                hit = true;
                            }
                        }
                    }
                }
                else
                {
                    double tSplit;
                    KDTreeNode firstNode, secondNode;

                    if (currentNode.SplitAxis == 0)
                    {
                        double side = ray.x - currentNode.SplitValue;
                        tSplit = -side / ray.dx;
                        double ySplit = ray.y + tSplit * ray.dy;
                        double zSplit = ray.z + tSplit * ray.dz;

                        if (ySplit <= currentNode.Max.y && ySplit >= currentNode.Min.y && zSplit <= currentNode.Max.z && zSplit >= currentNode.Min.z)
                        {
                            // Intersection is within the box
                            if (side >= 0)
                            {
                                firstNode = currentNode.Right;
                                secondNode = currentNode.Left;
                            }
                            else
                            {
                                firstNode = currentNode.Left;
                                secondNode = currentNode.Right;
                            }
                        }
                        else
                        {
                            if (side >= 0)
                            {
                                firstNode = currentNode.Left;
                                secondNode = currentNode.Right;
                            }
                            else
                            {
                                firstNode = currentNode.Right;
                                secondNode = currentNode.Left;
                            }
                        }
                    }
                    else if (currentNode.SplitAxis == 1)
                    {
                        double side = ray.y - currentNode.SplitValue;
                        tSplit = -side / ray.dy;
                        double xSplit = ray.x + tSplit * ray.dx;
                        double zSplit = ray.z + tSplit * ray.dz;

                        if (xSplit <= currentNode.Max.x && xSplit >= currentNode.Min.x && zSplit <= currentNode.Max.z && zSplit >= currentNode.Min.z)
                        {
                            // Intersection is within the box
                            if (side >= 0)
                            {
                                firstNode = currentNode.Right;
                                secondNode = currentNode.Left;
                            }
                            else
                            {
                                firstNode = currentNode.Left;
                                secondNode = currentNode.Right;
                            }
                        }
                        else
                        {
                            if (side >= 0)
                            {
                                firstNode = currentNode.Left;
                                secondNode = currentNode.Right;
                            }
                            else
                            {
                                firstNode = currentNode.Right;
                                secondNode = currentNode.Left;
                            }
                        }
                    }
                    else // currentNode.SplitAxis == 2
                    {
                        double side = ray.z - currentNode.SplitValue;
                        tSplit = -side / ray.dz;
                        double xSplit = ray.x + tSplit * ray.dx;
                        double ySplit = ray.y + tSplit * ray.dy;

                        if (xSplit <= currentNode.Max.x && xSplit >= currentNode.Min.x && ySplit <= currentNode.Max.y && ySplit >= currentNode.Min.y)
                        {
                            // Intersection is within the box
                            if (side >= 0)
                            {
                                firstNode = currentNode.Right;
                                secondNode = currentNode.Left;
                            }
                            else
                            {
                                firstNode = currentNode.Left;
                                secondNode = currentNode.Right;
                            }
                        }
                        else
                        {
                            if (side >= 0)
                            {
                                firstNode = currentNode.Left;
                                secondNode = currentNode.Right;
                            }
                            else
                            {
                                firstNode = currentNode.Right;
                                secondNode = currentNode.Left;
                            }
                        }
                    }

                    stack.Push(secondNode);
                    stack.Push(firstNode);
                }
            }

            return hit;
        }


        //private bool Shoot(KDTreeNode node, Ray ray, int top_index, uint rayid, out X_Event Ret_Event, int poly_origin1, int poly_origin2)
        //{
        //    if (node.IsLeaf)
        //    {
        //        Ret_Event = new X_Event();
        //        bool hit = false;
        //        double closestT = double.MaxValue;

        //        foreach (int polyId in node.Polygons)
        //        {
        //            if (polyId == poly_origin1 || polyId == poly_origin2) continue;

        //            // Check the mailbox to avoid redundant intersection tests
        //            if (Poly_Ray_ID[top_index, rayid][polyId] == ray.Ray_ID)
        //            {
        //                continue;
        //            }

        //            Poly_Ray_ID[top_index, rayid][polyId] = ray.Ray_ID;

        //            Point intersection;
        //            double u, v, t;
        //            if (Model[top_index].intersect(polyId, ray, out intersection, out u, out v, out t) && t > 0.0000000001)
        //            {
        //                if (t < closestT)
        //                {
        //                    closestT = t;
        //                    Ret_Event = new X_Event(intersection, u, v, t, polyId);
        //                    hit = true;
        //                }
        //            }
        //        }

        //        return hit;
        //    }
        //    else
        //    {
        //        double tSplit;
        //        KDTreeNode firstNode, secondNode;

        //        if (node.SplitAxis == 0)
        //        {
        //            double side = ray.x - node.SplitValue;
        //            tSplit = -side / ray.dx;
        //            double ySplit = ray.y + tSplit * ray.dy;
        //            double zSplit = ray.z + tSplit * ray.dz;

        //            if (ySplit <= node.Max.y && ySplit >= node.Min.y && zSplit <= node.Max.z && zSplit >= node.Min.z)
        //            {
        //                //intersection is within the box
        //                if (side >= 0)
        //                {
        //                    firstNode = node.Right;
        //                    secondNode = node.Left;
        //                }
        //                else
        //                {
        //                    firstNode = node.Left;
        //                    secondNode = node.Right;
        //                }
        //            }
        //            else
        //            {
        //                if (side >= 0)
        //                {
        //                    firstNode = node.Left;
        //                    secondNode = node.Right;
        //                }
        //                else
        //                {
        //                    firstNode = node.Right;
        //                    secondNode = node.Left;
        //                }
        //            }
        //        }
        //        else if (node.SplitAxis == 1)
        //        {
        //            double side = ray.y - node.SplitValue;
        //            tSplit = -side / ray.dy;
        //            double xSplit = ray.x + tSplit * ray.dx;
        //            double zSplit = ray.z + tSplit * ray.dz;

        //            if (xSplit <= node.Max.x && xSplit >= node.Min.x && zSplit <= node.Max.z && zSplit >= node.Min.z)
        //            {
        //                //intersection is within the box
        //                if (side >= 0)
        //                {
        //                    firstNode = node.Right;
        //                    secondNode = node.Left;
        //                }
        //                else
        //                {
        //                    firstNode = node.Left;
        //                    secondNode = node.Right;
        //                }
        //            }
        //            else
        //            {
        //                if (side >= 0)
        //                {
        //                    firstNode = node.Left;
        //                    secondNode = node.Right;
        //                }
        //                else
        //                {
        //                    firstNode = node.Right;
        //                    secondNode = node.Left;
        //                }
        //            }
        //        }
        //        else // node.SplitAxis == 2
        //        {
        //            double side = ray.z - node.SplitValue;
        //            tSplit = -side / ray.dz;
        //            double xSplit = ray.x + tSplit * ray.dx;
        //            double ySplit = ray.y + tSplit * ray.dy;

        //            if (xSplit <= node.Max.x && xSplit >= node.Min.x && ySplit <= node.Max.y && ySplit >= node.Min.y)
        //            {
        //                //intersection is within the box
        //                if (side >= 0)
        //                {
        //                    firstNode = node.Right;
        //                    secondNode = node.Left;
        //                }
        //                else
        //                {
        //                    firstNode = node.Left;
        //                    secondNode = node.Right;
        //                }
        //            }
        //            else
        //            {
        //                if (side >= 0)
        //                {
        //                    firstNode = node.Left;
        //                    secondNode = node.Right;
        //                }
        //                else
        //                {
        //                    firstNode = node.Right;
        //                    secondNode = node.Left;
        //                }
        //            }
        //        }

        //        if (Shoot(firstNode, ray, top_index, rayid, out Ret_Event, poly_origin1, poly_origin2))
        //        {
        //            return true;
        //        }
        //        else
        //        {
        //            return Shoot(secondNode, ray, top_index, rayid, out Ret_Event, poly_origin1, poly_origin2);
        //        }
        //    }
        //}

        //private bool Shoot(KDTreeNode node, Ray ray, int top_index, uint rayid, out X_Event Ret_Event, int poly_origin1, int poly_origin2)
        //{
        //    Ret_Event = new X_Event();

        //    if (node.IsLeaf)
        //    {
        //        bool hit = false;
        //        double closestT = double.MaxValue;

        //        foreach (int polyId in node.Polygons)
        //        {
        //            if (polyId == poly_origin1 || polyId == poly_origin2) continue;

        //            // Check the mailbox to avoid redundant intersection tests
        //            //if (Poly_Ray_ID[top_index, rayid] == ray.Ray_ID)
        //            //{
        //            //    continue;
        //            //}

        //            Poly_Ray_ID[top_index, rayid] = ray.Ray_ID;

        //            Point intersection;
        //            double u, v, t;
        //            if (Model[top_index].intersect(polyId, ray, out intersection, out u, out v, out t) && t > 0.0000000001)
        //            {
        //                if (t < closestT)
        //                {
        //                    closestT = t;
        //                    Ret_Event = new X_Event(intersection, u, v, t, polyId);
        //                    hit = true;
        //                }
        //            }
        //        }

        //        return hit;
        //    }
        //    else
        //    {
        //        double tSplit;
        //        KDTreeNode firstNode, secondNode;
        //        if (node.SplitAxis == 0)
        //        {
        //            tSplit = (node.SplitValue - ray.x) / ray.dx;
        //            if (ray.dx < 0)
        //            {
        //                firstNode = node.Right;
        //                secondNode = node.Left;
        //                //firstNode = node.Left;
        //                //secondNode = node.Right;
        //            }
        //            else
        //            {
        //                firstNode = node.Left;
        //                secondNode = node.Right;
        //                //firstNode = node.Right;
        //                //secondNode = node.Left;
        //            }
        //        }
        //        else if (node.SplitAxis == 1)
        //        {
        //            tSplit = (node.SplitValue - ray.y) / ray.dy;
        //            if (ray.dy < 0)
        //            {
        //                firstNode = node.Right;
        //                secondNode = node.Left;
        //                //firstNode = node.Left;
        //                //secondNode = node.Right;
        //            }
        //            else
        //            {
        //                firstNode = node.Left;
        //                secondNode = node.Right;
        //                //firstNode = node.Right;
        //                //secondNode = node.Left;
        //            }
        //        }
        //        else // node.SplitAxis == 2
        //        {
        //            tSplit = (node.SplitValue - ray.z) / ray.dz;
        //            if (ray.dz < 0)
        //            {
        //                firstNode = node.Right;
        //                secondNode = node.Left;
        //                //firstNode = node.Left;
        //                //secondNode = node.Right;
        //            }
        //            else
        //            {
        //                firstNode = node.Left;
        //                secondNode = node.Right;
        //                //firstNode = node.Right;
        //                //secondNode = node.Left;
        //            }
        //        }

        //        //if (tSplit < 0)
        //        //{
        //        //    return Shoot(firstNode, ray, top_index, rayid, out Ret_Event, poly_origin1, poly_origin2);

        //        //}
        //        //else
        //        //{
        //            if (Shoot(firstNode, ray, top_index, rayid, out Ret_Event, poly_origin1, poly_origin2))
        //            {
        //                return true;
        //            }
        //            else
        //            {
        //                return Shoot(secondNode, ray, top_index, rayid, out Ret_Event, poly_origin1, poly_origin2);
        //            }
        //        //}
        //    }
    }
}