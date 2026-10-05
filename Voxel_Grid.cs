//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)
//'
//'Copyright (c) 2008 - 2025, Open Research in Acoustical Science and Education, Inc. - a 501(c)3 nonprofit			
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
        /// Concept based on Amanatides Fast Ray-Voxel Traversal Algorithm.
        /// </summary>
        public class Voxel_Grid : Spatial_Partition
        {
                        // Legacy subclass interface; lanes are allocated only by assign_id().
            public int[,][] Poly_Ray_ID;
            private readonly System.Threading.ThreadLocal<PartitionScratch> queryScratch =
                new System.Threading.ThreadLocal<PartitionScratch>(() => new PartitionScratch());
            protected int VoxelCtX, VoxelCtY, VoxelCtZ;
            protected AABB[, ,] Voxels;
            uint no_of_boxes = 500;
            public List<int>[,,,] Voxel_Inv;
            protected Point BoxDims;
            protected Point BoxDims_Inv;
            protected Point VoxelDims;
            protected Point VoxelDims_Inv;
            protected AABB OBox;
            protected double Epsilon = 0.001;
            protected int XYTot;
            uint rayno = 0;
            object ctlock = new object();

            /// <summary>
            /// The voxel grid constructor. Concept based on Amanatides Fast Ray-Voxel Traversal Algorithm.
            /// </summary>
            /// <param name="Model_in"> The array of Topology to be entered into the Voxel Grid. For a single topology, enter an array with a single topology.</param> 
            public Voxel_Grid(Topology[] Model_in, int Domain)
            {
                if (Domain < 1) throw new ArgumentOutOfRangeException(nameof(Domain));
                //Initialize all variables
                ValidateModel(Model_in); Model = Model_in;
                Point MaxPT = new Point(Double.NegativeInfinity, Double.NegativeInfinity, Double.NegativeInfinity);
                Point MinPT = new Point(Double.PositiveInfinity, Double.PositiveInfinity, Double.PositiveInfinity);
                Poly_Ray_ID = new int[Model.Length, no_of_boxes][]; //bool[Model.Length, System.Environment.ProcessorCount][];



                //Get the max and min points of each topology...
                for (int m = 0; m < Model.Length; m++)
                {
                    if ((Model[m].Max.x + 0.01) > MaxPT.x) MaxPT.x = (Model[m].Max.x + Epsilon);
                    if ((Model[m].Max.y + 0.01) > MaxPT.y) MaxPT.y = (Model[m].Max.y + Epsilon);
                    if ((Model[m].Max.z + 0.01) > MaxPT.z) MaxPT.z = (Model[m].Max.z + Epsilon);
                    if ((Model[m].Min.x - 0.01) < MinPT.x) MinPT.x = (Model[m].Min.x - Epsilon);
                    if ((Model[m].Min.y - 0.01) < MinPT.y) MinPT.y = (Model[m].Min.y - Epsilon);
                    if ((Model[m].Min.z - 0.01) < MinPT.z) MinPT.z = (Model[m].Min.z - Epsilon);
                }

                OBox = new AABB(MinPT.x - .1, MinPT.y - .1, MinPT.z - .1, MaxPT.x + .1, MaxPT.y + .1, MaxPT.z + .1);

                VoxelCtX = Domain;
                VoxelCtY = Domain;
                VoxelCtZ = Domain;
                XYTot = VoxelCtX * VoxelCtY;

                Voxels = new AABB[VoxelCtX, VoxelCtY, VoxelCtZ];
                Voxel_Inv = new List<int>[VoxelCtX, VoxelCtY, VoxelCtZ, Model.Length];

                BoxDims = new Point(OBox.Max.x - OBox.Min.x, OBox.Max.y - OBox.Min.y, OBox.Max.z - OBox.Min.z);
                VoxelDims = new Point(BoxDims.x / VoxelCtX, BoxDims.y / VoxelCtY, BoxDims.z / VoxelCtZ);
                VoxelDims_Inv = new Point(1 / VoxelDims.x, 1 / VoxelDims.y, 1 / VoxelDims.z);
                BoxDims_Inv = new Point(1 / BoxDims.x, 1 / BoxDims.y, 1 / BoxDims.z);
                
                Char_Step = (VoxelDims.x < VoxelDims.y) ? ((VoxelDims.x < VoxelDims.z) ? VoxelDims.x : VoxelDims.z) : (VoxelDims.y < VoxelDims.z ? VoxelDims.y : VoxelDims.z);
                
                int processorCT = System.Environment.ProcessorCount;

                for (int m = 0; m < Model.Length; m++)
                {
                    System.Threading.Thread[] T_List = new System.Threading.Thread[processorCT];
                    for (int P_I = 0; P_I < processorCT; P_I++)
                    {
                        ThreadParams T = new ThreadParams(P_I * VoxelCtX / processorCT, (P_I + 1) * VoxelCtX / processorCT, P_I, m);
                        System.Threading.ParameterizedThreadStart TS = new System.Threading.ParameterizedThreadStart(delegate { Fill_Voxels(T); });
                        T_List[P_I] = new System.Threading.Thread(TS);
                        T_List[P_I].Start();
                    }

                    foreach (System.Threading.Thread worker in T_List) worker.Join();
                }
            }

            /// <summary>
            /// The adaptive version of a voxel grid.
            /// </summary>
            /// <param name="Model_in"></param>
            /// <param name="MaxDomain"></param>
            public Voxel_Grid(Topology[] Model_in, int MaxDomain, int Avg_polys)
            {
                if (MaxDomain < 0) throw new ArgumentOutOfRangeException(nameof(MaxDomain));
                if (Avg_polys < 1) throw new ArgumentOutOfRangeException(nameof(Avg_polys));
                //Initialize all variables
                ValidateModel(Model_in); Model = Model_in;
                Point MaxPT = new Point(Double.NegativeInfinity, Double.NegativeInfinity, Double.NegativeInfinity);
                Point MinPT = new Point(Double.PositiveInfinity, Double.PositiveInfinity, Double.PositiveInfinity);
                Poly_Ray_ID = new int[Model.Length, no_of_boxes][]; //bool[Model.Length, System.Environment.ProcessorCount][];



                //Get the max and min points of each topology...
                for (int m = 0; m < Model.Length; m++)
                {
                    if ((Model[m].Max.x + 0.01) > MaxPT.x) MaxPT.x = (Model[m].Max.x + Epsilon);
                    if ((Model[m].Max.y + 0.01) > MaxPT.y) MaxPT.y = (Model[m].Max.y + Epsilon);
                    if ((Model[m].Max.z + 0.01) > MaxPT.z) MaxPT.z = (Model[m].Max.z + Epsilon);
                    if ((Model[m].Min.x - 0.01) < MinPT.x) MinPT.x = (Model[m].Min.x - Epsilon);
                    if ((Model[m].Min.y - 0.01) < MinPT.y) MinPT.y = (Model[m].Min.y - Epsilon);
                    if ((Model[m].Min.z - 0.01) < MinPT.z) MinPT.z = (Model[m].Min.z - Epsilon);
                }

                OBox = new AABB(MinPT.x - .1, MinPT.y - .1, MinPT.z - .1, MaxPT.x + .1, MaxPT.y + .1, MaxPT.z + .1);

                VoxelCtX = VoxelCtY = VoxelCtZ = XYTot = 1;
                Voxels = new AABB[1,1,1];
                Voxels[0,0,0] = new AABB(MinPT.x - .1, MinPT.y - .1, MinPT.z - .1, MaxPT.x + .1, MaxPT.y + .1, MaxPT.z + .1);
                Voxel_Inv = new List<int>[1, 1, 1, Model.Length];
                for (int i = 0; i < Model.Length; i++) Voxel_Inv[0, 0, 0, i] = new List<int>();
                BoxDims = new Point(OBox.Max.x - OBox.Min.x, OBox.Max.y - OBox.Min.y, OBox.Max.z - OBox.Min.z);

                for (int i = 0; i < Model.Length; i++)
                {
                    for (int j = 0; j < Model[i].Polygon_Count; j++) Voxel_Inv[0, 0, 0, i].Add(j);
                }

                VoxelDims = new Point(BoxDims); VoxelDims_Inv = new Point(1 / BoxDims.x, 1 / BoxDims.y, 1 / BoxDims.z);
                BoxDims_Inv = new Point(VoxelDims_Inv);
                Char_Step = Math.Min(BoxDims.x, Math.Min(BoxDims.y, BoxDims.z));
                //Continuously subdivide voxels until either MaxDomain is reached, or the average number of polygons per voxel is reduced to the goal number...
                for (int k = 0; k < MaxDomain; k++)
                {
                    VoxelCtX = 2 * Voxels.GetLength(0);
                    VoxelCtY = 2 * Voxels.GetLength(1);
                    VoxelCtZ = 2 * Voxels.GetLength(2);
                    XYTot = VoxelCtX * VoxelCtY;

                    AABB[,,] Voxels_temp = new AABB[VoxelCtX, VoxelCtY, VoxelCtZ];
                    List<int>[,,,] Voxel_Inv_temp = new List<int>[VoxelCtX, VoxelCtY, VoxelCtZ, Model.Length];

                    VoxelDims = new Point(BoxDims.x / VoxelCtX, BoxDims.y / VoxelCtY, BoxDims.z / VoxelCtZ);
                    VoxelDims_Inv = new Point(1 / VoxelDims.x, 1 / VoxelDims.y, 1 / VoxelDims.z);
                    BoxDims_Inv = new Point(1 / BoxDims.x, 1 / BoxDims.y, 1 / BoxDims.z);

                    Char_Step = (VoxelDims.x < VoxelDims.y) ? ((VoxelDims.x < VoxelDims.z) ? VoxelDims.x : VoxelDims.z) : (VoxelDims.y < VoxelDims.z ? VoxelDims.y : VoxelDims.z);

                    int processorCT = System.Environment.ProcessorCount;

                    for (int m = 0; m < Model.Length; m++)
                    {
                        System.Threading.Thread[] T_List = new System.Threading.Thread[processorCT];
                        for (int P_I = 0; P_I < processorCT; P_I++)
                        {
                            ThreadParams T_ = new ThreadParams(P_I * VoxelCtX / processorCT, (P_I + 1) * VoxelCtX / processorCT, P_I, m);
                            System.Threading.ParameterizedThreadStart TS = new System.Threading.ParameterizedThreadStart(delegate(object T)
                                {
                                   ThreadParams Tp = (ThreadParams)T;
                                    for (int x = Tp.startvoxel; x < Tp.endvoxel; x++)
                                    {
                                        for (int y = 0; y < VoxelCtY; y++)
                                        {
                                            for (int z = 0; z < VoxelCtZ; z++)
                                            {
                                                Voxel_Inv_temp[x, y, z, Tp.m] = new List<int>();
                                                Point VoxelMin = new Point(x * VoxelDims.x - Epsilon, y * VoxelDims.y - Epsilon, z * VoxelDims.z - Epsilon);
                                                Point VoxelMax = new Point((x + 1) * VoxelDims.x + Epsilon, (y + 1) * VoxelDims.y + Epsilon, (z + 1) * VoxelDims.z + Epsilon);
                                                AABB Box = new AABB(VoxelMin + OBox.Min, VoxelMax + OBox.Min);
                                                Voxels_temp[x, y, z] = Box;
                                                int x_prev = (int)Math.Floor((double)x/2), y_prev = (int)Math.Floor((double)y/2), z_prev = (int)Math.Floor((double)z/2);
                                                foreach(int i in Voxel_Inv[x_prev, y_prev, z_prev, Tp.m])
                                                {
                                                    //Check for intersection between voxel x,y,z with Polygon i...
                                                    if (Box.PolyBoxOverlap(Model[Tp.m].Polys[i].Points))
                                                    {
                                                        Voxel_Inv_temp[x, y, z, Tp.m].Add(i);
                                                    }
                                                }
                                                //Check for Null Voxels
                                                if (Voxel_Inv_temp[x, y, z, Tp.m] == null)
                                                {
                                                    throw new Exception("Whoops... Null Voxels Detected");
                                                }
                                            }
                                        }
                                    }
                                });
                            T_List[P_I] = new System.Threading.Thread(TS);
                            T_List[P_I].Start(T_);
                        }
                        
                        foreach (System.Threading.Thread worker in T_List) worker.Join();
                    }

                    Voxels = Voxels_temp;
                    Voxel_Inv = Voxel_Inv_temp;

                    double sum = 0;
                    int ct = 0;
                    for (int m = 0; m < Model.Length; m++) for (int x = 0; x < Voxels.GetLength(0); x++) for (int y = 0; y < Voxels.GetLength(1); y++) for (int z = 0; z < Voxels.GetLength(2); z++) if (Voxel_Inv_temp[x, y, z, m].Count > 0) { sum += Voxel_Inv_temp[x, y, z, m].Count; ct++; }
                    if (ct == 0 || (k > 1 && sum / ct < Avg_polys)) return; //We are done...
                }
            }

            public void VoxelDecode(int Code, out int X, out int Y, out int Z)
            {
                Z = (int)Math.Floor((double)(Code / XYTot));
                Code -= Z * XYTot;
                X = Code / VoxelCtY;
                Y = Code - X * VoxelCtY;
            }

            public int VoxelCode(int X, int Y, int Z)
            {
                return XYTot * Z + VoxelCtY * X + Y;
            }

            /// <summary>
            /// Adaptive version
            /// </summary>
            /// <param name="o"></param>
            public void Fill_Voxels(object o)
            {
                ThreadParams worker = (ThreadParams)o;
                for (int x = worker.startvoxel; x < worker.endvoxel; x++)
                    for (int y = 0; y < VoxelCtY; y++)
                        for (int z = 0; z < VoxelCtZ; z++)
                        {
                            Voxel_Inv[x, y, z, worker.m] = new List<int>();
                            Voxels[x, y, z] = new AABB(
                                new Point(OBox.Min.x + x * VoxelDims.x - Epsilon, OBox.Min.y + y * VoxelDims.y - Epsilon, OBox.Min.z + z * VoxelDims.z - Epsilon),
                                new Point(OBox.Min.x + (x + 1) * VoxelDims.x + Epsilon, OBox.Min.y + (y + 1) * VoxelDims.y + Epsilon, OBox.Min.z + (z + 1) * VoxelDims.z + Epsilon));
                        }
                // Restrict exact overlap tests to each polygon's candidate cell range.
                for (int id = 0; id < Model[worker.m].Polygon_Count; id++)
                {
                    var polygon = Model[worker.m].Polys[id];
                    var bounds = PartitionBounds.Polygon(polygon);
                    int x0 = Math.Max(worker.startvoxel, Cell(bounds.X0 - Epsilon, OBox.Min.x, VoxelDims.x, VoxelCtX));
                    int x1 = Math.Min(worker.endvoxel - 1, Cell(bounds.X1 + Epsilon, OBox.Min.x, VoxelDims.x, VoxelCtX));
                    int y0 = Cell(bounds.Y0 - Epsilon, OBox.Min.y, VoxelDims.y, VoxelCtY);
                    int y1 = Cell(bounds.Y1 + Epsilon, OBox.Min.y, VoxelDims.y, VoxelCtY);
                    int z0 = Cell(bounds.Z0 - Epsilon, OBox.Min.z, VoxelDims.z, VoxelCtZ);
                    int z1 = Cell(bounds.Z1 + Epsilon, OBox.Min.z, VoxelDims.z, VoxelCtZ);
                    for (int x = x0; x <= x1; x++)
                        for (int y = y0; y <= y1; y++)
                            for (int z = z0; z <= z1; z++)
                                if (Voxels[x, y, z].PolyBoxOverlap(polygon.Points))
                                    Voxel_Inv[x, y, z, worker.m].Add(id);
                }
            }
            private struct ThreadParams
            {
                public int startvoxel;
                public int endvoxel;
                public int threadID;
                public int m;

                public ThreadParams(int start, int end, int thread, int min)
                {
                    startvoxel = start;
                    endvoxel = end;
                    threadID = thread;
                    m = min;
                }
            }

            public void PointInVoxel(Point Pt, out int X, out int Y, out int Z)
            {
                X = (int)Math.Floor((Pt.x - OBox.Min.x) / VoxelDims.x);
                Y = (int)Math.Floor((Pt.y - OBox.Min.y) / VoxelDims.y);
                Z = (int)Math.Floor((Pt.z - OBox.Min.z) / VoxelDims.z);
            }

            public int PointInVoxel(Point Pt)
            {
                return VoxelCode((int)Math.Floor((Pt.x - OBox.Min.x) / VoxelDims.x), (int)Math.Floor((Pt.y - OBox.Min.y) / VoxelDims.y), (int)Math.Floor((Pt.z - OBox.Min.z) / VoxelDims.z));    
            }

            protected uint assign_id()
            {
                lock (ctlock)
                {
                    rayno++;
                    if (rayno == no_of_boxes) { rayno = 0; }
                    for (int m = 0; m < Model.Length; m++)
                        if (Poly_Ray_ID[m, rayno] == null)
                            Poly_Ray_ID[m, rayno] = new int[Model[m].Polygon_Count];
                    return rayno;
                }
            }

            private static void ValidateModel(Topology[] model)
            {
                if (model == null || model.Length == 0) throw new ArgumentException("At least one topology is required.", nameof(model));
                foreach (var topology in model)
                    if (topology == null || !PartitionBounds.Finite(topology.Min.x) ||
                        !PartitionBounds.Finite(topology.Max.x) || !PartitionBounds.Finite(topology.Min.y) ||
                        !PartitionBounds.Finite(topology.Max.y) || !PartitionBounds.Finite(topology.Min.z) ||
                        !PartitionBounds.Finite(topology.Max.z) || topology.Min.x > topology.Max.x ||
                        topology.Min.y > topology.Max.y || topology.Min.z > topology.Max.z)
                        throw new ArgumentException("Grid topologies must have valid bounds.", nameof(model));
            }

            public override bool Shoot(Ray ray, int top_index, out X_Event result)
                => Shoot(ray, top_index, out result, -1, -1);

            /// <summary>Nearest polygon hit. The input ray and its caller-supplied ID are not modified.</summary>
            public override bool Shoot(Ray ray, int top_index, out X_Event result, int poly_origin1, int poly_origin2 = -1)
            {
                if ((uint)top_index >= (uint)Model.Length) throw new ArgumentOutOfRangeException(nameof(top_index));
                result = new X_Event();
                var state = queryScratch.Value;
                state.Begin(Model[top_index].Polygon_Count);
                if (!PartitionBounds.ValidRay(ray)) return false;
                double entry, exit;
                var bounds = PartitionBounds.From(OBox.Min_PT, OBox.Max_PT);
                if (!bounds.Intersect(ray, double.PositiveInfinity, out entry, out exit)) return false;
                int x = Cell(ray.x + entry * ray.dx, OBox.Min_PT.x, VoxelDims.x, VoxelCtX);
                int y = Cell(ray.y + entry * ray.dy, OBox.Min_PT.y, VoxelDims.y, VoxelCtY);
                int z = Cell(ray.z + entry * ray.dz, OBox.Min_PT.z, VoxelDims.z, VoxelCtZ);
                int sx = Math.Sign(ray.dx), sy = Math.Sign(ray.dy), sz = Math.Sign(ray.dz);
                double tx = NextBoundary(ray.x, ray.dx, OBox.Min_PT.x, VoxelDims.x, x, sx);
                double ty = NextBoundary(ray.y, ray.dy, OBox.Min_PT.y, VoxelDims.y, y, sy);
                double tz = NextBoundary(ray.z, ray.dz, OBox.Min_PT.z, VoxelDims.z, z, sz);
                double dx = sx == 0 ? double.PositiveInfinity : Math.Abs(VoxelDims.x / ray.dx);
                double dy = sy == 0 ? double.PositiveInfinity : Math.Abs(VoxelDims.y / ray.dy);
                double dz = sz == 0 ? double.PositiveInfinity : Math.Abs(VoxelDims.z / ray.dz);
                double closest = double.PositiveInfinity, bestU = 0, bestV = 0;
                int bestId = -1; Point bestPoint = null;
                while (true)
                {
                    foreach (int id in Voxel_Inv[x, y, z, top_index])
                    {
                        if (id == poly_origin1 || id == poly_origin2 || !state.FirstTest(id)) continue;
                        Point point; double u, v, t;
                        if (Model[top_index].intersect(id, ray, out point, out u, out v, out t) &&
                            t > 1e-10 && (t < closest || (t == closest && id < bestId)))
                        { closest = t; bestId = id; bestPoint = point; bestU = u; bestV = v; }
                    }
                    double next = Math.Min(tx, Math.Min(ty, tz));
                    // Exact cell boundaries, not padded overlap boxes, determine safe termination.
                    if (closest < next || next > exit || double.IsPositiveInfinity(next)) break;
                    // Visit ties one axis at a time; closed cell memberships preserve edge hits.
                    if (tx <= ty && tx <= tz) { x += sx; tx += dx; }
                    else if (ty <= tz) { y += sy; ty += dy; }
                    else { z += sz; tz += dz; }
                    if (x < 0 || x >= VoxelCtX || y < 0 || y >= VoxelCtY || z < 0 || z >= VoxelCtZ) break;
                }
                if (bestId < 0) return false;
                result = new X_Event(bestPoint, bestU, bestV, closest, bestId);
                return true;
            }

            private static int Cell(double p, double min, double width, int count)
                => Math.Max(0, Math.Min(count - 1, (int)Math.Floor((p - min) / width)));

            private static double NextBoundary(double origin, double direction, double min, double width, int cell, int step)
                => step == 0 ? double.PositiveInfinity : (min + (cell + (step > 0 ? 1 : 0)) * width - origin) / direction;
            public double Xdim
            {
                get 
                {
                    return BoxDims.x;
                }
            }
            public double Ydim
            {
                get
                {
                    return BoxDims.y;
                }
            }
            public double Zdim
            {
                get
                {
                    return BoxDims.z;
                }
            }

            public Point MinPt
            {
                get 
                {
                    return OBox.Min_PT;
                }
            }
        }
    }
}
