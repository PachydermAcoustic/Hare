//'Hare: Accelerated Multi-Resolution Ray Tracing (GPL)
//'
//'Copyright (c) 2008 - 2015, Arthur van der Harten			
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
using System.Collections.Concurrent;

namespace Hare
{
    namespace Geometry
    {
        /// <summary>
        /// A class describing a topological model.
        /// </summary>
        public class Topology
        {
            /// <summary>
            /// A list of vertices.
            /// </summary>
            private System.Collections.Generic.SortedDictionary<ulong, System.Collections.Generic.SortedDictionary<ulong, Point>> Vertices;

            private List<Point> Vertices_List = new List<Point>();

            /// <summary>
            /// A list of polygons.
            /// </summary>
            public List<Polygon> Polys = new List<Polygon>();
            /// <summary>
            /// A list of planes on which polygons lie.
            /// </summary>
            public List<plane> planeList = new List<plane>();
            /// <summary>
            /// The min point of the topology bounding  box.
            /// </summary>
            public Point Min;
            /// <summary>
            /// A list of planes and their polygons.
            /// </summary>
            public List<int>[] Plane_Members = new List<int>[0];
            /// <summary>
            /// The max point of the topology bounding  box.
            /// </summary>
            public Point Max;
            /// <summary>
            /// The number of significant digits to which all points are rounded.
            /// </summary>
            public int Prec;
            public MS_AABB Modspace;

            /// <summary>
            /// A simple constructor which initializes the topology at maximum precision.
            /// </summary>
            private Topology() : this(15) { }

            /// <summary>
            /// A simple constructor which allows the user to choose the precision.
            /// </summary>
            /// <param name="Precision">The number of significant digits the vertices will be rounded to.</param>
            private Topology(int Precision)
            {
                //Vertices = new List<Point>();
                Max = new Point(double.MinValue, double.MinValue, double.MinValue);
                Min = new Point(double.MaxValue, double.MaxValue, double.MaxValue);
                Vertices = new SortedDictionary<ulong, SortedDictionary<ulong, Point>>();
                Prec = Precision;
            }

            public Topology(Point Minpt, Point Maxpt)
            :this()
            {
                this.Max = Maxpt;
                this.Min = Minpt;
                Modspace = new MS_AABB(Min, Max);
            }

            public Topology(Point[][] T, int Precision)
                : this(Precision)
            {
                double Minx = double.MaxValue, Miny = double.MaxValue, Minz = double.MaxValue;
                double Maxx = double.MinValue, Maxy = double.MinValue, Maxz = double.MinValue;

                foreach (Point[] parr in T) foreach (Point p in parr)
                    {
                        if (Minx > p.x) Minx = p.x;
                        if (Miny > p.y) Miny = p.y;
                        if (Minz > p.z) Minz = p.z;
                        if (Maxx < p.x) Maxx = p.x;
                        if (Maxy < p.y) Maxy = p.y;
                        if (Maxz < p.z) Maxz = p.z;
                    }

                Min = new Point(Minx, Miny, Minz);
                Max = new Point(Maxx, Maxy, Maxz);
                Modspace = new MS_AABB(Min, Max);

                Build_Topology(T);
            }

            /// <summary>
            /// Initializes the topology with an explicit list of polygons and vertices.
            /// </summary>
            /// <param name="T">An array of arrays of points. The points represent vertices. The arrays represent the vertices of a single polygon.</param>

            public Topology(Point[][] T)
                : this()
            {
                double Minx = double.MaxValue, Miny = double.MaxValue, Minz = double.MaxValue;
                double Maxx = double.MinValue, Maxy = double.MinValue, Maxz = double.MinValue;

                foreach (Point[] parr in T) foreach (Point p in parr)
                    {
                        if (Minx > p.x) Minx = p.x;
                        if (Miny > p.y) Miny = p.y;
                        if (Minz > p.z) Minz = p.z;
                        if (Maxx < p.x) Maxx = p.x;
                        if (Maxy < p.y) Maxy = p.y;
                        if (Maxz < p.z) Maxz = p.z;
                    }

                Min = new Point(Minx, Miny, Minz);
                Max = new Point(Maxx, Maxy, Maxz);
                Modspace = new MS_AABB(Min, Max);

                Build_Topology(T);
            }

            /// <summary>
            /// Finalizes the topology by updating the axis-aligned bounding box of the model.
            /// </summary>
            public void Finish_Topology()
            {
                //double Minx;
                //double Miny;
                //double Minz;
                //double Maxx;
                //double Maxy;
                //double Maxz;
                //Minx = double.MaxValue;
                //Miny = double.MaxValue;
                //Minz = double.MaxValue;

                //Maxx = double.MinValue;
                //Maxy = double.MinValue;
                //Maxz = double.MinValue;

                //foreach (Point p in Vertices)
                //{
                //    if (p.x < Minx) Minx = p.x;
                //    if (p.y < Miny) Miny = p.y;
                //    if (p.z < Minz) Minz = p.z;

                //    if (p.x > Maxx) Maxx = p.x;
                //    if (p.y > Maxy) Maxy = p.y;
                //    if (p.z > Maxz) Maxz = p.z;
                //}
                //Min = new Point(Minx, Miny, Minz);
                //Max = new Point(Maxx, Maxy, Maxz);
            }

            /// <summary>
            /// Finalizes the topology by updating the axis-aligned bounding box of the model.
            /// </summary>
            /// <param name="EXPTS">A list of points which are not part of the model but need to be included in the axis-aligned bounding box.</param>
            public void Finish_Topology(List<Point> EXPTS)
            {
                double Minx = Modspace.Min.x;
                double Miny = Modspace.Min.y;
                double Minz = Modspace.Min.z;
                double Maxx = Modspace.Max.x;
                double Maxy = Modspace.Max.y;
                double Maxz = Modspace.Max.z;

                //Minx = double.MaxValue;
                //Miny = double.MaxValue;
                //Minz = double.MaxValue;
                //Maxx = double.MinValue;
                //Maxy = double.MinValue;
                //Maxz = double.MinValue;

                //for (int i = 0; i < Vertices.Count; i++) for (int j = 0; j < Vertices[i].Count; j++) foreach (Point p in Vertices[i][j])
                //{
                //    if (p.x < Minx) Minx = p.x;
                //    if (p.y < Miny) Miny = p.y;
                //    if (p.z < Minz) Minz = p.z;

                //    if (p.x > Maxx) Maxx = p.x;
                //    if (p.y > Maxy) Maxy = p.y;
                //    if (p.z > Maxz) Maxz = p.z;
                //}

                foreach (Point p in EXPTS)
                {
                    if (p.x < Minx) Minx = p.x;
                    if (p.y < Miny) Miny = p.y;
                    if (p.z < Minz) Minz = p.z;

                    if (p.x > Maxx) Maxx = p.x;
                    if (p.y > Maxy) Maxy = p.y;
                    if (p.z > Maxz) Maxz = p.z;
                }

                Min = new Point(Minx, Miny, Minz);
                Max = new Point(Maxx, Maxy, Maxz);
            }

            public void Add_Polygon(Point[] P, int Plane_ID)
            {
                List<Point> VertexList = new List<Point>(P.Length);

                if (Plane_ID > Plane_Members.Length-1)
                {
                    Array.Resize<List<int>>(ref Plane_Members, Plane_ID+1);
                    for (int i = 0; i < Plane_ID+1; i++)
                    {
                        if (Plane_Members[Plane_ID] == null) Plane_Members[Plane_ID] = new List<int>();
                    }
                }

                Plane_Members[Plane_ID].Add(Polys.Count);

                for (int p = 0; p < P.Length; p++)
                {
                    VertexList.Add(this.AddGetIndex(P[p]));
                }

                //Array.Resize(ref Polys, Polys.Length + 1);
                if (P.Length == 4)
                {
                    lock (Top_Lock) Polys.Add(new Quadrilateral(ref VertexList, 0, Polys.Count - 1, Plane_ID));
                    //Polys[Polys.Count - 1] = new Quadrilateral(ref Vertices, VertexList, 0, Polys.Length - 1);
                }
                else if (P.Length == 3)
                {
                    lock (Top_Lock) Polys.Add(new Triangle(ref VertexList, 0, Polys.Count - 1, Plane_ID));
                    //Polys[Polys.Count - 1] = new Triangle(ref Vertices, VertexList, 0, Polys.Length - 1);
                }
                else
                {
                    throw new NotImplementedException("Hare Does not yet support polygons of more than 4 sides.");
                }
            }

            /// <summary>
            /// Adds a polygon to the topology. Can be used immediately after initializing the topology.
            /// </summary>
            /// <param name="P">An array of Points which make up the vertices of the polygon.</param>
            public void Add_Polygon(Point[] P)
            {
                List<Point> VertexList = new List<Point>(P.Length);

                for (int p = 0; p < P.Length; p++)
                {
                    VertexList.Add(this.AddGetIndex(P[p]));
                }

                //Array.Resize(ref Polys, Polys.Length + 1);
                if (P.Length == 4)
                {
                    lock (Top_Lock) Polys.Add(new Quadrilateral(ref VertexList, 0, Polys.Count));
                    //Polys[Polys.Count - 1] = new Quadrilateral(ref Vertices, VertexList, 0, Polys.Length - 1);
                }
                else if (P.Length == 3)
                {
                    lock(Top_Lock) Polys.Add(new Triangle(ref VertexList, 0, Polys.Count));
                    //Polys[Polys.Count - 1] = new Triangle(ref Vertices, VertexList, 0, Polys.Length - 1);
                }
                else
                {
                    throw new NotImplementedException("Hare Does not yet support polygons of more than 4 sides.");
                }

                plane p1 = new plane(Polys[Polys.Count-1]);
                for(int i = 0; i < planeList.Count; i++) if (p1.GetHashCode() == planeList[i].GetHashCode())
                {
                    Polys[Polys.Count - 1].Plane_ID = i;
                    Plane_Members[i].Add(Polys.Count - 1);
                    return;
                }
                planeList.Add(p1);
                Array.Resize<List<int>>(ref Plane_Members, Plane_Members.Length+1);
                Plane_Members[Plane_Members.Length-1] = new List<int>();
                Plane_Members[Plane_Members.Length-1].Add(Polys.Count-1);
                Polys[Polys.Count - 1].Plane_ID = planeList.Count-1;
            }

            Object Top_Lock = new Object();

            public void Build_Topology(Point[][] T)
            {
                Min = new Point(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
                Max = new Point(double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity);

                Vertices = new SortedDictionary<ulong,SortedDictionary<ulong,Point>>();
                Polys = new List<Polygon>(T.Length);

                //for (int i = 0; i < T.Length; i++)
                System.Threading.Tasks.Parallel.For(0, T.Length, i => 
                {
                    List<Point> VertexList = new List<Point>();
                    for (int p = 0; p < T[i].Length; p++)
                    {
                        VertexList.Add(this.AddGetIndex(T[i][p]));
                    }

                    if (VertexList.Count == 4)
                    {
                        lock (Top_Lock) Polys.Add(new Quadrilateral(ref VertexList, 0, Polys.Count));    
                    }
                    else if (VertexList.Count == 3)
                    {
                        lock (Top_Lock) Polys.Add(new Triangle(ref VertexList, 0, Polys.Count));
                    }
                    else
                    {
                            throw new NotImplementedException("Hare Does not yet support polygons of more than 4 sides.");
                    }
                });
            }

            /// <summary>
            /// Checks to see if the topology has a given point. If it does, it returns the index of the point. If not, it adds it and returns the index of the new point.
            /// </summary>
            /// <param name="x">The point to be found or added.</param>
            /// <returns>The index of the point.</returns>
            //private int AddGetIndex(Point x)
            //{
            //    x.Round(Prec);
            //    try
            //    {
            //        for (int q = 0; q < Vertices.Length; q++)
            //        {
            //            if (Vertices[q].x == x.x && Vertices[q].y == x.y && Vertices[q].z == x.z)
            //            {
            //                return q;
            //            }
            //        }
            //        Array.Resize(ref Vertices, Vertices.Length + 1);
            //        Vertices[Vertices.Length - 1] = x;
            //        return Vertices.Length - 1;
            //    }
            //    catch (NullReferenceException)
            //    {
            //        Vertices[0] = x;
            //        return 0;
            //    }
            //}

            private Point AddGetIndex(Point x)
            {
                //Identify which grid point the point is located in...

                x.Round(Prec);
                lock (Top_Lock)
                {
                    SortedDictionary<ulong, Point> D;
                    Point p;
                    ulong Hash_1, Hash_2;
                    x.Hash2(this.Modspace, out Hash_1, out Hash_2);

                    if (Vertices.TryGetValue(Hash_1, out D))
                    {
                        if (D.TryGetValue(Hash_2, out p))
                        {
                            return p;//Vertices[q];
                        }
                        else
                        {
                            D.Add(Hash_2, x);
                            Vertices_List.Add(x);
                        }
                    }
                    else 
                    {
                        D = new SortedDictionary<ulong,Point>();
                        D.Add(Hash_2, x);
                        Vertices.Add(Hash_1, D);
                        Vertices_List.Add(x);
                    }
                }

                //if (x.x < Min.x) Min.x = x.x;
                //if (x.y < Min.y) Min.y = x.y;
                //if (x.z < Min.z) Min.z = x.z;

                //if (x.x > Max.x) Max.x = x.x;
                //if (x.y > Max.y) Max.y = x.y;
                //if (x.z > Max.z) Max.z = x.z;

                return x;
            }

            /// <summary>
            /// Returns a point referenced by a given index.
            /// </summary>
            /// <param name="index">The point index.</param>
            /// <returns>The indexed point.</returns>
            public Point this[int index]
            {
                get
                {
                    return this.Vertices_List[index];
                }
            }
            
            /// <summary>
            /// Returns a point referenced by a given polygon and vertex index.
            /// </summary>
            /// <param name="Poly_ID">The index of the polygon.</param>
            /// <param name="Corner_ID">The index of the vertex of a given polygon.</param>
            /// <returns>The indexed point.</returns>
            public Point this[int Poly_ID, int Corner_ID]
            {
                get
                {
                    return Polys[Poly_ID].Points[Corner_ID];
                }
            }

            /// <summary>
            /// Checks a ray for an intersection with a given polygon.
            /// </summary>
            /// <param name="Poly_ID">The index of the polygon to be checked.</param>
            /// <param name="R">The ray to be intersected with the polygon.</param>
            /// <param name="X">An event describing the intersection of the ray with the polygon.</param>
            /// <returns>true if ray intersects, false if not.</returns>
            public bool intersect(int Poly_ID, Ray R, out X_Event X)
            {
                double u, v, t;
                Point P;
                Polys[Poly_ID].Intersect(R,this.Polygon_Vertices(Poly_ID), out P, out u, out v, out t, out Poly_ID);

                X = new X_Event(P, u, v, t, Poly_ID);
                return true;
            }

            /// <summary>
            /// Checks a ray for an intersection with a given polygon.
            /// </summary>
            /// <param name="Poly_ID">The index of the polygon to be checked.</param>
            /// <param name="R">The ray to be intersected with the polygon.</param>
            /// <param name="X">An event describing the intersection of the ray with the polygon.</param>
            /// <returns>true if ray intersects, false if not.</returns>
            public bool intersect(int Poly_ID, Ray R, out Point P, out double u, out double v, out double t)
            {
                return Polys[Poly_ID].Intersect(R, this.Polygon_Vertices(Poly_ID), out P, out u, out v, out t, out Poly_ID);
            }


            /// <summary>
            /// Gets the array of vertices that make up a polygon.
            /// </summary>
            /// <param name="Poly_ID">The index of the polygon.</param>
            /// <returns>an array of points.</returns>
            public Point[] Polygon_Vertices(int Poly_ID)
            {
                Point[] P = new Point[Polys[Poly_ID].VertexCount];
                for (int i = 0; i < P.Length; i++)
                {
                    P[i] = Polys[Poly_ID].Points[i];
                }
                return P;
            }

            /// <summary>
            /// Gets the number of polygons in the topology.
            /// </summary>
            public int Polygon_Count 
            {
                get 
                {
                    return Polys.Count;
                }
            }

            /// <summary>
            /// Gets the number of vertices in the topology.
            /// </summary>
            public int Vertex_Count
            {
                get
                {
                    return Vertices_List.Count;
                }
            }

            /// <summary>
            /// Gets the normal of a referenced polygon.
            /// </summary>
            /// <param name="Poly_ID">The index of the polygon.</param>
            /// <returns>the normal of the polygon.</returns>
            public Vector Normal(int Poly_ID)
            {
                return Polys[Poly_ID].Normal;
            }

            /// <summary>
            /// Gets the area of the polygon.
            /// Adapted from http://www.softsurfer.com/Archive/algorithm_0101/algorithm_0101.htm, by Dan Sunday.
            /// </summary>
            /// <param name="Poly_ID">The index of the polygon.</param>
            /// <returns>the area of the polygon.</returns>
            public double Polygon_Area(int Poly_ID)
            {
                double area = 0;

                for (int i = 1, j = 2, k = 0; j < Polys[Poly_ID].VertexCount; i++, j++)
                {
                    area += .5 * Hare.Geometry.Hare_math.Cross(Polys[Poly_ID].Points[i] - Polys[Poly_ID].Points[k], Polys[Poly_ID].Points[i] - Polys[Poly_ID].Points[j]).Length();
                }
                return area;
            }

            /// <summary>
            /// Gets the centroid of a referenced polygon.
            /// </summary>
            /// <param name="Poly_ID">The index of the polygon.</param>
            /// <returns>the centroid of the polygon.</returns>
            public Point Polygon_Centroid(int Poly_ID)
            {
                Point P = new Point();
                for (int i = 0; i < Polys[Poly_ID].VertexCount; i++)
                {
                    P += Polys[Poly_ID].Points[i];
                }

                return P / Polys[Poly_ID].VertexCount;
            }

            /// <summary>
            /// Gets the distance from a point to the closest point on a plane.
            /// </summary>
            /// <param name="P">The point to reference.</param>
            /// <param name="Poly_ID">The polygon from which the plane to be used will be taken.</param>
            /// <returns>the distance from the point to the plane.</returns>
            public double DistToPlane (Point P, int Poly_ID)
            {
                return Polys[Poly_ID].DistToPlane(P);
            }

            /// <summary>
            /// Gets the closest point on a polygon to a given point.
            /// </summary>
            /// <param name="P">The point to reference.</param>
            /// <param name="Poly_ID">The index of the polygon.</param>
            /// <returns>The point on the polygon</returns>
            public Point Closest_Point(Point P, int Poly_ID)
            {
                Point[] PT = new Point[Polys[Poly_ID].VertexCount -2];
                for (int p = 0, i = 0, j = 1, k = 2; k < Polys[Poly_ID].VertexCount;p++, j++, k++)
                {
                    PT[p] = TriangleClosestPt(P, Polys[Poly_ID].Points[i], Polys[Poly_ID].Points[j], Polys[Poly_ID].Points[k]);
                }
                double PS, AS;
                Point A = PT[0];
                AS = A.x * A.x + A.y * A.y + A.z * A.z;
                for (int p = 1; p < PT.Length; p++)
                {
                    PS = PT[p].x * PT[p].x + PT[p].y * PT[p].y + PT[p].z * PT[p].z;
                    if (AS < PS)
                    {
                        AS = PS;
                        A = PT[p];
                    }
                }
                return A;                 
            }

            /// <summary>
            /// The closest point on a triangle to a given point.
            /// </summary>
            /// <param name="p">The referenced point.</param>
            /// <param name="a">Vertex a</param>
            /// <param name="b">Vertex b</param>
            /// <param name="c">Vertex c</param>
            /// <returns>The closest point.</returns>
            protected Point TriangleClosestPt(Point p, Point a, Point b, Point c)
            {
                // Check if P in vertex region outside A
                Vector ab = b - a;
                Vector ac = c - a;
                Vector ap = p - a;
                double d1 = Hare_math.Dot(ab, ap);
                double d2 = Hare_math.Dot(ac, ap);
                if (d1 <= 0.0f && d2 <= 0.0f) return a; // barycentric coordinates (1,0,0)

                // Check if P in vertex region outside B
                Vector bp = p - b;
                double d3 = Hare_math.Dot(ab, bp);
                double d4 = Hare_math.Dot(ac, bp);
                if (d3 >= 0.0f && d4 <= d3) return b; // barycentric coordinates (0,1,0)

                // Check if P in edge region of AB, if so return projection of P onto AB
                double vc = d1 * d4 - d3 * d2;
                if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
                {
                    double v = d1 / (d1 - d3);
                    return a + v * ab; // barycentric coordinates (1-v,v,0)
                }

                // Check if P in vertex region outside C
                Vector cp = p - c;
                double d5 = Hare_math.Dot(ab, cp);
                double d6 = Hare_math.Dot(ac, cp);
                if (d6 >= 0.0f && d5 <= d6) return c; // barycentric coordinates (0,0,1)

                // Check if P in edge region of AC, if so return projection of P onto AC
                double vb = d5 * d2 - d1 * d6;
                if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
                {
                    double w = d2 / (d2 - d6);
                    return a + w * ac; // barycentric coordinates (1-w,0,w)
                }

                // Check if P in edge region of BC, if so return projection of P onto BC
                double va = d3 * d6 - d5 * d4;
                if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
                {
                    double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                    return b + w * (c - b); // barycentric coordinates (0,1-w,w)
                }

                // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
                double denom = 1.0f / (va + vb + vc);
                double vl = vb * denom;
                double wl = vc * denom;
                return a + ab * vl + ac * wl; // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
            }

            public class MS_AABB : AABB
            {
                public int xdim, ydim, zdim, XYTot;   

                public MS_AABB(Point Point_Min_in, Point Point_Max_in)
                    :base(Point_Min_in, Point_Max_in)
                {

                    double xl = this.X_Length(), yl = this.Y_Length(), zl = this.Z_Length();

                    int max = Math.Max((int)Math.Ceiling(xl), Math.Max((int)Math.Ceiling(yl), (int)Math.Ceiling(zl)));

                    xdim = ydim = zdim = max;

                    //double x_2 = ((double)xdim - xl) / 2, y_2 = ((double)ydim - yl) / 2, z_2 = ((double)zdim - zl) / 2;
                    //Min_PT.x -= x_2;
                    //Min_PT.y -= y_2;
                    //Min_PT.z -= z_2;
                    XYTot = xdim * ydim;
                }
            }
        }
    }
}