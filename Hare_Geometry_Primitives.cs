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
using System.Runtime.CompilerServices;
namespace Hare
{
    namespace Geometry
    {
        /// <summary>
        /// Simple vector type variable, complete with certain common operators...
        /// </summary>
        public class Vector
        {
            public double dx, dy, dz;
            public Vector(Point p)
            {
                dx = p.x;
                dy = p.y;
                dz = p.z;
            }

            public Vector(double x_in, double y_in, double z_in)
            {
                dx = x_in;
                dy = y_in;
                dz = z_in;
            }

            public Vector()
                : base()
            {
            }

            public void Normalize()
            {
                double factor = dx * dx + dy * dy + dz * dz;
                if (factor == 0) return;
                factor = Math.Sqrt(factor);
                dx /= factor;
                dy /= factor;
                dz /= factor;
            }

            public double Length()
            {
                return Math.Sqrt(dx * dx + dy * dy + dz * dz);
            }

            public static Point operator +(Vector P, Point Q)
            {
                return new Point(P.dx + Q.x, P.dy + Q.y, P.dz + Q.z);
            }

            public static Vector operator +(Vector P, Vector Q)
            {
                return new Vector(P.dx + Q.dx, P.dy + Q.dy, P.dz + Q.dz);
            }

            public static Vector operator -(Vector P, Point Q)
            {
                return new Vector(P.dx - Q.x, P.dy - Q.y, P.dz - Q.z);
            }
            public static Vector operator -(Vector P, Vector Q)
            {
                return new Vector(P.dx - Q.dx, P.dy - Q.dy, P.dz - Q.dz);
            }

            public static Vector operator *(Vector P, double Q)
            {
                return new Vector(P.dx * Q, P.dy * Q, P.dz * Q);
            }

            public static Vector operator *(double Q, Vector P)
            {
                return new Vector(P.dx * Q, P.dy * Q, P.dz * Q);
            }

            public static Vector operator /(Vector P, double Q)
            {
                return new Vector(P.dx / Q, P.dy / Q, P.dz / Q);
            }

            /// <summary>
            /// Access x(0), y(1), z(2) variables by integers. (ie. for use with integer based loops)
            /// </summary>
            /// <param name="i"> Integer indicating which variable to return. x(0) : y(1) : z(2) </param>
            /// <returns> The double value of the variable. </returns>
            public double byint(int i)
            {
                switch (i)
                {
                    case 0:
                        return dx;
                    case 1:
                        return dy;
                    case 2:
                        return dz;
                }
                throw new Exception("Misrepresented Point-Dimension Index");
            }

            /// <summary>
            /// Checks for a vector of infinitesimal length.
            /// </summary>
            /// <returns> True or False... </returns>
            public bool IsZeroVector()
            {
                if ((dx * dx + dy * dy + dz * dz) < double.Epsilon) return true;
                return false;
            }
        }

        /// <summary>
        /// Simple point type variable, complete with certain common operators...
        /// </summary>
        public class Point
        {
            public double x;
            public double y;
            public double z;
            public int code;

            public Point(Point p)
                : this(p.x, p.y, p.z)
            {
            }

            public Point(double x_in, double y_in, double z_in)
            {
                x = x_in;
                y = y_in;
                z = z_in;

                code = (int)((((497 + x * 1000) * 71 + y * 1000) * 71) + z * 1000);
            }

            public Point()
            {
            }

            public static Point operator +(Point P, Point Q)
            {
                return new Point(P.x + Q.x, P.y + Q.y, P.z + Q.z);
            }

            public static Point operator +(Point P, Vertex Q)
            {
                return new Point(P.x + Q.x, P.y + Q.y, P.z + Q.z);
            }

            public static Point operator +(Point P, Vector Q)
            {
                return new Point(P.x + Q.dx, P.y + Q.dy, P.z + Q.dz);
            }

            public static Vector operator -(Point P, Point Q)
            {
                return new Vector(P.x - Q.x, P.y - Q.y, P.z - Q.z);
            }

            public static Point operator -(Point P, Vector Q)
            {
                return new Point(P.x - Q.dx, P.y - Q.dy, P.z - Q.dz);
            }


            public static Point operator *(Point P, double Q)
            {
                return new Point(P.x * Q, P.y * Q, P.z * Q);
            }

            public static Point operator *(double Q, Point P)
            {
                return new Point(P.x * Q, P.y * Q, P.z * Q);
            }

            public static Point operator /(Point P, double Q)
            {
                return new Point(P.x / Q, P.y / Q, P.z / Q);
            }

            /// <summary>
            /// Access x(0), y(1), z(2) variables by integers. (ie. for use with integer based loops)
            /// </summary>
            /// <param name="i"> Integer indicating which variable to return. x(0) : y(1) : z(2) </param>
            /// <returns> The double value of the variable. </returns>
            public double byint(int i)
            {
                switch (i)
                {
                    case 0:
                        return x;
                    case 1:
                        return y;
                    case 2:
                        return z;
                }
                throw new Exception("Misrepresented Point-Dimension Index");
            }

            /// <summary>
            /// Checks for a vector of infinitesimal length.
            /// </summary>
            /// <returns> True or False... </returns>
            public bool IsZeroVector()
            {
                if ((x * x + y * y + z * z) < double.Epsilon) return true;
                return false;
            }

            /// <summary>
            /// Rounds the variables of the point to a given precision... (Not adviseable...)
            /// </summary>
            /// <param name="Precision">The number of siginificant digits to round to...</param>
            public void Round(int Precision)
            {
                x = Math.Round(x, Precision);
                y = Math.Round(y, Precision);
                z = Math.Round(z, Precision);
            }

            public void Hash2(Topology.MS_AABB Modelspace, out ulong bucket, out ulong pos)
            {
                double Xoff = x - Modelspace.Min.x, Yoff = y - Modelspace.Min.y, Zoff = z - Modelspace.Min.z;

                UInt64 xlocus = (UInt64)Math.Floor(Xoff);
                UInt64 ylocus = (UInt64)Math.Floor(Yoff);
                UInt64 zlocus = (UInt64)Math.Floor(Zoff);

                bucket = (ulong)Modelspace.XYTot * zlocus + (ulong)Modelspace.ydim * xlocus + ylocus;

                UInt64 xpos = (UInt64)((Xoff - xlocus) * 1000), ypos = (UInt64)((Yoff - ylocus) * 1000), zpos = (UInt64)((Zoff - zlocus) * 1000);

                pos = 1000000 * zpos + 1000 * xpos + ypos;
            }
        }

        public class Vertex : Point
        {
            public int index;
            public System.Collections.Generic.List<Polygon> Polys = new System.Collections.Generic.List<Polygon>();
            public System.Collections.Generic.List<Edge> Edges = new System.Collections.Generic.List<Edge>();

            public Vertex(Point p, int id)
                : base(p.x, p.y, p.z)
            {
                index = id;
            }

            public Vertex(double x, double y, double z, int id)
                : base(x, y, z)
            {
                index = id;
            }
        }

        public class Edge
        {
            public System.Collections.Generic.List<Polygon> Polys = new System.Collections.Generic.List<Polygon>();
            public System.Collections.Generic.List<double> TributaryArea = new System.Collections.Generic.List<double>();
            public System.Collections.Generic.List<double> TributaryLength = new System.Collections.Generic.List<double>();
            public System.Collections.Generic.List<Hare.Geometry.Vector> Tangents = new System.Collections.Generic.List<Vector>();
            Vertex[] pts = new Vertex[2];
            int code;

            public Edge(Vertex a, Vertex b)
            {
                pts[0] = a;
                pts[1] = b;
                code = Hash(a, b);
            }

            public void Append_Poly_Relationship(Polygon p)
            {
                Polys.Add(p);
                TributaryArea.Add(0.5 * Hare_math.Cross((pts[1] - pts[0]), (p.Centroid - pts[0])).Length());
                Vector ab = pts[1] - pts[0];
                Vector av = p.Centroid - pts[0];
                Vector tan = p.Centroid - closestpoint(p.Centroid);
                //TributaryLength.Add(Hare.Geometry.Hare_math.Cross(ab, av).Length() / ab.Length());
                TributaryLength.Add(tan.Length());
                tan.Normalize();
                Tangents.Add(tan);
            }

            public Hare.Geometry.Point closestpoint(Hare.Geometry.Point P)
            {
                double abx = b.x - a.x;
                double aby = b.y - a.y;
                double abz = b.z - a.z;
                double P_ax = P.x - a.x;
                double P_ay = P.y - a.y;
                double P_az = P.z - a.z;

                double t = Hare_math.Dot(P_ax, P_ay,P_az, abx, aby, abz) / Hare_math.Dot(abx, aby, abz, abx, aby, abz);
                //if (t < 0) t = 0;
                //else if (t > 1) t = 1;
                return new Point(a.x + t * abx, a.y + t * aby, a.z + t * abz);
            }

            public Vertex a
            {
                get
                {
                    return pts[0];
                }
            }

            public Vertex b
            {
                get
                {
                    return pts[1];
                }
            }

            public Point mid
            {
                get
                {
                    return (a + b) / 2;
                }
            }

            public static System.Int64 Hash(Point a, Point b, Topology.MS_AABB mod)
            {
                Point pt1, pt2;
                if (a.x == b.x)
                {
                    if (a.y == b.y)
                    {
                        if (a.z == b.z) throw new Exception("Zero-length edge...");
                        else if (a.z > b.z) { pt1 = a; pt2 = b; }
                        else { pt2 = a; pt1 = b; }
                    }
                    else if (a.y > b.y) { pt1 = a; pt2 = b; }
                    else { pt2 = a; pt1 = b; }
                }
                else if (a.x > b.x) { pt1 = a; pt2 = b; }
                else { pt2 = a; pt1 = b; }
                ulong[] h = new ulong[4];
                pt1.Hash2(mod, out h[0], out h[1]);
                pt2.Hash2(mod, out h[2], out h[3]);

                return (System.Int64)(h[0] + 5 * h[1] + 11 * h[2] + 17 * h[3]);
            }

            public static int Hash(Point a, Point b)
            {
                Point pt1, pt2;
                if (a.x == b.x)
                {
                    if (a.y == b.y)
                    {
                        if (a.z == b.z) throw new Exception("Zero-length edge...");
                        else if (a.z > b.z) { pt1 = a; pt2 = b; }
                        else { pt2 = a; pt1 = b; }
                    }
                    else if (a.y > b.y) { pt1 = a; pt2 = b; }
                    else { pt2 = a; pt1 = b; }
                }
                else if (a.x > b.x) { pt1 = a; pt2 = b; }
                else { pt2 = a; pt1 = b; }

                int hash = (int)(pt1.x * 100);
                hash = hash * 5 + (int)(pt2.x * 100);
                hash = hash * 11 + (int)(pt1.y * 100);
                hash = hash * 17 + (int)(pt2.y * 100);
                hash = hash * 23 + (int)(pt1.z * 100);
                hash = hash * 29 + (int)(pt2.z * 100);
                return hash;
            }
        }

        /// <summary>
        /// A ray class storing all relevant data to accurately cast a ray.
        /// </summary>
        public class Ray
        {
            public Ray(double x, double y, double z, double dx, double dy, double dz, int ThreadID_IN, int ID)
            {
                this.x = x; this.y = y; this.z = z; this.dx = dx; this.dy = dy; this.dz = dz;
                ThreadID = ThreadID_IN; Ray_ID = ID;
                //a = 0;
            }

            public Ray(Point o, Vector d, int Thread_ID, int ID_IN)
            {
                Ray_ID = ID_IN;
                x = o.x;
                y = o.y;
                z = o.z;
                dx = d.dx;
                dy = d.dy;
                dz = d.dz;
                ThreadID = Thread_ID;
                //a = 0;
            }

            public void Reverse()
            {
                dx *= -1;
                dy *= -1; 
                dz *= -1;
            }

            public int ThreadID;
            public double x, y, z;
            public double dx, dy, dz;
            public int poly_origin1;
            public int poly_origin2;
            //public int a;
            public int Ray_ID;
        }


        /// <summary>
        /// A structure containing all data about the intersection resulting from a "shoot" operation in a spatial partition class.
        /// </summary>
        public class X_Event
        {
            public readonly double u;
            public readonly double v;
            public double t;
            public readonly bool Hit;
            public readonly Point X_Point;
            public readonly int Poly_id;
            /// <summary>
            /// Used to represent either an empty intersection event, or a no-intersection event...
            /// </summary>
            public X_Event(int Poly_index)
            {
                X_Point = default(Point);
                t = 0;
                Hit = false;
                Poly_id = Poly_index;
            }

            public X_Event()
            {
                u = 0;
                v = 0;
                t = 0;
                Hit = false;
                X_Point = default(Point);
                Poly_id = -1;
            }

            /// <summary>
            /// Used to store information about a discovered intersection event...
            /// </summary>
            /// <param name="P"> The intersection point. </param>
            /// <param name="u_in"> U coordinate of the intersection. </param>
            /// <param name="v_in"> V coordinate of the intersection.</param>
            /// <param name="t_in"> The distance along the ray in units related to the ray's length. If raylength is 1, this is the actual travel distance of the ray in model units. </param>
            /// <param name="Poly_index"> The index of the polygon upon which the intersection point may be found. </param>
            public X_Event(Point P, double u_in, double v_in, double t_in, int Poly_index)
            {
                u = u_in;
                v = v_in;
                t = t_in;
                Hit = true;
                X_Point = P;
                Poly_id = Poly_index;
            }
        }

        public class Plane
        {
            double a, b, c, d;
            int code;

            public Plane(Polygon p)
            {
                a = p.Normal.dx;
                b = p.Normal.dy;
                c = p.Normal.dz;
                d = -(a * p.Points[0].x + b * p.Points[0].y + c * p.Points[0].z);

                //a = (float)(p.Points[0].y * (p.Points[1].z - p.Points[2].z) + p.Points[2].y * (p.Points[2].z - p.Points[0].z) + p.Points[2].y * (p.Points[0].z - p.Points[1].z));
                //b = (float)(p.Points[0].z * (p.Points[1].x - p.Points[2].x) + p.Points[1].z * (p.Points[2].x - p.Points[0].x) + p.Points[2].z * (p.Points[0].x - p.Points[1].x));
                //c = (float)(p.Points[0].x * (p.Points[1].y - p.Points[2].y) + p.Points[1].x * (p.Points[2].y - p.Points[0].y) + p.Points[2].x * (p.Points[0].y - p.Points[1].y));
                //d = (float)-(p.Points[0].x * (p.Points[1].y * p.Points[2].z - p.Points[2].y * p.Points[1].z) + p.Points[1].x * (p.Points[2].y * p.Points[0].z - p.Points[0].y * p.Points[2].z) + p.Points[2].x * (p.Points[0].y * p.Points[1].z - p.Points[1].y * p.Points[0].z));

                if (d < 0)
                {
                    a *= -1;
                    b *= -1;
                    c *= -1;
                    d *= -1;
                }

                code = (int)(((Math.Round(d, 3).GetHashCode() * 7 + Math.Round(a, 3).GetHashCode()) * 11 + Math.Round(b, 3).GetHashCode()) * 17 + Math.Round(c, 3).GetHashCode());
            }

            public override int GetHashCode()
            {
                return code;
            }
        }
    }
}